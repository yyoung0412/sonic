"""One owned ROS camera publisher shared by real/sim readers and recording."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

from console_support import ConsoleError, age, atomic_json, read_json
from ros2_camera_bridge import validate_ros2_config
from g1_zmq_ros2_bridge import validate_g1_zmq_ros2_config


class CameraSourceManager:
    def __init__(self, driver):
        self.driver = driver
        self.directory = driver.directory/"camera-bridge"
        self.ingress_directory = driver.directory/"camera-zmq-ros2"

    def settings(self, mode):
        config = self.driver.config(mode)
        if config["camera_source"] != "ros2":
            return None
        if not config["camera_endpoint"].startswith("tcp://127.0.0.1:"):
            raise ConsoleError("托管 ROS 2 桥接仅绑定 127.0.0.1；远程 ZMQ 来源请选 zmq", 412, "camera_config")
        ingress = validate_g1_zmq_ros2_config(config["g1_zmq_ros2"], mode)
        if not ingress["enabled"]:
            # Disabled ingress details have no effect on the shared ROS reader.
            # Canonicalizing them preserves the existing real/sim process reuse
            # when both modes have the same active ROS camera configuration.
            ingress = validate_g1_zmq_ros2_config({"enabled": False}, "sim")
        result = {"ros2_camera": validate_ros2_config(config["ros2_camera"]),
                  "g1_zmq_ros2": ingress,
                  "camera_endpoint": config["camera_endpoint"], "camera_keys": config["camera_keys"]}
        code = b"".join((self.driver.app_root/name).read_bytes() for name in
                        ("ros2_camera_bridge.py", "g1_zmq_ros2_bridge.py", "console_support.py"))
        result["_worker_version"] = hashlib.sha256(code+json.dumps(result, sort_keys=True).encode()).hexdigest()[:20]
        return result

    def _ingress_status(self, settings):
        if not settings["g1_zmq_ros2"]["enabled"]:
            return {"enabled": False, "state": "disabled"}
        row = read_json(self.ingress_directory/"status.json")
        running = self.driver.supervisor.running("camera-zmq-ros2")
        record = self.driver.supervisor.record("camera-zmq-ros2") or {}
        same_process = row.get("pid") == record.get("pid")
        valid = running and same_process and row.get("worker_version") == settings["_worker_version"] and age(row.get("heartbeat_at")) < 3
        state = row.get("state", "waiting") if valid else "stopped" if not running else "starting"
        if same_process and row.get("worker_version") == settings["_worker_version"] and row.get("state") == "failed":
            state = "failed"
        return {"enabled": True, "state": state,
                "endpoint": settings["g1_zmq_ros2"]["endpoint"],
                "required_complete_set": True,
                "packets": row.get("packets", 0) if valid else 0,
                "rejected": row.get("rejected", 0) if valid else 0,
                "counts": row.get("counts", {}) if valid else {},
                "source_timestamps": row.get("source_timestamps", {}) if valid else {},
                "last_error": row.get("error") or row.get("last_error", "") if valid else ""}

    def status(self, mode):
        settings = self.settings(mode)
        if settings is None:
            return {"kind": "zmq", "state": "external", "message": "接收外部 ZMQ 图像发布流"}
        row = read_json(self.directory/"status.json")
        running = self.driver.supervisor.running("camera-bridge")
        record = self.driver.supervisor.record("camera-bridge") or {}
        same_process = row.get("pid") == record.get("pid")
        valid = running and same_process and row.get("worker_version") == settings["_worker_version"] and age(row.get("heartbeat_at")) < 3
        state = row.get("state", "waiting") if valid else "stopped" if not running else "starting"
        if same_process and row.get("worker_version") == settings["_worker_version"] and row.get("state") == "failed":
            state = "failed"
        ingress = self._ingress_status(settings)
        if ingress["enabled"]:
            if ingress["state"] == "failed": state = "failed"
            elif ingress["state"] in {"starting", "waiting", "stopped"} and state != "failed": state = "waiting" if running else state
            elif ingress["state"] == "streaming" and row.get("state") == "streaming" and valid: state = "streaming"
        return {"kind": "ros2", "domain_id": settings["ros2_camera"]["domain_id"], "state": state,
                "topics": settings["ros2_camera"]["topics"], "endpoint": settings["camera_endpoint"],
                "source_ingress": ingress,
                "views": row.get("views", {}) if valid else {}, "errors": row.get("errors", {}) if valid else {},
                "packets": row.get("packets", 0) if valid else 0,
                "message": (ingress.get("last_error") or row.get("error")) if state == "failed" else
                    f"G1 ZMQ → ROS 2 Domain {settings['ros2_camera']['domain_id']} · {state}" if ingress["enabled"] else
                    f"ROS 2 Domain {settings['ros2_camera']['domain_id']} · {state}"}

    def start(self, mode):
        settings = self.settings(mode)
        if settings is None:
            return {"kind": "zmq", "reused": True}
        path = self.directory/"config.json"
        supervisor = self.driver.supervisor
        with supervisor.lock:
            changed = read_json(path) != settings
            if changed:
                if supervisor.running("camera-bridge"): supervisor.stop("camera-bridge")
                if supervisor.running("camera-zmq-ros2"): supervisor.stop("camera-zmq-ros2")
            if not supervisor.running("camera-bridge") or settings["g1_zmq_ros2"]["enabled"] and not supervisor.running("camera-zmq-ros2"):
                atomic_json(path, settings)
            launcher = self.driver.app_root.parent/"scripts/run_ros2_camera_bridge.sh"
            ingress_launcher = self.driver.app_root.parent/"scripts/run_g1_zmq_ros2_bridge.sh"
            python = self.driver.app_root/".venv_ros2_camera/bin/python"
            if not launcher.is_file() or not python.is_file() or settings["g1_zmq_ros2"]["enabled"] and not ingress_launcher.is_file():
                raise ConsoleError("ROS 2 相机环境未安装，请运行 scripts/install_ros2_camera.sh", 412, "camera_environment")
            if settings["g1_zmq_ros2"]["enabled"]:
                self.ingress_directory.mkdir(parents=True, exist_ok=True)
                supervisor.start("camera-zmq-ros2", ["bash", str(ingress_launcher), "--config", str(path),
                    "--directory", str(self.ingress_directory)], self.driver.app_root.parent)
            elif supervisor.running("camera-zmq-ros2"):
                supervisor.stop("camera-zmq-ros2")
            result = supervisor.start("camera-bridge", ["bash", str(launcher), "--config", str(path),
                "--directory", str(self.directory)], self.driver.app_root.parent)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            row = self.status(mode)
            if row["state"] in {"waiting", "streaming"}:
                return {**result, **row}
            if row["state"] == "failed" or not supervisor.running("camera-bridge"):
                raise ConsoleError("ROS 2 相机桥接启动失败："+supervisor.logs("camera-bridge", 1600)[-900:], 412, "camera_start_failed")
            time.sleep(.05)
        raise ConsoleError("ROS 2 相机桥接启动超时，请查看桥接日志", 412, "camera_start_timeout")

    def stop(self):
        stopped_bridge = self.driver.supervisor.stop("camera-bridge")
        stopped_ingress = self.driver.supervisor.stop("camera-zmq-ros2")
        return stopped_bridge or stopped_ingress
