"""Bridge a complete three-camera G1 ZMQ packet into ROS 2 CompressedImage topics.

The worker is read-only. It never opens camera devices and never creates robot,
DDS, hand, or motion command publishers. ROS imports stay inside ``main`` so
configuration validation and unit tests do not join a ROS domain.
"""
from __future__ import annotations

import argparse
import copy
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import signal
import time

from console_support import atomic_json, now, read_json

CAMERAS = ("head", "left_wrist", "right_wrist")


def default_g1_zmq_ros2_config(mode="real"):
    return {
        "enabled": False,
        "endpoint": "tcp://192.168.123.164:15555" if mode == "real" else "tcp://127.0.0.1:15555",
        "required_complete_set": True,
        "max_message_bytes": 48 * 1024 * 1024,
        "frame_ids": {
            "head": "head_camera_optical_frame",
            "left_wrist": "left_wrist_camera_optical_frame",
            "right_wrist": "right_wrist_camera_optical_frame",
        },
    }


def validate_g1_zmq_ros2_config(value, mode="real"):
    if not isinstance(value, dict):
        raise ValueError("g1_zmq_ros2 必须是对象")
    result = default_g1_zmq_ros2_config(mode)
    if set(value) - set(result):
        raise ValueError("未知 G1 ZMQ 相机配置项：" + ", ".join(sorted(set(value) - set(result))))
    result.update(copy.deepcopy(value))
    if type(result["enabled"]) is not bool or result["required_complete_set"] is not True:
        raise ValueError("G1 ZMQ 第一版只支持显式启用并要求完整三路")
    match = re.fullmatch(r"tcp://([0-9.]+):([1-9][0-9]{0,4})", result["endpoint"]) if isinstance(result["endpoint"], str) else None
    if not match:
        raise ValueError("G1 ZMQ endpoint 必须是 tcp://IPv4:端口")
    try:
        address = ipaddress.ip_address(match.group(1))
    except ValueError as exc:
        raise ValueError("G1 ZMQ endpoint IPv4 无效") from exc
    if not (address.is_private or address.is_loopback) or int(match.group(2)) > 65535:
        raise ValueError("G1 ZMQ endpoint 只能使用本机或私有局域网地址")
    size = result["max_message_bytes"]
    if type(size) is not int or not 1 * 1024 * 1024 <= size <= 128 * 1024 * 1024:
        raise ValueError("G1 ZMQ 消息上限必须在 1MiB 到 128MiB 之间")
    frame_ids = result["frame_ids"]
    if not isinstance(frame_ids, dict) or set(frame_ids) != set(CAMERAS):
        raise ValueError("G1 ZMQ 必须配置三路 frame_id")
    for frame_id in frame_ids.values():
        if not isinstance(frame_id, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]{0,127}", frame_id):
            raise ValueError("ROS 相机 frame_id 无效")
    return result


class CompletePacketGate:
    """Validate fresh, increasing, synchronized three-view packets."""

    def __init__(self, keys, max_age_seconds=.5, max_skew_seconds=.1):
        if not isinstance(keys, dict) or set(keys) != set(CAMERAS) or len(set(keys.values())) != 3:
            raise ValueError("camera_keys 必须包含三个不同的来源键")
        self.keys = dict(keys)
        self.max_age_seconds = float(max_age_seconds)
        self.max_skew_seconds = float(max_skew_seconds)
        self.previous = {name: 0. for name in CAMERAS}
        self.accepted = 0
        self.rejected = 0
        self.last_error = ""

    def accept(self, packet, wall=None):
        wall = time.time() if wall is None else wall
        try:
            if not isinstance(packet, dict):
                raise ValueError("ZMQ相机消息必须是对象")
            images, timestamps = packet.get("images"), packet.get("timestamps")
            if not isinstance(images, dict) or not isinstance(timestamps, dict):
                raise ValueError("ZMQ相机消息缺少 images/timestamps")
            rows = {}
            for name, key in self.keys.items():
                image, stamp = images.get(key), timestamps.get(key)
                if not isinstance(image, (bytes, bytearray, memoryview)):
                    raise ValueError(f"{name} 缺少JPEG字节")
                image = bytes(image)
                if not 128 <= len(image) <= 24 * 1024 * 1024 or image[:2] != b"\xff\xd8" or image[-2:] != b"\xff\xd9":
                    raise ValueError(f"{name} JPEG无效或尺寸超限")
                if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError(f"{name} 时间戳无效")
                stamp = float(stamp)
                age = wall - stamp
                if not -.5 <= age <= self.max_age_seconds:
                    raise ValueError(f"{name} 时间戳过期或时钟未同步")
                if stamp <= self.previous[name]:
                    raise ValueError(f"{name} 时间戳重复或回退")
                rows[name] = {"jpeg": image, "stamp": stamp, "source_key": key}
            stamps = [row["stamp"] for row in rows.values()]
            if max(stamps) - min(stamps) > self.max_skew_seconds:
                raise ValueError("三路相机时间差超过上限")
            for name, row in rows.items():
                self.previous[name] = row["stamp"]
            self.accepted += 1
            self.last_error = ""
            return rows
        except Exception as exc:
            self.rejected += 1
            self.last_error = str(exc)[:300]
            return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    config = read_json(args.config)
    source = validate_g1_zmq_ros2_config(config["g1_zmq_ros2"], config.get("mode", "real"))
    ros = config["ros2_camera"]
    if not source["enabled"]:
        raise ValueError("G1 ZMQ → ROS 2桥接未启用")
    for name, topic in ros["topics"].items():
        if topic["message_type"] != "sensor_msgs/msg/CompressedImage":
            raise ValueError(f"{name} 必须配置为 sensor_msgs/msg/CompressedImage")
    os.environ.update(
        ROS_DOMAIN_ID=str(ros["domain_id"]),
        RMW_IMPLEMENTATION=ros["rmw"],
        ROS_LOCALHOST_ONLY="1" if ros["localhost_only"] else "0",
    )
    import msgpack
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
    import zmq

    gate = CompletePacketGate(config["camera_keys"], ros["max_age_seconds"], ros["max_skew_seconds"])
    args.directory.mkdir(parents=True, exist_ok=True)
    zmq_context = zmq.Context()
    sub = zmq_context.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.RCVHWM, 1)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(source["endpoint"])
    context = Context()
    rclpy.init(context=context, domain_id=ros["domain_id"])
    node = Node("robot_station_g1_zmq_camera_source", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publishers = {}
    for name, row in ros["topics"].items():
        qos = QoSProfile(
            depth=row["depth"],
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE if row["reliability"] == "reliable" else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        publishers[name] = node.create_publisher(CompressedImage, row["topic"], qos)
    stopping = False
    counts = {name: 0 for name in CAMERAS}
    last_packet_wall = 0.
    last_source_stamps = {}

    def stop(*_):
        nonlocal stopping
        stopping = True

    def write_status(state, error=None):
        row = {
            "state": state,
            "endpoint": source["endpoint"],
            "domain_id": ros["domain_id"],
            "topics": {name: value["topic"] for name, value in ros["topics"].items()},
            "required_complete_set": True,
            "packets": gate.accepted,
            "rejected": gate.rejected,
            "counts": counts,
            "source_timestamps": last_source_stamps,
            "last_error": gate.last_error,
            "heartbeat_at": now(),
            "pid": os.getpid(),
            "worker_version": config["_worker_version"],
        }
        if error:
            row["error"] = str(error)[:400]
        atomic_json(args.directory / "status.json", row)

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)
    print(json.dumps({"state": "waiting", "endpoint": source["endpoint"], "domain_id": ros["domain_id"]}, ensure_ascii=False), flush=True)
    last_status = 0.
    try:
        while not stopping and context.ok():
            executor.spin_once(timeout_sec=.01)
            if sub.poll(10):
                raw = sub.recv()
                if len(raw) > source["max_message_bytes"]:
                    gate.rejected += 1
                    gate.last_error = "ZMQ相机消息超过配置上限"
                else:
                    try:
                        packet = msgpack.unpackb(raw, raw=False)
                    except Exception as exc:
                        gate.rejected += 1
                        gate.last_error = ("msgpack解码失败：" + str(exc))[:300]
                    else:
                        rows = gate.accept(packet)
                        if rows:
                            # Preserve the three-view packet as one atomic ROS
                            # batch.  The downstream bridge requires this exact
                            # shared stamp and therefore cannot mix adjacent
                            # source packets while callbacks are interleaved.
                            bundle_stamp = max(row["stamp"] for row in rows.values())
                            for name, row in rows.items():
                                message = CompressedImage()
                                seconds = int(bundle_stamp)
                                message.header.stamp.sec = seconds
                                message.header.stamp.nanosec = int(round((bundle_stamp - seconds) * 1_000_000_000))
                                if message.header.stamp.nanosec >= 1_000_000_000:
                                    message.header.stamp.sec += 1
                                    message.header.stamp.nanosec = 0
                                message.header.frame_id = source["frame_ids"][name]
                                message.format = "jpeg"
                                message.data = row["jpeg"]
                                publishers[name].publish(message)
                                counts[name] += 1
                            last_source_stamps = {name: row["stamp"] for name, row in rows.items()}
                            last_packet_wall = time.time()
            if time.monotonic() - last_status >= .25:
                live = last_packet_wall and time.time() - last_packet_wall <= ros["max_age_seconds"]
                write_status("streaming" if live else "waiting")
                last_status = time.monotonic()
        write_status("stopped")
    except Exception as exc:
        write_status("failed", exc)
        print(f"G1 ZMQ camera bridge failed: {exc}", flush=True)
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            context.shutdown()
        sub.close()
        zmq_context.term()


if __name__ == "__main__":
    main()
