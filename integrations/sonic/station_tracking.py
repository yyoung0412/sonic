"""Receive XR tracking in one SDK process and share it with SONIC."""
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import time
import threading
import socket
import subprocess
import sys

import msgpack

import numpy as np


class StationTracking:
    def __init__(self, filename, sdk_lock_fd=None):
        self.path = Path(filename)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = os.fdopen(sdk_lock_fd, 'a') if sdk_lock_fd is not None else (self.path.parents[3] / 'pico-sdk.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise
        self.previous = None
        self.consecutive = 0
        self.last_write = 0
        self.record = {'body_ready': False, 'source': 'XRoboToolkit SDK / SONIC Manager'}
        self.controllers = {}
        self.controller_stamp = None

    def sdk_proxy(self):
        return SDKRelay(self.path, self.lock.fileno())

    def update(self, sample, sdk_snapshot=None):
        mono = time.monotonic()
        if mono - self.last_write < .05:
            return
        self.last_write = mono
        if sdk_snapshot is not None:
            stamp = sdk_snapshot.get('get_time_stamp_ns', 0)
            values = [sdk_snapshot.get('get_' + side + '_trigger') for side in ('left', 'right')]
            if stamp > 0 and stamp != self.controller_stamp and all(
                isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value) and 0 <= value <= 1
                for value in values):
                self.controller_stamp = stamp
                self.controllers = {'observed_at': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                    'device_timestamp_ns': int(stamp), 'left_trigger': float(values[0]), 'right_trigger': float(values[1])}
        if sample is not None:
            stamp = int(sample['timestamp_ns'])
            if stamp != self.previous:
                poses = np.asarray(sample['body_poses_np'])
                valid = stamp > 0 and poses.shape == (24, 7) and np.isfinite(poses).all()
                if valid:
                    norms = np.linalg.norm(poses[:, 3:7], axis=1)
                    valid = bool(((norms > .5) & (norms < 1.5)).all())
                advancing = self.previous is None or stamp > self.previous
                self.consecutive = self.consecutive + 1 if valid and advancing else 0
                self.previous = stamp
                self.record = {'body_ready': self.consecutive >= 2,
                    'observed_at': datetime.fromtimestamp(sample['timestamp_realtime'], timezone.utc).isoformat(timespec='milliseconds'),
                    'body_timestamp_ns': stamp, 'source': 'XRoboToolkit SDK / SONIC Manager'}
            self.record['body_ready'] = bool(self.record['body_ready'] and 0 <= mono - sample['timestamp_monotonic'] <= .5)
        record = {**self.record, 'controllers': self.controllers,
            'heartbeat_at': datetime.now(timezone.utc).isoformat(timespec='milliseconds')}
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(record, allow_nan=False))
        os.replace(temporary, self.path)

    def close(self):
        self.record['body_ready'] = False
        self.last_write = 0
        self.update(None)
        self.lock.close()


SDK_DEFAULTS = {'is_body_data_available': False, 'get_body_timestamp_ns': 0,
    'get_time_stamp_ns': 0, 'get_body_joints_pose': [],
    'get_A_button': False, 'get_B_button': False, 'get_X_button': False, 'get_Y_button': False,
    'get_left_trigger': 0., 'get_right_trigger': 0., 'get_left_grip': 0., 'get_right_grip': 0.,
    'get_left_menu_button': False, 'get_right_menu_button': False,
    'get_left_axis_click': False, 'get_right_axis_click': False,
    'get_left_axis': [0., 0.], 'get_right_axis': [0., 0.]}


class SDKRelay:
    """Cache one SDK process's snapshots without blocking policy initialization."""
    def __init__(self, path, lock_fd):
        self.path, self.lock_fd = Path(path), lock_fd
        self.snapshot = dict(SDK_DEFAULTS)
        self.child = self.socket = self.thread = None
        self.stop = threading.Event()

    def init(self):
        if self.child is not None:
            raise RuntimeError('Station SDK relay is already initialized')
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        parent.settimeout(.2)
        child.setblocking(False)
        try:
            self.child = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()),
                '--sdk-bridge', str(self.path), str(child.fileno()), str(self.lock_fd), str(os.getpid())],
                pass_fds=(child.fileno(), self.lock_fd))
        except Exception:
            parent.close()
            raise
        finally:
            child.close()
        self.socket = parent
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()

    def receive(self):
        while not self.stop.is_set():
            try:
                self.snapshot = msgpack.unpackb(self.socket.recv(65536), raw=False)
            except socket.timeout:
                continue
            except OSError:
                return

    def __getattr__(self, name):
        if name not in SDK_DEFAULTS:
            raise AttributeError(name)
        def read():
            if self.child is None or self.child.poll() is not None:
                return SDK_DEFAULTS[name]
            return self.snapshot.get(name, SDK_DEFAULTS[name])
        return read

    def close(self):
        if self.child is None:
            return
        if self.child.poll() is None:
            try:
                self.socket.send(b'stop')
            except OSError:
                pass
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                self.child.wait(timeout=5)
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
        self.socket.close()
        self.child = None


def run_sdk_bridge(filename, socket_fd, lock_fd, parent_pid):
    import xrobotoolkit_sdk as sdk
    channel = socket.socket(fileno=socket_fd)
    channel.setblocking(False)
    tracker = StationTracking(filename, sdk_lock_fd=lock_fd)
    previous = None
    sample = None
    try:
        sdk.init()
        while os.getppid() == parent_pid:
            try:
                if channel.recv(16) == b'stop':
                    break
            except BlockingIOError:
                pass
            snapshot = {name: getattr(sdk, name)() for name in SDK_DEFAULTS}
            stamp = int(snapshot['get_body_timestamp_ns'])
            if snapshot['is_body_data_available'] and stamp > 0 and stamp != previous:
                previous = stamp
                sample = {'body_poses_np': snapshot['get_body_joints_pose'], 'timestamp_ns': stamp,
                    'timestamp_realtime': time.time(), 'timestamp_monotonic': time.monotonic()}
            tracker.update(sample, snapshot)
            try:
                channel.send(msgpack.packb(snapshot, use_bin_type=True))
            except BlockingIOError:
                pass
            time.sleep(.02)
    finally:
        try:
            sdk.close()
        finally:
            tracker.close()
            channel.close()


if __name__ == '__main__':
    if len(sys.argv) != 6 or sys.argv[1] != '--sdk-bridge':
        raise SystemExit('This helper is launched by a managed SONIC session')
    run_sdk_bridge(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
