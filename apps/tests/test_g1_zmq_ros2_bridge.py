import copy
import time
import unittest

from g1_zmq_ros2_bridge import CompletePacketGate, default_g1_zmq_ros2_config, validate_g1_zmq_ros2_config
from ros2_camera_bridge import CameraFrames, default_ros2_config


KEYS = {"head": "ego_view", "left_wrist": "left_wrist", "right_wrist": "right_wrist"}


class G1ZmqRos2Tests(unittest.TestCase):
    def test_downstream_requires_one_complete_identical_g1_batch(self):
        keys = {"head": "ego_view", "left_wrist": "left_wrist", "right_wrist": "right_wrist"}
        frames = CameraFrames(default_ros2_config(), keys, require_complete_set=True)
        wall = time.time()
        frames.frames = {
            "head": {"stamp": wall, "jpeg": b"h"},
            "left_wrist": {"stamp": wall, "jpeg": b"l"},
        }
        self.assertIsNone(frames.packet(wall))
        frames.frames["right_wrist"] = {"stamp": wall - .01, "jpeg": b"r"}
        self.assertIsNone(frames.packet(wall))
        frames.frames["right_wrist"]["stamp"] = wall
        packet = frames.packet(wall)
        self.assertEqual(set(packet["images"]), set(keys.values()))

    @classmethod
    def setUpClass(cls):
        # The packet gate validates bounded JPEG framing; image decoding is
        # exercised by the ROS wire test in the isolated camera environment.
        cls.jpeg = b"\xff\xd8" + b"x" * 124 + b"\xff\xd9"

    def packet(self, stamps=(100., 100.01, 100.02), missing=None):
        images = {key: self.jpeg for key in KEYS.values()}
        timestamps = {key: stamp for key, stamp in zip(KEYS.values(), stamps)}
        if missing:
            images.pop(KEYS[missing], None)
            timestamps.pop(KEYS[missing], None)
        return {"images": images, "timestamps": timestamps}

    def test_config_is_private_complete_and_disabled_by_default(self):
        value = default_g1_zmq_ros2_config("real")
        self.assertFalse(validate_g1_zmq_ros2_config(value)["enabled"])
        self.assertEqual(value["endpoint"], "tcp://192.168.123.164:15555")
        mutations = [
            lambda c: c.update(endpoint="tcp://8.8.8.8:15555"),
            lambda c: c.update(required_complete_set=False),
            lambda c: c.update(max_message_bytes=20),
            lambda c: c.update(extra=True),
            lambda c: c["frame_ids"].pop("head"),
        ]
        for mutate in mutations:
            bad = copy.deepcopy(value)
            mutate(bad)
            with self.assertRaises(ValueError):
                validate_g1_zmq_ros2_config(bad)

    def test_only_complete_fresh_synchronized_sets_are_accepted(self):
        gate = CompletePacketGate(KEYS, max_age_seconds=.5, max_skew_seconds=.1)
        rows = gate.accept(self.packet(), wall=100.1)
        self.assertEqual(set(rows), set(KEYS))
        self.assertEqual(gate.accepted, 1)
        self.assertIsNone(gate.accept(self.packet(missing="head"), wall=100.1))
        self.assertIn("head", gate.last_error)

    def test_stale_skewed_duplicate_and_invalid_jpeg_are_rejected(self):
        for packet, wall, expected in [
            (self.packet(), 101., "过期"),
            (self.packet((100., 100., 100.3)), 100.3, "时间差"),
        ]:
            gate = CompletePacketGate(KEYS, max_age_seconds=.5, max_skew_seconds=.1)
            self.assertIsNone(gate.accept(packet, wall=wall))
            self.assertIn(expected, gate.last_error)
        gate = CompletePacketGate(KEYS)
        self.assertIsNotNone(gate.accept(self.packet(), wall=100.1))
        self.assertIsNone(gate.accept(self.packet(), wall=100.1))
        self.assertIn("重复", gate.last_error)
        bad = self.packet((101., 101.01, 101.02))
        bad["images"]["left_wrist"] = b"not-jpeg"
        self.assertIsNone(gate.accept(bad, wall=101.1))
        self.assertIn("JPEG", gate.last_error)


if __name__ == "__main__":
    unittest.main()
