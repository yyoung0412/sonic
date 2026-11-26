import importlib.util
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest
import uuid
import sys
from types import SimpleNamespace
from unittest import mock

import numpy as np

from console_driver import default_config
from console_pico import active_owner
from console_probe_worker import PassiveProbe
from console_sonic_worker import Session
from console_support import ConsoleError, age, atomic_json, now, process_identity, read_json

HELPER = Path(__file__).resolve().parents[2] / 'integrations/sonic/station_tracking.py'
spec = importlib.util.spec_from_file_location('station_tracking', HELPER)
tracking = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracking)


class PicoOwnershipTests(unittest.TestCase):
    def test_trigger_values_only_refresh_when_the_original_sdk_clock_advances(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sonic-control/runs' / uuid.uuid4().hex / 'pico_tracking.json'
            writer = tracking.StationTracking(path)
            try:
                first = {'get_time_stamp_ns': 10, 'get_left_trigger': 0., 'get_right_trigger': 1.}
                writer.update(None, first)
                self.assertEqual(read_json(path)['controllers']['left_trigger'], 0.)
                writer.last_write = 0
                writer.update(None, {**first, 'get_left_trigger': 1.})
                self.assertEqual(read_json(path)['controllers']['left_trigger'], 0.)
                writer.last_write = 0
                writer.update(None, {**first, 'get_time_stamp_ns': 11, 'get_left_trigger': 1., 'get_right_trigger': 0.})
                self.assertEqual(read_json(path)['controllers']['left_trigger'], 1.)
                self.assertEqual(read_json(path)['controllers']['right_trigger'], 0.)
                writer.last_write = 0
                writer.update(None, {**first, 'get_time_stamp_ns': 12, 'get_left_trigger': float('nan')})
                self.assertEqual(read_json(path)['controllers']['device_timestamp_ns'], 11)
            finally:
                writer.close()

    def test_sdk_process_keeps_tracking_fresh_while_policy_holds_the_gil(self):
        import ctypes
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sonic-control/runs' / uuid.uuid4().hex / 'pico_tracking.json'
            fake = Path(tmp) / 'xrobotoolkit_sdk.py'
            freeze = Path(tmp) / 'freeze'
            fake.write_text('import time\nfrom pathlib import Path\n'
                'stamp = 1\ndef init(): pass\ndef close(): pass\n'
                'def is_body_data_available(): return True\n'
                'def get_body_timestamp_ns():\n global stamp\n'
                ' if not Path(' + repr(str(freeze)) + ').exists(): stamp = time.time_ns()\n'
                ' return stamp\n'
                'def get_time_stamp_ns(): return time.time_ns()\n'
                'def get_body_joints_pose(): return [[0,0,0,0,0,0,1] for _ in range(24)]\n'
                'def __getattr__(name): return lambda: False\n')
            # Editable SDK finders may precede PathFinder. Load the exact fixture
            # at interpreter startup so the bridge never connects to a real SDK.
            (Path(tmp) / 'sitecustomize.py').write_text(
                'import importlib.util, sys\n'
                'spec = importlib.util.spec_from_file_location("xrobotoolkit_sdk", ' + repr(str(fake)) + ')\n'
                'module = importlib.util.module_from_spec(spec)\n'
                'sys.modules["xrobotoolkit_sdk"] = module\n'
                'spec.loader.exec_module(module)\n')
            writer = tracking.StationTracking(path)
            sdk = writer.sdk_proxy()
            try:
                with mock.patch.dict(os.environ, {'PYTHONPATH': tmp}): sdk.init()
                deadline = time.monotonic() + 5
                while not read_json(path).get('body_ready') and time.monotonic() < deadline: time.sleep(.02)
                self.assertTrue(read_json(path).get('body_ready'))
                before = read_json(path)['body_timestamp_ns']
                # PyDLL retains the GIL throughout this native initialization delay.
                ctypes.PyDLL(None).usleep(1000000)
                record = read_json(path)
                self.assertTrue(record['body_ready'])
                self.assertGreater(record['body_timestamp_ns'], before)
                self.assertLess(age(record['observed_at']), .3)
                freeze.touch()
                time.sleep(.7)
                self.assertFalse(read_json(path)['body_ready'])
            finally:
                sdk.close()
                writer.close()
            self.assertFalse(read_json(path)['body_ready'])

    def failure_session(self, tmp, pico):
        directory = Path(tmp) / 'sonic-control'; directory.mkdir()
        atomic_json(directory / 'request.json', {'run_id': uuid.uuid4().hex,
            'mode': 'sim', 'connection': default_config('sim')})
        session = Session(tmp, directory)
        session.initialized = session.manager_started = session.had_input = True
        session.last_manager_packet = session.manager_started_at = time.monotonic()
        session.config['pico_calibration_confirmed'] = True
        atomic_json(session.run_dir / 'pico_tracking.json', pico)
        session.prepare = mock.Mock()
        session.read_feedback = mock.Mock()
        session.requested_stop = mock.Mock(return_value=False)
        session.children = mock.Mock()
        session.children.running.return_value = True
        session.stop_controller = mock.Mock(return_value=True)
        session.cleanup = mock.Mock(side_effect=lambda: atomic_json(
            session.run_dir / 'pico_tracking.json', {'body_ready': False, 'cleanup': True}))
        return session

    def test_tracking_failure_keeps_pre_cleanup_evidence_and_stops(self):
        cases = [{}, {'body_ready': True, 'observed_at': '2000-01-01T00:00:00+00:00',
                  'heartbeat_at': now(), 'body_timestamp_ns': 42},
                 {'body_ready': False, 'observed_at': now(), 'heartbeat_at': now()}]
        for pico in cases:
            with self.subTest(pico=pico), tempfile.TemporaryDirectory() as tmp:
                session = self.failure_session(tmp, pico)
                def stop_before_diagnostic():
                    self.assertFalse((session.run_dir / 'tracking_failure.json').exists())
                    return True
                session.stop_controller.side_effect = stop_before_diagnostic
                with mock.patch('console_sonic_worker.signal.signal'):
                    self.assertEqual(session.run(), 0)
                session.stop_controller.assert_called_once()
                self.assertTrue(session.state['stop_confirmed'])
                self.assertEqual(session.state['fault']['code'], 'sonic_tracking_lost')
                failure = read_json(session.run_dir / 'tracking_failure.json')
                self.assertEqual(failure['latest']['pico'], pico)
                self.assertFalse(failure['latest']['input_ready'])
                self.assertTrue(failure['recent'])
                self.assertEqual(session.state['tracking_failure']['latest']['pico'], pico)
                self.assertTrue(read_json(session.run_dir / 'pico_tracking.json')['cleanup'])

    def test_tracking_diagnostic_write_failure_does_not_skip_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.failure_session(tmp, {'body_ready': False})
            (session.run_dir / 'tracking_failure.json').mkdir()
            with mock.patch('console_sonic_worker.signal.signal'):
                self.assertEqual(session.run(), 0)
            session.stop_controller.assert_called_once()
            self.assertEqual(session.state['fault']['code'], 'sonic_tracking_lost')
            self.assertIn('tracking_diagnostic_error', session.state)
            self.assertTrue(session.state['stop_confirmed'])

    def test_real_and_sim_probes_share_one_sdk_and_release_before_manager(self):
        with tempfile.TemporaryDirectory() as tmp:
            console = Path(tmp)
            probes = []
            poses = np.zeros((24, 7)); poses[:, 6] = 1
            calls = {'init': 0, 'active': 0, 'maximum': 0}
            def init():
                calls['init'] += 1
                calls['active'] += 1
                calls['maximum'] = max(calls['maximum'], calls['active'])
            def close(): calls['active'] -= 1
            sdk = SimpleNamespace(init=init, close=close, is_body_data_available=lambda: True,
                get_body_timestamp_ns=time.time_ns, get_time_stamp_ns=time.time_ns,
                get_body_joints_pose=lambda: poses)
            for mode in ('real', 'sim'):
                directory = console / 'telemetry' / mode; directory.mkdir(parents=True)
                probes.append(PassiveProbe(tmp, directory, default_config(mode), mode))
            def wait_for(condition):
                deadline = time.monotonic() + 4
                while not condition() and time.monotonic() < deadline: time.sleep(.02)
                self.assertTrue(condition(), [p.data for p in probes])
            with mock.patch.dict(sys.modules, {'xrobotoolkit_sdk': sdk}), \
                 mock.patch('console_probe_worker.socket.create_connection', return_value=mock.MagicMock()):
                threads = [threading.Thread(target=p.run_pico) for p in probes]
                for thread in threads: thread.start()
                try:
                    wait_for(lambda: all(p.data['pico'].get('body_ready') for p in probes))
                    self.assertEqual(calls['init'], 1)
                    owner = {'run_id': uuid.uuid4().hex, 'pid': os.getpid(), 'start_ticks': process_identity(os.getpid())}
                    owner_path = console / 'sonic-control/pico-owner.json'
                    atomic_json(owner_path, owner)
                    wait_for(lambda: calls['active'] == 0 and all(p.data['pico'].get('sdk_owner') == owner['run_id'] for p in probes))
                    owner_path.unlink()
                    wait_for(lambda: calls['init'] == 2)
                    self.assertEqual(calls['maximum'], 1)
                finally:
                    for probe in probes: probe.stop.set()
                    for thread in threads: thread.join(timeout=3)
                self.assertEqual(calls['active'], 0)

    def test_owner_requires_live_process_identity_and_valid_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sonic-control/pico-owner.json'
            owner = {'run_id': uuid.uuid4().hex, 'pid': os.getpid(), 'start_ticks': process_identity(os.getpid())}
            atomic_json(path, owner)
            self.assertEqual(active_owner(tmp), owner)
            for patch in ({'start_ticks': 'reused'}, {'run_id': '../escape'}, {'pid': -1}):
                atomic_json(path, {**owner, **patch})
                self.assertEqual(active_owner(tmp), {})

    def test_shared_body_tracking_rejects_frozen_invalid_and_regressed_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sonic-control/runs' / uuid.uuid4().hex / 'pico_tracking.json'
            writer = tracking.StationTracking(path)
            poses = np.zeros((24, 7)); poses[:, 6] = 1
            def sample(stamp):
                return {'timestamp_ns': stamp, 'timestamp_realtime': time.time(),
                    'timestamp_monotonic': time.monotonic(), 'body_poses_np': poses.copy()}
            try:
                for stamp in (10, 11):
                    writer.last_write = 0
                    writer.update(sample(stamp))
                self.assertTrue(read_json(path)['body_ready'])
                frozen = sample(11); frozen['timestamp_monotonic'] -= 1
                observed = read_json(path)['observed_at']
                writer.last_write = 0; writer.update(frozen)
                self.assertFalse(read_json(path)['body_ready'])
                self.assertEqual(read_json(path)['observed_at'], observed)
                writer.last_write = 0; writer.update(sample(1))
                self.assertFalse(read_json(path)['body_ready'])
                invalid = sample(2); invalid['body_poses_np'][0, 0] = np.nan
                writer.last_write = 0; writer.update(invalid)
                self.assertFalse(read_json(path)['body_ready'])
                with self.assertRaises(BlockingIOError):
                    tracking.StationTracking(path)
            finally:
                writer.close()

    def test_manager_starts_before_body_readiness(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / 'sonic-control'; directory.mkdir()
            atomic_json(directory / 'request.json', {'run_id': uuid.uuid4().hex,
                'mode': 'sim', 'connection': default_config('sim')})
            session = Session(tmp, directory)
            session.initialized = True
            session.config['pico_calibration_confirmed'] = True
            atomic_json(session.run_dir / 'pico_tracking.json', {'body_ready': True,
                'observed_at': now()})
            session.requested_stop = mock.Mock(side_effect=[False, True])
            session.read_feedback = mock.Mock()
            session.children = mock.Mock()
            def start(): session.manager_started = True
            session.start_manager = mock.Mock(side_effect=start)
            session.loop()
            session.start_manager.assert_called_once()
            self.assertFalse(session.state['input_ready'])
            session.last_manager_packet = time.monotonic()
            session.requested_stop = mock.Mock(side_effect=[False, True])
            session.loop()
            self.assertTrue(session.state['input_ready'])

    def test_live_port_listener_still_blocks_restart(self):
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('127.0.0.1', 0)); listener.listen()
            with self.assertRaises(ConsoleError):
                Session.require_free_port(listener.getsockname()[1])

    def test_closed_reusable_connection_does_not_block_restart(self):
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('127.0.0.1', 0)); listener.listen()
            port = listener.getsockname()[1]
            with socket.create_connection(('127.0.0.1', port)) as client:
                accepted, _ = listener.accept()
                accepted.close()
        Session.require_free_port(port)
