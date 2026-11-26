import copy
import json
import os
from pathlib import Path
import tempfile
import time
import threading
import unittest
from unittest import mock

from console_driver import default_config
from console_sonic import SonicControl, connection, controller_argv
from console_support import ConsoleError, atomic_json, now
from sonic_container import BINARY, inspect as inspect_controller
from sonic_feedback import SonicFeedback
from sonic_commission import FLAGS, LEGACY_FLAGS, FILES, load_gate, runtime_fingerprint, template
from unitree_g1_control import digest
from unitree_g1_transport import UnitreeTransport

APP = Path(__file__).resolve().parents[1]
PROFILE_PATH = APP / 'static/assets/g1/profile.json'
if not PROFILE_PATH.exists(): PROFILE_PATH = APP.parent / 'static/assets/g1/profile.json'
PROFILE = json.loads(PROFILE_PATH.read_text())


def message(index=0, **patch):
    return {'control_loop_type': 'cpp', 'index': index, 'ros_timestamp': 0.,
            'body_q': [0.] * 29, 'body_q_measured': [.2] * 29, 'body_dq': [0.] * 29,
            'last_action': [0.] * 29, **patch}


class FeedbackTests(unittest.TestCase):
    def test_actual_cpp_index_with_zero_ros_time_requires_three_advancing_frames(self):
        tracker = SonicFeedback()
        tracker.configuration({'control_frequency': 50})
        self.assertFalse(tracker.status()['valid'])
        for index in range(3):
            self.assertTrue(tracker.feed(message(index)))
            self.assertEqual(tracker.status()['valid'], index == 2)
        self.assertEqual(tracker.values, [.2] * 29)
        self.assertEqual(tracker.status()['control_index'], 2)
        self.assertTrue(tracker.status()['valid'])

    def test_repeated_packet_and_headless_ros_time_never_refresh_validity(self):
        tracker = SonicFeedback(max_age=.05)
        for index in range(3): tracker.feed(message(index))
        stamp = tracker.observed_at
        for _ in range(4): self.assertFalse(tracker.feed(message(2)))
        self.assertEqual(tracker.observed_at, stamp)
        time.sleep(.07)
        self.assertFalse(tracker.status()['valid'])
        self.assertEqual(tracker.status()['duplicates'], 4)

    def test_reorder_regression_malformed_values_and_wrong_source_fail_closed(self):
        for patch in [{'index': True}, {'index': -1}, {'index': 1.1}, {'body_q_measured': [0.] * 28},
                      {'body_q_measured': [float('nan')] * 29}, {'last_action': [float('inf')] * 29},
                      {'control_loop_type': 'fake'}, {'ros_timestamp': time.time() - 10}]:
            tracker = SonicFeedback()
            for index in range(3): tracker.feed(message(index))
            self.assertFalse(tracker.feed({**message(3), **patch}))
            self.assertFalse(tracker.status()['valid'], patch)
        tracker = SonicFeedback()
        for index in (8, 9, 10): tracker.feed(message(index))
        self.assertFalse(tracker.feed(message(0)))
        self.assertFalse(tracker.status()['valid'])
        self.assertEqual(tracker.last_index, 10)

    def test_constant_configuration_heartbeat_does_not_invent_control_feedback(self):
        tracker = SonicFeedback()
        for _ in range(4): tracker.configuration({'control_frequency': 50})
        self.assertFalse(tracker.status()['valid'])
        self.assertIsNone(tracker.status()['control_index'])
        self.assertIsNotNone(tracker.status()['configuration_age_seconds'])


class MemorySupervisor:
    def __init__(self): self.children = {}
    def running(self, name): return name in self.children
    def record(self, name): return self.children.get(name, {})
    def start(self, name, argv, cwd):
        self.children[name] = {'pid': 12345, 'start_ticks': 'fixture', 'argv': argv, 'started_at': now()}
        return self.children[name]
    def logs(self, name): return 'fixture log'


class Driver:
    def __init__(self, root):
        self.root, self.app_root = Path(root), APP
        self.supervisor = MemorySupervisor()
        self.settings = {mode: default_config(mode) for mode in ('sim', 'real')}
    def config(self, mode): return copy.deepcopy(self.settings[mode])


class ControllerManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.driver = Driver(self.temp.name)
        self.directory = self.driver.root / 'runtime'
        self.control = SonicControl(self.driver, self.directory, PROFILE)
    def tearDown(self): self.temp.cleanup()
    def prepare_environment(self):
        python = self.driver.root / '.venv_teleop/bin/python'; python.parent.mkdir(parents=True); python.touch()
        manager = self.driver.root / 'gear_sonic/scripts/pico_manager_thread_server.py'; manager.parent.mkdir(parents=True)
        manager.write_text('# Robot Station: single SDK reader with shared tracking.\n')

    def test_simulation_is_hard_bound_to_loopback_and_domain_zero(self):
        config = connection(self.driver, 'sim')
        argv = controller_argv(config, 'sim')
        self.assertEqual(argv[:2], [BINARY, 'lo'])
        self.assertIn('--disable-crc-check', argv)
        self.assertNotIn('--disable-crc-check', controller_argv(self.driver.config('real'), 'real'))
        for patch in [{'interface': 'eno1'}, {'dds_domain': 99}, {'pose_endpoint': 'tcp://192.168.123.164:5556'},
                      {'state_endpoint': 'tcp://127.0.0.1:15556'}]:
            self.driver.settings['sim'] = {**default_config('sim'), **patch}
            with self.assertRaises(ConsoleError): connection(self.driver, 'sim')

    def test_real_start_cannot_be_enabled_by_http_boolean_or_manual_approval(self):
        self.prepare_environment()
        self.assertFalse(self.control.gate('real')['ready'])
        with self.assertRaises(ConsoleError) as error:
            self.control.start('real', {'service_id': 'controller', 'confirmed_real_start': True})
        self.assertEqual(error.exception.code, 'sonic_onsite_required')
        self.assertFalse(self.driver.supervisor.children)

    def test_explicit_start_has_one_durable_session_and_rejects_duplicates_or_unknown_args(self):
        self.prepare_environment()
        result = self.control.start('sim', {'service_id': 'controller'})
        self.assertEqual(result['state'], 'accepted')
        request = json.loads((self.control.directory / 'request.json').read_text())
        self.assertEqual(request['mode'], 'sim')
        self.assertEqual(request['connection']['interface'], 'lo')
        self.assertTrue(self.control.active())
        self.assertEqual(self.control.status('sim')['phase'], 'starting')
        self.assertFalse(self.control.status('sim')['feedback_valid'])
        for mode in ('sim', 'real'):
            with self.assertRaises(ConsoleError): self.control.start(mode, {'service_id': 'controller'})
        other = self.control.status('real')
        self.assertTrue(other['mode_conflict'])
        self.assertFalse(other['stop_allowed'])
        with self.assertRaises(ConsoleError): self.control.stop('real')
        self.driver.supervisor.children.clear()
        atomic_json(self.control.directory / 'status.json', {'run_id': request['run_id'], 'stop_confirmed': True})
        with self.assertRaises(ConsoleError): self.control.start('sim', {'service_id': 'controller', 'argv': ['bad']})

    def test_dead_worker_does_not_claim_container_process_was_stopped(self):
        self.prepare_environment(); self.control.start('sim', {'service_id': 'controller'})
        request = json.loads((self.control.directory / 'request.json').read_text())
        self.driver.supervisor.children.clear()
        atomic_json(self.control.directory / 'status.json', {'run_id': request['run_id'], 'container_id': 'immutable-id', 'phase': 'running', 'stop_confirmed': False})
        status = self.control.status('sim')
        self.assertTrue(self.control.active()); self.assertFalse(status['feedback_valid'])
        self.assertEqual(status['phase'], 'fault'); self.assertFalse(status['stop_confirmed'])
        self.assertFalse(status['start_allowed'])

    def test_log_components_are_allowlisted(self):
        for component in ('../../operation-key', '/etc/passwd', 'request.json'):
            with self.assertRaises(ConsoleError): self.control.logs('sim', component)

    def observation(self, **changes):
        return {'ok': True, 'samples': 150, 'mode_machine': 5, 'stationary': True,
                'max_joint_excursion_rad': .002, 'max_joint_speed_rad_s': .01,
                'unique_lowstate_source': True, 'foreign_lowcmd_publishers': 0,
                'unitree_motion_service_empty': True,
                'lowstate_source_guid': '5c4b43bc-3101-410b-b20b-3c3e698b0038',
                'connection': {'interface': 'eno1', 'dds_domain': 0}, **changes}

    def test_web_confirmation_requires_every_explicit_attestation(self):
        for payload in ({}, {'confirmed': True, 'confirmations': {}},
                        {'confirmed': True, 'confirmations': {flag: 'true' for flag in FLAGS}},
                        {'confirmed': True, 'confirmations': {flag: True for flag in FLAGS}, 'approved': True}):
            with mock.patch.object(self.control, 'inspect_commissioning') as inspect:
                with self.assertRaises(ConsoleError): self.control.commission('approve', payload)
                inspect.assert_not_called()
        self.assertFalse((self.control.directory / 'commissioning.json').exists())

    def test_web_approval_rechecks_live_state_and_saves_without_starting_controller(self):
        payload = {'confirmed': True, 'confirmations': {flag: True for flag in FLAGS}}
        observation = self.observation(stationary=False, unitree_motion_service_empty=False, unitree_motion_service_name='ai', sonic_takeover_ready=True)
        with mock.patch.object(self.control, 'inspect_commissioning', return_value=observation) as inspect, \
             mock.patch('sonic_commission.machine_fingerprint', return_value='host'), \
             mock.patch('sonic_commission.boot_id', return_value='boot'), \
             mock.patch('sonic_commission.sdk_fingerprint', return_value='sdk'), \
             mock.patch('sonic_commission.runtime_fingerprint', return_value='runtime'):
            result = self.control.commission('approve', payload)
            inspect.assert_called_once()
            self.assertTrue(result['gate']['ready'])
            path = self.control.directory / 'commissioning.json'
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            document = json.loads(path.read_text())
            self.assertEqual(document['schema_version'], 2)
            self.assertTrue(document['onsite_confirmed'])
            self.assertNotIn('dex3_hand_configuration_verified', document)
            self.assertEqual(document['mode_machine'], 5)
            self.assertAlmostEqual(document['expires_at_unix'] - document['approved_at_unix'], 4 * 3600)
            self.assertFalse(self.driver.supervisor.children)
            self.assertFalse((self.control.directory / 'request.json').exists())
            # A failed renewal leaves the last valid approval untouched.
            original = path.read_bytes()
            with mock.patch.object(self.control, 'inspect_commissioning', return_value=self.observation(foreign_lowcmd_publishers=1)):
                with self.assertRaises(ConsoleError): self.control.commission('approve', payload)
            self.assertEqual(path.read_bytes(), original)

    def test_read_only_inspection_reports_motion_and_handles_dds_output(self):
        observation = self.observation(stationary=False)
        process = mock.Mock(returncode=0, stdout='DDS startup log\n' + json.dumps(observation), stderr='')
        with mock.patch('console_sonic.subprocess.run', return_value=process):
            report = self.control.commission('inspect', {})
        self.assertTrue(report['inspection']['ready'])
        self.assertTrue(report['inspection']['checks'][0]['passed'])
        self.assertIn('现场目视确认', report['inspection']['motion_reference'])
        self.assertFalse((self.control.directory / 'commissioning.json').exists())
        self.assertFalse(self.driver.supervisor.children)
        for output in ('{}', 'not JSON', json.dumps(self.observation(connection={'interface': 'lo', 'dds_domain': 0}))):
            with mock.patch('console_sonic.subprocess.run', return_value=mock.Mock(returncode=0, stdout=output)):
                with self.assertRaises(ConsoleError): self.control.inspect_commissioning()


class GateTests(unittest.TestCase):
    def test_current_onsite_approval_is_boot_model_source_and_runtime_bound(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); app = root / 'apps'; app.mkdir()
            for name in ('console_sonic.py', 'console_sonic_worker.py', 'sonic_container.py', 'sonic_feedback.py',
                         'sonic_commission.py', 'unitree_commission.py', 'unitree_g1_transport.py', 'console_pico.py', 'console_dex1.py'):
                (app / name).write_text('reviewed fixture')
            for name in FILES:
                path = root / 'gear_sonic_deploy' / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b'fixture')
            manager = root / 'gear_sonic/scripts/pico_manager_thread_server.py'; manager.parent.mkdir(parents=True); manager.write_text('fixture')
            helper = root / 'gear_sonic/utils/teleop/station_tracking.py'; helper.parent.mkdir(parents=True); helper.write_text('fixture')
            path = root / 'approval.json'; config = default_config('real'); stamp = time.time()
            document = {**template(config, PROFILE), **{flag: True for flag in FLAGS}, 'approved': True,
                        'mode_machine': 1, 'lowstate_source_guid': '5c4b43bc-3101-410b-b20b-3c3e698b0038',
                        'machine_fingerprint': 'host', 'boot_id': 'boot', 'sdk_fingerprint': 'sdk',
                        'approved_at_unix': stamp - 1, 'expires_at_unix': stamp + 60,
                        'runtime_fingerprint': runtime_fingerprint(root, app)}
            atomic_json(path, document); path.chmod(0o600)
            with mock.patch('sonic_commission.machine_fingerprint', return_value='host'), mock.patch('sonic_commission.boot_id', return_value='boot'), mock.patch('sonic_commission.sdk_fingerprint', return_value='sdk'):
                self.assertTrue(load_gate(path, root, app, config, PROFILE)['approved'])
                path.chmod(0o644)
                with self.assertRaises(ConsoleError): load_gate(path, root, app, config, PROFILE)
                path.chmod(0o600)
                legacy = {**document, 'schema_version': 1, **{flag: True for flag in LEGACY_FLAGS}}
                legacy.pop('onsite_confirmed')
                atomic_json(path, legacy); path.chmod(0o600)
                self.assertTrue(load_gate(path, root, app, config, PROFILE)['approved'])
                atomic_json(path, {**legacy, 'normal_stop_verified': False}); path.chmod(0o600)
                with self.assertRaises(ConsoleError): load_gate(path, root, app, config, PROFILE)
                for patch in [{'schema_version': 3}, {'approved': False}, {'expires_at_unix': stamp - .1}, {'boot_id': 'another-boot'}, {'onsite_confirmed': False}, {'model_hash': 'wrong'}]:
                    atomic_json(path, {**document, **patch}); path.chmod(0o600)
                    with self.assertRaises(ConsoleError): load_gate(path, root, app, config, PROFILE)
                atomic_json(path, document); path.chmod(0o600)
                (app / 'console_sonic_worker.py').write_text('changed worker')
                with self.assertRaises(ConsoleError): load_gate(path, root, app, config, PROFILE)


class MotionModeHandoverTests(unittest.TestCase):
    def test_sonic_start_checks_feedback_identity_without_automatic_motion_threshold(self):
        from console_sonic_worker import Session
        session = Session.__new__(Session)
        session.app = APP
        session.root = Path('/unused-fixture')
        session.directory = Path('/unused-fixture/runtime')
        session.config = default_config('real')
        document = {'lowstate_source_guid': 'robot', 'mode_machine': 5}
        observation = {'ok': True, 'stationary': False, 'lowstate_source_guid': 'robot', 'mode_machine': 5}
        with mock.patch('console_sonic_worker.load_gate', return_value=document), \
             mock.patch('sonic_commission.inspect_sonic_hardware', return_value=observation):
            self.assertEqual(session.check_real(observe=True), document)
            observation['mode_machine'] = 6
            with self.assertRaises(ConsoleError): session.check_real(observe=True)

    def transport(self):
        transport = UnitreeTransport.__new__(UnitreeTransport)
        transport.lock = threading.RLock()
        transport.feedback = mock.Mock(reason='', at=time.monotonic())
        transport.feedback.view.return_value = {'mode_machine': 5}
        transport.mode_ok = True
        transport.mode_at = transport.graph_at = time.monotonic()
        transport.started = time.monotonic() - 3
        transport.mode_name = 'ai'
        transport.graph_error = None
        transport.own_guid = 'station'
        transport.endpoints = {'robot-state': {'topic': 'rt/lowstate', 'type': 'unitree_hg.msg.dds_.LowState_', 'participant': 'robot'}}
        return transport

    def test_sonic_accepts_registered_ai_only_with_clear_lowcmd_discovery(self):
        transport = self.transport()
        config = {'feedback_timeout_seconds': .1}
        with self.assertRaises(ConsoleError) as error:
            transport.observe(config)
        self.assertEqual(error.exception.code, 'controller_conflict')
        result = transport.observe(config, allowed_motion_modes=('ai',))
        self.assertEqual(result['motion_service_name'], 'ai')
        self.assertEqual(result['foreign_publishers'], 0)
        transport.endpoints['other-controller'] = {'topic': 'rt/lowcmd', 'type': 'unitree_hg.msg.dds_.LowCmd_', 'participant': 'other'}
        with self.assertRaises(ConsoleError) as error:
            transport.observe(config, allowed_motion_modes=('ai',))
        self.assertEqual(error.exception.code, 'foreign_publisher')

    def test_unknown_registered_modes_remain_blocked_for_sonic(self):
        transport = self.transport()
        transport.mode_name = 'unknown'
        with self.assertRaises(ConsoleError) as error:
            transport.observe({'feedback_timeout_seconds': .1}, allowed_motion_modes=('ai',))
        self.assertEqual(error.exception.code, 'controller_conflict')


class ContainerInspectorTests(unittest.TestCase):
    def test_invalid_inspector_result_is_not_treated_as_exit_confirmation(self):
        with mock.patch('sonic_container.subprocess.run', return_value=mock.Mock(returncode=0, stdout='{}', stderr='')):
            with self.assertRaises(ConsoleError): inspect_controller('immutable-id')
        with mock.patch('sonic_container.subprocess.run', return_value=mock.Mock(returncode=1, stdout='', stderr='daemon unavailable')):
            with self.assertRaises(ConsoleError): inspect_controller('immutable-id')




class StreamDeadlineTests(unittest.TestCase):
    def session(self):
        from console_sonic_worker import Session
        session = Session.__new__(Session)
        session.manager_started_at = session.last_manager_packet = session.policy_start_at = None
        session.had_feedback = session.had_input = False
        return session

    def test_idle_wait_allowed_but_frozen_input_and_missing_first_feedback_stop(self):
        session = self.session()
        session.check_stream_deadlines()
        session.manager_started_at = time.monotonic() - 31
        session.check_stream_deadlines()
        session.had_input = True
        with self.assertRaises(ConsoleError) as error: session.check_stream_deadlines()
        self.assertEqual(error.exception.code, 'sonic_input_lost')
        session.last_manager_packet = time.monotonic() - 2
        session.check_stream_deadlines()
        session.policy_start_at = time.monotonic()
        with self.assertRaises(ConsoleError): session.check_stream_deadlines()
        session.last_manager_packet = time.monotonic()
        session.policy_start_at = time.monotonic() - 6
        with self.assertRaises(ConsoleError) as error: session.check_stream_deadlines()
        self.assertEqual(error.exception.code, 'sonic_feedback_missing')
        session.had_feedback = True
        session.check_stream_deadlines()

    def test_official_command_start_sets_deadline_once(self):
        session = self.session()
        header = {'fields': [{'name': name, 'dtype': 'u8', 'shape': [1]} for name in ('start', 'stop', 'planner')]}
        message = b'command' + json.dumps(header).encode().ljust(1280, b'\0') + bytes([1, 0, 1])
        session.observe_input(message)
        self.assertIsNotNone(session.policy_start_at)
        stamp = session.policy_start_at
        session.observe_input(message)
        self.assertEqual(session.policy_start_at, stamp)

if __name__ == '__main__': unittest.main()
