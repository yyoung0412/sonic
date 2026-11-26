import copy
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock
import sys

from console_dex1 import Dex1Control, Dex1Transport, trigger_target
from console_support import ConsoleError, now
from unitree_g1_transport import initialize_channel


class MemoryTransport:
    def __init__(self):
        self.states = {side: {'valid': True, 'received_at': time.monotonic(), 'position_rad': 2.7}
                       for side in ('left', 'right')}
        self.competing = False
        self.writes = []
        self.disarmed = 0
        self.closed = False

    def observe(self): return copy.deepcopy(self.states), self.competing
    def write(self, positions):
        self.writes.append(dict(positions))
        for side, value in positions.items():
            self.states[side].update(position_rad=value, received_at=time.monotonic())
    def disarm(self): self.disarmed += 1
    def close(self): self.closed = True


class Dex1ControlTests(unittest.TestCase):
    def setUp(self):
        self.transport = MemoryTransport()
        self.control = Dex1Control({}, self.transport)

    def controllers(self, left=0., right=1.):
        return {'observed_at': now(), 'left_trigger': left, 'right_trigger': right}

    def test_robot_preflight_and_grippers_share_one_channel_initialization(self):
        participant = object()
        factory = SimpleNamespace(_ChannelFactory__participant=participant)
        initialize = mock.Mock()
        channel = SimpleNamespace(ChannelFactoryInitialize=initialize, ChannelFactory=lambda: factory)
        with mock.patch.dict(sys.modules, {'unitree_sdk2py.core.channel': channel}), \
             mock.patch('unitree_g1_transport._channel_connection', None):
            connection = {'dds_domain': 0, 'interface': 'eno1'}
            self.assertIs(initialize_channel(connection), participant)
            self.assertIs(initialize_channel(connection), participant)
            initialize.assert_called_once_with(0, 'eno1')
            with self.assertRaises(ConsoleError):
                initialize_channel({'dds_domain': 0, 'interface': 'lo'})
            initialize.assert_called_once()

    def test_full_press_is_closed_release_is_open_and_half_press_is_half_stroke(self):
        self.assertEqual(trigger_target(1.), 0.)
        self.assertEqual(trigger_target(0.), 5.4)
        self.assertEqual(trigger_target(.5), 2.7)
        for value in (None, True, '1', -.01, 1.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError): trigger_target(value)

    def test_left_and_right_follow_their_own_trigger_and_reach_full_endpoints(self):
        for _ in range(35):
            status = self.control.update(self.controllers(), enabled=True)
        self.assertTrue(status['active'])
        self.assertAlmostEqual(self.transport.states['left']['position_rad'], 5.4)
        self.assertAlmostEqual(self.transport.states['right']['position_rad'], 0.)
        self.assertAlmostEqual(self.transport.writes[0]['left'], 2.88)
        self.assertAlmostEqual(self.transport.writes[0]['right'], 2.52)
        for _ in range(35): self.control.update(self.controllers(1., 0.), enabled=True)
        self.assertAlmostEqual(self.transport.states['left']['position_rad'], 0.)
        self.assertAlmostEqual(self.transport.states['right']['position_rad'], 5.4)

    def test_waiting_policy_missing_controller_data_and_invalid_values_do_not_publish(self):
        self.control.update(self.controllers(), enabled=False)
        self.control.update(None, enabled=True)
        self.control.update({'observed_at': '2000-01-01T00:00:00+00:00', 'left_trigger': 0., 'right_trigger': 0.}, enabled=True)
        self.control.update(self.controllers(float('nan'), 0.), enabled=True)
        self.assertFalse(self.transport.writes)
        self.assertEqual(self.transport.disarmed, 4)

    def test_state_loss_or_other_publisher_disarms_both_sides(self):
        self.control.update(self.controllers(), enabled=True)
        count = len(self.transport.writes)
        self.transport.states['right']['received_at'] -= 1
        self.assertFalse(self.control.update(self.controllers(), enabled=True)['active'])
        self.transport.states['right']['received_at'] = time.monotonic()
        self.transport.competing = True
        self.assertFalse(self.control.update(self.controllers(), enabled=True)['active'])
        self.assertEqual(len(self.transport.writes), count)
        self.control.disarm()
        self.assertFalse(self.control.status()['active'])
        self.control.close()
        self.assertTrue(self.transport.closed)

    def test_feedback_requires_one_finite_motor(self):
        transport = Dex1Transport.__new__(Dex1Transport)
        transport.lock = threading.RLock(); transport.states = {}
        for positions in ([], [1., 2.], [float('nan')], [8.]):
            transport.receive('left', SimpleNamespace(states=[SimpleNamespace(q=q) for q in positions]))
            self.assertFalse(transport.states['left']['valid'])
        transport.receive('right', SimpleNamespace(states=[SimpleNamespace(q=1.25)]))
        self.assertEqual(transport.states['right']['position_rad'], 1.25)
        self.assertTrue(transport.states['right']['valid'])

    def test_dds_wire_topics_gains_and_partial_failure_cleanup(self):
        messages = []
        class Publisher:
            def __init__(self, topic, typ): self.topic, self.closed = topic, False
            def Init(self): pass
            def Write(self, message): messages.append((self.topic, message)); return True
            def Close(self): self.closed = True
        transport = Dex1Transport.__new__(Dex1Transport); transport.publishers = {}
        modules = {'unitree_sdk2py.core.channel': SimpleNamespace(ChannelPublisher=Publisher),
            'unitree_sdk2py.idl.unitree_go.msg.dds_': SimpleNamespace(MotorCmds_=SimpleNamespace),
            'unitree_sdk2py.idl.default': SimpleNamespace(unitree_go_msg_dds__MotorCmd_=SimpleNamespace)}
        with mock.patch.dict(sys.modules, modules):
            transport.write({'left': 0., 'right': 5.4})
            self.assertEqual([row[0] for row in messages], ['rt/dex1/left/cmd', 'rt/dex1/right/cmd'])
            for side, (_, message) in zip(('left', 'right'), messages):
                motor = message.cmds[0]
                self.assertEqual((motor.mode, motor.kp, motor.kd, motor.dq, motor.tau), (1, 5., .05, 0., 0.))
                self.assertEqual(motor.q, 0. if side == 'left' else 5.4)
            publishers = dict(transport.publishers)
            publishers['right'].Write = lambda _: False
            with self.assertRaises(ConsoleError): transport.write({'left': 1., 'right': 2.})
            self.assertFalse(transport.publishers)
            self.assertTrue(all(p.closed for p in publishers.values()))
