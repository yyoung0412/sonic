"""PICO trigger control for the external Unitree Dex1-1 serial2dds service."""
from __future__ import annotations

import threading
import time

from console_support import ConsoleError, age, finite

SIDES = ('left', 'right')
OPEN_POSITION = 5.4
CLOSED_POSITION = 0.0
MAX_POSITION_STEP = .18


def trigger_target(value):
    if not finite(value) or not 0 <= value <= 1:
        raise ValueError('PICO扳机值必须为0到1之间的有限数值')
    return OPEN_POSITION * (1.0 - value)


class Dex1Transport:
    def __init__(self, connection):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        from unitree_g1_transport import initialize_channel
        participant = initialize_channel(connection)
        self.own_guid = str(participant.guid)
        self.graph = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        self.endpoints = {}
        self.lock = threading.RLock()
        self.states = {}
        self.subscribers = []
        self.publishers = {}
        for side in SIDES:
            subscriber = ChannelSubscriber(f'rt/dex1/{side}/state', MotorStates_)
            subscriber.Init(lambda message, side=side: self.receive(side, message), 1)
            self.subscribers.append(subscriber)

    def receive(self, side, message):
        motors = getattr(message, 'states', [])
        valid = len(motors) == 1 and finite(motors[0].q) and -.1 <= motors[0].q <= 5.6
        with self.lock:
            self.states[side] = {'position_rad': float(motors[0].q) if valid else None,
                                 'received_at': time.monotonic(), 'valid': valid}

    def observe(self):
        for endpoint in self.graph.take(100):
            key = str(endpoint.key)
            if endpoint.sample_info.valid_data:
                self.endpoints[key] = {'topic': endpoint.topic_name,
                    'participant': str(endpoint.participant_key)}
            else:
                self.endpoints.pop(key, None)
        topics = {f'rt/dex1/{side}/cmd' for side in SIDES}
        competing = any(row['topic'] in topics and row['participant'] != self.own_guid
                        for row in self.endpoints.values())
        with self.lock:
            states = {side: dict(self.states.get(side, {})) for side in SIDES}
        return states, competing

    def write(self, positions):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        try:
            for side in SIDES:
                if side not in self.publishers:
                    publisher = ChannelPublisher(f'rt/dex1/{side}/cmd', MotorCmds_)
                    self.publishers[side] = publisher
                    publisher.Init()
                message = MotorCmds_()
                message.cmds = [unitree_go_msg_dds__MotorCmd_()]
                motor = message.cmds[0]
                motor.mode, motor.q, motor.dq, motor.tau = 1, positions[side], 0., 0.
                motor.kp, motor.kd = 5., .05
                if self.publishers[side].Write(message) is not True:
                    raise ConsoleError('Dex1夹爪指令发送失败', 503, 'dex1_write_failed')
        except Exception:
            self.disarm()
            raise

    def disarm(self):
        publishers, self.publishers = self.publishers, {}
        error = None
        for publisher in publishers.values():
            try:
                publisher.Close()
            except Exception as exc:
                error = exc
        if error is not None:
            raise error

    def close(self):
        try:
            self.disarm()
        finally:
            for subscriber in self.subscribers:
                subscriber.Close()


class Dex1Control:
    def __init__(self, connection, transport=None):
        self.transport = transport if transport is not None else Dex1Transport(connection)
        self.report = {'type': 'dex1', 'active': False, 'message': '等待Dex1状态和SONIC策略启用', 'sides': {}}

    def update(self, controllers, enabled):
        states, competing = self.transport.observe()
        stamp = time.monotonic()
        sides = {side: {'online': row.get('valid') is True and stamp - row.get('received_at', 0) <= .5,
                        'position_rad': row.get('position_rad')}
                 for side, row in states.items()}
        self.report = {'type': 'dex1', 'active': False, 'sides': sides}
        if not enabled:
            message = '等待SONIC策略启用'
        elif competing:
            message = '已有其他Dex1控制发布器，夹爪输出暂停'
        elif not all(sides.get(side, {}).get('online') for side in SIDES):
            message = '等待左右Dex1夹爪的新鲜状态'
        elif not isinstance(controllers, dict) or age(controllers.get('observed_at')) > .5:
            message = '等待PICO扳机数据'
        else:
            try:
                targets = {side: trigger_target(controllers.get(side + '_trigger')) for side in SIDES}
            except ValueError:
                message = 'PICO扳机数据无效，夹爪输出暂停'
            else:
                commands = {side: states[side]['position_rad'] +
                    max(-MAX_POSITION_STEP, min(MAX_POSITION_STEP, targets[side] - states[side]['position_rad']))
                    for side in SIDES}
                commands = {side: max(CLOSED_POSITION, min(OPEN_POSITION, value)) for side, value in commands.items()}
                self.transport.write(commands)
                for side in SIDES:
                    sides[side].update(trigger=controllers[side + '_trigger'],
                        target_rad=targets[side], command_rad=commands[side])
                self.report.update(active=True, message='左右PICO扳机正在控制对应Dex1夹爪')
                return self.status()
        self.transport.disarm()
        self.report['message'] = message
        return self.status()

    def status(self):
        return self.report

    def disarm(self):
        self.transport.disarm()
        self.report = {**self.report, 'active': False, 'message': 'SONIC停止，Dex1指令输出已结束'}

    def close(self):
        self.transport.close()
