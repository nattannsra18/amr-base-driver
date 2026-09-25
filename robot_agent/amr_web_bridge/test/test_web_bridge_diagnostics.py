from queue import Queue
import threading
import time

from amr_web_bridge.stall_escape import StallEscapeDecision
from amr_web_bridge.web_bridge_node import WebBridgeNode
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest


class StubLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class StubBridge:
    def __init__(self):
        self.logger = StubLogger()

    def get_logger(self):
        return self.logger


def test_precreated_recovery_service_returns_without_cli_discovery():
    calls = []

    class Future:
        def result(self):
            return type('Response', (), {
                'success': True,
                'message': 'pre-stall recovery authorized',
            })()

        def add_done_callback(self, callback):
            callback(self)

    class Client:
        def wait_for_service(self, timeout_sec):
            calls.append(('wait', timeout_sec))
            return True

        def call_async(self, request):
            calls.append(('call', request))
            return Future()

    succeeded, detail = WebBridgeNode._service_response(
        Client(), object(), timeout_seconds=0.2,
    )

    assert succeeded
    assert detail == 'pre-stall recovery authorized'
    assert calls[0] == ('wait', 0.25)


@pytest.mark.parametrize(
    ('level', 'expected'),
    [
        (b'\x00', 'OK'),
        (b'\x01', 'WARN'),
        (b'\x02', 'ERROR'),
        (b'\x03', 'STALE'),
        (0, 'OK'),
        (1, 'WARN'),
        (2, 'ERROR'),
        (3, 'STALE'),
        (bytearray(b'\x01'), 'WARN'),
        (memoryview(b'\x02'), 'ERROR'),
    ],
)
def test_diagnostic_level_name(level, expected):
    bridge = StubBridge()

    assert (
        WebBridgeNode.diagnostic_level_name(bridge, level)
        == expected
    )
    assert bridge.logger.warnings == []


@pytest.mark.parametrize('level', [b'', b'\x04', 4, None])
def test_unknown_diagnostic_level_is_stale(level):
    bridge = StubBridge()

    assert (
        WebBridgeNode.diagnostic_level_name(bridge, level)
        == 'STALE'
    )
    assert len(bridge.logger.warnings) == 1


def make_bridge():
    bridge = object.__new__(WebBridgeNode)
    bridge.diagnostics_lock = threading.Lock()
    bridge.latest_diagnostics = None
    bridge.diagnostics_revision = 0
    bridge.diagnostics_stale_after_seconds = 3.0
    bridge.diagnostics_expire_after_seconds = 60.0
    bridge.scan_lock = threading.Lock()
    bridge.latest_scan = None
    bridge.last_scan_monotonic = None
    return bridge


def diagnostics_message(name, hardware_id='robot01'):
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = hardware_id
    status.level = DiagnosticStatus.OK
    status.message = 'Healthy'
    status.values = [KeyValue(key='Topic', value='/example')]
    message.status = [status]
    return message


def test_profile_motion_gate_ignores_stationary_ekf_angular_noise():
    assert not WebBridgeNode.velocity_indicates_motion(
        {'linear_velocity': 0.0, 'angular_velocity': 0.0249},
        linear_threshold=0.02,
        angular_threshold=0.05,
    )


def test_profile_motion_gate_keeps_real_motion_strict():
    assert WebBridgeNode.velocity_indicates_motion(
        {'linear_velocity': 0.021, 'angular_velocity': 0.0},
        linear_threshold=0.02,
        angular_threshold=0.05,
    )
    assert WebBridgeNode.velocity_indicates_motion(
        {'linear_velocity': 0.0, 'angular_velocity': 0.051},
        linear_threshold=0.02,
        angular_threshold=0.05,
    )


def test_start_robot_stack_uses_fixed_system_action():
    bridge = make_bridge()
    bridge.robot_operation_lock = threading.Lock()
    bridge.robot_operation_active = True
    statuses = []
    bridge.send_command_status = lambda command, lifecycle, detail: statuses.append(
        (command, lifecycle, detail)
    )

    class Runner:
        def system_action(self, action):
            assert action == 'start-navigation'
            return True, ''

    bridge.navigation_recovery_runner = Runner()
    command = {
        'action': 'system.start_navigation',
        'command_id': 'robot-op:start',
    }

    bridge.run_robot_operation(command)

    assert statuses[0][1] == 'started'
    assert statuses[-1][1] == 'succeeded'
    assert 'stack start requested' in statuses[-1][2]
    assert bridge.robot_operation_active is False


def test_diagnostics_callback_merges_independent_publishers():
    bridge = make_bridge()

    bridge.diagnostics_callback(diagnostics_message('LiDAR'))
    bridge.diagnostics_callback(diagnostics_message('Nav2 Health'))

    snapshot = bridge.diagnostics_snapshot()
    assert snapshot is not None
    assert [status['name'] for status in snapshot['statuses']] == [
        'LiDAR',
        'Nav2 Health',
    ]
    assert all(
        '_received_at' not in status
        for status in snapshot['statuses']
    )


def test_diagnostics_snapshot_marks_only_old_entry_stale():
    bridge = make_bridge()
    bridge.diagnostics_callback(diagnostics_message('LiDAR'))
    bridge.diagnostics_callback(diagnostics_message('Nav2 Health'))
    lidar, nav2 = bridge.latest_diagnostics['statuses']
    lidar['_received_at'] = 10.0
    nav2['_received_at'] = 14.0

    snapshot = bridge.diagnostics_snapshot(now=14.5)
    assert snapshot is not None
    assert snapshot['statuses'][0]['level'] == 'STALE'
    assert snapshot['statuses'][1]['level'] == 'OK'


def test_diagnostics_snapshot_expires_abandoned_entry():
    bridge = make_bridge()
    bridge.diagnostics_callback(diagnostics_message('LiDAR'))
    bridge.latest_diagnostics['statuses'][0]['_received_at'] = 10.0

    snapshot = bridge.diagnostics_snapshot(now=71.0)
    assert snapshot is not None
    assert snapshot['statuses'] == []
    assert bridge.latest_diagnostics['statuses'] == []


def test_motor_fault_snapshot_reads_esp32_stall_state():
    bridge = make_bridge()
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    assert bridge.motor_fault_snapshot() == {
        'mcu_fault': 2,
        'host_motion_fault': 'LEFT_ENCODER_STALL',
        'host_motion_state': '',
        'host_motion_reason': '',
        'motors_enabled': True,
    }


def test_motor_fault_snapshot_merges_separate_fault_sources():
    bridge = make_bridge()
    message = DiagnosticArray()
    mcu = diagnostics_message('ESP32 MCU fault', 'esp32-uart-c').status[0]
    mcu.values = [KeyValue(key='mcu_fault', value='0')]
    host = diagnostics_message(
        'Host wheel feedback', 'odroid-motion-guard').status[0]
    host.values = [
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='LEFT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    message.status = [mcu, host]
    bridge.diagnostics_callback(message)

    assert bridge.motor_fault_snapshot() == {
        'mcu_fault': 0,
        'host_motion_fault': '',
        'host_motion_state': 'PRE_STALL',
        'host_motion_reason': 'LEFT_NO_WHEEL_FEEDBACK',
        'motors_enabled': True,
    }


def test_pre_stall_is_resettable_before_fault_latch():
    bridge = make_bridge()
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='LEFT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    fault = bridge.motor_fault_snapshot()
    assert fault is not None
    assert bridge.resettable_wheel_fault(fault)


def test_reset_motor_stall_requires_stop_and_diagnostic_confirmation():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: None
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='3'),
        KeyValue(key='host_motion_fault', value='RIGHT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class Runner:
        def clear_motor_fault(self):
            status = bridge.latest_diagnostics['statuses'][0]
            status['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'success: true'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()
    assert succeeded is True
    assert 'confirmed' in detail
    assert 'motors remain enabled' in detail
    assert 'no navigation goal was active' in detail


def test_reset_motor_stall_clears_host_no_wheel_feedback_fault():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: None
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value='LEFT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class Runner:
        def clear_motor_fault(self):
            status = bridge.latest_diagnostics['statuses'][0]
            status['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'success: true'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()
    assert succeeded is True
    assert 'confirmed' in detail
    assert 'motors remain enabled' in detail


def test_reset_motor_stall_refuses_non_stall_hardware_fault():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='4'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    succeeded, detail = bridge.reset_motor_stall()
    assert succeeded is False
    assert 'not a resettable wheel feedback fault' in detail


def test_reset_motor_stall_uses_safe_scan_escape_before_resuming_goal():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.latest_scan = {
        'ranges': tuple([1.0] * 360),
        'angle_min': -3.141592653589793,
        'angle_increment': 3.141592653589793 / 180.0,
        'range_min': 0.05,
        'range_max': 8.0,
    }
    bridge.last_scan_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-1',
        'task_id': 'TASK-1',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.command_queue = Queue()
    results = []
    bridge.send_navigation_result = (
        lambda command, status, detail: results.append(
            (command, status, detail)
        )
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class Runner:
        def clear_motor_fault(self):
            status = bridge.latest_diagnostics['statuses'][0]
            status['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'success: true'

        def guarded_reverse(self):
            return True, 'status: SUCCEEDED'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()
    assert succeeded is True
    assert 'short reverse' in detail
    assert bridge.command_queue.get_nowait() == active
    assert results == []


def test_reset_motor_stall_uses_rolling_arc_when_reverse_is_refused():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.latest_scan = {
        'ranges': tuple([1.0] * 360),
        'angle_min': -3.141592653589793,
        'angle_increment': 3.141592653589793 / 180.0,
        'range_min': 0.05,
        'range_max': 8.0,
    }
    bridge.last_scan_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-turn-fallback',
        'task_id': 'TASK-TURN',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.command_queue = Queue()
    bridge.send_navigation_result = lambda *_args: None
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='3'),
        KeyValue(key='host_motion_fault', value='RIGHT_ENCODER_STALL'),
        KeyValue(key='host_motion_state', value='FAULTED'),
        KeyValue(key='host_motion_reason', value='RIGHT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    maneuvers = []

    class Runner:
        def clear_motor_fault(self):
            status = bridge.latest_diagnostics['statuses'][0]
            status['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'host_motion_state', 'value': 'NORMAL'},
                {'key': 'host_motion_reason', 'value': ''},
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'success: true'

        def guarded_reverse(self):
            maneuvers.append(('guarded_reverse', 0.10))
            return False, 'backup timed out'

        def guarded_arc(self, yaw):
            maneuvers.append(('arc', yaw))
            return True, 'status: SUCCEEDED'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is True
    assert maneuvers == [
        ('guarded_reverse', 0.10),
        ('arc', 0.22),
    ]
    assert 'guarded rolling left arc' in detail
    assert bridge.command_queue.get_nowait() == active


def test_reset_motor_stall_tries_other_arc_and_replans_before_resume():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-alternate-turn',
        'task_id': 'TASK-ALTERNATE',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.command_queue = Queue()
    bridge.send_navigation_result = lambda *_args: None
    bridge.stall_escape_decision = lambda **_kwargs: StallEscapeDecision(
        action='arc',
        value=0.22,
        clearance=0.50,
        detail='rolling reverse left arc',
    )
    bridge.stall_arc_decisions = lambda: (
        StallEscapeDecision(
            action='arc', value=0.22, clearance=0.50,
            detail='rolling reverse left arc',
        ),
        StallEscapeDecision(
            action='arc', value=-0.22, clearance=0.48,
            detail='rolling reverse right arc',
        ),
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    maneuvers = []

    class Runner:
        def clear_motor_fault(self):
            status = bridge.latest_diagnostics['statuses'][0]
            status['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'cleared'

        def guarded_arc(self, yaw):
            maneuvers.append(('arc', yaw))
            return (yaw < 0.0), ('right completed' if yaw < 0.0 else 'timeout')

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (
        maneuvers.append(('clear', 0.0)) or (True, 'cleared')
    )
    bridge.compute_recovery_path = lambda target: (
        maneuvers.append(('plan', target['x'])) or (True, 'path available')
    )

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is True
    assert maneuvers == [
        ('arc', 0.22),
        ('arc', -0.22),
        ('clear', 0.0),
        ('plan', 1.0),
    ]
    assert 'guarded rolling right arc' in detail
    assert bridge.command_queue.get_nowait() == active


def test_fresh_pre_stall_during_escape_opens_one_final_recovery_window():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-fresh-pre-stall',
        'task_id': 'TASK-FRESH-PRE-STALL',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.command_queue = Queue()
    bridge.send_navigation_result = lambda *_args: None
    bridge.stall_escape_decision = lambda **_kwargs: StallEscapeDecision(
        action='arc', value=0.22, clearance=0.50,
        detail='rolling reverse left arc',
    )
    bridge.stall_arc_decisions = lambda: (
        StallEscapeDecision(
            action='arc', value=0.22, clearance=0.50,
            detail='rolling reverse left arc',
        ),
        StallEscapeDecision(
            action='arc', value=-0.22, clearance=0.48,
            detail='rolling reverse right arc',
        ),
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='host_motion_state', value='FAULTED'),
        KeyValue(key='host_motion_reason', value='LEFT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    calls = []

    def set_motor_state(state, *, fault='', reason='', mcu=0):
        bridge.latest_diagnostics['statuses'][0]['values'] = [
            {'key': 'mcu_fault', 'value': str(mcu)},
            {'key': 'host_motion_fault', 'value': fault},
            {'key': 'host_motion_state', 'value': state},
            {'key': 'host_motion_reason', 'value': reason},
            {'key': 'motors_enabled', 'value': 'True'},
        ]

    class Runner:
        def clear_motor_fault(self):
            calls.append('clear')
            if calls.count('clear') == 1:
                set_motor_state('NORMAL')
                return True, 'latched fault cleared'
            set_motor_state(
                'RECOVERING', reason='RIGHT_NO_WHEEL_FEEDBACK',
            )
            return True, 'pre-stall recovery authorized once'

        def guarded_arc(self, yaw):
            calls.append(('arc', yaw))
            if yaw > 0.0:
                set_motor_state(
                    'PRE_STALL', reason='RIGHT_NO_WHEEL_FEEDBACK',
                )
                return False, 'left arc lost right wheel feedback'
            set_motor_state(
                'RECOVERING', reason='RIGHT_NO_WHEEL_FEEDBACK',
            )
            return True, 'right arc completed'

        def finish_motor_recovery(self, success):
            calls.append(('finish', success))
            set_motor_state(
                'VERIFYING_RECOVERY' if success else 'FAULTED',
                fault='' if success else 'RIGHT_NO_WHEEL_FEEDBACK',
                reason='RIGHT_NO_WHEEL_FEEDBACK',
            )
            return True, 'recovery result recorded'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is True
    assert calls == [
        'clear',
        ('arc', 0.22),
        'clear',
        ('arc', -0.22),
        ('finish', True),
    ]
    assert 'guarded rolling right arc' in detail
    assert bridge.command_queue.get_nowait() == active


def test_pre_stall_recovery_failure_latches_without_second_maneuver():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-existing-pre-stall',
        'task_id': 'TASK-EXISTING-PRE-STALL',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.send_navigation_result = lambda *_args: None
    bridge.stall_escape_decision = lambda **_kwargs: StallEscapeDecision(
        action='arc', value=0.22, clearance=0.50,
        detail='rolling reverse left arc',
    )
    bridge.stall_arc_decisions = lambda: (
        StallEscapeDecision(
            action='arc', value=-0.22, clearance=0.48,
            detail='rolling reverse right arc',
        ),
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='LEFT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    calls = []

    class Runner:
        def clear_motor_fault(self):
            calls.append('clear')
            bridge.latest_diagnostics['statuses'][0]['values'] = [
                {'key': 'mcu_fault', 'value': '0'},
                {'key': 'host_motion_fault', 'value': ''},
                {'key': 'host_motion_state', 'value': 'RECOVERING'},
                {
                    'key': 'host_motion_reason',
                    'value': 'LEFT_NO_WHEEL_FEEDBACK',
                },
                {'key': 'motors_enabled', 'value': 'True'},
            ]
            return True, 'pre-stall recovery authorized once'

        def guarded_arc(self, yaw):
            calls.append(('arc', yaw))
            return False, 'arc timed out'

        def finish_motor_recovery(self, success):
            calls.append(('finish', success))
            return True, 'recovery failed and fault latched'

    bridge.navigation_recovery_runner = Runner()

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is False
    assert calls == ['clear', ('arc', 0.22), ('finish', False)]
    assert 'one authorized recovery attempt' in detail


def test_pre_stall_guard_refusal_tries_scan_safe_arc_in_same_window():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {
        'command_id': 'command-pre-stall-refusal',
        'task_id': 'TASK-PRE-STALL-REFUSAL',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    bridge.command_queue = Queue()
    results = []
    bridge.send_navigation_result = lambda *_args: results.append(_args)
    bridge.stall_escape_decision = lambda **_kwargs: StallEscapeDecision(
        action='back_up', value=0.10, clearance=0.45,
        detail='rear clearance 0.45 m',
    )
    bridge.stall_arc_decisions = lambda: (
        StallEscapeDecision(
            action='arc', value=-0.22, clearance=0.52,
            detail='rolling reverse right arc',
        ),
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='RIGHT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    calls = []

    def set_motor_state(state):
        bridge.latest_diagnostics['statuses'][0]['values'] = [
            {'key': 'mcu_fault', 'value': '0'},
            {'key': 'host_motion_fault', 'value': ''},
            {'key': 'host_motion_state', 'value': state},
            {'key': 'host_motion_reason', 'value': 'RIGHT_NO_WHEEL_FEEDBACK'},
            {'key': 'motors_enabled', 'value': 'True'},
        ]

    class Runner:
        def clear_motor_fault(self):
            calls.append('clear')
            set_motor_state('RECOVERING')
            return True, 'pre-stall recovery authorized once'

        def guarded_reverse(self):
            calls.append('reverse')
            return False, (
                'Guarded reverse refused: rear clearance 0.44 m is below 0.45 m'
            )

        def guarded_arc(self, yaw):
            calls.append(('arc', yaw))
            return True, 'Guarded right arc completed'

        def finish_motor_recovery(self, success):
            calls.append(('finish', success))
            set_motor_state('VERIFYING_RECOVERY' if success else 'FAULTED')
            return True, 'recovery result recorded'

    bridge.navigation_recovery_runner = Runner()
    bridge.wait_for_recovery_lifecycle = lambda: (True, 'active')
    bridge.clear_recovery_costmaps = lambda: (True, 'cleared')
    bridge.compute_recovery_path = lambda _target: (True, 'path available')

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is True
    assert calls == [
        'clear',
        'reverse',
        ('arc', -0.22),
        ('finish', True),
    ]
    assert 'guarded rolling right arc' in detail
    assert bridge.command_queue.get_nowait() == active
    assert results == []


def test_pre_stall_without_active_goal_is_blocked_without_moving():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: None
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='LEFT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)
    calls = []

    class Runner:
        def clear_motor_fault(self):
            calls.append('clear')
            return True, 'pre-stall recovery authorized once'

        def finish_motor_recovery(self, success):
            calls.append(('finish', success))
            return True, 'pending fault latched'

        def guarded_reverse(self):
            raise AssertionError('the robot must not move without an active goal')

        def guarded_arc(self, _yaw):
            raise AssertionError('the robot must not move without an active goal')

    bridge.navigation_recovery_runner = Runner()
    bridge.block_motor_recovery_for_safety = lambda: (
        calls.append('block') or (True, 'RECOVERY_BLOCKED')
    )

    succeeded, detail = bridge.reset_motor_stall()

    assert succeeded is False
    assert calls == ['clear', 'block']
    assert detail.startswith('RECOVERY_BLOCKED:')
    assert 'no motor fault was asserted' in detail


def test_reset_motor_stall_keeps_fault_latched_without_safe_escape():
    bridge = make_bridge()
    bridge.velocity_lock = threading.Lock()
    bridge.latest_velocity = {
        'linear_velocity': 0.0,
        'angular_velocity': 0.0,
    }
    bridge.last_odom_monotonic = time.monotonic()
    bridge.publish_zero_velocity = lambda: None
    active = {'command_id': 'command-2', 'task_id': 'TASK-2'}
    bridge.cancel_active_navigation_for_operation = lambda **_kwargs: active
    results = []
    bridge.send_navigation_result = (
        lambda command, status, detail: results.append(
            (command, status, detail)
        )
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='3'),
        KeyValue(key='host_motion_fault', value='RIGHT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class Runner:
        def clear_motor_fault(self):
            raise AssertionError('fault must remain latched')

    bridge.navigation_recovery_runner = Runner()

    succeeded, detail = bridge.reset_motor_stall()
    assert succeeded is False
    assert 'fresh LaserScan is unavailable' in detail
    assert results[0][1] == 'aborted'


def test_later_recovery_episodes_choose_alternative_safe_arcs():
    bridge = make_bridge()
    reverse = StallEscapeDecision(action='back_up', value=0.10)
    left = StallEscapeDecision(action='arc', value=0.22)
    right = StallEscapeDecision(action='arc', value=-0.22)
    bridge.stall_escape_decision = lambda: reverse
    bridge.stall_arc_decisions = lambda: (left, right)

    assert bridge.stall_escape_for_recovery_episode(1) == reverse
    assert bridge.stall_escape_for_recovery_episode(2) == left
    assert bridge.stall_escape_for_recovery_episode(3) == right


def configure_automatic_stall_recovery(bridge):
    bridge.command_lock = threading.Lock()
    bridge.emergency_stop_latched = threading.Event()
    bridge.physical_estop_latched = threading.Event()
    bridge.robot_operation_lock = threading.Lock()
    bridge.robot_operation_active = False
    bridge.automatic_stall_recovery_lock = threading.Lock()
    bridge.automatic_stall_recovery_active = False
    bridge.automatic_stall_recovery_attempt_counts = {}
    bridge.automatic_stall_fault_active = False
    bridge.automatic_stall_fault_epoch = 0
    bridge.automatic_stall_recovery_last_epochs = {}
    bridge.automatic_stall_recovery_max_episodes = 3


def test_automatic_stall_recovery_runs_once_per_fault_episode(monkeypatch):
    bridge = make_bridge()
    configure_automatic_stall_recovery(bridge)
    bridge.active_command = {'command_id': 'command-auto'}
    bridge.get_logger = lambda: StubLogger()
    calls = []
    bridge.reset_motor_stall = lambda **_kwargs: (
        calls.append('recover') or (True, 'escaped')
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)

    bridge.maybe_start_automatic_stall_recovery()
    bridge.maybe_start_automatic_stall_recovery()
    bridge.maybe_start_automatic_stall_recovery()

    assert calls == ['recover']
    assert bridge.automatic_stall_recovery_attempt_counts == {'command-auto': 1}
    assert bridge.robot_operation_active is False


def test_automatic_stall_recovery_allows_new_episode_after_healthy_state(
    monkeypatch,
):
    bridge = make_bridge()
    configure_automatic_stall_recovery(bridge)
    bridge.active_command = {'command_id': 'command-auto'}
    bridge.get_logger = lambda: StubLogger()
    calls = []
    bridge.reset_motor_stall = lambda **_kwargs: (
        calls.append('recover') or (True, 'escaped')
    )

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)

    stalled = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    stalled.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='RIGHT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    healthy = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    healthy.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='NORMAL'),
        KeyValue(key='host_motion_reason', value=''),
        KeyValue(key='motors_enabled', value='True'),
    ]

    bridge.diagnostics_callback(stalled)
    bridge.maybe_start_automatic_stall_recovery()
    bridge.maybe_start_automatic_stall_recovery()
    bridge.diagnostics_callback(healthy)
    bridge.maybe_start_automatic_stall_recovery()
    bridge.diagnostics_callback(stalled)
    bridge.maybe_start_automatic_stall_recovery()

    assert calls == ['recover', 'recover']
    assert bridge.automatic_stall_recovery_attempt_counts == {'command-auto': 2}
    assert bridge.automatic_stall_fault_epoch == 2


def test_automatic_stall_recovery_does_not_rearm_while_base_is_recovering(
    monkeypatch,
):
    bridge = make_bridge()
    configure_automatic_stall_recovery(bridge)
    bridge.active_command = {'command_id': 'command-auto'}
    bridge.get_logger = lambda: StubLogger()
    calls = []
    bridge.reset_motor_stall = lambda **_kwargs: (
        calls.append('recover') or (True, 'escaped')
    )

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)

    stalled = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    stalled.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='RIGHT_NO_WHEEL_FEEDBACK'),
    ]
    recovering = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    recovering.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='RECOVERING'),
        KeyValue(key='host_motion_reason', value='RIGHT_NO_WHEEL_FEEDBACK'),
    ]

    bridge.diagnostics_callback(stalled)
    bridge.maybe_start_automatic_stall_recovery()
    bridge.diagnostics_callback(recovering)
    bridge.maybe_start_automatic_stall_recovery()
    bridge.diagnostics_callback(stalled)
    bridge.maybe_start_automatic_stall_recovery()

    assert calls == ['recover']
    assert bridge.automatic_stall_fault_epoch == 1


def test_automatic_stall_recovery_caps_independent_episodes(monkeypatch):
    bridge = make_bridge()
    configure_automatic_stall_recovery(bridge)
    bridge.active_command = {'command_id': 'command-auto'}
    bridge.get_logger = lambda: StubLogger()
    calls = []
    bridge.reset_motor_stall = lambda **_kwargs: (
        calls.append('recover') or (True, 'escaped')
    )

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)

    stalled = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    stalled.status[0].values = [
        KeyValue(key='mcu_fault', value='2'),
        KeyValue(key='host_motion_fault', value='LEFT_ENCODER_STALL'),
        KeyValue(key='host_motion_state', value='FAULTED'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    healthy = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    healthy.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='NORMAL'),
        KeyValue(key='motors_enabled', value='True'),
    ]

    for _ in range(4):
        bridge.diagnostics_callback(stalled)
        bridge.maybe_start_automatic_stall_recovery()
        bridge.diagnostics_callback(healthy)
        bridge.maybe_start_automatic_stall_recovery()

    assert calls == ['recover', 'recover', 'recover']
    assert bridge.automatic_stall_recovery_attempt_counts == {'command-auto': 3}


def test_automatic_recovery_starts_for_pre_stall(monkeypatch):
    bridge = make_bridge()
    configure_automatic_stall_recovery(bridge)
    bridge.active_command = {'command_id': 'command-pre-stall'}
    bridge.get_logger = lambda: StubLogger()
    calls = []
    bridge.reset_motor_stall = lambda **_kwargs: (
        calls.append('recover') or (True, 'escaped before latch')
    )
    message = diagnostics_message('ESP32 base controller', 'esp32-uart-c')
    message.status[0].values = [
        KeyValue(key='mcu_fault', value='0'),
        KeyValue(key='host_motion_fault', value=''),
        KeyValue(key='host_motion_state', value='PRE_STALL'),
        KeyValue(key='host_motion_reason', value='RIGHT_NO_WHEEL_FEEDBACK'),
        KeyValue(key='motors_enabled', value='True'),
    ]
    bridge.diagnostics_callback(message)

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)
    bridge.maybe_start_automatic_stall_recovery()

    assert calls == ['recover']
    assert bridge.automatic_stall_recovery_attempt_counts == {
        'command-pre-stall': 1,
    }
