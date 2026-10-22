import threading

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
    bridge.blocked_start_recovery_lock = threading.Lock()
    bridge.drive_stall_event_pending = False
    bridge.latest_scan = None
    bridge.last_scan_monotonic = None
    return bridge


def test_drive_blocked_diagnostic_is_latched_until_worker_consumes_it():
    bridge = make_bridge()

    bridge.diagnostics_callback(diagnostics_message(
        'Base drive telemetry',
        values=[KeyValue(key='drive_blocked', value='True')],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base drive telemetry',
        values=[KeyValue(key='drive_blocked', value='False')],
    ))

    assert bridge.drive_stall_event_pending is True


def diagnostics_message(
    name,
    hardware_id='robot01',
    values=None,
    *,
    level=DiagnosticStatus.OK,
    status_message='Healthy',
):
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = hardware_id
    status.level = level
    status.message = status_message
    status.values = values or [KeyValue(key='Topic', value='/example')]
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


def test_motor_fault_snapshot_combines_mcu_and_drive_diagnostics():
    bridge = make_bridge()
    bridge.diagnostics_callback(diagnostics_message(
        'ESP32 MCU fault', values=[KeyValue(key='mcu_fault', value='0')],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base drive telemetry', values=[
            KeyValue(key='motors_enabled', value='True'),
        ],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base serial telemetry',
        status_message='Fresh telemetry',
    ))

    assert WebBridgeNode.motor_fault_snapshot(bridge) == {
        'mcu_fault': 0,
        'motors_enabled': True,
        'telemetry_fresh': True,
        'drive_blocked': False,
        'drive_stall_recent': False,
    }


def test_motor_fault_snapshot_accepts_host_wheel_feedback_name():
    bridge = make_bridge()
    bridge.diagnostics_callback(diagnostics_message(
        'ESP32 MCU fault', values=[KeyValue(key='mcu_fault', value='0')],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Host wheel feedback', values=[
            KeyValue(key='motors_enabled', value='True'),
            KeyValue(key='drive_blocked', value='False'),
        ],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base serial telemetry',
        status_message='Fresh telemetry',
    ))

    snapshot = WebBridgeNode.motor_fault_snapshot(bridge)
    assert snapshot is not None
    assert snapshot['motors_enabled'] is True
    assert snapshot['mcu_fault'] == 0
    assert snapshot['telemetry_fresh'] is True


def test_motor_fault_snapshot_rejects_nonfresh_serial_telemetry():
    bridge = make_bridge()
    bridge.diagnostics_callback(diagnostics_message(
        'ESP32 MCU fault', values=[KeyValue(key='mcu_fault', value='0')],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base drive telemetry', values=[
            KeyValue(key='motors_enabled', value='True'),
        ],
    ))
    bridge.diagnostics_callback(diagnostics_message(
        'Base serial telemetry',
        level=DiagnosticStatus.ERROR,
        status_message='No fresh ESP32 telemetry',
    ))

    snapshot = WebBridgeNode.motor_fault_snapshot(bridge)
    assert snapshot is not None
    assert snapshot['telemetry_fresh'] is False


def test_active_drive_stall_cancels_nav_before_guarded_recovery():
    bridge = make_bridge()
    bridge.blocked_start_recovery_lock = threading.Lock()
    bridge.drive_stall_recovery_pending = True
    active = {'command_id': 'TASK-STALL:pickup'}
    calls = []
    bridge.cancel_active_navigation_for_operation = lambda *, resume: (
        calls.append(('cancel', resume)) or active
    )
    bridge.maybe_start_blocked_pose_recovery = (
        lambda command, detail: calls.append(('recover', command, detail)) or True
    )

    bridge.run_active_drive_stall_recovery(active)

    assert calls[0] == ('cancel', True)
    assert calls[1][0] == 'recover'
    assert bridge.drive_stall_recovery_pending is False


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
