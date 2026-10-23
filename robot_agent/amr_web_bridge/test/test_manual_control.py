import asyncio
import threading
from types import SimpleNamespace

from amr_web_bridge import web_bridge_node
from amr_web_bridge.web_bridge_node import WebBridgeNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def bridge(*, phase='IDLE', stopped=False):
    value = SimpleNamespace(
        robot_id='robot01',
        map_revision=5,
        mapping_runtime=SimpleNamespace(
            snapshot=lambda _revision: {'phase': phase},
        ),
        mapping_velocity_lock=threading.Lock(),
        mapping_velocity_deadline=0.0,
        mapping_velocity_active=False,
        mapping_teleop_deadman_seconds=0.35,
        manual_velocity_publisher=Publisher(),
        logger=Logger(),
    )
    value.motion_stop_latched = lambda: stopped
    value.get_logger = lambda: value.logger
    return value


def send(value, **overrides):
    message = {
        'robot_id': 'robot01',
        'linear_x': 0.15,
        'angular_z': -0.6,
    }
    message.update(overrides)
    asyncio.run(WebBridgeNode.handle_manual_teleop(value, object(), message))


def test_manual_teleop_publishes_bounded_command_and_refreshes_deadman():
    value = bridge()

    send(value)

    assert len(value.manual_velocity_publisher.messages) == 1
    command = value.manual_velocity_publisher.messages[0]
    assert command.linear.x == 0.15
    assert command.angular.z == -0.6
    assert value.mapping_velocity_active is True
    assert value.mapping_velocity_deadline > 0.0


def test_manual_teleop_rejects_mapping_estop_and_out_of_range_commands():
    cases = [
        (bridge(phase='MAPPING'), {}),
        (bridge(stopped=True), {}),
        (bridge(), {'linear_x': 0.31}),
        (bridge(), {'angular_z': 1.01}),
        (bridge(), {'linear_x': True}),
        (bridge(), {'robot_id': 'robot02'}),
    ]

    for value, overrides in cases:
        send(value, **overrides)
        assert value.manual_velocity_publisher.messages == []
        assert value.mapping_velocity_active is False
        assert value.logger.warnings == [
            'Rejected invalid manual teleoperation command',
        ]


def test_system_metrics_is_cached_and_uses_standard_linux_sources(monkeypatch):
    class HostPath:
        stat_reads = 0

        def __init__(self, path):
            self.path = path

        def exists(self):
            return self.path == '/sys/class/thermal/thermal_zone0/temp'

        def read_text(self):
            if self.path == '/proc/stat':
                HostPath.stat_reads += 1
                return 'cpu  100 0 0 100 0 0 0 0 0 0\n'
            if self.path == '/proc/meminfo':
                return 'MemTotal:       4000 kB\nMemAvailable:   1000 kB\n'
            return '51234\n'

    ticks = iter((10.0, 11.0, 12.1))
    value = SimpleNamespace(
        _system_metrics_last_read=0.0,
        _system_metrics=None,
        _cpu_sample=None,
    )
    monkeypatch.setattr(web_bridge_node, 'FilePath', HostPath)
    monkeypatch.setattr(web_bridge_node.time, 'monotonic', lambda: next(ticks))

    first = WebBridgeNode.system_metrics(value)
    cached = WebBridgeNode.system_metrics(value)
    refreshed = WebBridgeNode.system_metrics(value)

    assert first == {
        'cpu_percent': 0.0,
        'memory_percent': 75.0,
        'temperature_c': 51.234,
    }
    assert cached == first
    assert refreshed == first
    assert HostPath.stat_reads == 2
