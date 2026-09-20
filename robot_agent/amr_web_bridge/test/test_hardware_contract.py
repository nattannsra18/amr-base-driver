import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

from amr_web_bridge.web_bridge_node import WebBridgeNode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CONTRACT_CHECK = REPOSITORY_ROOT / 'scripts' / 'verify_hardware_contract.sh'


def test_hardware_contract_check_is_valid_bash_and_covers_every_gate():
    subprocess.run(['bash', '-n', str(CONTRACT_CHECK)], check=True)
    script = CONTRACT_CHECK.read_text(encoding='utf-8')
    for check_id in (
        'HIC-T01',
        'HIC-T02',
        'HIC-T03',
        'HIC-T04',
        'HIC-T05',
        'HIC-T06',
        'HIC-T07',
        'HIC-T08',
        'HIC-T09',
        'HIC-F01',
        'HIC-F02',
        'HIC-A01',
        'HIC-A02',
        'HIC-S01',
        'HIC-S02',
        'HIC-S03',
    ):
        assert check_id in script


def test_hardware_contract_check_documents_physical_and_simulation_modes():
    result = subprocess.run(
        ['bash', str(CONTRACT_CHECK), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--physical|--simulation' in result.stdout
    assert 'PHYSICAL_ESTOP_TOPIC' in result.stdout
    assert 'BATTERY_TOPIC' in result.stdout


def test_hardware_contract_check_accepts_a_complete_physical_ros_graph(tmp_path):
    fake_ros2 = tmp_path / 'ros2'
    fake_ros2.write_text(
        """#!/usr/bin/env bash
if [[ \"$1 $2\" == \"topic type\" ]]; then
  case \"$3\" in
    /odom) echo nav_msgs/msg/Odometry ;;
    /scan) echo sensor_msgs/msg/LaserScan ;;
    /map) echo nav_msgs/msg/OccupancyGrid ;;
    /amcl_pose) echo geometry_msgs/msg/PoseWithCovarianceStamped ;;
    /plan) echo nav_msgs/msg/Path ;;
    /diagnostics) echo diagnostic_msgs/msg/DiagnosticArray ;;
    /cmd_vel) echo geometry_msgs/msg/Twist ;;
    /battery_state) echo sensor_msgs/msg/BatteryState ;;
    /safety/physical_estop) echo std_msgs/msg/Bool ;;
  esac
elif [[ \"$1 $2\" == \"topic echo\" ]]; then
  exit 0
elif [[ \"$1 $2\" == \"action list\" ]]; then
  echo '/navigate_to_pose [nav2_msgs/action/NavigateToPose]'
  echo '/compute_path_to_pose [nav2_msgs/action/ComputePathToPose]'
elif [[ \"$1 $2\" == \"service list\" ]]; then
  echo '/map_server/load_map [nav2_msgs/srv/LoadMap]'
  echo '/reinitialize_global_localization [std_srvs/srv/Empty]'
elif [[ \"$1 $2\" == \"lifecycle get\" ]]; then
  echo 'active [3]'
elif [[ \"$1 $2 $3\" == \"run tf2_ros tf2_echo\" ]]; then
  echo 'Translation: [0.000, 0.000, 0.000]'
else
  exit 1
fi
""",
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    environment = dict(os.environ)
    environment['PATH'] = f"{tmp_path}:{environment['PATH']}"
    result = subprocess.run(
        ['bash', str(CONTRACT_CHECK), '--physical'],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert 'Hardware Interface Contract v1.0: 23 passed, 0 failed' in result.stdout
    assert 'SKIP' not in result.stdout


def test_sensor_battery_replaces_the_simulated_charge_source():
    bridge = SimpleNamespace(
        telemetry_lock=threading.Lock(),
        battery_percent=100,
        battery_source='SIMULATED',
        last_battery_monotonic=None,
    )
    message = BatteryState()
    message.percentage = 0.634
    WebBridgeNode.battery_callback(bridge, message)
    assert bridge.battery_percent == 63
    assert bridge.battery_source == 'SENSOR'
    assert bridge.last_battery_monotonic is not None


def test_physical_estop_clears_motion_and_does_not_resume_on_release():
    class Goal:
        cancelled = False

        def cancel_goal_async(self):
            self.cancelled = True

    goal = Goal()
    bridge = SimpleNamespace(
        physical_estop_latched=threading.Event(),
        last_physical_estop_monotonic=None,
        command_lock=threading.Lock(),
        active_goal_handle=goal,
        active_command={'command_id': 'active'},
        pending_command_ids={'active'},
        pending_cancel_requests={'active': {}},
        zero_count=0,
        queue_clear_count=0,
        path_clear_count=0,
    )
    bridge.publish_emergency_zero = lambda: setattr(
        bridge, 'zero_count', bridge.zero_count + 1
    )
    bridge.publish_zero_velocity = bridge.publish_emergency_zero
    bridge.latch_physical_estop = (
        lambda: WebBridgeNode.latch_physical_estop(bridge)
    )
    bridge.clear_command_queue = lambda: setattr(
        bridge, 'queue_clear_count', bridge.queue_clear_count + 1
    )
    bridge.clear_navigation_path = lambda **_kwargs: setattr(
        bridge, 'path_clear_count', bridge.path_clear_count + 1
    )

    latched = Bool()
    latched.data = True
    WebBridgeNode.physical_estop_callback(bridge, latched)
    assert bridge.physical_estop_latched.is_set()
    assert bridge.active_command is None
    assert bridge.active_goal_handle is None
    assert goal.cancelled is True
    assert bridge.queue_clear_count == 1
    assert bridge.path_clear_count == 1

    released = Bool()
    released.data = False
    WebBridgeNode.physical_estop_callback(bridge, released)
    assert not bridge.physical_estop_latched.is_set()
    assert bridge.active_command is None


def test_physical_estop_watchdog_fails_safe_when_signal_is_missing():
    bridge = SimpleNamespace(
        hardware_contract_mode='physical',
        physical_estop_stale_seconds=2.0,
        last_physical_estop_monotonic=None,
        latch_count=0,
    )
    bridge.latch_physical_estop = lambda: setattr(
        bridge, 'latch_count', bridge.latch_count + 1
    )

    WebBridgeNode.enforce_physical_estop_watchdog(bridge)
    assert bridge.latch_count == 1

    bridge.last_physical_estop_monotonic = time.monotonic()
    WebBridgeNode.enforce_physical_estop_watchdog(bridge)
    assert bridge.latch_count == 1


def test_physical_estop_watchdog_is_disabled_for_simulation():
    bridge = SimpleNamespace(
        hardware_contract_mode='simulation',
        physical_estop_stale_seconds=2.0,
        last_physical_estop_monotonic=None,
        latch_count=0,
    )
    bridge.latch_physical_estop = lambda: setattr(
        bridge, 'latch_count', bridge.latch_count + 1
    )

    WebBridgeNode.enforce_physical_estop_watchdog(bridge)
    assert bridge.latch_count == 0


def test_physical_estop_watchdog_does_not_fabricate_a_prototype_stop():
    bridge = SimpleNamespace(
        hardware_contract_mode='prototype',
        physical_estop_stale_seconds=2.0,
        last_physical_estop_monotonic=None,
        latch_count=0,
    )
    bridge.latch_physical_estop = lambda: setattr(
        bridge, 'latch_count', bridge.latch_count + 1
    )

    WebBridgeNode.enforce_physical_estop_watchdog(bridge)
    assert bridge.latch_count == 0
