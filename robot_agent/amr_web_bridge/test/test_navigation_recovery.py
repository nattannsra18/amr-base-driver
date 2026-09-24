from subprocess import CompletedProcess

import pytest

from amr_web_bridge.navigation_recovery import (
    classify_plan_failure,
    MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS,
    NavigationRecoveryRunner,
)


def test_classifies_required_navigation_failures():
    assert classify_plan_failure('Start is occupied') == 'Start blocked'
    assert classify_plan_failure('Goal pose is invalid') == 'Goal blocked'
    assert classify_plan_failure('planner failed') == 'No connected path'
    assert classify_plan_failure('error_code=205') == 'Start blocked'
    assert classify_plan_failure('error_code: 206') == 'Goal blocked'
    assert classify_plan_failure('Command timed out after 15 seconds') == 'Planner timeout'


def test_lifecycle_health_checks_only_fixed_nav2_nodes():
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        return CompletedProcess(arguments, 0, stdout='active [3]', stderr='')

    healthy, _ = NavigationRecoveryRunner(run=run).nav2_lifecycle_healthy()
    assert healthy is True
    assert calls == [
        ['ros2', 'lifecycle', 'get', '/planner_server'],
        ['ros2', 'lifecycle', 'get', '/controller_server'],
        ['ros2', 'lifecycle', 'get', '/bt_navigator'],
    ]


@pytest.mark.parametrize('output', [
    'inactive [2]', 'unconfigured [1]', 'activating [13]',
    'node is not active', 'active [2]', '',
])
def test_lifecycle_health_rejects_non_active_states(output):
    def run(arguments, **_kwargs):
        return CompletedProcess(arguments, 0, stdout=output, stderr='')

    assert NavigationRecoveryRunner(run=run).nav2_lifecycle_healthy()[0] is False


def test_system_helper_rejects_arbitrary_actions():
    runner = NavigationRecoveryRunner(run=lambda *_args, **_kwargs: None)
    accepted, detail = runner.system_action('shell-command')
    assert accepted is False
    assert detail == 'Unsupported system action'


def test_system_helper_allows_fixed_start_navigation_action():
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        return CompletedProcess(arguments, 0, stdout='', stderr='')

    accepted, _ = NavigationRecoveryRunner(run=run).system_action(
        'start-navigation'
    )
    assert accepted is True
    assert calls == [[
        'sudo', '/usr/local/sbin/indoor-delivery-robot-control',
        'start-navigation',
    ]]


def test_restart_gate_checks_lifecycle_manager_service_type():
    def run(arguments, **_kwargs):
        return CompletedProcess(
            arguments,
            0,
            stdout='nav2_msgs/srv/ManageLifecycleNodes',
            stderr='',
        )

    available, _ = NavigationRecoveryRunner(run=run).lifecycle_manager_available()
    assert available is True


def test_costmap_clear_targets_local_then_global():
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        return CompletedProcess(arguments, 0, stdout='response', stderr='')

    cleared, _ = NavigationRecoveryRunner(run=run).clear_costmaps()
    assert cleared is True
    assert calls[0][3] == '/local_costmap/clear_entirely_local_costmap'
    assert calls[1][3] == '/global_costmap/clear_entirely_global_costmap'


def test_motor_fault_clear_uses_only_fixed_trigger_service():
    calls = []

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return CompletedProcess(arguments, 0, stdout='success: true', stderr='')

    cleared, _ = NavigationRecoveryRunner(run=run).clear_motor_fault()
    assert cleared is True
    assert calls[0][0] == [
        'ros2', 'service', 'call', '/clear_motor_fault',
        'std_srvs/srv/Trigger', '{}',
    ]
    assert calls[0][1]['timeout'] == MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS
    assert MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS == 15.0


def test_motor_fault_clear_accepts_jazzy_trigger_response_format():
    def run(arguments, **_kwargs):
        return CompletedProcess(
            arguments,
            0,
            stdout='std_srvs.srv.Trigger_Response(success=True, message="cleared")',
            stderr='',
        )

    cleared, _ = NavigationRecoveryRunner(run=run).clear_motor_fault()
    assert cleared is True


def test_finish_motor_recovery_uses_fixed_set_bool_service():
    calls = []

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return CompletedProcess(arguments, 0, stdout='success: true', stderr='')

    finished, _ = NavigationRecoveryRunner(run=run).finish_motor_recovery(True)
    assert finished is True
    assert calls[0][0] == [
        'ros2', 'service', 'call', '/finish_motor_recovery',
        'std_srvs/srv/SetBool', '{"data":true}',
    ]
    assert calls[0][1]['timeout'] == MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS


def test_escape_behaviors_use_only_bounded_nav2_actions():
    calls = []

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return CompletedProcess(
            arguments, 0, stdout='status: SUCCEEDED\nerror_code=0', stderr='')

    runner = NavigationRecoveryRunner(run=run)
    assert runner.back_up(0.10)[0] is True
    assert runner.spin(-0.26)[0] is True
    assert calls[0][0][:5] == [
        'ros2', 'action', 'send_goal', '/backup', 'nav2_msgs/action/BackUp']
    assert '"x":0.1' in calls[0][0][5]
    assert '"speed":0.05' in calls[0][0][5]
    assert calls[0][1]['timeout'] == 15.0
    assert calls[1][0][:5] == [
        'ros2', 'action', 'send_goal', '/spin', 'nav2_msgs/action/Spin']
    assert '"target_yaw":-0.26' in calls[1][0][5]
    assert calls[1][1]['timeout'] == 15.0
