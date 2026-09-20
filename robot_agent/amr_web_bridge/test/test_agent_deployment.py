from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_systemd_service_starts_agent_from_external_profile_and_env():
    service = (
        PACKAGE_ROOT
        / 'deploy'
        / 'systemd'
        / 'indoor-delivery-robot-agent.service'
    ).read_text(encoding='utf-8')
    assert 'EnvironmentFile=/etc/indoor-delivery-robot/agent.env' in service
    assert 'source "$ROBOT_WORKSPACE/setup.bash"' in service
    assert '--params-file "$ROBOT_PROFILE_FILE"' in service
    assert 'Restart=always' in service
    assert 'NoNewPrivileges=true' in service
    assert 'ReadWritePaths=/var/lib/indoor-delivery-robot' in service


def test_deployment_environment_keeps_runtime_values_out_of_source():
    environment = (
        PACKAGE_ROOT / 'deploy' / 'robot-agent.env.example'
    ).read_text(encoding='utf-8')
    assert 'ROBOT_CONTROL_URL=wss://' in environment
    assert 'ROBOT_CREDENTIAL_FILE=/var/lib/' in environment
    assert 'replace-with-limited-bootstrap-secret' in environment
    assert 'ROBOT_WS_TOKEN' not in environment


def test_installer_supports_secure_repeatable_deployment():
    installer = (
        PACKAGE_ROOT.parents[1] / 'scripts' / 'install_robot_agent.sh'
    ).read_text(encoding='utf-8')
    assert '--dry-run' in installer
    assert '--enrollment-token-file' in installer
    assert 'useradd --system' in installer
    assert 'colcon build --merge-install' in installer
    assert 'ros2 pkg prefix' in installer
    assert 'import websockets, yaml' in installer
    assert 'systemctl enable' in installer
    assert 'Robot Registry:' in installer
    assert 'Robot pairing required. Code:' in installer
    assert 'REPLACE-|CHANGE-ME|CHANGEME' in installer
