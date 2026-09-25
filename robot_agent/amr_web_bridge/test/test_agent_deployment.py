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
    assert 'NoNewPrivileges=false' in service
    assert 'ReadWritePaths=/var/lib/indoor-delivery-robot' in service


def test_camera_relay_is_separate_hardened_outbound_service():
    service = (
        PACKAGE_ROOT
        / 'deploy'
        / 'systemd'
        / 'indoor-delivery-robot-camera-relay.service'
    ).read_text(encoding='utf-8')
    assert 'camera_relay' in service
    assert 'EnvironmentFile=/etc/indoor-delivery-robot/agent.env' in service
    assert 'User=indoor-robot' in service
    assert 'NoNewPrivileges=true' in service
    assert 'Restart=always' in service
    assert 'CPUQuota=' not in service
    assert 'Nice=0' in service


def test_camera_capture_prioritizes_stable_low_latency_frames():
    service = (
        PACKAGE_ROOT.parents[1]
        / 'deploy'
        / 'systemd'
        / 'indoor-delivery-robot-camera.service'
    ).read_text(encoding='utf-8')
    environment = (
        PACKAGE_ROOT.parents[1] / 'deploy' / 'camera.env.example'
    ).read_text(encoding='utf-8')
    assert '--buffers=1' in service
    assert '--desired-fps=30' in service
    assert '--resolution=${CAMERA_RESOLUTION}' in service
    assert 'exposure_dynamic_framerate=${CAMERA_DYNAMIC_FRAMERATE}' in service
    assert 'CPUQuota=' not in service
    assert 'Nice=0' in service
    assert 'CAMERA_DYNAMIC_FRAMERATE=0' in environment
    assert 'CAMERA_RESOLUTION=352x288' in environment


def test_deployment_environment_keeps_runtime_values_out_of_source():
    environment = (
        PACKAGE_ROOT / 'deploy' / 'robot-agent.env.example'
    ).read_text(encoding='utf-8')
    assert 'ROBOT_CONTROL_URL=wss://' in environment
    assert 'ROBOT_CREDENTIAL_FILE=/var/lib/' in environment
    assert 'replace-with-limited-bootstrap-secret' in environment
    assert 'ROBOT_WS_TOKEN' not in environment
    assert 'CAMERA_LOCAL_STREAM_URL=http://127.0.0.1:8081/stream' in environment


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
    assert 'indoor-delivery-robot-camera-relay.service' in installer
    assert 'Robot Registry:' in installer
    assert 'Robot pairing required. Code:' in installer
    assert 'REPLACE-|CHANGE-ME|CHANGEME' in installer
    assert '/etc/sudoers.d/indoor-delivery-robot-agent' in installer
    assert 'indoor-delivery-robot-control start-navigation' in installer
    assert 'indoor-delivery-robot-control restart-navigation' in installer
    assert 'indoor-delivery-robot-control stop-navigation' in installer
    assert 'indoor-delivery-robot-control poweroff' in installer
    assert 'systemctl *' not in installer
