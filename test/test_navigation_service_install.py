from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_and_agent_installers_keep_dds_local_to_odroid():
    navigation = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')
    agent = (
        PACKAGE_ROOT / 'scripts' / 'install_robot_agent.sh'
    ).read_text(encoding='utf-8')

    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET' in navigation
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET' in agent


def test_navigation_service_has_bounded_restart_policy():
    unit = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-navigation.service'
    ).read_text(encoding='utf-8')

    assert 'Restart=on-failure' in unit
    assert 'StartLimitIntervalSec=120' in unit
    assert 'StartLimitBurst=5' in unit
    assert 'Requires=indoor-delivery-robot-base.service' in unit
    assert 'AllowedCPUs=0-2' in unit
    assert 'CPUWeight=800' in unit
    assert 'Nice=-2' in unit


def test_navigation_entrypoint_makes_composition_an_explicit_env_toggle():
    entrypoint = (
        PACKAGE_ROOT / 'deploy' / 'indoor-delivery-robot-navigation'
    ).read_text(encoding='utf-8')

    assert 'use_composition:="${AMR_NAV2_USE_COMPOSITION:-false}"' in entrypoint


def test_installer_keeps_black_box_opt_in_and_composition_off_by_default():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')

    assert "printf 'AMR_NAV2_USE_COMPOSITION=false\\n'" in installer
    assert 'enable "$BASE_SERVICE_NAME" "$SERVICE_NAME"' in installer
    assert 'disable --now "$BLACK_BOX_SERVICE_NAME"' in installer
    assert 'restart "$BLACK_BOX_SERVICE_NAME"' not in installer


def test_installer_prevents_package_updates_from_restarting_motion_services():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')
    policy = (
        PACKAGE_ROOT / 'deploy' / 'needrestart' /
        '99-indoor-delivery-robot.conf'
    ).read_text(encoding='utf-8')

    assert '/etc/needrestart/conf.d/$NEEDRESTART_CONFIG_NAME' in installer
    assert 'indoor-delivery-robot-' in policy
    assert '= 0;' in policy


def test_navigation_install_records_source_commit():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')

    assert 'AMR_SOURCE_COMMIT=%s' in installer


def test_black_box_capture_runs_below_control_priority():
    launch = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "prefix='nice -n 10'" in launch
    assert "prefix=['nice', '-n', '10']" not in launch
    assert "parameters=[{'max_topic_hz': 1.0}]" in launch


def test_base_and_observability_have_separate_cpu_priorities():
    base = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-base.service'
    ).read_text(encoding='utf-8')
    black_box = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-black-box.service'
    ).read_text(encoding='utf-8')

    assert 'AllowedCPUs=2-3' in base
    assert 'Slice=indoor-delivery-robot-critical.slice' in base
    assert 'CPUWeight=1000' in base
    assert 'Nice=-5' in base
    assert 'AllowedCPUs=0-2' in black_box
    assert 'Slice=indoor-delivery-robot-background.slice' in black_box
    assert 'CPUWeight=10' in black_box
    assert 'Nice=10' in black_box
    assert '-p record_scan:=false' in black_box
    assert '-p record_tf:=false' in black_box
    assert '-p record_extended_navigation:=false' in black_box
    assert 'CPUQuota=' not in base + black_box
    navigation = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-navigation.service'
    ).read_text(encoding='utf-8')
    # Evidence capture is opt-in on a fresh install and independent from Nav2,
    # so switching between localization and mapping does not stop recording.
    assert 'Wants=indoor-delivery-robot-black-box.service' not in navigation
    assert 'After=indoor-delivery-robot-black-box.service' not in navigation
    assert 'PartOf=indoor-delivery-robot-navigation.service' not in black_box
    wrapper = (
        PACKAGE_ROOT / 'deploy' / 'indoor-delivery-robot-base'
    ).read_text(encoding='utf-8')
    assert 'enable_base_control:=true' in wrapper
    assert 'enable_support:=false' in wrapper


def test_navigation_install_keeps_black_box_on_demand_across_reinstalls():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')

    assert 'BLACK_BOX_WAS_ENABLED' not in installer
    assert 'disable --now "$BLACK_BOX_SERVICE_NAME"' in installer
    assert 'enable "$BLACK_BOX_SERVICE_NAME"' not in installer


def test_systemd_slices_shed_background_load_before_base_control():
    critical = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-critical.slice'
    ).read_text(encoding='utf-8')
    background = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-background.slice'
    ).read_text(encoding='utf-8')
    camera = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-camera.service'
    ).read_text(encoding='utf-8')
    camera_relay = (
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' / 'deploy' /
        'systemd' / 'indoor-delivery-robot-camera-relay.service'
    ).read_text(encoding='utf-8')

    assert 'CPUWeight=1000' in critical
    assert 'AllowedCPUs=2-3' in critical
    assert 'CPUWeight=10' in background
    assert 'AllowedCPUs=0-2' in background
    assert 'Slice=indoor-delivery-robot-background.slice' in camera
    assert 'Slice=indoor-delivery-robot-background.slice' in camera_relay
    assert 'CPUQuota=' not in critical + background


def test_navigation_installer_deploys_split_services_in_order():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')

    assert 'indoor-delivery-robot-base.service' in installer
    assert 'indoor-delivery-robot-black-box.service' in installer
    assert 'systemctl stop "$SERVICE_NAME"' in installer
    assert 'systemctl restart "$BASE_SERVICE_NAME"' in installer
    assert 'systemctl disable --now "$BLACK_BOX_SERVICE_NAME"' in installer


def test_uart_sensor_publishers_cannot_block_on_slow_dds_readers():
    serial_bridge = (
        PACKAGE_ROOT / 'amr_base_driver' / 'serial_bridge.py'
    ).read_text(encoding='utf-8')
    base_launch = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "Imu, 'imu/data_raw', qos_profile_sensor_data" in serial_bridge
    assert (
        "Odometry, 'wheel/odometry', qos_profile_sensor_data"
        in serial_bridge
    )
    assert "Imu, 'imu/data', qos_profile_sensor_data" in serial_bridge
    assert "executable='imu_conditioner'" not in base_launch
