from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_and_agent_installers_keep_dds_local_to_odroid():
    navigation = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')
    agent = (
        PACKAGE_ROOT / 'scripts' / 'install_robot_agent.sh'
    ).read_text(encoding='utf-8')

    assert 'ROS_LOCALHOST_ONLY=1' in navigation
    assert 'ROS_LOCALHOST_ONLY=1' in agent


def test_navigation_service_has_bounded_restart_policy():
    unit = (
        PACKAGE_ROOT / 'deploy' / 'systemd' /
        'indoor-delivery-robot-navigation.service'
    ).read_text(encoding='utf-8')

    assert 'Restart=on-failure' in unit
    assert 'StartLimitIntervalSec=120' in unit
    assert 'StartLimitBurst=5' in unit


def test_navigation_install_records_source_commit():
    installer = (
        PACKAGE_ROOT / 'scripts' / 'install_navigation_service.sh'
    ).read_text(encoding='utf-8')

    assert 'AMR_SOURCE_COMMIT=%s' in installer
