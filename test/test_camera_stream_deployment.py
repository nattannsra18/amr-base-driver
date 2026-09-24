from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_camera_stream_uses_hardware_mjpeg_outside_ros():
    service = (
        ROOT / 'deploy' / 'systemd' / 'indoor-delivery-robot-camera.service'
    ).read_text(encoding='utf-8')
    assert '--format=MJPEG' in service
    assert '--encoder=HW' in service
    assert '--resolution=640x480' in service
    assert '--desired-fps=30' in service
    assert '--tcp-nodelay' in service
    assert '--slowdown' not in service
    assert 'EnvironmentFile=/etc/indoor-delivery-robot/camera.env' in service
    assert 'ros2' not in service.lower()
    assert 'CPUQuota=25%' in service


def test_camera_installer_defaults_to_stable_private_endpoint():
    installer = (
        ROOT / 'scripts' / 'install_camera_stream.sh'
    ).read_text(encoding='utf-8')
    assert '/dev/v4l/by-id/' in installer
    assert '--dry-run' in installer
    assert 'ip -brief address show' in installer
    assert 'systemctl enable' in installer
