from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_base_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/udev', ['99-amr-serial.rules']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees',
         glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='nattanns18',
    maintainer_email='nattanns18@example.com',
    description='ESP32 serial bridge for the physical AMR base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_conditioner = amr_base_driver.imu_conditioner:main',
            'nav2_activation_gate = '
            'amr_base_driver.nav2_activation_gate:main',
            'drive_supervisor = amr_base_driver.drive_supervisor:main',
            'cmd_vel_passthrough = '
            'amr_base_driver.cmd_vel_passthrough:main',
            'serial_bridge = amr_base_driver.serial_bridge:main',
            'safe_keyboard_teleop = amr_base_driver.safe_keyboard_teleop:main',
            'scan_resampler = amr_base_driver.scan_resampler:main',
            'sensor_dashboard = amr_base_driver.sensor_dashboard:main',
            'calibration_run = amr_base_driver.calibration_run:main',
        ],
    },
)
