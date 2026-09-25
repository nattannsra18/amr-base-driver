from pathlib import Path

import yaml


PROFILE_DIR = Path(__file__).parents[1] / 'config' / 'profiles'


def parameters(name):
    payload = yaml.safe_load((PROFILE_DIR / name).read_text(encoding='utf-8'))
    return payload['amr_web_bridge']['ros__parameters']


def test_sim_profile_declares_identity_capabilities_and_standard_interfaces():
    profile = parameters('turtlebot3_waffle_sim.yaml')
    assert profile['robot_serial_number'] == 'SIM-0001'
    assert profile['profile_version'] == 'turtlebot3-waffle-sim-v1'
    assert profile['hardware_contract_mode'] == 'simulation'
    assert profile['navigate_action'] == '/navigate_to_pose'
    assert profile['map_topic'] == '/map'
    assert profile['odom_topic'] == '/odom'
    assert profile['scan_topic'] == '/scan'


def test_physical_profile_is_an_explicit_non_secret_template():
    profile = parameters('scuttle_real.example.yaml')
    assert profile['robot_serial_number'] == 'REPLACE-WITH-CHASSIS-SERIAL'
    assert profile['profile_version'] == 'scuttle-real-v1'
    assert profile['hardware_contract_mode'] == 'physical'
    assert profile['battery_topic'] == '/battery_state'
    assert profile['physical_estop_topic'] == '/safety/physical_estop'
    assert 'credential' not in profile
    assert 'token' not in profile


def test_odroid_prototype_profile_uses_real_robot_interfaces():
    profile = parameters('odroid_c4_tt_prototype.yaml')
    assert profile['robot_serial_number'] == 'AMR-ODROID-C4-001'
    assert profile['profile_version'] == 'odroid-c4-tt-prototype-v1'
    assert profile['hardware_contract_mode'] == 'prototype'
    assert profile['odom_topic'] == '/odometry/filtered'
    assert profile['scan_topic'] == '/scan_recovery'
    assert profile['diagnostics_topic'] == '/diagnostics'
    assert profile['emergency_stop_cmd_vel_topic'] == '/cmd_vel_stop'
    assert profile['manual_cmd_vel_topic'] == '/cmd_vel_manual'
    assert 'mapping' not in profile['agent_capabilities'].split(',')
    assert 'credential' not in profile
    assert 'token' not in profile
