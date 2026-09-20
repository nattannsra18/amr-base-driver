from amr_web_bridge.profile_validator import validate_robot_profile


def healthy_observation():
    return {
        'navigate_action': True,
        'navigate_action_name': '/navigate_to_pose',
        'compute_path_action': True,
        'compute_path_action_name': '/compute_path_to_pose',
        'odom_age_seconds': 0.2,
        'scan_age_seconds': 0.2,
        'amcl_pose_age_seconds': 0.3,
        'robot_moving': False,
        'map_age_seconds': 12.0,
        'diagnostics_age_seconds': 0.4,
        'tf_map_to_odom': True,
        'tf_odom_to_base': True,
        'amcl_state': 'ACTIVE',
        'global_localization_service': True,
        'global_localization_service_name': '/reinitialize_global_localization',
        'maps_directory': '/var/lib/robot/maps',
        'load_map_service': True,
        'load_map_service_name': '/map_server/load_map',
    }


def test_all_declared_interfaces_produce_a_ready_report():
    report = validate_robot_profile(
        ['navigation', 'localization', 'mapping', 'diagnostics'],
        healthy_observation(),
    )
    assert report['status'] == 'READY'
    assert all(item['status'] == 'PASS' for item in report['checks'])
    assert {item['category'] for item in report['checks']} == {
        'INTERFACE', 'DATA', 'TF', 'LIFECYCLE', 'CAPABILITY'
    }


def test_missing_required_interface_and_stale_data_are_not_ready():
    observation = healthy_observation()
    observation['navigate_action'] = False
    observation['odom_age_seconds'] = 8.0
    report = validate_robot_profile(['navigation'], observation)
    failed = {item['check_id'] for item in report['checks'] if item['status'] == 'FAIL'}
    assert report['status'] == 'NOT_READY'
    assert {'interface.navigate_action', 'data.odom'} <= failed


def test_stationary_amcl_pose_remains_valid_but_moving_pose_must_be_fresh():
    observation = healthy_observation()
    observation['amcl_pose_age_seconds'] = 30.0
    stationary = validate_robot_profile(['localization'], observation)
    assert stationary['status'] == 'READY'

    observation['robot_moving'] = True
    moving = validate_robot_profile(['localization'], observation)
    assert moving['status'] == 'NOT_READY'
    assert next(
        item for item in moving['checks']
        if item['check_id'] == 'data.amcl_pose'
    )['status'] == 'FAIL'


def test_unknown_capability_is_degraded_instead_of_silently_accepted():
    report = validate_robot_profile(['experimental_sensor'], healthy_observation())
    assert report['status'] == 'DEGRADED'
    assert report['checks'][0]['check_id'] == 'capability.experimental_sensor'
    assert report['checks'][0]['status'] == 'WARN'


def test_physical_contract_requires_fresh_battery_and_estop_state():
    observation = healthy_observation()
    observation.update({
        'hardware_contract_mode': 'physical',
        'battery_age_seconds': 0.4,
        'physical_estop_age_seconds': 0.2,
        'physical_estop_latched': False,
    })
    report = validate_robot_profile(['navigation'], observation)
    assert report['status'] == 'READY'
    assert {
        item['check_id'] for item in report['checks']
    } >= {
        'data.lidar',
        'data.battery',
        'data.physical_estop',
        'capability.physical_estop_clear',
    }

    observation['physical_estop_latched'] = True
    report = validate_robot_profile(['navigation'], observation)
    assert report['status'] == 'NOT_READY'
    failed = {
        item['check_id']
        for item in report['checks']
        if item['status'] == 'FAIL'
    }
    assert 'capability.physical_estop_clear' in failed


def test_prototype_reports_missing_safety_interfaces_without_faking_ready():
    observation = healthy_observation()
    observation.update({
        'hardware_contract_mode': 'prototype',
        'battery_age_seconds': None,
        'physical_estop_age_seconds': None,
        'physical_estop_latched': False,
    })
    report = validate_robot_profile(['navigation'], observation)
    assert report['status'] == 'DEGRADED'
    status_by_id = {
        item['check_id']: item['status']
        for item in report['checks']
    }
    assert status_by_id['data.battery'] == 'WARN'
    assert status_by_id['data.physical_estop'] == 'WARN'
    assert status_by_id['capability.physical_estop_clear'] == 'PASS'
    assert status_by_id['capability.prototype_safety'] == 'WARN'


def test_prototype_still_fails_when_physical_estop_is_latched():
    observation = healthy_observation()
    observation.update({
        'hardware_contract_mode': 'prototype',
        'battery_age_seconds': None,
        'physical_estop_age_seconds': 0.2,
        'physical_estop_latched': True,
    })
    report = validate_robot_profile(['navigation'], observation)
    assert report['status'] == 'NOT_READY'
    assert next(
        item for item in report['checks']
        if item['check_id'] == 'capability.physical_estop_clear'
    )['status'] == 'FAIL'
