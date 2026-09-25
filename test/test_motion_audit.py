from amr_base_driver.motion_audit import audit_events


def event(time, stream, data):
    return {'wall_time': time, 'monotonic': time, 'stream': stream, 'data': data}


def diagnostics(time, *statuses):
    return event(time, '/diagnostics/test', {'statuses': list(statuses)})


def status(name, **values):
    return {'name': name, 'message': values.get('owner', ''), 'values': values}


def test_audit_accepts_healthy_motion_and_matching_signs():
    result = audit_events([
        event(1.0, '/cmd_vel_safe', {'linear_x': -0.06, 'angular_z': 0.0}),
        diagnostics(1.2, status('ESP32 MCU fault', mcu_fault='0')),
        diagnostics(1.2, status(
            'Base drive telemetry', target_left_rpm='-18',
            target_right_rpm='-18', measured_left_rpm='-17',
            measured_right_rpm='-18', encoder_left_delta='-12',
            encoder_right_delta='-13', requested_linear_mps='-0.06',
            requested_angular_rps='0', wheel_linear_mps='-0.058',
            wheel_angular_rps='0.01', imu_angular_z_rps='0.01')),
        event(1.5, '/cmd_vel_safe', {'linear_x': 0.0, 'angular_z': 0.0}),
        diagnostics(1.7, status('Base command ownership', owner='STOPPED')),
    ])

    assert result['esp32_fault']['status'] == 'PASS'
    assert result['false_stall']['status'] == 'PASS'
    assert result['command_stuck']['status'] == 'PASS'
    assert result['odometry_imu_sign']['status'] == 'PASS'


def test_audit_fails_fault_false_stall_stuck_command_and_reversed_yaw():
    result = audit_events([
        event(1.0, '/cmd_vel_safe', {'linear_x': 0.0, 'angular_z': 0.3}),
        diagnostics(1.8, status('Base command ownership', owner='STOPPED')),
        diagnostics(1.8, status('ESP32 MCU fault', mcu_fault='2')),
        diagnostics(1.8, status(
            'Base drive telemetry',
            target_left_rpm='12', target_right_rpm='12',
            measured_left_rpm='4', measured_right_rpm='4',
            encoder_left_delta='2', encoder_right_delta='2',
            requested_linear_mps='0', requested_angular_rps='0.3',
            wheel_linear_mps='0', wheel_angular_rps='-0.2',
            imu_angular_z_rps='-0.18')),
    ])

    assert result['esp32_fault']['status'] == 'FAIL'
    assert result['false_stall']['status'] == 'FAIL'
    assert result['command_stuck']['status'] == 'FAIL'
    assert result['odometry_imu_sign']['status'] == 'FAIL'


def test_audit_reports_no_direction_data_while_stationary():
    result = audit_events([
        diagnostics(1.0, status('ESP32 MCU fault', mcu_fault='0')),
    ])

    assert result['odometry_imu_sign']['status'] == 'NO_DATA'


def test_mcu_stall_is_false_stall_when_encoder_still_moves():
    result = audit_events([
        diagnostics(1.0, status('ESP32 MCU fault', mcu_fault='3')),
        diagnostics(1.0, status(
            'Base drive telemetry', target_left_rpm='15',
            target_right_rpm='15', measured_left_rpm='14',
            measured_right_rpm='12', encoder_left_delta='8',
            encoder_right_delta='7')),
    ])

    assert result['false_stall']['status'] == 'FAIL'
    assert result['false_stall']['findings'][0]['source'] == 'MCU'
