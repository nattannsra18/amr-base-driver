"""Audit rolling black-box evidence without sending robot commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _number(values, key):
    try:
        return float(values.get(key, ''))
    except (TypeError, ValueError):
        return None


def _sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def load_events(paths):
    files = []
    for raw_path in paths:
        path = Path(raw_path).expanduser()
        files.extend(path.rglob('*.jsonl') if path.is_dir() else [path])
    events = []
    for path in sorted(set(files)):
        with path.open(encoding='utf-8') as source:
            for line_number, line in enumerate(source, 1):
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError as error:
                    raise ValueError(
                        f'{path}:{line_number}: invalid JSONL: {error}'
                    ) from error
    return sorted(events, key=lambda event: (
        event.get('wall_time', 0.0), event.get('monotonic', 0.0)))


def audit_events(events, *, command_timeout=0.6):
    """Return evidence-backed checks for one rolling telemetry window."""
    faults = []
    false_stalls = []
    stuck_commands = []
    sign_errors = []
    sign_samples = 0
    active_mcu_stall = 0
    last_command = {'linear_x': 0.0, 'angular_z': 0.0}
    last_command_time = None

    for event in events:
        timestamp = float(event.get('wall_time', 0.0))
        stream = str(event.get('stream', ''))
        data = event.get('data', {})
        if stream == '/cmd_vel_safe':
            last_command = data
            last_command_time = timestamp
            continue
        if not stream.startswith('/diagnostics'):
            continue

        for status in data.get('statuses', []):
            name = status.get('name')
            values = status.get('values', {})
            if name == 'ESP32 MCU fault':
                fault = int(_number(values, 'mcu_fault') or 0)
                active_mcu_stall = fault if fault in (2, 3) else 0
                if fault:
                    faults.append({'time': timestamp, 'fault': fault})
            elif name == 'Base command ownership':
                owner = values.get('owner', status.get('message'))
                moving = (
                    abs(float(last_command.get('linear_x', 0.0))) > 0.01
                    or abs(float(last_command.get('angular_z', 0.0))) > 0.03
                )
                age = (
                    timestamp - last_command_time
                    if last_command_time is not None else 0.0
                )
                if owner == 'STOPPED' and moving and age >= command_timeout:
                    stuck_commands.append({
                        'time': timestamp,
                        'command_age_s': round(age, 3),
                        'command': last_command,
                    })
            elif name == 'Base drive telemetry':
                left_target = abs(_number(values, 'target_left_rpm') or 0.0)
                right_target = abs(_number(values, 'target_right_rpm') or 0.0)
                left_measured = abs(
                    _number(values, 'measured_left_rpm') or 0.0)
                right_measured = abs(
                    _number(values, 'measured_right_rpm') or 0.0)
                left_delta = abs(_number(values, 'encoder_left_delta') or 0.0)
                right_delta = abs(
                    _number(values, 'encoder_right_delta') or 0.0)
                if active_mcu_stall == 2 and (
                    left_target < 8.0 or left_measured >= 0.5 or left_delta > 0
                ):
                    false_stalls.append({
                        'time': timestamp,
                        'wheel': 'left', 'source': 'MCU'})
                if active_mcu_stall == 3 and (
                    right_target < 8.0
                    or right_measured >= 0.5 or right_delta > 0
                ):
                    false_stalls.append({
                        'time': timestamp,
                        'wheel': 'right', 'source': 'MCU'})

                requested_linear = _number(values, 'requested_linear_mps')
                requested_angular = _number(values, 'requested_angular_rps')
                wheel_linear = _number(values, 'wheel_linear_mps')
                wheel_angular = _number(values, 'wheel_angular_rps')
                imu_angular = _number(values, 'imu_angular_z_rps')
                if None in (
                    requested_linear, requested_angular, wheel_linear,
                    wheel_angular, imu_angular,
                ):
                    continue
                if abs(requested_linear) >= 0.03 and abs(wheel_linear) >= 0.01:
                    sign_samples += 1
                    if _sign(requested_linear) != _sign(wheel_linear):
                        sign_errors.append({
                            'time': timestamp, 'axis': 'linear',
                            'requested': requested_linear,
                            'wheel_odometry': wheel_linear,
                        })
                if (
                    abs(requested_angular) >= 0.10
                    and abs(wheel_angular) >= 0.03
                    and abs(imu_angular) >= 0.03
                ):
                    sign_samples += 1
                    expected = _sign(requested_angular)
                    if (
                        _sign(wheel_angular) != expected
                        or _sign(imu_angular) != expected
                    ):
                        sign_errors.append({
                            'time': timestamp, 'axis': 'angular',
                            'requested': requested_angular,
                            'wheel_odometry': wheel_angular,
                            'imu': imu_angular,
                        })

    return {
        'esp32_fault': _result(faults),
        'false_stall': _result(false_stalls),
        'command_stuck': _result(stuck_commands),
        'odometry_imu_sign': {
            'status': 'FAIL' if sign_errors else (
                'PASS' if sign_samples else 'NO_DATA'),
            'samples': sign_samples,
            'findings': sign_errors,
        },
        'events': len(events),
    }


def _result(findings):
    return {
        'status': 'FAIL' if findings else 'PASS',
        'findings': findings,
    }


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Audit AMR black-box JSONL without moving the robot.')
    parser.add_argument('paths', nargs='+', help='JSONL file or directory')
    parser.add_argument('--json', action='store_true')
    options = parser.parse_args(args)
    result = audit_events(load_events(options.paths))
    if options.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        for name in (
            'esp32_fault', 'false_stall', 'command_stuck',
            'odometry_imu_sign',
        ):
            check = result[name]
            suffix = (
                f" ({check.get('samples')} samples)"
                if 'samples' in check else ''
            )
            print(f'{name}: {check["status"]}{suffix}')
        print(f'events: {result["events"]}')
    return 1 if any(
        result[name]['status'] == 'FAIL'
        for name in result if isinstance(result[name], dict)
    ) else 0


if __name__ == '__main__':
    raise SystemExit(main())
