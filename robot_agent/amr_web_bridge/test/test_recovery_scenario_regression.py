"""Twenty deterministic recovery decisions before attended floor HIL."""

import math

from amr_web_bridge.stall_escape import choose_stall_escape

import pytest


def scan_with(*, front, rear, left, right):
    values = [max(front, rear, left, right)] * 360
    for degrees in range(360):
        signed = degrees if degrees <= 180 else degrees - 360
        if abs(signed) <= 30:
            values[degrees] = front
        elif abs(signed) >= 150:
            values[degrees] = rear
        elif 50 <= signed <= 130:
            values[degrees] = left
        elif -130 <= signed <= -50:
            values[degrees] = right
    return values


SCENARIOS = []
for repeat in range(4):
    variation = repeat * 0.01
    SCENARIOS.extend([
        pytest.param(
            {'front': 1.2, 'rear': 0.80 + variation,
             'left': 1.0, 'right': 1.0},
            'back_up', 0.10,
            id=f'clear-route-{repeat + 1}',
        ),
        pytest.param(
            {'front': 0.18, 'rear': 0.70 + variation,
             'left': 0.65, 'right': 0.62},
            'back_up', 0.10,
            id=f'front-obstacle-{repeat + 1}',
        ),
        pytest.param(
            {'front': 0.20, 'rear': 0.24, 'left': 0.28, 'right': 0.29},
            'none', 0.0,
            id=f'narrow-corner-{repeat + 1}',
        ),
        pytest.param(
            {'front': 0.22, 'rear': 0.36,
             'left': 0.72 + variation, 'right': 0.31},
            'arc', 0.22,
            id=f'left-recovery-{repeat + 1}',
        ),
        pytest.param(
            {'front': 0.22, 'rear': 0.36,
             'left': 0.31, 'right': 0.72 + variation},
            'arc', -0.22,
            id=f'right-recovery-{repeat + 1}',
        ),
    ])


@pytest.mark.parametrize(('clearance', 'action', 'value'), SCENARIOS)
def test_twenty_delivery_recovery_scenarios(clearance, action, value):
    """Fail closed or choose the expected bounded maneuver for 20 runs."""
    decision = choose_stall_escape(
        scan_with(**clearance),
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=8.0,
    )

    assert decision.action == action
    assert decision.value == value
