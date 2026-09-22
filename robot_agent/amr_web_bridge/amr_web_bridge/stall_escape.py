"""Conservative LaserScan-based escape selection for wheel stalls."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class StallEscapeDecision:
    """One bounded Nav2 behavior selected from fresh obstacle clearance."""

    action: str
    value: float = 0.0
    clearance: float = 0.0
    detail: str = ''


def choose_stall_escape(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    rear_required: float = 0.45,
    side_required: float = 0.40,
) -> StallEscapeDecision:
    """
    Choose reverse first, then a short turn toward the clearer side.

    The behavior server performs the final footprint/costmap collision check.
    This scan gate prevents even requesting a maneuver into a visibly blocked
    sector and never returns an unconstrained velocity command.
    """
    sectors: dict[str, list[float]] = {
        'rear': [],
        'left': [],
        'right': [],
    }
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or distance < range_min
            or distance > range_max
        ):
            continue
        angle = _normalize_angle(angle_min + index * angle_increment)
        degrees = math.degrees(angle)
        if abs(degrees) >= 150.0:
            sectors['rear'].append(distance)
        elif 50.0 <= degrees <= 130.0:
            sectors['left'].append(distance)
        elif -130.0 <= degrees <= -50.0:
            sectors['right'].append(distance)

    clearances = {
        name: _robust_clearance(values)
        for name, values in sectors.items()
    }
    rear = clearances['rear']
    left = clearances['left']
    right = clearances['right']
    if rear >= rear_required:
        return StallEscapeDecision(
            action='back_up',
            value=0.10,
            clearance=rear,
            detail=f'rear clearance {rear:.2f} m',
        )
    best_side = max(left, right)
    if best_side >= side_required:
        turn_left = left >= right
        return StallEscapeDecision(
            action='spin',
            value=0.26 if turn_left else -0.26,
            clearance=best_side,
            detail=(
                f'{"left" if turn_left else "right"} clearance '
                f'{best_side:.2f} m; rear clearance {rear:.2f} m'
            ),
        )
    return StallEscapeDecision(
        action='none',
        detail=(
            'no safe escape sector '
            f'(rear={rear:.2f} m, left={left:.2f} m, right={right:.2f} m)'
        ),
    )


def _robust_clearance(values: list[float]) -> float:
    if len(values) < 3:
        return 0.0
    ordered = sorted(values)
    # Ignore at most the lowest ten percent of isolated speckle returns while
    # remaining conservative over the rest of the sector.
    index = min(len(ordered) - 1, max(0, int(len(ordered) * 0.10)))
    return ordered[index]


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))
