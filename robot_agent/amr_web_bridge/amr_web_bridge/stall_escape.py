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
    arc_rear_required: float = 0.32,
    side_required: float = 0.40,
) -> StallEscapeDecision:
    """
    Choose a straight reverse first, then a rolling reverse arc.

    Pure in-place spins are intentionally excluded because the physical
    passive-caster base has not passed an attended pivot floor test.  The
    selected primitive still performs its own live scan and odometry checks.
    """
    clearances = _sector_clearances(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
    )
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
    arcs = _arc_decisions(
        clearances,
        rear_required=arc_rear_required,
        side_required=side_required,
    )
    if arcs:
        return arcs[0]
    return StallEscapeDecision(
        action='none',
        detail=(
            'no safe escape sector '
            f'(rear={rear:.2f} m, left={left:.2f} m, right={right:.2f} m)'
        ),
    )


def choose_stall_arcs(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    rear_required: float = 0.32,
    side_required: float = 0.40,
) -> tuple[StallEscapeDecision, ...]:
    """Return scan-safe rolling reverse arcs, clearest side first."""
    clearances = _sector_clearances(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
    )
    return _arc_decisions(
        clearances,
        rear_required=rear_required,
        side_required=side_required,
    )


def choose_stall_turns(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    side_required: float = 0.40,
) -> tuple[StallEscapeDecision, ...]:
    """Return every scan-safe short turn, ordered by available clearance."""
    clearances = _sector_clearances(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
    )
    return _turn_decisions(clearances, side_required=side_required)


def _sector_clearances(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> dict[str, float]:
    ranges = tuple(ranges)
    sectors: dict[str, list[tuple[int, float]]] = {
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
            sectors['rear'].append((index, distance))
        elif 50.0 <= degrees <= 130.0:
            sectors['left'].append((index, distance))
        elif -130.0 <= degrees <= -50.0:
            sectors['right'].append((index, distance))

    return {
        name: _adjacent_cluster_clearance(values, beam_count=len(ranges))
        for name, values in sectors.items()
    }


def _turn_decisions(
    clearances: dict[str, float],
    *,
    side_required: float,
) -> tuple[StallEscapeDecision, ...]:
    candidates = sorted(
        (
            ('left', clearances['left'], 0.26),
            ('right', clearances['right'], -0.26),
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    return tuple(
        StallEscapeDecision(
            action='spin',
            value=yaw,
            clearance=clearance,
            detail=(
                f'{side} clearance {clearance:.2f} m; '
                f'rear clearance {clearances["rear"]:.2f} m'
            ),
        )
        for side, clearance, yaw in candidates
        if clearance >= side_required
    )


def _arc_decisions(
    clearances: dict[str, float],
    *,
    rear_required: float,
    side_required: float,
) -> tuple[StallEscapeDecision, ...]:
    rear = clearances['rear']
    if rear < rear_required:
        return ()
    candidates = sorted(
        (
            ('left', clearances['left'], 0.22),
            ('right', clearances['right'], -0.22),
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    return tuple(
        StallEscapeDecision(
            action='arc',
            value=yaw,
            clearance=min(rear, side_clearance),
            detail=(
                f'rolling reverse {side} arc; rear clearance {rear:.2f} m, '
                f'{side} clearance {side_clearance:.2f} m'
            ),
        )
        for side, side_clearance, yaw in candidates
        if side_clearance >= side_required
    )


def _adjacent_cluster_clearance(
    values: list[tuple[int, float]],
    *,
    beam_count: int,
) -> float:
    """Use the nearest two adjacent beams so thin chair legs are not hidden."""
    ordered = sorted(values, key=lambda item: item[0])
    candidates = [
        max(previous[1], current[1])
        for previous, current in zip(ordered, ordered[1:])
        if current[0] == previous[0] + 1
    ]
    if (
        len(ordered) >= 2
        and ordered[0][0] == 0
        and ordered[-1][0] == beam_count - 1
    ):
        candidates.append(max(ordered[0][1], ordered[-1][1]))
    return min(candidates) if candidates else 0.0


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))
