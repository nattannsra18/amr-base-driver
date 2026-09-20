from __future__ import annotations

import math
from typing import Any, Sequence, TypeVar


T = TypeVar('T')


def downsample_preserving_endpoints(
    values: Sequence[T],
    maximum: int,
) -> list[T]:
    """Deterministically bound a sequence while retaining both ends."""
    if maximum < 2:
        raise ValueError('maximum must be at least 2')
    if len(values) <= maximum:
        return list(values)

    last_index = len(values) - 1
    return [
        values[(index * last_index) // (maximum - 1)]
        for index in range(maximum)
    ]


def quaternion_yaw(orientation: Any) -> float | None:
    components = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in components):
        return None

    x, y, z, w = components
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return yaw if math.isfinite(yaw) else None


def serialize_path(
    message: Any,
    command: dict[str, Any] | None,
    maximum: int,
    timestamp: str,
) -> dict[str, Any] | None:
    """Serialize a real ROS Path only for a valid active command."""
    if command is None:
        return None

    required = ('command_id', 'task_id', 'stage')
    if any(not command.get(key) for key in required):
        return None

    poses: list[dict[str, float]] = []
    for stamped_pose in message.poses:
        pose = stamped_pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            continue

        serialized_pose = {'x': x, 'y': y}
        yaw = quaternion_yaw(pose.orientation)
        if yaw is not None:
            serialized_pose['yaw'] = yaw
        poses.append(serialized_pose)

    if not poses:
        return None

    bounded = downsample_preserving_endpoints(
        poses,
        maximum,
    )
    return {
        'type': 'navigation_path',
        'command_id': str(command['command_id']),
        'task_id': str(command['task_id']),
        'stage': str(command['stage']),
        'frame_id': message.header.frame_id or 'map',
        'timestamp': timestamp,
        'poses': bounded,
    }


def serialize_preview_path(
    message: Any,
    maximum: int,
) -> tuple[str, list[dict[str, float]]] | None:
    """Serialize a planner result without active-navigation metadata."""
    poses: list[dict[str, float]] = []
    for stamped_pose in message.poses:
        pose = stamped_pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        serialized_pose = {'x': x, 'y': y}
        yaw = quaternion_yaw(pose.orientation)
        if yaw is not None:
            serialized_pose['yaw'] = yaw
        poses.append(serialized_pose)

    if not poses:
        return None
    return (
        message.header.frame_id or 'map',
        downsample_preserving_endpoints(poses, maximum),
    )


def path_signature(path: dict[str, Any]) -> tuple[Any, ...]:
    return (
        path['command_id'],
        path['task_id'],
        path['stage'],
        path['frame_id'],
        tuple(
            (pose['x'], pose['y'], pose.get('yaw'))
            for pose in path['poses']
        ),
    )
