from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from typing import Any

import yaml


MAP_ID_PATTERN = re.compile(r'^[A-Za-z0-9_.-]{1,120}$')
METADATA_SUFFIX = '.metadata.json'


def _metadata_path(yaml_path: Path) -> Path:
    return yaml_path.with_name(f'{yaml_path.stem}{METADATA_SUFFIX}')


def _read_metadata(yaml_path: Path) -> dict[str, Any]:
    path = _metadata_path(yaml_path)
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _optional_text(value: Any, maximum: int) -> str | None:
    if not isinstance(value, str):
        return None
    normalized = value.strip()
    return normalized[:maximum] if normalized else None


def _modified_at(*paths: Path) -> str | None:
    timestamps = [path.stat().st_mtime for path in paths if path.is_file()]
    if not timestamps:
        return None
    return datetime.fromtimestamp(max(timestamps), tz=timezone.utc).isoformat()


def _safe_number(value: Any, expected_type: type) -> int | float | None:
    try:
        parsed = expected_type(value)
    except (TypeError, ValueError):
        return None
    return parsed


def read_map_record(yaml_path: Path, active_map_id: str | None) -> dict[str, Any]:
    issue = None
    values: dict[str, Any] = {}
    try:
        loaded = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        if not isinstance(loaded, dict):
            raise ValueError('map YAML must contain an object')
        values = loaded
    except (OSError, UnicodeError, yaml.YAMLError, ValueError) as error:
        issue = str(error)

    image_value = values.get('image')
    image_path = None
    if isinstance(image_value, str) and image_value.strip():
        candidate = Path(image_value.strip())
        image_path = candidate if candidate.is_absolute() else yaml_path.parent / candidate
    available = issue is None and image_path is not None and image_path.is_file()
    if issue is None and image_path is None:
        issue = 'map YAML does not declare an image'
    elif issue is None and not available:
        issue = 'map image file is missing'

    metadata_path = _metadata_path(yaml_path)
    size_bytes = yaml_path.stat().st_size if yaml_path.is_file() else 0
    if image_path is not None and image_path.is_file():
        size_bytes += image_path.stat().st_size
    if metadata_path.is_file():
        size_bytes += metadata_path.stat().st_size
    map_id = yaml_path.stem
    metadata = _read_metadata(yaml_path)
    return {
        'id': map_id,
        'name': _optional_text(metadata.get('name'), 160)
        or map_id.replace('_', ' ').replace('-', ' ').title(),
        'yaml_file': yaml_path.name,
        'image_file': image_path.name if image_path is not None else None,
        'resolution': _safe_number(values.get('resolution'), float),
        'size_bytes': size_bytes,
        'modified_at': (
            _modified_at(yaml_path, image_path, metadata_path)
            if image_path else _modified_at(yaml_path, metadata_path)
        ),
        'available': available,
        'active': map_id == active_map_id,
        'issue': issue,
        'building': _optional_text(metadata.get('building'), 120),
        'floor': _optional_text(metadata.get('floor'), 80),
        'area_description': _optional_text(
            metadata.get('area_description'), 240
        ),
    }


def build_map_catalog(
    maps_directory: str,
    active_map_id: str | None,
    robot_id: str,
) -> dict[str, Any]:
    directory_value = maps_directory.strip()
    records: list[dict[str, Any]] = []
    if directory_value:
        directory = Path(directory_value).expanduser()
        if directory.is_dir():
            records = [
                read_map_record(path, active_map_id)
                for path in sorted(directory.glob('*.yaml'))
            ]
    return {
        'type': 'map_catalog',
        'robot_id': robot_id,
        'source': 'ROS_FILESYSTEM',
        'active_map_id': active_map_id or None,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'maps': records,
    }


def available_map_yaml(maps_directory: str, map_id: str) -> Path | None:
    """Resolve only a ready YAML map contained by the configured directory."""
    directory_value = maps_directory.strip()
    if not directory_value or MAP_ID_PATTERN.fullmatch(map_id) is None:
        return None
    directory = Path(directory_value).expanduser().resolve()
    candidate = (directory / f'{map_id}.yaml').resolve()
    if not candidate.is_relative_to(directory) or not candidate.is_file():
        return None
    record = read_map_record(candidate, None)
    return candidate if record['available'] else None


def update_map_metadata(
    maps_directory: str,
    map_id: str,
    metadata: dict[str, Any],
) -> None:
    yaml_path = available_map_yaml(maps_directory, map_id)
    if yaml_path is None:
        raise ValueError('Map is unavailable on the robot')
    name = _optional_text(metadata.get('name'), 160)
    if name is None:
        raise ValueError('Map name is required')
    value = {
        'name': name,
        'building': _optional_text(metadata.get('building'), 120),
        'floor': _optional_text(metadata.get('floor'), 80),
        'area_description': _optional_text(
            metadata.get('area_description'), 240
        ),
    }
    target = _metadata_path(yaml_path)
    temporary = target.with_name(f'.{target.name}.tmp')
    try:
        temporary.write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)


def rename_map(maps_directory: str, map_id: str, new_map_id: str) -> None:
    yaml_path = available_map_yaml(maps_directory, map_id)
    if yaml_path is None:
        raise ValueError('Map is unavailable on the robot')
    if MAP_ID_PATTERN.fullmatch(new_map_id) is None:
        raise ValueError('New map ID is invalid')
    target = yaml_path.with_name(f'{new_map_id}.yaml')
    target_metadata = _metadata_path(target)
    source_metadata = _metadata_path(yaml_path)
    if target.exists() or target_metadata.exists():
        raise ValueError('New map ID already exists')
    yaml_path.rename(target)
    try:
        if source_metadata.exists():
            source_metadata.rename(target_metadata)
    except OSError:
        target.rename(yaml_path)
        raise


def delete_map(maps_directory: str, map_id: str) -> None:
    yaml_path = available_map_yaml(maps_directory, map_id)
    if yaml_path is None:
        raise ValueError('Map is unavailable on the robot')
    directory = yaml_path.parent.resolve()
    record = read_map_record(yaml_path, None)
    image_path = None
    if record['image_file']:
        candidate = (directory / record['image_file']).resolve()
        if candidate.is_relative_to(directory):
            image_path = candidate

    yaml_path.unlink()
    _metadata_path(yaml_path).unlink(missing_ok=True)
    if image_path is None or not image_path.is_file():
        return
    for other_yaml in directory.glob('*.yaml'):
        other = read_map_record(other_yaml, None)
        if other['image_file'] and (
            other_yaml.parent / other['image_file']
        ).resolve() == image_path:
            return
    image_path.unlink()
