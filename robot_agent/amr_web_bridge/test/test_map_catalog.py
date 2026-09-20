from pathlib import Path

from amr_web_bridge.map_catalog import (
    available_map_yaml,
    build_map_catalog,
    delete_map,
    read_map_record,
    rename_map,
    update_map_metadata,
)


def test_map_catalog_reads_ros_map_files(tmp_path: Path):
    (tmp_path / 'floor_one.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    yaml_path = tmp_path / 'floor_one.yaml'
    yaml_path.write_text(
        'image: floor_one.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        encoding='utf-8',
    )

    catalog = build_map_catalog(str(tmp_path), 'floor_one', 'robot01')

    assert catalog['source'] == 'ROS_FILESYSTEM'
    assert catalog['active_map_id'] == 'floor_one'
    assert catalog['maps'][0]['active'] is True
    assert catalog['maps'][0]['available'] is True
    assert catalog['maps'][0]['resolution'] == 0.05


def test_map_record_reports_missing_image(tmp_path: Path):
    yaml_path = tmp_path / 'broken.yaml'
    yaml_path.write_text('image: missing.pgm\nresolution: 0.05\n', encoding='utf-8')

    record = read_map_record(yaml_path, None)

    assert record['available'] is False
    assert record['issue'] == 'map image file is missing'


def test_map_catalog_does_not_scan_working_directory_when_unconfigured():
    catalog = build_map_catalog('', 'warehouse_map', 'robot01')

    assert catalog['maps'] == []


def test_available_map_yaml_rejects_traversal_and_missing_assets(tmp_path: Path):
    (tmp_path / 'ready.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (tmp_path / 'ready.yaml').write_text(
        'image: ready.pgm\nresolution: 0.05\n',
        encoding='utf-8',
    )
    (tmp_path / 'broken.yaml').write_text(
        'image: missing.pgm\nresolution: 0.05\n',
        encoding='utf-8',
    )

    assert available_map_yaml(str(tmp_path), 'ready') == tmp_path / 'ready.yaml'
    assert available_map_yaml(str(tmp_path), '../ready') is None
    assert available_map_yaml(str(tmp_path), 'broken') is None


def test_metadata_and_rename_are_persisted_as_robot_sidecar(tmp_path: Path):
    (tmp_path / 'ready.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (tmp_path / 'ready.yaml').write_text(
        'image: ready.pgm\nresolution: 0.05\n', encoding='utf-8'
    )
    update_map_metadata(str(tmp_path), 'ready', {
        'name': 'Engineering Floor',
        'building': 'Building A',
        'floor': '2',
        'area_description': 'East wing',
    })
    record = read_map_record(tmp_path / 'ready.yaml', None)
    assert record['name'] == 'Engineering Floor'
    assert record['building'] == 'Building A'

    rename_map(str(tmp_path), 'ready', 'engineering_floor')
    assert not (tmp_path / 'ready.yaml').exists()
    assert (tmp_path / 'engineering_floor.yaml').exists()
    assert (tmp_path / 'engineering_floor.metadata.json').exists()
    assert read_map_record(
        tmp_path / 'engineering_floor.yaml', None
    )['name'] == 'Engineering Floor'


def test_delete_preserves_an_image_referenced_by_another_map(tmp_path: Path):
    (tmp_path / 'shared.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    for map_id in ('one', 'two'):
        (tmp_path / f'{map_id}.yaml').write_text(
            'image: shared.pgm\nresolution: 0.05\n', encoding='utf-8'
        )
    delete_map(str(tmp_path), 'one')
    assert not (tmp_path / 'one.yaml').exists()
    assert (tmp_path / 'shared.pgm').exists()
    delete_map(str(tmp_path), 'two')
    assert not (tmp_path / 'shared.pgm').exists()
