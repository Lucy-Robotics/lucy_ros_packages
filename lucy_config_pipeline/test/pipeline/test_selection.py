from pathlib import Path

import pytest
from src.config_store import ConfigStore
from src.pipeline.selection import board_build_plan
from src.pipeline.selection import resolve_firmware_paths
from src.pipeline.selection import resolve_mapping_input
from src.pipeline.selection import select_boards_to_process


def test_resolve_mapping_input_active_and_named(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()
    store.write_named_yaml('default', 'version: 1\nrobot_name: t\n')
    store.write_named_yaml('alt', 'version: 1\nrobot_name: alt\n')
    store.activate('default')

    name, txt = resolve_mapping_input(store, '')
    assert name == 'default'
    assert 'robot_name: t' in txt

    name, txt = resolve_mapping_input(store, 'alt')
    assert name == 'alt'
    assert 'robot_name: alt' in txt


def test_resolve_mapping_input_from_yaml_path(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()

    p = tmp_path / 'custom.yaml'
    p.write_text('version: 1\nrobot_name: custom\n', encoding='utf-8')

    name, txt = resolve_mapping_input(store, str(p))
    assert name == 'custom'
    assert 'robot_name: custom' in txt


def test_select_boards_to_process():
    data = {'boards': {'rp2040_left_arm': {}, 'rp2040_right_arm': {}}}

    assert select_boards_to_process(data, []) is None
    assert select_boards_to_process(data, ['rp2040_left_arm']) == {'rp2040_left_arm'}

    with pytest.raises(ValueError, match='unknown boards'):
        select_boards_to_process(data, ['rp2040_foo'])


def test_resolve_firmware_paths_relative(tmp_path: Path):
    data = {'firmware': {'source_dir': 'lucy_embedded_firmware', 'build_dir': 'build/firmware'}}
    paths = resolve_firmware_paths(data, tmp_path)
    assert paths.source_dir == (tmp_path / 'lucy_embedded_firmware').resolve()
    assert paths.build_dir == (
        tmp_path / 'lucy_embedded_firmware' / 'build' / 'firmware'
    ).resolve()


def test_resolve_firmware_paths_requires_source_dir(tmp_path: Path):
    with pytest.raises(ValueError, match='missing firmware.source_dir'):
        resolve_firmware_paths({'firmware': {}}, tmp_path)


def test_board_build_plan_all_and_selected():
    data = {
        'boards': {
            'rp2040_left_arm': {'firmware_target': 'lucy_left_arm'},
            'rp2040_right_arm': {'firmware_target': 'lucy_right_arm'},
        }
    }

    all_plan = board_build_plan(data, None)
    assert all_plan == [
        ('rp2040_left_arm', 'lucy_left_arm'),
        ('rp2040_right_arm', 'lucy_right_arm'),
    ]

    selected_plan = board_build_plan(data, {'rp2040_right_arm'})
    assert selected_plan == [('rp2040_right_arm', 'lucy_right_arm')]


def test_board_build_plan_requires_firmware_target():
    data = {'boards': {'rp2040_right_arm': {}}}
    with pytest.raises(ValueError, match='missing firmware_target'):
        board_build_plan(data, {'rp2040_right_arm'})
