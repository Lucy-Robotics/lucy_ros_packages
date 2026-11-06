from pathlib import Path

from src.config_store import ConfigStore


def test_save_list_activate_delete(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()

    a = 'version: 1\nrobot_name: t\n'
    b = 'version: 1\nrobot_name: t2\n'
    store.write_named_yaml('default', a)
    store.write_named_yaml('alt', b)

    assert store.list_configs() == ['alt', 'default']

    backup = store.activate('default')
    assert backup == ''
    assert store.get_active_name() == 'default'

    backup = store.activate('alt')
    assert backup != ''
    assert (store.backups_dir / f'{backup}.yaml').is_file()

    try:
        store.delete('default')
        assert False, 'default deletion should fail'
    except ValueError:
        pass


def test_activate_preserves_flashed_meta(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()
    store.write_named_yaml('a', 'version: 1\nrobot_name: x\n')
    store.activate('a')
    store.record_flashed_preset('a')
    assert store.get_flashed_name() == 'a'
    assert store.get_flashed_at()

    store.write_named_yaml('b', 'version: 1\nrobot_name: y\n')
    store.activate('b')
    assert store.get_active_name() == 'b'
    assert store.get_flashed_name() == 'a'


def test_atomic_writes_leave_no_temp_files(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.write_named_yaml('a', 'version: 1\n')
    store.activate('a')
    store.activate('a')
    assert [p.name for p in store.named_dir.iterdir()] == ['a.yaml']
    assert not list(store.config_dir.glob('.*tmp'))


def test_activate_keeps_flashed_fields(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.write_named_yaml('a', 'version: 1\n')
    store.record_flashed_preset('a')
    flashed_at = store.get_flashed_at()
    store.activate('a')
    assert store.get_flashed_name() == 'a'
    assert store.get_flashed_at() == flashed_at


def test_activating_a_calibrated_preset_binds_its_adapter(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.write_named_yaml('a', 'calibration:\n  adapter_serial: ABC\n')
    store.write_named_yaml('b', 'calibration:\n  adapter_serial: ABC\n')
    store.write_named_yaml('plain', 'version: 1\n')

    store.activate('a')
    assert store.bound_config('ABC') == 'a'
    store.activate('b')
    assert store.bound_config('ABC') == 'b', 'the latest activation wins'
    store.activate('plain')
    assert store.bound_config('ABC') == 'b'
    assert store.bound_config('other') == ''


def test_cli_preset_without_a_serial_binds_nothing(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.write_named_yaml('a', "calibration:\n  adapter_serial: ''\n")
    store.activate('a')
    assert store.read_bindings() == {}


def test_bindings_survive_selecting_an_adapter(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()
    store.bind_adapter('A', 'x')
    store.select_adapter('A')
    assert store.bound_config('A') == 'x'
    assert store.selected_adapter() == 'A'


def test_journal_round_trip(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    assert store.read_journal() == {}
    store.write_journal({'old': [{'id': 1, 'homing_offset': -5}]})
    assert store.read_journal()['old'][0]['homing_offset'] == -5
    store.clear_journal()
    assert store.read_journal() == {}
    store.clear_journal()


def test_corrupt_journal_reads_as_none(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.ensure_layout()
    store.session_journal.write_text('{not yaml', encoding='utf-8')
    assert store.read_journal() == {}


def test_delete_removes_the_lerobot_sibling(tmp_path: Path):
    store = ConfigStore(tmp_path / 'hardware')
    store.write_named_yaml('a', 'version: 1\n')
    sibling = store.lerobot_json_path('a')
    sibling.write_text('{}', encoding='utf-8')
    store.delete('a')
    assert not sibling.exists()
