from pathlib import Path

from src.config_store import ConfigStore


def test_save_list_activate_delete(tmp_path: Path):
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()

    a = "version: 1\nrobot_name: t\n"
    b = "version: 1\nrobot_name: t2\n"
    store.write_named_yaml("default", a)
    store.write_named_yaml("alt", b)

    assert store.list_configs() == ["alt", "default"]

    backup = store.activate("default")
    assert backup == ""
    assert store.get_active_name() == "default"

    backup = store.activate("alt")
    assert backup != ""
    assert (store.backups_dir / f"{backup}.yaml").is_file()

    try:
        store.delete("default")
        assert False, "default deletion should fail"
    except ValueError:
        pass


def test_activate_preserves_flashed_meta(tmp_path: Path):
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()
    store.write_named_yaml("a", "version: 1\nrobot_name: x\n")
    store.activate("a")
    store.record_flashed_preset("a")
    assert store.get_flashed_name() == "a"
    assert store.get_flashed_at()

    store.write_named_yaml("b", "version: 1\nrobot_name: y\n")
    store.activate("b")
    assert store.get_active_name() == "b"
    assert store.get_flashed_name() == "a"
