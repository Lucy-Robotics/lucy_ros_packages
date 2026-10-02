from pathlib import Path

import pytest
from src.pipeline import build as pipeline_build


def _sample_data() -> dict:
    return {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {
            'rp2040_right_arm': {
                'firmware_target': 'lucy_right_arm',
                'board_class': 'internal_servo_only',
            },
        },
    }


def _make_fw_tree(tmp_path: Path, crate: str = 'rp2040_servo2040') -> Path:
    fw_src = tmp_path / 'fw'
    (fw_src / 'firmwares' / crate).mkdir(parents=True)
    (fw_src / 'firmwares' / crate / 'Cargo.toml').write_text('[package]\nname="x"\n')
    (fw_src / 'build').mkdir(parents=True)
    return fw_src


def test_firmware_crate_relpath_by_board_class():
    assert (
        pipeline_build.firmware_crate_relpath({'board_class': 'internal_servo_only'})
        == 'firmwares/rp2040_servo2040'
    )
    assert (
        pipeline_build.firmware_crate_relpath({'board_class': 'bus_servo_only'})
        == 'firmwares/rp2040_bus_servo'
    )
    assert (
        pipeline_build.firmware_crate_relpath({'board_class': 'internal_servo_i2c_pwm'})
        == 'firmwares/rp2040_servo2040'
    )
    assert (
        pipeline_build.firmware_crate_relpath(
            {
                'board_class': 'internal_servo_only',
                'firmware_crate': 'firmwares/custom',
            }
        )
        == 'firmwares/custom'
    )


def test_firmware_package_name():
    assert (
        pipeline_build.firmware_package_name('firmwares/rp2040_servo2040')
        == 'lucy_embedded_firmware_rp2040_servo2040'
    )


def test_run_build_phase_requires_source_dir(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        pipeline_build.run_build_phase(
            data=_sample_data(),
            selected_boards=None,
            workspace_src=tmp_path,
            timeout_seconds=10,
            feedback=lambda **kwargs: None,
            log_error=lambda _msg: None,
        )


def test_run_build_phase_reports_missing_uf2(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    _make_fw_tree(tmp_path)
    errors: list[str] = []

    monkeypatch.setattr(
        pipeline_build.shutil,
        'which',
        lambda name: '/usr/bin/cargo' if name == 'cargo' else None,
    )
    monkeypatch.setattr(pipeline_build, '_run_command', lambda **_k: None)

    failed = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda msg: errors.append(msg),
    )

    assert failed == ['rp2040_right_arm']
    assert errors


def test_run_build_phase_success(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    fw_src = _make_fw_tree(tmp_path)
    release = fw_src / 'target' / 'thumbv6m-none-eabi' / 'release'
    release.mkdir(parents=True)
    elf = release / 'lucy_embedded_firmware_rp2040_servo2040'
    elf.write_bytes(b'elf')

    monkeypatch.setattr(pipeline_build.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(pipeline_build, '_run_command', lambda **_k: None)

    def fake_elf_to_uf2(elf_path: Path, uf2: Path, *, timeout_seconds: int):
        uf2.write_bytes(b'uf2')

    monkeypatch.setattr(pipeline_build, '_elf_to_uf2', fake_elf_to_uf2)

    failed = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert (fw_src / 'build' / 'lucy_right_arm.uf2').is_file()


def test_run_build_phase_selects_bus_servo_crate(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    fw_src = _make_fw_tree(tmp_path, crate='rp2040_bus_servo')
    release = fw_src / 'target' / 'thumbv6m-none-eabi' / 'release'
    release.mkdir(parents=True)
    (release / 'lucy_embedded_firmware_rp2040_bus_servo').write_bytes(b'elf')

    data = {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {
            'rp2040_so_arm': {
                'firmware_target': 'so_arm',
                'board_class': 'bus_servo_only',
            },
        },
    }

    seen_manifests: list[str] = []

    def capture_run(**kwargs):
        cmd = kwargs['cmd']
        seen_manifests.append(cmd[cmd.index('--manifest-path') + 1])

    monkeypatch.setattr(pipeline_build.shutil, 'which', lambda name: f'/usr/bin/{name}')
    monkeypatch.setattr(pipeline_build, '_run_command', capture_run)
    monkeypatch.setattr(
        pipeline_build,
        '_elf_to_uf2',
        lambda elf_path, uf2, *, timeout_seconds: uf2.write_bytes(b'uf2'),
    )

    failed = pipeline_build.run_build_phase(
        data=data,
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert any('rp2040_bus_servo' in m for m in seen_manifests)
    assert (fw_src / 'build' / 'so_arm.uf2').is_file()
