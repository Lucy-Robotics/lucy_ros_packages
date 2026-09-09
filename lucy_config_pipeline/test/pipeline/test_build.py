from pathlib import Path

import pytest
from src.pipeline import build as pipeline_build


def _sample_data() -> dict:
    return {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {'rp2040_right_arm': {'firmware_target': 'lucy_right_arm'}},
    }


def _make_fw_tree(tmp_path: Path) -> Path:
    fw_src = tmp_path / 'fw'
    (fw_src / 'firmwares' / 'rp2040').mkdir(parents=True)
    (fw_src / 'firmwares' / 'rp2040' / 'Cargo.toml').write_text('[package]\nname="x"\n')
    (fw_src / 'build').mkdir(parents=True)
    return fw_src


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

    monkeypatch.setattr(pipeline_build.shutil, 'which', lambda name: '/usr/bin/cargo' if name == 'cargo' else None)
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
    elf = release / 'lucy_embedded_firmware_rp2040'
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
