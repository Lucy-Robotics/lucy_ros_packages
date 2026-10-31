from pathlib import Path

import pytest
from src.pipeline import build as pipeline_build


def _sample_data() -> dict:
    return {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {'rp2040_right_arm': {'firmware_target': 'pico_micro_ros_right_arm'}},
    }


def test_run_build_phase_requires_source_dir(tmp_path: Path):
    feedback_calls: list[dict] = []
    with pytest.raises(FileNotFoundError):
        pipeline_build.run_build_phase(
            data=_sample_data(),
            selected_boards=None,
            workspace_src=tmp_path,
            timeout_seconds=10,
            feedback=lambda **kwargs: feedback_calls.append(kwargs),
            log_error=lambda _msg: None,
        )


def test_run_build_phase_reports_missing_uf2(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    fw_src = tmp_path / 'fw'
    fw_src.mkdir(parents=True)
    errors: list[str] = []

    def fake_run_command(**_kwargs):
        return None

    monkeypatch.setattr(pipeline_build, '_run_command', fake_run_command)

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
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    (fw_build / 'CMakeCache.txt').write_text('cached', encoding='utf-8')

    def fake_run_command(*, cmd: list[str], cwd: Path, **_kwargs):
        if cmd[0] == 'make':
            (cwd / 'pico_micro_ros_right_arm.uf2').write_text('ok', encoding='utf-8')

    monkeypatch.setattr(pipeline_build, '_run_command', fake_run_command)

    failed = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []


def test_mega_board_builds_with_cargo_and_its_generated_tables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    crate = tmp_path / 'fw' / 'firmwares' / 'arduino_mega'
    artifacts = crate / 'target' / 'lucy'
    artifacts.mkdir(parents=True)
    (artifacts / 'config_mega_left.rs').write_text('// tables', encoding='utf-8')
    calls: list[dict] = []

    def fake_run_command(*, cmd: list[str], cwd: Path, env=None, **_kwargs):
        calls.append({'cmd': cmd, 'cwd': cwd, 'env': env})
        elf = cwd / pipeline_build.MEGA_ELF
        elf.parent.mkdir(parents=True, exist_ok=True)
        elf.write_bytes(b'elf')

    monkeypatch.setattr(pipeline_build, '_run_command', fake_run_command)
    failed = pipeline_build.run_build_phase(
        data={
            'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
            'boards': {
                'mega_left': {'board_class': 'i2c_pwm_only', 'firmware_target': 'arduino_mega'},
            },
        },
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    # No cmake for a Mega-only plan; cargo runs in the crate so its toolchain file applies.
    assert [c['cmd'] for c in calls] == [['cargo', 'build', '--release']]
    assert calls[0]['cwd'] == crate
    assert calls[0]['env']['LUCY_MEGA_BOARD_CONFIG'] == str(artifacts / 'config_mega_left.rs')
    assert (artifacts / 'mega_left.elf').read_bytes() == b'elf'
