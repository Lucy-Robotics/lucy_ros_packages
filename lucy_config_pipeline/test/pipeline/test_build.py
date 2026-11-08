from pathlib import Path

import pytest
from src.pipeline import build as pipeline_build


def _sample_data() -> dict:
    return {
        "firmware": {"source_dir": "fw", "build_dir": "build"},
        "boards": {"rp2040_right_arm": {"firmware_target": "pico_micro_ros_right_arm"}},
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


def test_run_build_phase_reports_missing_uf2(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    fw_src = tmp_path / "fw"
    fw_src.mkdir(parents=True)
    errors: list[str] = []

    def fake_run_command(**_kwargs):
        return None

    monkeypatch.setattr(pipeline_build, "_run_command", fake_run_command)

    failed = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda msg: errors.append(msg),
    )

    assert failed == ["rp2040_right_arm"]
    assert errors


def test_run_build_phase_success(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    fw_src = tmp_path / "fw"
    fw_build = fw_src / "build"
    fw_build.mkdir(parents=True)
    (fw_build / "CMakeCache.txt").write_text("cached", encoding="utf-8")

    def fake_run_command(*, cmd: list[str], cwd: Path, **_kwargs):
        if cmd[0] == "make":
            (cwd / "pico_micro_ros_right_arm.uf2").write_text("ok", encoding="utf-8")

    monkeypatch.setattr(pipeline_build, "_run_command", fake_run_command)

    failed = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
