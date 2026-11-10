from pathlib import Path
import sys

import pytest
from src.pipeline import build


def _run(script: str, tmp_path: Path, lines: list[str], timeout: int = 10) -> None:
    build._run_command(
        phase="build",
        board="left",
        cmd=[sys.executable, "-c", script],
        cwd=tmp_path,
        timeout_seconds=timeout,
        feedback=lambda **kw: lines.append(kw["detail"]),
    )


def test_failure_message_ends_with_the_last_twelve_output_lines(tmp_path: Path):
    script = "for i in range(50): print(f'line {i}')\nraise SystemExit(2)"
    with pytest.raises(RuntimeError) as failed:
        _run(script, tmp_path, [])
    message = str(failed.value)
    assert message.startswith("command failed (2):")
    assert message.splitlines()[-13:] == ["raise SystemExit(2)"] + [
        f"line {i}" for i in range(38, 50)
    ]


def test_silent_failure_says_there_was_no_output(tmp_path: Path):
    with pytest.raises(RuntimeError, match=r"\(no output\)"):
        _run("raise SystemExit(1)", tmp_path, [])


def test_feedback_stops_after_two_hundred_lines(tmp_path: Path):
    lines: list[str] = []
    _run("for i in range(250): print(i)", tmp_path, lines)
    assert lines == [str(i) for i in range(200)]


def test_hanging_build_is_killed_at_its_timeout(tmp_path: Path):
    with pytest.raises(TimeoutError, match="timed out after 1s"):
        _run("import time; time.sleep(30)", tmp_path, [], timeout=1)


def test_cargo_env_prefers_the_pixi_toolchain(tmp_path: Path, monkeypatch):
    (tmp_path / "cargo" / "bin").mkdir(parents=True)
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path))
    monkeypatch.setenv("PATH", "/usr/bin")
    env = build._cargo_env()
    assert env["CARGO_HOME"] == str(tmp_path / "cargo")
    assert env["RUSTUP_HOME"] == str(tmp_path / "rustup")
    assert env["PATH"].split(":")[0] == str(tmp_path / "cargo" / "bin")
