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

    failed, built = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda msg: errors.append(msg),
    )

    assert failed == ["rp2040_right_arm"]
    assert built == set()
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

    failed, built = pipeline_build.run_build_phase(
        data=_sample_data(),
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert built == {"rp2040_right_arm"}


def test_mega_board_builds_with_cargo_and_its_generated_tables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    crate = tmp_path / "fw" / "firmwares" / "arduino_mega"
    artifacts = crate / "target" / "lucy"
    artifacts.mkdir(parents=True)
    (artifacts / "config_mega_left.rs").write_text("// tables", encoding="utf-8")
    calls: list[dict] = []

    def fake_run_command(*, cmd: list[str], cwd: Path, env=None, **_kwargs):
        calls.append({"cmd": cmd, "cwd": cwd, "env": env})
        elf = cwd / pipeline_build.MEGA_ELF
        elf.parent.mkdir(parents=True, exist_ok=True)
        elf.write_bytes(b"elf")

    monkeypatch.setattr(pipeline_build, "_run_command", fake_run_command)
    failed, built = pipeline_build.run_build_phase(
        data={
            "firmware": {"source_dir": "fw", "build_dir": "build"},
            "boards": {
                "mega_left": {
                    "board_class": "i2c_pwm_only",
                    "firmware_target": "arduino_mega",
                },
            },
        },
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert built == {"mega_left"}
    # No cmake for a Mega-only plan; cargo runs in the crate so its toolchain file applies.
    assert [c["cmd"] for c in calls] == [["cargo", "build", "--release"]]
    assert calls[0]["cwd"] == crate
    assert calls[0]["env"]["LUCY_MEGA_BOARD_CONFIG"] == str(
        artifacts / "config_mega_left.rs"
    )
    assert (artifacts / "mega_left.elf").read_bytes() == b"elf"


def test_mega_board_builds_beside_the_rp2040_cargo_crate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    rp2040 = tmp_path / "fw" / "firmwares" / "rp2040"
    rp2040.mkdir(parents=True)
    (rp2040 / "Cargo.toml").write_text("", encoding="utf-8")
    megas: list[str] = []
    monkeypatch.setattr(pipeline_build, "firmware_tool_env", lambda base=None: {})
    monkeypatch.setattr(
        pipeline_build, "resolve_firmware_tool", lambda name, env=None: name
    )
    monkeypatch.setattr(
        pipeline_build, "_build_mega", lambda board, *_args: megas.append(board)
    )
    monkeypatch.setattr(pipeline_build, "_run_command", lambda **_kwargs: None)
    failed, built = pipeline_build.run_build_phase(
        data={
            "firmware": {"source_dir": "fw", "build_dir": "build"},
            "boards": {
                "mega_left": {
                    "board_class": "i2c_pwm_only",
                    "firmware_target": "arduino_mega",
                },
            },
            "actuators": [],
        },
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert (failed, built, megas) == ([], {"mega_left"}, ["mega_left"])


def test_cargo_build_writes_one_uf2_per_board(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    source = tmp_path / "lucy_embedded_firmware"
    (source / "firmwares" / "rp2040").mkdir(parents=True)
    (source / "firmwares" / "rp2040" / "Cargo.toml").write_text("", encoding="utf-8")
    calls: list[list[str]] = []
    cargo = tmp_path / "cargo"
    elf2uf2 = tmp_path / "elf2uf2-rs"
    cargo.write_text("", encoding="utf-8")
    elf2uf2.write_text("", encoding="utf-8")

    def fake_run_command(*, cmd: list[str], **_kwargs):
        calls.append(cmd)
        if Path(cmd[0]) == cargo:
            elf = source / "target" / "thumbv6m-none-eabi" / "release"
            elf.mkdir(parents=True, exist_ok=True)
            (elf / "lucy-embedded-firmware-rp2040").write_bytes(b"\0" * 10_000)
        if Path(cmd[0]).name in ("elf2uf2-rs", "elf2uf2-rs.exe"):
            Path(cmd[2]).write_text("uf2", encoding="utf-8")

    monkeypatch.setattr(pipeline_build, "_run_command", fake_run_command)
    monkeypatch.setattr(
        pipeline_build,
        "resolve_firmware_tool",
        lambda name, env=None: cargo if name == "cargo" else elf2uf2,
    )
    monkeypatch.setattr(
        pipeline_build, "firmware_tool_env", lambda base=None: {"PATH": "/tmp"}
    )
    data = {
        "firmware": {"source_dir": "lucy_embedded_firmware", "build_dir": "build"},
        "boards": {
            "rp2040_left_arm": {
                "firmware_target": "pico_left_arm",
                "serial_id": "ABC",
                "board_class": "internal_servo_only",
            },
            "rp2040_so_arm": {
                "firmware_target": "so_arm",
                "serial_id": "",
                "board_class": "bus_servo_only",
            },
        },
        "actuators": [],
    }
    failed, built = pipeline_build.run_build_phase(
        data=data,
        selected_boards=None,
        workspace_src=tmp_path,
        timeout_seconds=10,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )
    assert failed == []
    assert built == {"rp2040_left_arm"}
    assert Path(calls[0][0]) == cargo
    assert Path(calls[1][0]) == elf2uf2
    assert calls[1][2].endswith("rp2040_left_arm.uf2")


def test_cargo_build_fails_before_cargo_when_elf2uf2_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Regression: bare elf2uf2-rs on PATH used to fail mid-board with Errno 2."""
    source = tmp_path / "lucy_embedded_firmware"
    (source / "firmwares" / "rp2040").mkdir(parents=True)
    (source / "firmwares" / "rp2040" / "Cargo.toml").write_text("", encoding="utf-8")
    ran: list[list[str]] = []

    monkeypatch.setattr(
        pipeline_build,
        "_run_command",
        lambda **kwargs: ran.append(kwargs["cmd"]),
    )
    monkeypatch.setattr(pipeline_build, "firmware_tool_env", lambda base=None: {})
    monkeypatch.setattr(
        pipeline_build,
        "resolve_firmware_tool",
        lambda name, env=None: (_ for _ in ()).throw(
            pipeline_build.FirmwareToolchainError(
                "elf2uf2-rs not found — run: pixi run firmware-setup"
            )
        ),
    )
    data = {
        "firmware": {"source_dir": "lucy_embedded_firmware", "build_dir": "build"},
        "boards": {
            "rp2040_left_arm": {
                "firmware_target": "pico_left_arm",
                "board_class": "internal_servo_only",
            },
        },
        "actuators": [],
    }
    errors: list[str] = []
    with pytest.raises(pipeline_build.FirmwareToolchainError, match="firmware-setup"):
        pipeline_build.run_build_phase(
            data=data,
            selected_boards=None,
            workspace_src=tmp_path,
            timeout_seconds=10,
            feedback=lambda **_kwargs: None,
            log_error=lambda msg: errors.append(msg),
        )
    assert ran == []
    assert errors and "firmware-setup" in errors[0]
