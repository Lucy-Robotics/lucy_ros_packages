import io
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
from src.pipeline import flash


def _run(cmd: list[str], tmp_path: Path, timeout: int = 10) -> list[str]:
    lines: list[str] = []
    flash._run_command(
        phase="flash",
        board="left",
        cmd=cmd,
        cwd=tmp_path,
        timeout_seconds=timeout,
        feedback=lambda **kw: lines.append(kw["detail"]),
    )
    return lines


def test_command_output_lines_reach_the_feedback_without_blanks(tmp_path: Path):
    script = "print('one'); print(''); print('  two  ')"
    assert _run([sys.executable, "-c", script], tmp_path) == ["one", "two"]


def test_failing_command_names_its_exit_code(tmp_path: Path):
    with pytest.raises(RuntimeError, match=r"command failed \(3\)"):
        _run([sys.executable, "-c", "raise SystemExit(3)"], tmp_path)


def test_hanging_command_is_killed_at_its_timeout(tmp_path: Path):
    start = time.monotonic()
    with pytest.raises(TimeoutError, match="timed out after 1s"):
        _run([sys.executable, "-c", "import time; time.sleep(30)"], tmp_path, timeout=1)
    assert time.monotonic() - start < 10


def _link_at(monkeypatch: pytest.MonkeyPatch, path: Path) -> None:
    monkeypatch.setattr(flash, "_bridge_link_path", lambda node: path)


def test_bridge_link_counts_a_stamp_written_after_the_flash(tmp_path, monkeypatch):
    link = tmp_path / "link"
    _link_at(monkeypatch, link)
    link.write_text(f"{time.time():.3f}\n", encoding="utf-8")
    assert flash._wait_for_bridge_link("left", since=time.time(), timeout_seconds=1)


def test_bridge_link_ignores_an_old_stamp(tmp_path, monkeypatch):
    link = tmp_path / "link"
    _link_at(monkeypatch, link)
    link.write_text("100.0\n", encoding="utf-8")
    os.utime(link, (100, 100))
    assert not flash._wait_for_bridge_link(
        "left", since=time.time(), timeout_seconds=0.3
    )


def test_unreadable_stamp_falls_back_to_the_file_time(tmp_path, monkeypatch):
    link = tmp_path / "link"
    _link_at(monkeypatch, link)
    link.write_text("not-a-time\n", encoding="utf-8")
    assert flash._wait_for_bridge_link("left", since=time.time(), timeout_seconds=1)


def test_missing_link_file_times_out(tmp_path, monkeypatch):
    _link_at(monkeypatch, tmp_path / "never")
    assert not flash._wait_for_bridge_link("left", since=0.0, timeout_seconds=0.3)


def test_bridge_process_is_found_by_its_node_name(monkeypatch):
    calls = []

    def run(cmd, **kwargs):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0 if "left" in cmd[-1] else 1)

    monkeypatch.setattr(subprocess, "run", run)
    assert flash._mcu_bridge_process_running("left")
    assert not flash._mcu_bridge_process_running("right")
    assert calls[0] == ["pgrep", "-f", "lucy-embedded-firmware-mcu-bridge left"]


def test_no_pgrep_means_no_bridge(monkeypatch):
    def run(cmd, **kwargs):
        raise OSError("no pgrep")

    monkeypatch.setattr(subprocess, "run", run)
    assert not flash._mcu_bridge_process_running("left")


def test_signalling_a_bridge_stops_it_and_forgets_its_link(tmp_path, monkeypatch):
    link = tmp_path / "link"
    link.write_text("1\n", encoding="utf-8")
    _link_at(monkeypatch, link)
    calls = []
    monkeypatch.setattr(subprocess, "run", lambda cmd, **kw: calls.append(cmd))
    flash._signal_mcu_bridge("left")
    flash._signal_mcu_bridge("")
    assert calls == [["pkill", "-f", "lucy-embedded-firmware-mcu-bridge left"]]
    assert not link.exists()


def test_flash_waits_follow_their_environment(monkeypatch):
    monkeypatch.setenv("LUCY_PIPELINE_FLASH_TIMEOUT_SEC", "7")
    monkeypatch.setenv("LUCY_PIPELINE_FLASH_WAIT_SEC", "8")
    monkeypatch.setenv("LUCY_PIPELINE_FLASH_UPTIME_WAIT_SEC", "9")
    monkeypatch.setenv("LUCY_PIPELINE_FLASH_BRIDGE_LINK_WAIT_SEC", "2.5")
    assert flash.flash_picotool_timeout_seconds() == 7
    assert flash.flash_usb_wait_seconds() == 8
    assert flash.flash_uptime_wait_seconds() == 9
    assert flash.flash_bridge_link_wait_seconds() == 2.5


def _bootsel(root: Path, name: str) -> Path:
    volume = root / name
    volume.mkdir(parents=True)
    (volume / "INFO_UF2.TXT").write_text("UF2 Bootloader\n", encoding="utf-8")
    return volume


def test_only_bootsel_mounts_are_returned_once_each(tmp_path, monkeypatch):
    volume = _bootsel(tmp_path, "RPI-RP2")
    no_info = tmp_path / "RP2350"
    no_info.mkdir()
    mounts = f"/dev/sda1 {volume} vfat rw 0 0\n{volume} {volume} vfat rw 0 0\n"
    mounts += (
        f"/dev/sdb1 {no_info} vfat rw 0 0\n/dev/sdc1 {tmp_path} ext4 rw 0 0\nbroken\n"
    )
    monkeypatch.setattr(
        flash, "open", lambda *a, **k: io.StringIO(mounts), raising=False
    )
    monkeypatch.setenv("USER", "lucy-test-user-without-media")
    assert flash._find_rpi_rp2_mounts() == [volume.resolve()]


def test_unreadable_proc_mounts_finds_nothing(monkeypatch):
    def no_mounts(*args, **kwargs):
        raise OSError("no /proc")

    monkeypatch.setattr(flash, "open", no_mounts, raising=False)
    monkeypatch.setenv("USER", "lucy-test-user-without-media")
    assert flash._find_rpi_rp2_mounts() == []
