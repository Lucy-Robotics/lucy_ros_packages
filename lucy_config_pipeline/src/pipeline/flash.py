from __future__ import annotations

from collections.abc import Callable
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import TYPE_CHECKING

from .selection import board_build_plan
from .selection import is_mega_board
from .selection import mega_artifact_dir
from .selection import resolve_firmware_paths

if TYPE_CHECKING:
    from rclpy.node import Node


def run_flash_phase(
    *,
    data: dict,
    selected_boards: set[str] | None,
    boards_built_ok: set[str],
    workspace_src: Path,
    picotool_timeout_seconds: int,
    usb_wait_seconds: int,
    uptime_wait_seconds: int,
    node: Node | None,
    feedback: Callable[..., None],
    log_error: Callable[[str], None],
) -> tuple[list[str], list[str]]:
    """
    Flash built UF2 images with picotool.

    Only boards present in ``boards_built_ok`` are considered. Boards without a
    non-empty ``serial_id`` are skipped (not counted as failure).

    After USB serial re-enumeration, optionally waits for ``std_msgs/Int32`` on
    the board uptime topic (see ``_uptime_topic``) when ``node`` is set.

    Returns ``(failed_board_ids, flashed_board_ids)`` in stable board order.
    """
    paths = resolve_firmware_paths(data, workspace_src)
    plan = board_build_plan(data, selected_boards)
    flashable = [(b, t) for b, t in plan if b in boards_built_ok]
    boards_map = data.get('boards', {})
    needs_uf2 = any(not is_mega_board(boards_map.get(b, {})) for b, _ in flashable)
    if needs_uf2 and not paths.build_dir.is_dir():
        raise FileNotFoundError(paths.build_dir)
    total = max(len(flashable), 1)

    failed: list[str] = []
    flashed: list[str] = []

    for step_i, (board, target) in enumerate(flashable):
        row = step_i / total
        boards_entry = data.get('boards', {}).get(board, {})
        raw_serial = boards_entry.get('serial_id')
        serial = str(raw_serial).strip() if raw_serial is not None else ''
        if not serial:
            feedback(
                phase='flash',
                progress=min(1.0, row + 0.99 / total),
                detail=f'skip flash for {board}: no serial_id',
                board=board,
            )
            continue

        if is_mega_board(boards_entry):
            try:
                _flash_mega(board, serial, paths, picotool_timeout_seconds, feedback, row)
                flashed.append(board)
                feedback(
                    phase='flash',
                    progress=min(1.0, row + 0.95 / total),
                    detail=f'flash completed for {board}',
                    board=board,
                )
            except Exception as exc:
                failed.append(board)
                log_error(f'Flash failed for {board}: {exc}')
                feedback(
                    phase='flash',
                    progress=min(1.0, row + 0.9 / total),
                    detail=f'flash failed for {board}: {exc}',
                    board=board,
                )
            continue

        uf2 = paths.build_dir / f'{target}.uf2'
        if not uf2.is_file():
            msg = f'missing UF2 for flash: {uf2}'
            log_error(f'Flash failed for {board}: {msg}')
            failed.append(board)
            feedback(
                phase='flash',
                progress=min(1.0, row + 0.5 / total),
                detail=msg,
                board=board,
            )
            continue

        try:
            feedback(
                phase='flash',
                progress=row + 0.1 / total,
                detail=f'picotool load {uf2.name}',
                board=board,
            )
            load_progress = min(1.0, row + 0.25 / total)
            _run_command(
                phase='flash',
                board=board,
                cmd=['sudo', 'picotool', 'load', str(uf2), '-f', '--ser', serial],
                cwd=paths.build_dir,
                timeout_seconds=picotool_timeout_seconds,
                feedback=feedback,
                stream_progress=load_progress,
            )
            # ``picotool load`` already reboots the board into application mode; a
            # follow-up ``picotool reboot`` races USB re-enumeration and often
            # fails (e.g. exit 249) while the device is offline.
            post_load_delay = float(os.environ.get('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '1'))
            if post_load_delay > 0:
                time.sleep(post_load_delay)
            feedback(
                phase='flash',
                progress=min(1.0, row + 0.5 / total),
                detail=f'waiting up to {usb_wait_seconds}s for USB serial',
                board=board,
            )
            if not _wait_for_usb_serial(serial, usb_wait_seconds):
                raise TimeoutError(
                    f'USB serial did not become ready within {usb_wait_seconds}s '
                    f'(board {board})'
                )
            if node is not None and uptime_wait_seconds > 0:
                topic = _uptime_topic(boards_entry)
                feedback(
                    phase='flash',
                    progress=min(1.0, row + 0.75 / total),
                    detail=f'waiting for uptime on {topic} (up to {uptime_wait_seconds}s)',
                    board=board,
                )
                if not _wait_uptime_message(node, topic, float(uptime_wait_seconds)):
                    raise TimeoutError(
                        f'no uptime message on {topic!r} within {uptime_wait_seconds}s '
                        f'(board {board})'
                    )
            flashed.append(board)
            feedback(
                phase='flash',
                progress=min(1.0, row + 0.95 / total),
                detail=f'flash completed for {board}',
                board=board,
            )
        except Exception as exc:
            failed.append(board)
            log_error(f'Flash failed for {board}: {exc}')
            feedback(
                phase='flash',
                progress=min(1.0, row + 0.9 / total),
                detail=f'flash failed for {board}: {exc}',
                board=board,
            )

    feedback(
        phase='flash',
        progress=1.0,
        detail='flash phase completed',
        board='',
    )
    return failed, flashed


def _uptime_topic(boards_entry: dict) -> str:
    """Resolve absolute ROS 2 topic for ``std_msgs/msg/Int32`` uptime ticks."""
    raw = boards_entry.get('topic_uptime')
    if isinstance(raw, str):
        t = raw.strip()
        if t:
            return t if t.startswith('/') else f'/{t}'
    env = os.environ.get('LUCY_PIPELINE_UPTIME_TOPIC', '').strip()
    if env:
        return env if env.startswith('/') else f'/{env}'
    return '/uptime_publisher'


def _wait_uptime_message(node: Node, topic: str, timeout_sec: float) -> bool:
    """
    Wait for one ``std_msgs/msg/Int32`` on ``topic`` within ``timeout_sec``.

    Returns True if a message was received, False on timeout.
    """
    from rclpy.wait_for_message import wait_for_message
    from std_msgs.msg import Int32

    ok, _msg = wait_for_message(Int32, node, topic, time_to_wait=timeout_sec)
    return bool(ok)


MEGA_USB_ID = (0x2341, 0x0042)


def _flash_mega(
    board: str,
    serial: str,
    paths,
    timeout_seconds: int,
    feedback: Callable[..., None],
    progress: float,
) -> None:
    """Flash the board's ELF with avrdude onto the Mega whose USB serial is ``serial``."""
    elf = mega_artifact_dir(paths) / f'{board}.elf'
    if not elf.is_file():
        raise FileNotFoundError(f'missing ELF for flash: {elf}')
    port = mega_port(serial)
    feedback(phase='flash', progress=progress, detail=f'avrdude {elf.name} -> {port}', board=board)
    # Same programmer settings ravedude uses for mega2560 (stk500v2 bootloader at 115200).
    with mega_flash_lock(serial):
        _run_command(
            phase='flash',
            board=board,
            cmd=['avrdude', '-p', 'atmega2560', '-c', 'wiring', '-P', port, '-b', '115200',
                 '-D', '-U', f'flash:w:{elf}:e'],
            cwd=elf.parent,
            timeout_seconds=timeout_seconds,
            feedback=feedback,
            stream_progress=progress,
        )


@contextmanager
def mega_flash_lock(serial: str):
    """
    Keep this Mega's mcu_bridge off the port while avrdude owns it.

    The bridge closes the port once it sees the lock file and reconnects once it is gone,
    so it never has to be stopped. It can hold the port for 2 s after opening it (the Mega
    resets), so wait longer than that before avrdude opens it.
    """
    lock = Path(f'/tmp/.lucy_mcu_flashing_{serial}')
    lock.touch()
    try:
        time.sleep(3.0)
        yield
    finally:
        lock.unlink(missing_ok=True)


def mega_port(serial_id: str) -> str:
    """Return the serial device of the plugged Arduino Mega with this USB serial number."""
    from serial.tools import list_ports

    megas = [p for p in list_ports.comports() if (p.vid, p.pid) == MEGA_USB_ID]
    for p in megas:
        if p.serial_number == serial_id:
            return p.device
    found = ', '.join(f'{p.device}={p.serial_number}' for p in megas) or 'none'
    raise RuntimeError(
        f'no Arduino Mega with USB serial {serial_id} plugged in (found: {found})'
    )


def _wait_for_usb_serial(serial_id: str, timeout_seconds: int) -> bool:
    """Return whether a /dev/serial/by-id entry containing ``serial_id`` resolves."""
    needle = serial_id.strip().lower()
    if not needle:
        return False
    deadline = time.monotonic() + max(0.1, float(timeout_seconds))
    by_id = Path('/dev/serial/by-id')
    while time.monotonic() < deadline:
        if by_id.is_dir():
            for entry in by_id.iterdir():
                name = entry.name.lower()
                if needle not in name:
                    continue
                try:
                    resolved = entry.resolve()
                    if resolved.exists():
                        return True
                except OSError:
                    continue
        time.sleep(0.5)
    return False


def _run_command(
    *,
    phase: str,
    board: str,
    cmd: list[str],
    cwd: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float = 0.0,
) -> None:
    process = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    # The timeout has to arm before the read loop: that loop runs until the pipe
    # closes, so a child that hangs without exiting never reaches process.wait().
    # Killing closes the pipe, which ends the loop.
    watchdog = threading.Timer(timeout_seconds, process.kill)
    watchdog.start()
    emitted = 0
    try:
        for line in process.stdout:
            text = line.strip()
            if not text:
                continue
            if emitted < 200:
                feedback(phase=phase, progress=stream_progress, detail=text, board=board)
            emitted += 1
        return_code = process.wait()
    finally:
        timed_out = not watchdog.is_alive()
        watchdog.cancel()
    if timed_out:
        raise TimeoutError(f"command timed out after {timeout_seconds}s: {' '.join(cmd)}")
    if return_code != 0:
        raise RuntimeError(f"command failed ({return_code}): {' '.join(cmd)}")


def flash_picotool_timeout_seconds() -> int:
    return int(os.environ.get('LUCY_PIPELINE_FLASH_TIMEOUT_SEC', '120'))


def flash_usb_wait_seconds() -> int:
    return int(os.environ.get('LUCY_PIPELINE_FLASH_WAIT_SEC', '5'))


def flash_uptime_wait_seconds() -> int:
    return int(os.environ.get('LUCY_PIPELINE_FLASH_UPTIME_WAIT_SEC', '30'))
