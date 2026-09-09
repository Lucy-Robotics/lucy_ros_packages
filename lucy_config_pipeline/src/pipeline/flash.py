from __future__ import annotations

from collections.abc import Callable
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import TYPE_CHECKING

from .selection import board_build_plan
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

    After USB serial re-enumeration, optionally verifies the board answers a
    Modbus read (holding register 0) when ``uptime_wait_seconds > 0``.

    Returns ``(failed_board_ids, flashed_board_ids)`` in stable board order.
    """
    paths = resolve_firmware_paths(data, workspace_src)
    if not paths.build_dir.is_dir():
        raise FileNotFoundError(paths.build_dir)

    plan = board_build_plan(data, selected_boards)
    flashable = [(b, t) for b, t in plan if b in boards_built_ok]
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
            if uptime_wait_seconds > 0:
                feedback(
                    phase='flash',
                    progress=min(1.0, row + 0.75 / total),
                    detail=f'verifying Modbus on serial {serial} (up to {uptime_wait_seconds}s)',
                    board=board,
                )
                if not _wait_modbus_ready(serial, float(uptime_wait_seconds)):
                    raise TimeoutError(
                        f'Modbus verify failed within {uptime_wait_seconds}s '
                        f'(board {board}, serial {serial})'
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


def _modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')


def _wait_modbus_ready(serial_id: str, timeout_sec: float) -> bool:
    """Probe the board with Modbus FC03 read of register 0 after flash."""
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        # pyserial missing: fall back to USB presence only.
        return True

    needle = serial_id.strip().lower()
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    port_name = None
    while time.monotonic() < deadline and port_name is None:
        for info in list_ports.comports():
            hay = ' '.join(
                filter(
                    None,
                    [
                        info.device,
                        info.serial_number or '',
                        info.description or '',
                        info.hwid or '',
                    ],
                )
            ).lower()
            if needle and needle in hay:
                port_name = info.device
                break
        if port_name is None:
            time.sleep(0.5)
    if port_name is None:
        return False

    # FC03: slave 1, start 0, qty 1
    req = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
    req.extend(_modbus_crc(req))

    while time.monotonic() < deadline:
        try:
            with serial.Serial(port_name, 115200, timeout=0.5) as ser:
                ser.reset_input_buffer()
                ser.write(req)
                resp = ser.read(7)
                if len(resp) >= 5 and resp[0] == 0x01 and resp[1] == 0x03:
                    return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


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
