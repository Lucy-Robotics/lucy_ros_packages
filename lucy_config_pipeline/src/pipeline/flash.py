from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
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
    Flash built UF2 images without sudo.

    Prefer copying the UF2 onto an RPI-RP2 BOOTSEL volume (user-accessible after
    ``pixi run firmware-setup`` udev rules). Fall back to unprivileged
    ``picotool load -f`` when the board is already running application firmware.

    Only boards present in ``boards_built_ok`` are considered. Boards without a
    non-empty ``serial_id`` are skipped (not counted as failure).

    After USB serial re-enumeration, optionally waits for ``std_msgs/Int32`` on
    the board uptime topic (see ``_uptime_topic``) when ``node`` is set.

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

        uf2 = paths.build_dir / f'{board}.uf2'
        if not uf2.is_file():
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
                detail=f'flash {uf2.name}',
                board=board,
            )
            load_progress = min(1.0, row + 0.25 / total)
            bridge_node = _board_bridge_node_name(board)
            # Do NOT kill mcu_bridge here: during BOOTSEL a respawn crash-loops.
            # The bridge reopens CDC itself after the board leaves BOOTSEL.
            if bridge_node:
                try:
                    _bridge_link_path(bridge_node).unlink(missing_ok=True)
                except OSError:
                    pass
            flash_started = time.time()
            _flash_uf2(
                uf2=uf2,
                configured_serial=serial,
                board=board,
                cwd=paths.build_dir,
                timeout_seconds=picotool_timeout_seconds,
                feedback=feedback,
                stream_progress=load_progress,
            )
            # UF2 copy / picotool load already reboots into application mode.
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
            # USB-CDC RP2040 images do not publish micro-ROS uptime topics.
            cargo_firmware = (
                paths.source_dir / 'firmwares' / 'rp2040' / 'Cargo.toml'
            ).is_file()
            if (
                not cargo_firmware
                and node is not None
                and uptime_wait_seconds > 0
            ):
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
            # Launch respawns the bridge after we stopped it; require a live framed
            # link — USB by-id alone is not enough.
            link_wait = flash_bridge_link_wait_seconds()
            if bridge_node and link_wait > 0:
                feedback(
                    phase='flash',
                    progress=min(1.0, row + 0.85 / total),
                    detail=(
                        f'waiting up to {link_wait}s for mcu_bridge link '
                        f'({bridge_node})'
                    ),
                    board=board,
                )
                # Respawn may lag a second behind CDC re-enum.
                respawn_deadline = time.monotonic() + min(5.0, link_wait)
                while (
                    time.monotonic() < respawn_deadline
                    and not _mcu_bridge_process_running(bridge_node)
                ):
                    time.sleep(0.2)
                if _mcu_bridge_process_running(bridge_node):
                    if not _wait_for_bridge_link(
                        bridge_node, since=flash_started, timeout_seconds=link_wait
                    ):
                        raise TimeoutError(
                            f'mcu_bridge for {bridge_node} did not restore a live '
                            f'serial link within {link_wait}s after flash '
                            '(CRC/resync or stale CDC). Check board USB and bridge logs.'
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


def _board_bridge_node_name(board_id: str) -> str:
    """ROS / SHM node name used by ``mcu_bridge_<node>`` for ``board_id``."""
    try:
        from lucy_config_generator.schema import derive_ros2_node_name

        return str(derive_ros2_node_name(str(board_id))).strip()
    except Exception:
        return str(board_id).strip()


def _bridge_link_path(node_name: str) -> Path:
    return Path(f'/tmp/.lucy_mcu_bridge_link.{node_name}')


def _mcu_bridge_process_running(node_name: str) -> bool:
    """True when a host mcu_bridge for ``node_name`` is in the process table."""
    import subprocess

    needle = f'lucy-embedded-firmware-mcu-bridge {node_name}'
    try:
        result = subprocess.run(
            ['pgrep', '-f', needle],
            capture_output=True,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0


def _signal_mcu_bridge(node_name: str) -> None:
    """SIGTERM one mcu_bridge so launch respawn reopens CDC after flash."""
    import subprocess

    if not node_name:
        return
    try:
        subprocess.run(
            ['pkill', '-f', f'lucy-embedded-firmware-mcu-bridge {node_name}'],
            capture_output=True,
            check=False,
        )
    except OSError:
        pass
    try:
        _bridge_link_path(node_name).unlink(missing_ok=True)
    except OSError:
        pass


def _wait_for_bridge_link(
    node_name: str, *, since: float, timeout_seconds: float
) -> bool:
    """Wait until mcu_bridge records a successful framed I/O after ``since``."""
    path = _bridge_link_path(node_name)
    deadline = time.monotonic() + max(0.1, float(timeout_seconds))
    while time.monotonic() < deadline:
        try:
            if path.is_file():
                # Prefer stamped content; fall back to mtime.
                raw = path.read_text(encoding='utf-8').strip().split()
                if raw:
                    try:
                        if float(raw[0]) >= since - 2.0:
                            return True
                    except ValueError:
                        pass
                if path.stat().st_mtime >= since - 2.0:
                    return True
        except OSError:
            pass
        time.sleep(0.2)
    return False


def flash_bridge_link_wait_seconds() -> float:
    return float(os.environ.get('LUCY_PIPELINE_FLASH_BRIDGE_LINK_WAIT_SEC', '20'))


def _list_bootsel_serials() -> list[str]:
    """USB serial strings for devices whose product is RP2 Boot (BOOTSEL).

    Application firmware also uses VID ``2e8a`` / PID ``0003``, so product
    string is the reliable discriminator. BOOTSEL serial ≠ flash unique ID
    baked into ``serial_id``, so ``picotool --ser <serial_id>`` misses a board
    that is already sitting in the bootrom.
    """
    root = Path('/sys/bus/usb/devices')
    if not root.is_dir():
        return []
    found: list[str] = []
    for entry in root.iterdir():
        vid_path = entry / 'idVendor'
        if not vid_path.is_file():
            continue
        try:
            vid = vid_path.read_text(encoding='utf-8').strip().lower()
            pid = (entry / 'idProduct').read_text(encoding='utf-8').strip().lower()
            product = (entry / 'product').read_text(encoding='utf-8').strip().lower()
            serial = (entry / 'serial').read_text(encoding='utf-8').strip()
        except OSError:
            continue
        if vid != '2e8a' or pid != '0003':
            continue
        if 'rp2 boot' not in product and 'bootsel' not in product:
            continue
        if serial:
            found.append(serial)
    return found


def _is_bootsel_volume(path: Path) -> bool:
    """True when ``path`` is a writable RP2 BOOTSEL mass-storage mount."""
    name = path.name.upper()
    if name not in {'RPI-RP2', 'RP2350'} and 'RPI-RP2' not in name:
        return False
    try:
        if not path.is_dir():
            return False
        if not ((path / 'INFO_UF2.TXT').is_file() or (path / 'INFO_UF2.txt').is_file()):
            return False
        return os.access(path, os.W_OK)
    except OSError:
        return False


def _find_rpi_rp2_mounts() -> list[Path]:
    """Writable RPI-RP2 / RP2350 BOOTSEL volumes (never probe unrelated mounts)."""
    candidates: list[Path] = []
    seen: set[Path] = set()
    mount_points: list[Path] = []
    try:
        with open('/proc/mounts', encoding='utf-8') as fh:
            for line in fh:
                parts = line.split()
                if len(parts) < 2:
                    continue
                mount_points.append(Path(parts[1]))
    except OSError:
        mount_points = []
    for root in (
        Path('/run/media') / os.environ.get('USER', ''),
        Path('/media') / os.environ.get('USER', ''),
    ):
        if root.is_dir():
            try:
                mount_points.extend(p for p in root.iterdir() if p.is_dir())
            except OSError:
                pass
    for path in mount_points:
        if not _is_bootsel_volume(path):
            continue
        try:
            resolved = path.resolve()
        except OSError:
            continue
        if resolved in seen:
            continue
        seen.add(resolved)
        candidates.append(resolved)
    return candidates


def _find_rpi_rp2_block_devices() -> list[Path]:
    """Block devices labelled ``RPI-RP2`` / ``RP2350`` (may be unmounted)."""
    by_label = Path('/dev/disk/by-label')
    found: list[Path] = []
    if not by_label.is_dir():
        return found
    for name in ('RPI-RP2', 'RP2350'):
        link = by_label / name
        if not link.exists():
            continue
        try:
            found.append(link.resolve())
        except OSError:
            continue
    return found


def _udisksctl_mount(device: Path) -> Path | None:
    """Mount ``device`` via udisksctl (no sudo). Return the mount point."""
    import shutil

    if shutil.which('udisksctl') is None:
        return None
    result = subprocess.run(
        ['udisksctl', 'mount', '-b', str(device)],
        capture_output=True,
        text=True,
        check=False,
    )
    # "Mounted /dev/sdc1 at /run/media/USER/RPI-RP2" or already-mounted text.
    combined = f'{result.stdout}\n{result.stderr}'
    for token in combined.replace('`', ' ').split():
        if 'RPI-RP2' in token or 'RP2350' in token or token.startswith('/run/media'):
            path = Path(token.rstrip('.,;'))
            if _is_bootsel_volume(path):
                return path
    mounts = _find_rpi_rp2_mounts()
    return mounts[0] if mounts else None


def _ensure_bootsel_mounts(
    *,
    feedback: Callable[..., None],
    stream_progress: float,
    board: str,
) -> list[Path]:
    """Return writable BOOTSEL mounts, auto-mounting via udisksctl if needed."""
    mounts = _find_rpi_rp2_mounts()
    if mounts:
        return mounts
    devices = _find_rpi_rp2_block_devices()
    if not devices:
        return []
    feedback(
        phase='flash',
        progress=stream_progress,
        detail=f'mounting BOOTSEL volume {devices[0]} via udisksctl (no sudo)',
        board=board,
    )
    mounted = _udisksctl_mount(devices[0])
    if mounted is not None:
        return [mounted]
    return _find_rpi_rp2_mounts()


def _copy_uf2_to_bootsel_volume(
    *,
    uf2: Path,
    mount: Path,
    feedback: Callable[..., None],
    stream_progress: float,
    board: str,
) -> None:
    dest = mount / uf2.name
    feedback(
        phase='flash',
        progress=stream_progress,
        detail=f'copy {uf2.name} → {mount} (BOOTSEL, no sudo)',
        board=board,
    )
    # Binary copy; the volume remounts / disappears once the UF2 is accepted.
    data = uf2.read_bytes()
    try:
        dest.write_bytes(data)
    except OSError as exc:
        # EIO / disappearing mount after the write is common and means success.
        if getattr(exc, 'errno', None) not in {5, 2}:  # EIO, ENOENT
            raise RuntimeError(
                f'failed to copy {uf2} onto BOOTSEL volume {mount}: {exc}'
            ) from exc


def _picotool_bin() -> str:
    """Resolve picotool on PATH / under ``$CONDA_PREFIX/bin``."""
    import shutil

    found = shutil.which('picotool')
    if found:
        return found
    prefix = (os.environ.get('CONDA_PREFIX') or '').strip()
    if prefix:
        candidate = Path(prefix) / 'bin' / 'picotool'
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError(
        'picotool not found on PATH — launch Lucy from the Pixi env '
        '(pixi run …) after `pixi run firmware-setup`'
    )


def _picotool_env() -> dict[str, str]:
    """Env for picotool: Pixi ``lib/`` must be on ``LD_LIBRARY_PATH`` (libusb)."""
    env = dict(os.environ)
    prefix = (env.get('CONDA_PREFIX') or '').strip()
    if not prefix:
        return env
    lib = str(Path(prefix) / 'lib')
    bin_dir = str(Path(prefix) / 'bin')
    path = env.get('PATH', '')
    if path != bin_dir and not path.startswith(bin_dir + os.pathsep):
        env['PATH'] = f'{bin_dir}{os.pathsep}{path}' if path else bin_dir
    ld = env.get('LD_LIBRARY_PATH', '')
    if ld != lib and not ld.startswith(lib + os.pathsep):
        env['LD_LIBRARY_PATH'] = f'{lib}{os.pathsep}{ld}' if ld else lib
    return env


# Written to the board CDC port when raw USB (picotool) is not writable.
# Must match firmwares/rp2040 ``BOOTSEL_MAGIC``.
_BOOTSEL_CDC_MAGIC = b'LUCYBOOTSEL\n'


def _usb_udev_hint() -> str:
    return (
        'USB device node is not writable for picotool (needs MODE 0666 / uaccess). '
        'Run once: `pixi run setup-udev` (or `pixi run firmware-setup`), '
        'unplug/replug the board, then retry — no sudo at flash time. '
        'Or hold BOOTSEL while plugging once; newer firmware accepts CDC '
        '`LUCYBOOTSEL\\n` when the tty is writable (dialout / MODE 0666).'
    )


def _try_make_node_writable(node: Path) -> bool:
    """Best-effort chmod so flash can proceed without a prompt (sudo -n only)."""
    try:
        if os.access(node, os.W_OK):
            return True
    except OSError:
        return False
    try:
        os.chmod(node, 0o666)
        return os.access(node, os.W_OK)
    except OSError:
        pass
    try:
        result = subprocess.run(
            ['sudo', '-n', 'chmod', '0666', str(node)],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return False
    if result.returncode != 0:
        return False
    try:
        return os.access(node, os.W_OK)
    except OSError:
        return False


@dataclass(frozen=True)
class _UsbLoc:
    bus: int
    address: int
    serial: str
    product: str

    @property
    def node(self) -> Path:
        return Path(f'/dev/bus/usb/{self.bus:03d}/{self.address:03d}')

    @property
    def writable(self) -> bool:
        try:
            return os.access(self.node, os.W_OK)
        except OSError:
            return False


def _find_usb_loc_by_serial(serial: str) -> _UsbLoc | None:
    """Locate a 2e8a USB device whose serial matches ``serial`` (app firmware)."""
    want = serial.strip()
    if not want:
        return None
    root = Path('/sys/bus/usb/devices')
    if not root.is_dir():
        return None
    for entry in root.iterdir():
        vid_path = entry / 'idVendor'
        if not vid_path.is_file():
            continue
        try:
            vid = vid_path.read_text(encoding='utf-8').strip().lower()
            if vid != '2e8a':
                continue
            ser = (entry / 'serial').read_text(encoding='utf-8').strip()
            if ser != want:
                continue
            bus = int((entry / 'busnum').read_text(encoding='utf-8').strip())
            addr = int((entry / 'devnum').read_text(encoding='utf-8').strip())
            product = (entry / 'product').read_text(encoding='utf-8').strip()
        except (OSError, ValueError):
            continue
        return _UsbLoc(bus=bus, address=addr, serial=ser, product=product)
    return None


def _flash_uf2(
    *,
    uf2: Path,
    configured_serial: str,
    board: str,
    cwd: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float,
) -> None:
    """Flash ``uf2`` without sudo: BOOTSEL volume copy, else picotool force-reboot."""
    bootsel_serials = _list_bootsel_serials()
    mounts = _ensure_bootsel_mounts(
        feedback=feedback,
        stream_progress=stream_progress,
        board=board,
    )

    if mounts:
        if len(mounts) > 1 and len(bootsel_serials) > 1:
            raise RuntimeError(
                f'multiple BOOTSEL volumes ({", ".join(str(m) for m in mounts)}); '
                'unplug other boards and retry flash'
            )
        _copy_uf2_to_bootsel_volume(
            uf2=uf2,
            mount=mounts[0],
            feedback=feedback,
            stream_progress=stream_progress,
            board=board,
        )
        return

    if bootsel_serials and not mounts:
        # Already in BOOTSEL but volume not mounted — try udisks again, else picotool.
        devices = _find_rpi_rp2_block_devices()
        if devices:
            feedback(
                phase='flash',
                progress=stream_progress,
                detail=f'BOOTSEL disk {devices[0]} present; mounting via udisksctl',
                board=board,
            )
            mounted = _udisksctl_mount(devices[0])
            if mounted is not None:
                _copy_uf2_to_bootsel_volume(
                    uf2=uf2,
                    mount=mounted,
                    feedback=feedback,
                    stream_progress=stream_progress,
                    board=board,
                )
                return
        boot_ser = bootsel_serials[0] if len(bootsel_serials) == 1 else None
        if boot_ser is None:
            raise RuntimeError(
                f'multiple BOOTSEL boards ({", ".join(bootsel_serials)}) and no '
                'writable RPI-RP2 mount — unplug extras, or open the RPI-RP2 volume '
                'in the file manager, then retry'
            )
        loc = _find_usb_loc_by_serial(boot_ser)
        _run_picotool_load_bootsel(
            uf2=uf2,
            loc=loc,
            serial=boot_ser,
            board=board,
            cwd=cwd,
            timeout_seconds=timeout_seconds,
            feedback=feedback,
            stream_progress=stream_progress,
        )
        return

    # Application firmware running: force into BOOTSEL, then UF2 volume copy.
    # Do NOT keep --ser <app serial> across the reboot — BOOTSEL enumerates with a
    # different USB serial, which is why plain `picotool load -f --ser <app>` fails.
    _force_reflash_running_board(
        uf2=uf2,
        configured_serial=configured_serial,
        board=board,
        cwd=cwd,
        timeout_seconds=timeout_seconds,
        feedback=feedback,
        stream_progress=stream_progress,
    )


def _find_cdc_tty_for_serial(serial_id: str) -> Path | None:
    """Resolve the CDC tty path for a board USB serial string."""
    needle = serial_id.strip().lower()
    if not needle:
        return None
    by_id = Path('/dev/serial/by-id')
    if by_id.is_dir():
        for entry in sorted(by_id.iterdir()):
            if needle not in entry.name.lower():
                continue
            try:
                resolved = entry.resolve()
            except OSError:
                continue
            if resolved.exists():
                return resolved
    return None


def _serial_request_bootsel(
    *,
    configured_serial: str,
    board: str,
    feedback: Callable[..., None],
    stream_progress: float,
) -> None:
    """Ask running firmware to enter BOOTSEL by writing CDC magic (no picotool)."""
    tty = _find_cdc_tty_for_serial(configured_serial)
    if tty is None:
        raise RuntimeError(
            f'no CDC tty for serial_id={configured_serial} '
            '(board must be running application firmware with USB serial)'
        )
    if not _try_make_node_writable(tty):
        raise RuntimeError(
            f'CDC {tty} is not writable — add your user to dialout, or chmod the tty'
        )
    feedback(
        phase='flash',
        progress=stream_progress,
        detail=(
            f'CDC force BOOTSEL via {tty} (write LUCYBOOTSEL; no picotool USB access)'
        ),
        board=board,
    )
    try:
        # Exclusive open can fail if mcu_bridge holds the port; use non-blocking RDWR.
        fd = os.open(str(tty), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as exc:
        raise RuntimeError(
            f'cannot open CDC {tty} for BOOTSEL request: {exc} '
            '(stop anything holding the serial port, then retry)'
        ) from exc
    try:
        os.write(fd, _BOOTSEL_CDC_MAGIC)
    except OSError as exc:
        raise RuntimeError(f'failed writing BOOTSEL magic to {tty}: {exc}') from exc
    finally:
        os.close(fd)


def _wait_bootsel_volume_and_copy(
    *,
    uf2: Path,
    board: str,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float,
) -> None:
    deadline = time.monotonic() + max(5.0, float(timeout_seconds))
    while time.monotonic() < deadline:
        mounts = _ensure_bootsel_mounts(
            feedback=feedback,
            stream_progress=stream_progress,
            board=board,
        )
        if mounts:
            _copy_uf2_to_bootsel_volume(
                uf2=uf2,
                mount=mounts[0],
                feedback=feedback,
                stream_progress=stream_progress,
                board=board,
            )
            return
        time.sleep(0.4)

    raise RuntimeError(
        f'forced {board} into BOOTSEL but no writable RPI-RP2 volume appeared within '
        f'{timeout_seconds}s — open the volume in the file manager and retry, or check '
        'udisks automount.'
    )


def _force_reflash_running_board(
    *,
    uf2: Path,
    configured_serial: str,
    board: str,
    cwd: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float,
) -> None:
    """Force BOOTSEL (picotool USB or CDC magic), then copy UF2 onto RPI-RP2."""
    loc = _find_usb_loc_by_serial(configured_serial)
    if loc is None:
        raise RuntimeError(
            f'no USB device with serial_id={configured_serial} — is the board plugged '
            'in and running Lucy firmware (or in BOOTSEL)?'
        )

    # Session chmod may unlock picotool after replug lost udev MODE bits.
    usb_ok = loc.writable or _try_make_node_writable(loc.node)
    if usb_ok:
        picotool = _picotool_bin()
        feedback(
            phase='flash',
            progress=stream_progress,
            detail=(
                f'picotool reboot --bus {loc.bus} --address {loc.address} -f -u '
                f'(force BOOTSEL for {configured_serial}, no sudo)'
            ),
            board=board,
        )
        try:
            # picotool v2.x: device-selection (--bus/--address/--ser) MUST come
            # before -f/-u or the CLI rejects them as "unexpected option".
            _run_command(
                phase='flash',
                board=board,
                cmd=[
                    picotool, 'reboot',
                    '--bus', str(loc.bus),
                    '--address', str(loc.address),
                    '-f', '-u',
                ],
                cwd=cwd,
                timeout_seconds=min(30, timeout_seconds),
                feedback=feedback,
                stream_progress=stream_progress,
                env=_picotool_env(),
            )
        except Exception as exc:
            hint = ''
            err = str(exc).lower()
            if 'permission' in err or 'accessible' in err:
                hint = ' ' + _usb_udev_hint()
            raise RuntimeError(
                f'picotool force-reboot failed for {board} ({configured_serial}): '
                f'{exc}.{hint}'
            ) from exc
    else:
        # Raw USB not writable (missing udev / permissions): try CDC magic.
        try:
            _serial_request_bootsel(
                configured_serial=configured_serial,
                board=board,
                feedback=feedback,
                stream_progress=stream_progress,
            )
        except Exception as cdc_exc:
            raise RuntimeError(
                f'cannot force-flash {board} at {loc.node} ({loc.product}): '
                f'{_usb_udev_hint()} '
                f'(CDC fallback also failed: {cdc_exc})'
            ) from cdc_exc

    _wait_bootsel_volume_and_copy(
        uf2=uf2,
        board=board,
        timeout_seconds=timeout_seconds,
        feedback=feedback,
        stream_progress=stream_progress,
    )


def _run_picotool_load_bootsel(
    *,
    uf2: Path,
    loc: _UsbLoc | None,
    serial: str,
    board: str,
    cwd: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float,
) -> None:
    """Load UF2 onto a board already in BOOTSEL (prefer bus/address, else serial)."""
    picotool = _picotool_bin()
    if loc is not None:
        if not loc.writable:
            raise RuntimeError(
                f'BOOTSEL device {loc.node} not writable: ' + _usb_udev_hint()
            )
        # Filename before device-selection is fine; keep selection before any -f.
        cmd = [
            picotool, 'load', str(uf2),
            '--bus', str(loc.bus),
            '--address', str(loc.address),
        ]
        detail = f'picotool load --bus {loc.bus} --address {loc.address} (BOOTSEL)'
    else:
        cmd = [picotool, 'load', str(uf2), '--ser', serial]
        detail = f'picotool load --ser {serial} (BOOTSEL)'
    feedback(phase='flash', progress=stream_progress, detail=detail, board=board)
    _run_command(
        phase='flash',
        board=board,
        cmd=cmd,
        cwd=cwd,
        timeout_seconds=timeout_seconds,
        feedback=feedback,
        stream_progress=stream_progress,
        env=_picotool_env(),
    )


def _run_command(
    *,
    phase: str,
    board: str,
    cmd: list[str],
    cwd: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    stream_progress: float = 0.0,
    env: dict[str, str] | None = None,
) -> None:
    process = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
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
    # UF2 reboot + CDC re-enum is often >5s on USB hubs.
    return int(os.environ.get('LUCY_PIPELINE_FLASH_WAIT_SEC', '15'))


def flash_uptime_wait_seconds() -> int:
    return int(os.environ.get('LUCY_PIPELINE_FLASH_UPTIME_WAIT_SEC', '30'))
