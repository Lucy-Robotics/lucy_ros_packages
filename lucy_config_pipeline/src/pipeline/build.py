from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import subprocess
import threading

from .selection import board_build_plan
from .selection import resolve_firmware_paths
from .toolchain import FirmwareToolchainError
from .toolchain import firmware_tool_env
from .toolchain import resolve_firmware_tool


def run_build_phase(
    *,
    data: dict,
    selected_boards: set[str] | None,
    workspace_src: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    log_error: Callable[[str], None],
) -> tuple[list[str], set[str]]:
    """Return ``(failed_board_ids, built_board_ids)``."""
    paths = resolve_firmware_paths(data, workspace_src)
    if not paths.source_dir.exists():
        raise FileNotFoundError(paths.source_dir)

    if (paths.source_dir / 'firmwares' / 'rp2040' / 'Cargo.toml').is_file():
        return _run_cargo_builds(
            data=data,
            selected_boards=selected_boards,
            source_dir=paths.source_dir,
            build_dir=paths.build_dir,
            timeout_seconds=timeout_seconds,
            feedback=feedback,
            log_error=log_error,
        )

    paths.build_dir.mkdir(parents=True, exist_ok=True)
    plan = board_build_plan(data, selected_boards)
    total_steps = len(plan) + 1
    step_idx = 0

    cmake_cache = paths.build_dir / 'CMakeCache.txt'
    if not cmake_cache.exists():
        feedback(
            phase='build',
            progress=step_idx / total_steps,
            detail=f'running cmake in {paths.build_dir}',
            board='',
        )
        _run_command(
            phase='build',
            board='',
            cmd=['cmake', '..'],
            cwd=paths.build_dir,
            timeout_seconds=timeout_seconds,
            feedback=feedback,
            stream_progress=step_idx / total_steps,
        )
    step_idx += 1

    failed_boards: list[str] = []
    for board, target in plan:
        feedback(
            phase='build',
            progress=step_idx / total_steps,
            detail=f'building target {target}',
            board=board,
        )
        try:
            _run_command(
                phase='build',
                board=board,
                cmd=['make', target],
                cwd=paths.build_dir,
                timeout_seconds=timeout_seconds,
                feedback=feedback,
                stream_progress=step_idx / total_steps,
            )
            uf2 = paths.build_dir / f'{target}.uf2'
            if not uf2.exists():
                raise RuntimeError(f'missing build artifact: {uf2}')
        except Exception as exc:
            failed_boards.append(board)
            log_error(f'Build failed for {board}: {exc}')
            feedback(
                phase='build',
                progress=step_idx / total_steps,
                detail=f'build failed for {board}: {exc}',
                board=board,
            )
        finally:
            step_idx += 1

    built_ok = {board for board, _target in plan if board not in failed_boards}
    feedback(
        phase='build',
        progress=1.0,
        detail='build phase completed',
        board='',
    )
    return failed_boards, built_ok


def _run_cargo_builds(
    *,
    data: dict,
    selected_boards: set[str] | None,
    source_dir: Path,
    build_dir: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    log_error: Callable[[str], None],
) -> tuple[list[str], set[str]]:
    """One cargo release build and ``elf2uf2-rs`` per internal-servo board."""
    from lucy_config_generator.firmware_spec import render_firmware_spec

    build_dir.mkdir(parents=True, exist_ok=True)
    plan = board_build_plan(data, selected_boards)
    failed: list[str] = []
    built: set[str] = set()
    total = max(len(plan), 1)
    elf = (
        source_dir
        / 'target'
        / 'thumbv6m-none-eabi'
        / 'release'
        / 'lucy-embedded-firmware-rp2040'
    )
    try:
        tool_env = firmware_tool_env()
        cargo = resolve_firmware_tool('cargo', env=tool_env)
        elf2uf2 = resolve_firmware_tool('elf2uf2-rs', env=tool_env)
    except FirmwareToolchainError as exc:
        log_error(str(exc))
        feedback(phase='build', progress=0.0, detail=str(exc), board='')
        raise

    for index, (board, _target) in enumerate(plan):
        board_def = (data.get('boards') or {}).get(board) or {}
        if str(board_def.get('board_class') or '') == 'bus_servo_only':
            feedback(
                phase='build',
                progress=(index + 1) / total,
                detail=f'skip cargo build for bus_servo board {board}',
                board=board,
            )
            continue
        spec_path = build_dir / f'config_{board}.yaml'
        uf2 = build_dir / f'{board}.uf2'
        try:
            spec_path.write_text(render_firmware_spec(data, board), encoding='utf-8')
            env = dict(tool_env)
            env['LUCY_FIRMWARE_CONFIG'] = str(spec_path)
            serial = str((data.get('boards') or {}).get(board, {}).get('serial_id') or '').strip()
            if serial:
                env['DEVICE_SERIAL_NUMBER'] = serial
            feedback(
                phase='build',
                progress=index / total,
                detail=f'cargo build {board}',
                board=board,
            )
            _run_command(
                phase='build',
                board=board,
                cmd=[
                    str(cargo), 'build', '--release', '--target', 'thumbv6m-none-eabi',
                    '-p', 'lucy-embedded-firmware-rp2040',
                ],
                cwd=source_dir,
                timeout_seconds=timeout_seconds,
                feedback=feedback,
                stream_progress=index / total,
                env=env,
            )
            _run_command(
                phase='build',
                board=board,
                cmd=[str(elf2uf2), str(elf), str(uf2)],
                cwd=source_dir,
                timeout_seconds=timeout_seconds,
                feedback=feedback,
                stream_progress=(index + 0.5) / total,
                env=env,
            )
            if not uf2.is_file():
                raise RuntimeError(f'missing build artifact: {uf2}')
            built.add(board)
        except Exception as exc:
            failed.append(board)
            log_error(f'Build failed for {board}: {exc}')
            feedback(
                phase='build',
                progress=(index + 1) / total,
                detail=f'build failed for {board}: {exc}',
                board=board,
            )
    feedback(phase='build', progress=1.0, detail='build phase completed', board='')
    return failed, built


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
