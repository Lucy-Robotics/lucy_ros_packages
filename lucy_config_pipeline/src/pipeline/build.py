from __future__ import annotations

from collections.abc import Callable
import os
from pathlib import Path
import shutil
import subprocess
import threading

from .selection import board_build_plan
from .selection import is_mega_board
from .selection import mega_artifact_dir
from .selection import mega_crate_dir
from .selection import resolve_firmware_paths

MEGA_ELF = Path('target/avr-none/release/lucy-embedded-firmware-arduino-mega.elf')


def run_build_phase(
    *,
    data: dict,
    selected_boards: set[str] | None,
    workspace_src: Path,
    timeout_seconds: int,
    feedback: Callable[..., None],
    log_error: Callable[[str], None],
) -> list[str]:
    paths = resolve_firmware_paths(data, workspace_src)
    if not paths.source_dir.exists():
        raise FileNotFoundError(paths.source_dir)

    plan = board_build_plan(data, selected_boards)
    boards_map = data.get('boards', {})
    total_steps = len(plan) + 1
    step_idx = 0

    cmake_cache = paths.build_dir / 'CMakeCache.txt'
    needs_cmake = any(not is_mega_board(boards_map.get(b, {})) for b, _ in plan)
    if needs_cmake:
        paths.build_dir.mkdir(parents=True, exist_ok=True)
    if needs_cmake and not cmake_cache.exists():
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
            if is_mega_board(boards_map.get(board, {})):
                _build_mega(board, paths, timeout_seconds, feedback, step_idx / total_steps)
                continue
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

    feedback(
        phase='build',
        progress=1.0,
        detail='build phase completed',
        board='',
    )
    return failed_boards


def _build_mega(
    board: str,
    paths,
    timeout_seconds: int,
    feedback: Callable[..., None],
    progress: float,
) -> None:
    """Cargo-build firmwares/arduino_mega with the board's generated tables; keep its ELF."""
    artifacts = mega_artifact_dir(paths)
    config = artifacts / f'config_{board}.rs'
    if not config.is_file():
        raise RuntimeError(f'missing generated board config: {config}')
    env = _cargo_env()
    env['LUCY_MEGA_BOARD_CONFIG'] = str(config)
    crate = mega_crate_dir(paths)
    # cwd matters: the crate's rust-toolchain.toml and .cargo/config.toml resolve from it.
    _run_command(
        phase='build',
        board=board,
        cmd=['cargo', 'build', '--release'],
        cwd=crate,
        timeout_seconds=timeout_seconds,
        feedback=feedback,
        stream_progress=progress,
        env=env,
    )
    shutil.copy2(crate / MEGA_ELF, artifacts / f'{board}.elf')


def _cargo_env() -> dict[str, str]:
    """Prefer the Pixi env's rustup/cargo (`pixi run firmware-setup`) over the host's."""
    env = dict(os.environ)
    prefix = env.get('CONDA_PREFIX', '')
    if prefix and (Path(prefix) / 'cargo' / 'bin').is_dir():
        env['CARGO_HOME'] = str(Path(prefix) / 'cargo')
        env['RUSTUP_HOME'] = str(Path(prefix) / 'rustup')
        env['PATH'] = f"{Path(prefix) / 'cargo' / 'bin'}{os.pathsep}{env.get('PATH', '')}"
    return env


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
        env=env,
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
