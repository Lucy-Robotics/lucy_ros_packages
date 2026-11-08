from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import subprocess
import threading

from .selection import board_build_plan
from .selection import resolve_firmware_paths


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

    feedback(
        phase='build',
        progress=1.0,
        detail='build phase completed',
        board='',
    )
    return failed_boards


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
                feedback(
                    phase=phase, progress=stream_progress, detail=text, board=board
                )
            emitted += 1
        return_code = process.wait()
    finally:
        timed_out = not watchdog.is_alive()
        watchdog.cancel()
    if timed_out:
        raise TimeoutError(
            f"command timed out after {timeout_seconds}s: {' '.join(cmd)}"
        )
    if return_code != 0:
        raise RuntimeError(f"command failed ({return_code}): {' '.join(cmd)}")
