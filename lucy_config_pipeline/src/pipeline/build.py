from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import shutil
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
    """Build RP2040 firmware with Cargo and produce per-board UF2 artifacts."""
    paths = resolve_firmware_paths(data, workspace_src)
    if not paths.source_dir.exists():
        raise FileNotFoundError(paths.source_dir)

    paths.build_dir.mkdir(parents=True, exist_ok=True)
    plan = board_build_plan(data, selected_boards)
    total_steps = max(len(plan), 1)
    step_idx = 0

    cargo = shutil.which('cargo')
    if cargo is None:
        raise RuntimeError(
            "cargo not found; run `pixi run firmware-setup` to install the Rust toolchain"
        )

    manifest = paths.source_dir / 'firmwares' / 'rp2040' / 'Cargo.toml'
    if not manifest.is_file():
        raise FileNotFoundError(manifest)

    failed_boards: list[str] = []
    for board, target in plan:
        feedback(
            phase='build',
            progress=step_idx / total_steps,
            detail=f'building cargo target for {board} ({target})',
            board=board,
        )
        try:
            # Install board config.yaml next to the RP2040 crate for build.rs.
            board_cfg = paths.source_dir / 'config' / f'config_{board}.yaml'
            crate_cfg = paths.source_dir / 'firmwares' / 'rp2040' / 'config.yaml'
            if board_cfg.is_file():
                shutil.copy2(board_cfg, crate_cfg)

            _run_command(
                phase='build',
                board=board,
                cmd=[
                    cargo,
                    'build',
                    '--release',
                    '--target',
                    'thumbv6m-none-eabi',
                    '--manifest-path',
                    str(manifest),
                ],
                cwd=paths.source_dir,
                timeout_seconds=timeout_seconds,
                feedback=feedback,
                stream_progress=step_idx / total_steps,
            )

            elf = (
                paths.source_dir
                / 'target'
                / 'thumbv6m-none-eabi'
                / 'release'
                / 'lucy_embedded_firmware_rp2040'
            )
            # Binary name may match package name without extension.
            if not elf.exists():
                # Fallback: first non-.d file in release dir matching package.
                release_dir = paths.source_dir / 'target' / 'thumbv6m-none-eabi' / 'release'
                candidates = [
                    p
                    for p in release_dir.glob('*')
                    if p.is_file() and p.suffix == '' and not p.name.startswith('.')
                ]
                if not candidates:
                    raise RuntimeError(f'missing ELF under {release_dir}')
                elf = candidates[0]

            uf2 = paths.build_dir / f'{target}.uf2'
            _elf_to_uf2(elf, uf2, timeout_seconds=timeout_seconds)
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


def _elf_to_uf2(elf: Path, uf2: Path, *, timeout_seconds: int) -> None:
    converter = shutil.which('elf2uf2-rs')
    if converter is None:
        raise RuntimeError(
            "elf2uf2-rs not found; run `pixi run firmware-setup` to install it"
        )
    uf2.parent.mkdir(parents=True, exist_ok=True)
    # elf2uf2-rs writes alongside ELF by default; copy/rename to expected path.
    cmd = [converter, str(elf)]
    process = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_seconds,
        check=False,
    )
    if process.returncode != 0:
        raise RuntimeError(
            f'elf2uf2-rs failed ({process.returncode}): {process.stdout} {process.stderr}'
        )
    produced = elf.with_suffix('.uf2')
    if not produced.exists():
        raise RuntimeError(f'elf2uf2-rs did not produce {produced}')
    if produced.resolve() != uf2.resolve():
        shutil.copy2(produced, uf2)


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
