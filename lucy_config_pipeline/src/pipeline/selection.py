from __future__ import annotations

from pathlib import Path

from .models import FirmwarePaths
from ..config_store import ConfigStore


def resolve_mapping_input(store: ConfigStore, mapping_file: str) -> tuple[str, str]:
    if mapping_file:
        mapping_path = Path(mapping_file)
        if mapping_file.endswith('.yaml') and mapping_path.is_file():
            return mapping_path.stem, mapping_path.read_text(encoding='utf-8')
        return mapping_file, store.read_named_yaml(mapping_file)
    return store.get_active_name(), store.read_active_yaml()


def select_boards_to_process(data: dict, requested: list[str]) -> set[str] | None:
    if not requested:
        return None
    boards = {b for b in requested if b}
    known = set(data.get('boards', {}).keys())
    unknown = sorted(boards - known)
    if unknown:
        raise ValueError(f"unknown boards requested: {', '.join(unknown)}")
    return boards


def resolve_firmware_paths(data: dict, workspace_src: Path) -> FirmwarePaths:
    firmware = data.get('firmware', {})
    source_dir_raw = str(firmware.get('source_dir', '')).strip()
    if not source_dir_raw:
        raise ValueError('missing firmware.source_dir in mapping')

    build_dir_raw = str(firmware.get('build_dir', '')).strip() or 'build'
    source_dir = Path(source_dir_raw)
    if not source_dir.is_absolute():
        source_dir = (workspace_src / source_dir).resolve()

    build_dir = Path(build_dir_raw)
    if not build_dir.is_absolute():
        build_dir = (source_dir / build_dir).resolve()

    return FirmwarePaths(source_dir=source_dir, build_dir=build_dir)


def board_build_plan(data: dict, selected_boards: set[str] | None) -> list[tuple[str, str]]:
    boards_map = data.get('boards', {})
    board_ids = sorted(selected_boards) if selected_boards else sorted(boards_map.keys())
    plan: list[tuple[str, str]] = []
    for board in board_ids:
        firmware_target = str(boards_map.get(board, {}).get('firmware_target', '')).strip()
        if not firmware_target:
            raise ValueError(f"missing firmware_target for board '{board}'")
        plan.append((board, firmware_target))
    return plan
