"""Backward-compatible import wrapper."""

from .pipeline.selection import board_build_plan
from .pipeline.selection import resolve_firmware_paths
from .pipeline.selection import resolve_mapping_input
from .pipeline.selection import select_boards_to_process

__all__ = [
    "resolve_mapping_input",
    "select_boards_to_process",
    "resolve_firmware_paths",
    "board_build_plan",
]
