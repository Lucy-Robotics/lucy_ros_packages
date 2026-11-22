from __future__ import annotations

from pathlib import Path
import shutil

from lucy_config_generator.calibration import apply_urdf_limits
from lucy_config_generator.calibration import urdf_limits_from_config
from lucy_config_generator.generate import generate
from lucy_config_generator.schema import resolve_generated_files

from .models import PipelinePaths
from .selection import is_mega_board
from .selection import mega_artifact_dir
from .selection import resolve_firmware_paths
from ..config_store import ConfigStore
from ..config_store.io import atomic_write_text


def generate_to_dir(
    paths: PipelinePaths,
    *,
    out_dir: Path,
    config_yaml: str,
    boards_filter: set[str] | None,
    targets: set[str],
    simulation_only: bool,
) -> None:
    (out_dir / "input.yaml").write_text(config_yaml, encoding="utf-8")
    generate(
        input_yaml=out_dir / "input.yaml",
        urdf_xacro=paths.urdf_xacro,
        base_path=paths.base_path,
        controller_config=paths.controller_config,
        output_dir=out_dir,
        targets=targets,
        boards_filter=boards_filter,
        simulation_only=simulation_only,
    )


def limits_file(
    store: ConfigStore, paths: PipelinePaths, relative: str | None = None
) -> Path:
    return paths.robot_root / (
        relative if relative is not None else store.read_stock_urdf_limits()[0]
    )


def sync_urdf_limits(
    store: ConfigStore, paths: PipelinePaths, data: dict
) -> str | None:
    """
    Make the URDF's limits those of the preset being activated.

    URDF limits are shared by all presets, but a calibration belongs to one arm.
    A preset with no calibration goes back to the robot's stock limits. Robots
    with no stock limits file don't take part. Returns the previous URDF text.
    """
    relative, stock = store.read_stock_urdf_limits()
    if not relative or not stock:
        return None
    limits = urdf_limits_from_config(data) if "calibration" in data else stock
    path = limits_file(store, paths, relative)
    before = path.read_text(encoding="utf-8")
    after = apply_urdf_limits(before, limits)
    if after != before:
        atomic_write_text(path, after)
    return before


def install_ros2_control_outputs(
    paths: PipelinePaths, out_dir: Path, data: dict
) -> None:
    names = resolve_generated_files(data)
    xacro_name = names["ros2_control_xacro"]
    ctrl_name = names["controllers_yaml"]
    xacro_dst = paths.robot_root / "description" / "ros2_control" / xacro_name
    ctrl_dst = paths.robot_root / "config" / ctrl_name
    xacro_dst.parent.mkdir(parents=True, exist_ok=True)
    ctrl_dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out_dir / xacro_name, xacro_dst)
    shutil.copy2(out_dir / ctrl_name, ctrl_dst)


def install_firmware_outputs(paths: PipelinePaths, out_dir: Path, data: dict) -> None:
    if not str(data.get("firmware", {}).get("source_dir", "")).strip():
        return
    fw_cfg_dir = resolve_firmware_paths(data, paths.workspace_src).source_dir / "config"
    fw_cfg_dir.mkdir(parents=True, exist_ok=True)
    for cfile in out_dir.glob("config_*.c"):
        shutil.copy2(cfile, fw_cfg_dir / cfile.name)
    mega_dir = mega_artifact_dir(resolve_firmware_paths(data, paths.workspace_src))
    for board_id, board in (data.get("boards") or {}).items():
        generated = out_dir / f"config_{board_id}.rs"
        if is_mega_board(board) and generated.is_file():
            mega_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(generated, mega_dir / generated.name)
