from __future__ import annotations

from contextlib import ExitStack
import json
from pathlib import Path
import tempfile
from typing import Callable

from lucy_config_generator.calibration import generated_limit_mismatches
from lucy_config_generator.schema import resolve_generated_files
from lucy_msgs.action import ConfigurePipeline
from std_srvs.srv import Trigger

from .build import run_build_phase
from .flash import flash_picotool_timeout_seconds
from .flash import flash_uptime_wait_seconds
from .flash import flash_usb_wait_seconds
from .flash import run_flash_phase
from .generate_phase import generate_to_dir
from .generate_phase import install_firmware_outputs
from .generate_phase import install_ros2_control_outputs
from .generate_phase import limits_file
from .generate_phase import sync_urdf_limits
from .models import PipelinePaths
from .selection import all_selected_serials_empty
from .selection import board_build_plan
from .selection import resolve_mapping_input
from .selection import select_boards_to_process
from .selection import skip_mcu_reason
from ..config_store import ConfigStore
from ..config_store.io import atomic_write_text
from ..error_format import format_error_lines
from ..ros_wait import wait_for_future
from ..validation import urdf_crosscheck
from ..validation import validate_schema


def feedback(
    goal_handle, *, phase: str, progress: float, detail: str, board: str = ""
) -> None:
    fb = ConfigurePipeline.Feedback()
    fb.phase = phase
    q = round(max(0.0, min(1.0, float(progress))), 2)
    fb.progress = q
    fb.detail = json.dumps(
        {"phase": phase, "board": board, "progress": q, "message": detail}
    )
    fb.board = board
    goal_handle.publish_feedback(fb)


def call_reload_service(
    goal_handle,
    reload_client,
    *,
    reload_service: str,
    reload_timeout_seconds: float,
    reload_restart_timeout_seconds: float,
) -> tuple[bool, str]:
    if not reload_client.wait_for_service(timeout_sec=reload_timeout_seconds):
        return False, f"service {reload_service} not available"

    feedback(
        goal_handle, phase="reload", progress=0.0, detail="restarting control stack"
    )
    future = reload_client.call_async(Trigger.Request())

    # The reload client lives in its own ReentrantCallbackGroup, so the
    # MultiThreadedExecutor can dispatch the response on a different thread
    # while this _execute call stays parked here.
    completed = wait_for_future(future, reload_restart_timeout_seconds)
    if goal_handle.is_cancel_requested:
        return False, "reload cancelled"
    if not completed:
        return False, f"reload timed out after {reload_restart_timeout_seconds}s"
    resp = future.result()
    if resp is None:
        return False, "reload service call failed"
    if not resp.success:
        return False, resp.message or "reload failed"
    feedback(
        goal_handle,
        phase="reload",
        progress=1.0,
        detail=resp.message or "reload complete",
    )
    return True, resp.message or "reload complete"


def run_execute(
    goal_handle,
    *,
    node,
    store: ConfigStore,
    paths: PipelinePaths,
    reload_client,
    reload_service: str,
    reload_timeout_seconds: float,
    reload_restart_timeout_seconds: float,
    build_timeout_seconds: int,
    log_error: Callable[[str], None],
):
    result = ConfigurePipeline.Result()
    result.success = False
    result.config_name = store.get_active_name()
    result.boards_flashed = []
    result.errors = []
    mega_flash_locks = ExitStack()
    urdf_before: str | None = None

    try:
        goal = goal_handle.request
        result.config_name, config_yaml = resolve_mapping_input(
            store, goal.mapping_file
        )

        feedback(
            goal_handle, phase="validate", progress=0.1, detail="schema validation"
        )
        data = validate_schema(config_yaml)

        if not goal.dry_run:
            urdf_before = sync_urdf_limits(store, paths, data)

        for path in (paths.urdf_xacro, paths.base_path, paths.controller_config):
            if not path.exists():
                raise FileNotFoundError(path)

        feedback(goal_handle, phase="validate", progress=0.6, detail="urdf cross-check")
        report = urdf_crosscheck(
            data, paths.urdf_xacro, paths.base_path, paths.controller_config
        )
        if report.errors:
            result.errors.extend(format_error_lines(report.errors))
            result.message = "validation failed"
            goal_handle.abort()
            return result

        boards = select_boards_to_process(data, list(goal.boards_to_flash))
        boards_set = set(boards) if boards else None

        with tempfile.TemporaryDirectory(prefix="lucy_config_pipeline_") as tmp:
            out_dir = Path(tmp)

            # ros2_control + controllers are always regenerated, regardless of
            # simulation_only / build_only / dry_run
            feedback(
                goal_handle,
                phase="generate",
                progress=0.3,
                detail=(
                    "rendering simulation ros2_control"
                    if goal.simulation_only
                    else "rendering ros2_control"
                ),
            )
            generate_to_dir(
                paths,
                out_dir=out_dir,
                config_yaml=config_yaml,
                boards_filter=None if goal.simulation_only else boards_set,
                targets={"ros2_control", "controllers"},
                simulation_only=goal.simulation_only,
            )
            if (
                urdf_before is not None
                and "calibration" in data
                and not goal.simulation_only
            ):
                names = resolve_generated_files(data)
                problems = generated_limit_mismatches(
                    (out_dir / names["ros2_control_xacro"]).read_text(encoding="utf-8"),
                    data,
                )
                if problems:
                    result.errors.extend(format_error_lines(problems))
                    result.message = "generated limits do not match the preset"
                    goal_handle.abort()
                    return result

            if not goal.dry_run:
                feedback(
                    goal_handle,
                    phase="generate",
                    progress=0.5,
                    detail="installing ros2_control outputs",
                )
                install_ros2_control_outputs(paths, out_dir, data)
                # Generated files now match the URDF, so it must not be rolled back alone.
                urdf_before = None

            if not goal.simulation_only:
                feedback(
                    goal_handle,
                    phase="generate",
                    progress=0.7,
                    detail="rendering firmware",
                )
                generate_to_dir(
                    paths,
                    out_dir=out_dir,
                    config_yaml=config_yaml,
                    boards_filter=boards_set,
                    targets={"firmware"},
                    simulation_only=False,
                )
                if not goal.dry_run:
                    install_firmware_outputs(paths, out_dir, data)

        plan_boards = [b for b, _ in board_build_plan(data, boards)]
        plan_set = set(plan_boards)
        skip_mcu = (
            not goal.dry_run
            and not goal.simulation_only
            and all_selected_serials_empty(data, boards)
        )

        build_failed_boards: list[str] = []
        if skip_mcu:
            feedback(
                goal_handle,
                phase="build",
                progress=0.85,
                detail=f"skip build: {skip_mcu_reason(data, boards)}",
            )
            built_ok = set(plan_boards)
        elif not goal.dry_run and not goal.simulation_only:
            build_failed_boards = run_build_phase(
                data=data,
                selected_boards=boards,
                workspace_src=paths.workspace_src,
                timeout_seconds=build_timeout_seconds,
                feedback=lambda **kwargs: feedback(goal_handle, **kwargs),
                log_error=log_error,
            )
            built_ok = {b for b in plan_boards if b not in set(build_failed_boards)}
        else:
            built_ok = {b for b in plan_boards if b not in set(build_failed_boards)}

        if not goal.dry_run and goal.build_only and build_failed_boards:
            result.errors.extend(_build_failed_lines(build_failed_boards))
            result.message = "build failed"
            goal_handle.abort()
            return result

        if not goal.dry_run and plan_set and not built_ok and not goal.simulation_only:
            result.errors.extend(_build_failed_lines(build_failed_boards))
            result.message = "build failed"
            goal_handle.abort()
            return result

        if build_failed_boards:
            result.errors.extend(_build_failed_lines(build_failed_boards))

        flash_failed: list[str] = []
        if skip_mcu:
            feedback(
                goal_handle,
                phase="flash",
                progress=0.9,
                detail=f"skip flash: {skip_mcu_reason(data, boards)}",
            )
            result.boards_flashed = []
        elif not goal.dry_run and not goal.build_only and not goal.simulation_only:
            flash_failed, flashed_ok = run_flash_phase(
                data=data,
                selected_boards=boards,
                boards_built_ok=built_ok,
                workspace_src=paths.workspace_src,
                picotool_timeout_seconds=flash_picotool_timeout_seconds(),
                usb_wait_seconds=flash_usb_wait_seconds(),
                uptime_wait_seconds=flash_uptime_wait_seconds(),
                node=node,
                feedback=lambda **kwargs: feedback(goal_handle, **kwargs),
                log_error=log_error,
                locks=mega_flash_locks,
            )
            result.boards_flashed = flashed_ok
            if flash_failed:
                result.errors.extend(
                    format_error_lines(
                        [f"flash failed for boards: {', '.join(sorted(flash_failed))}"]
                    )
                )
                result.message = "flash failed"
                goal_handle.abort()
                return result
            if flashed_ok:
                store.record_flashed_preset(result.config_name)

        if not goal.dry_run:
            ok, reload_msg = call_reload_service(
                goal_handle,
                reload_client,
                reload_service=reload_service,
                reload_timeout_seconds=reload_timeout_seconds,
                reload_restart_timeout_seconds=reload_restart_timeout_seconds,
            )
            if not ok:
                result.errors.extend(format_error_lines([reload_msg]))
                result.message = reload_msg
                goal_handle.abort()
                return result

        result.success = True
        if build_failed_boards:
            result.message = "pipeline completed with partial build failure"
        else:
            result.message = "pipeline completed"
        goal_handle.succeed()
        return result
    except Exception as e:  # pragma: no cover - top-level action guard
        lines = [line for line in str(e).splitlines() if line.strip()]
        raw = lines if lines else [str(e)]
        result.errors.extend(format_error_lines(raw))
        result.message = f"pipeline failed: {e}"
        goal_handle.abort()
        return result
    finally:
        mega_flash_locks.close()
        if urdf_before is not None and not result.success:
            atomic_write_text(limits_file(store, paths), urdf_before)


def _build_failed_lines(build_failed_boards: list[str]) -> list[str]:
    return format_error_lines(
        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
    )
