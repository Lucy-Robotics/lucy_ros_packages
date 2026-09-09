from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import tempfile
import threading

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from lucy_config_generator.generate import generate
from lucy_config_generator.schema import resolve_generated_files
from lucy_msgs.action import ConfigurePipeline

from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import urdf_crosscheck
from ..validation import validate_schema
from .build import run_build_phase
from .flash import flash_picotool_timeout_seconds
from .flash import flash_uptime_wait_seconds
from .flash import flash_usb_wait_seconds
from .flash import run_flash_phase
from .models import PipelinePaths
from .selection import board_build_plan
from .selection import resolve_mapping_input
from .selection import select_boards_to_process


class PipelineActionServer(Node):
    _BUILD_TIMEOUT_SECONDS = int(os.environ.get('LUCY_PIPELINE_BUILD_TIMEOUT_SEC', '300'))
    _RELOAD_SERVICE = '/lucy_control/restart'
    _RELOAD_TIMEOUT_SECONDS = 15.0

    def __init__(self, *, paths: PipelinePaths, config_store: ConfigStore):
        super().__init__('lucy_config_pipeline')
        self._paths = paths
        self._store = config_store
        self._busy_lock = threading.Lock()
        self._busy = False
        # The action's _execute callback blocks while waiting for the reload
        # service response. To avoid a single-callback-group deadlock (and the
        # related corruption from re-entering the executor via
        # spin_until_future_complete), give the action and the reload client
        # independent callback groups so the response can be dispatched on a
        # different thread while _execute is still running.
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._reload_cb_group = ReentrantCallbackGroup()
        self._reload_client = self.create_client(
            Trigger, self._RELOAD_SERVICE, callback_group=self._reload_cb_group
        )
        self._action_server = ActionServer(
            self,
            ConfigurePipeline,
            'configure_pipeline',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def _goal_callback(self, _goal_request: ConfigurePipeline.Goal) -> GoalResponse:
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _feedback(
        self,
        goal_handle,
        *,
        phase: str,
        progress: float,
        detail: str,
        board: str = '',
    ) -> None:
        fb = ConfigurePipeline.Feedback()
        fb.phase = phase
        q = round(max(0.0, min(1.0, float(progress))), 2)
        fb.progress = q
        fb.detail = json.dumps(
            {
                'phase': phase,
                'board': board,
                'progress': q,
                'message': detail,
            }
        )
        fb.board = board
        goal_handle.publish_feedback(fb)

    def _call_reload_service(self, goal_handle) -> tuple[bool, str]:
        if not self._reload_client.wait_for_service(timeout_sec=self._RELOAD_TIMEOUT_SECONDS):
            return False, f'service {self._RELOAD_SERVICE} not available'

        self._feedback(
            goal_handle,
            phase='reload',
            progress=0.0,
            detail='restarting control stack',
        )
        req = Trigger.Request()
        future = self._reload_client.call_async(req)

        # Wait for the response *without* re-entering the executor. The reload
        # client lives in its own ReentrantCallbackGroup, so the
        # MultiThreadedExecutor can dispatch the response on a different thread
        # while this _execute call stays parked here.
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        completed = done_event.wait(timeout=self._RELOAD_TIMEOUT_SECONDS)
        if goal_handle.is_cancel_requested:
            return False, 'reload cancelled'
        if not completed or not future.done():
            return False, f'reload timed out after {self._RELOAD_TIMEOUT_SECONDS}s'
        resp = future.result()
        if resp is None:
            return False, 'reload service call failed'
        if not resp.success:
            return False, resp.message or 'reload failed'
        self._feedback(
            goal_handle,
            phase='reload',
            progress=1.0,
            detail=resp.message or 'reload complete',
        )
        return True, resp.message or 'reload complete'

    def _generate_to_dir(
        self,
        *,
        out_dir: Path,
        config_yaml: str,
        boards_filter: set[str] | None,
        targets: set[str],
        simulation_only: bool,
    ) -> None:
        (out_dir / 'input.yaml').write_text(config_yaml, encoding='utf-8')
        generate(
            input_yaml=out_dir / 'input.yaml',
            urdf_xacro=self._paths.urdf_xacro,
            base_path=self._paths.base_path,
            controller_config=self._paths.controller_config,
            output_dir=out_dir,
            targets=targets,
            boards_filter=boards_filter,
            simulation_only=simulation_only,
        )

    def _execute(self, goal_handle):
        result = ConfigurePipeline.Result()
        result.success = False
        result.config_name = self._store.get_active_name()
        result.boards_flashed = []
        result.errors = []

        try:
            goal = goal_handle.request
            result.config_name, config_yaml = resolve_mapping_input(self._store, goal.mapping_file)

            self._feedback(goal_handle, phase='validate', progress=0.1, detail='schema validation')
            data = validate_schema(config_yaml)

            for path in (
                self._paths.urdf_xacro,
                self._paths.base_path,
                self._paths.controller_config,
            ):
                if not path.exists():
                    raise FileNotFoundError(path)

            self._feedback(goal_handle, phase='validate', progress=0.6, detail='urdf cross-check')
            report = urdf_crosscheck(
                data,
                self._paths.urdf_xacro,
                self._paths.base_path,
                self._paths.controller_config,
            )
            if report.errors:
                result.errors.extend(format_error_lines(report.errors))
                result.message = 'validation failed'
                goal_handle.abort()
                return result

            boards = select_boards_to_process(data, list(goal.boards_to_flash))
            boards_set = set(boards) if boards else None

            with tempfile.TemporaryDirectory(prefix='lucy_config_pipeline_') as tmp:
                out_dir = Path(tmp)

                # ros2_control + controllers are always regenerated, regardless of
                # simulation_only / build_only / dry_run
                self._feedback(
                    goal_handle,
                    phase='generate',
                    progress=0.3,
                    detail=(
                        'rendering simulation ros2_control'
                        if goal.simulation_only
                        else 'rendering ros2_control'
                    ),
                )
                self._generate_to_dir(
                    out_dir=out_dir,
                    config_yaml=config_yaml,
                    boards_filter=None if goal.simulation_only else boards_set,
                    targets={'ros2_control', 'controllers'},
                    simulation_only=goal.simulation_only,
                )
                if not goal.dry_run:
                    self._feedback(
                        goal_handle,
                        phase='generate',
                        progress=0.5,
                        detail='installing ros2_control outputs',
                    )
                    self._install_ros2_control_outputs(out_dir, data)

                if not goal.simulation_only:
                    self._feedback(
                        goal_handle,
                        phase='generate',
                        progress=0.7,
                        detail='rendering firmware',
                    )
                    self._generate_to_dir(
                        out_dir=out_dir,
                        config_yaml=config_yaml,
                        boards_filter=boards_set,
                        targets={'firmware'},
                        simulation_only=False,
                    )
                    if not goal.dry_run:
                        self._install_firmware_outputs(out_dir, data)

            build_failed_boards: list[str] = []
            if not goal.dry_run and not goal.simulation_only:
                build_failed_boards = run_build_phase(
                    data=data,
                    selected_boards=boards,
                    workspace_src=self._paths.workspace_src,
                    timeout_seconds=self._BUILD_TIMEOUT_SECONDS,
                    feedback=lambda **kwargs: self._feedback(goal_handle, **kwargs),
                    log_error=lambda msg: self.get_logger().error(msg),
                )

            plan_boards = [b for b, _ in board_build_plan(data, boards)]
            plan_set = set(plan_boards)
            built_ok = {b for b in plan_boards if b not in set(build_failed_boards)}

            if not goal.dry_run and goal.build_only and build_failed_boards:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )
                result.message = 'build failed'
                goal_handle.abort()
                return result

            if not goal.dry_run and plan_set and not built_ok and not goal.simulation_only:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )
                result.message = 'build failed'
                goal_handle.abort()
                return result

            if build_failed_boards:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )

            flash_failed: list[str] = []
            if not goal.dry_run and not goal.build_only and not goal.simulation_only:
                flash_failed, flashed_ok = run_flash_phase(
                    data=data,
                    selected_boards=boards,
                    boards_built_ok=built_ok,
                    workspace_src=self._paths.workspace_src,
                    picotool_timeout_seconds=flash_picotool_timeout_seconds(),
                    usb_wait_seconds=flash_usb_wait_seconds(),
                    uptime_wait_seconds=flash_uptime_wait_seconds(),
                    node=self,
                    feedback=lambda **kwargs: self._feedback(goal_handle, **kwargs),
                    log_error=lambda msg: self.get_logger().error(msg),
                )
                result.boards_flashed = flashed_ok
                if flash_failed:
                    result.errors.extend(
                        format_error_lines(
                            [f"flash failed for boards: {', '.join(sorted(flash_failed))}"]
                        )
                    )
                    result.message = 'flash failed'
                    goal_handle.abort()
                    return result
                if flashed_ok:
                    self._store.record_flashed_preset(result.config_name)

            if not goal.dry_run:
                ok, reload_msg = self._call_reload_service(goal_handle)
                if not ok:
                    result.errors.extend(format_error_lines([reload_msg]))
                    result.message = reload_msg
                    goal_handle.abort()
                    return result

            result.success = True
            if build_failed_boards:
                result.message = 'pipeline completed with partial build failure'
            else:
                result.message = 'pipeline completed'
            goal_handle.succeed()
            return result
        except Exception as e:  # pragma: no cover - top-level action guard
            lines = [line for line in str(e).splitlines() if line.strip()]
            raw = lines if lines else [str(e)]
            result.errors.extend(format_error_lines(raw))
            result.message = f'pipeline failed: {e}'
            goal_handle.abort()
            return result
        finally:
            with self._busy_lock:
                self._busy = False

    def _install_ros2_control_outputs(self, out_dir: Path, data: dict) -> None:
        names = resolve_generated_files(data)
        xacro_name = names['ros2_control_xacro']
        ctrl_name = names['controllers_yaml']
        xacro_dst = (
            self._paths.robot_root / 'description' / 'ros2_control' / xacro_name
        )
        ctrl_dst = self._paths.robot_root / 'config' / ctrl_name
        xacro_dst.parent.mkdir(parents=True, exist_ok=True)
        ctrl_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(out_dir / xacro_name, xacro_dst)
        shutil.copy2(out_dir / ctrl_name, ctrl_dst)

    def _install_firmware_outputs(self, out_dir: Path, data: dict) -> None:
        firmware_src_dir = str(data.get('firmware', {}).get('source_dir', '')).strip()
        if not firmware_src_dir:
            return
        fw_cfg_dir = (self._paths.workspace_src / firmware_src_dir / 'config').resolve()
        fw_cfg_dir.mkdir(parents=True, exist_ok=True)
        for yfile in out_dir.glob('config_*.yaml'):
            shutil.copy2(yfile, fw_cfg_dir / yfile.name)
        # Also keep a default config.yaml for single-board cargo builds.
        yaml_files = sorted(out_dir.glob('config_*.yaml'))
        if yaml_files:
            rp2040_cfg = (
                self._paths.workspace_src
                / firmware_src_dir
                / 'firmwares'
                / 'rp2040'
                / 'config.yaml'
            )
            rp2040_cfg.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(yaml_files[0], rp2040_cfg)
