# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""ConfigurePipeline simulation_only branch and reload phase (requires ROS env)."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest

pytest.importorskip('rclpy')

from src.pipeline.action_server import PipelineActionServer  # noqa: E402
from src.pipeline.models import PipelinePaths  # noqa: E402
import yaml  # noqa: E402

_FIXTURE = (
    Path(__file__).resolve().parents[2]
    / '..'
    / 'lucy_config_generator'
    / 'test'
    / 'fixtures'
    / 'test_mapping.yaml'
)


@pytest.fixture
def pipeline_paths(tmp_path: Path) -> PipelinePaths:
    robot_root = tmp_path / 'thais_urdf'
    (robot_root / 'description' / 'urdf').mkdir(parents=True)
    (robot_root / 'description' / 'ros2_control').mkdir(parents=True)
    (robot_root / 'config').mkdir(parents=True)
    (robot_root / 'config' / 'hardware').mkdir(parents=True)
    urdf_xacro = robot_root / 'description' / 'urdf' / 'inmoov.urdf.xacro'
    urdf_xacro.write_text(
        (_FIXTURE.parent / 'test_robot.urdf.xacro').read_text(encoding='utf-8'),
        encoding='utf-8',
    )
    (robot_root / 'config' / 'controllers.yaml').write_text(
        'controller_manager:\n  ros__parameters:\n    update_rate: 100\n',
        encoding='utf-8',
    )
    return PipelinePaths(
        config_dir=robot_root / 'config' / 'hardware',
        urdf_xacro=urdf_xacro,
        base_path=robot_root / 'description',
        controller_config=robot_root / 'config' / 'controllers.yaml',
        robot_root=robot_root,
        workspace_src=tmp_path / 'src',
    )


@pytest.mark.skipif(
    not Path('/opt/ros/jazzy').exists(),
    reason='ROS 2 Jazzy overlay required for lucy_msgs/rclpy',
)
def test_simulation_only_skips_build_flash_and_calls_reload(
    pipeline_paths: PipelinePaths, rclpy_init_shutdown
):
    data = yaml.safe_load(_FIXTURE.read_text(encoding='utf-8'))
    config_yaml = yaml.dump(data)

    store = MagicMock()
    store.get_active_name.return_value = 'default'

    node = PipelineActionServer(paths=pipeline_paths, config_store=store)
    try:
        node._reload_client.wait_for_service = MagicMock(  # type: ignore[method-assign]
            return_value=True
        )

        future = MagicMock()
        future.done.return_value = True
        resp = MagicMock()
        resp.success = True
        resp.message = 'ok'
        future.result.return_value = resp
        # The new wait path uses future.add_done_callback + threading.Event,
        # not rclpy.spin_until_future_complete. Make the callback fire immediately
        # so done_event is set before .wait() is reached.
        future.add_done_callback.side_effect = lambda cb: cb(future)
        node._reload_client.call_async = MagicMock(  # type: ignore[method-assign]
            return_value=future
        )

        goal_handle = MagicMock()
        goal_handle.request.mapping_file = ''
        goal_handle.request.boards_to_flash = []
        goal_handle.request.dry_run = False
        goal_handle.request.build_only = False
        goal_handle.request.simulation_only = True
        goal_handle.is_cancel_requested = False

        with (
            patch(
                'src.pipeline.action_server.resolve_mapping_input',
                return_value=('default', config_yaml),
            ),
            patch('src.pipeline.action_server.validate_schema', return_value=data),
            patch('src.pipeline.action_server.urdf_crosscheck') as cross,
            patch('src.pipeline.action_server.generate') as gen,
            patch('src.pipeline.action_server.run_build_phase') as build,
            patch('src.pipeline.action_server.run_flash_phase') as flash,
        ):
            cross.return_value = MagicMock(errors=[])
            gen.side_effect = lambda **kwargs: _write_ros2_outputs(kwargs['output_dir'])

            result = node._execute(goal_handle)

        assert result.success is True
        build.assert_not_called()
        flash.assert_not_called()
        assert any(c.kwargs.get('simulation_only') is True for c in gen.call_args_list)
        node._reload_client.call_async.assert_called_once()
    finally:
        node.destroy_node()


def _write_ros2_outputs(out_dir: Path) -> None:
    (out_dir / 'inmoov_ros2_control.xacro').write_text('<robot/>', encoding='utf-8')
    (out_dir / 'controllers.yaml').write_text(
        'controller_manager:\n  ros__parameters:\n    update_rate: 100\n',
        encoding='utf-8',
    )
