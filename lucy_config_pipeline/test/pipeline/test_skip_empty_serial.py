# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Skip MCU build/flash when every selected board has an empty serial_id."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest

from src.pipeline.selection import all_selected_serials_empty

pytest.importorskip('rclpy')


def test_all_selected_serials_empty_true_when_blank():
    data = {
        'boards': {
            'rp2040_so_arm': {'serial_id': '', 'firmware_target': 'lucy_so_arm'},
        }
    }
    assert all_selected_serials_empty(data, None) is True
    assert all_selected_serials_empty(data, {'rp2040_so_arm'}) is True


def test_all_selected_serials_empty_false_when_any_set():
    data = {
        'boards': {
            'rp2040_left_arm': {'serial_id': 'ABC', 'firmware_target': 'lucy_left_arm'},
            'rp2040_right_arm': {'serial_id': '', 'firmware_target': 'lucy_right_arm'},
        }
    }
    assert all_selected_serials_empty(data, None) is False
    assert all_selected_serials_empty(data, {'rp2040_right_arm'}) is True


@pytest.mark.skipif(
    not Path('/opt/ros/jazzy').exists(),
    reason='ROS 2 Jazzy overlay required for lucy_msgs/rclpy',
)
def test_empty_serials_skip_build_flash_and_reload(
    pipeline_paths,
):
    from src.pipeline.action_server import PipelineActionServer
    import yaml

    fixture = (
        Path(__file__).resolve().parents[2]
        / '..'
        / 'lucy_config_generator'
        / 'test'
        / 'fixtures'
        / 'test_mapping.yaml'
    )
    data = yaml.safe_load(fixture.read_text(encoding='utf-8'))
    for board in data.get('boards', {}).values():
        if isinstance(board, dict):
            board['serial_id'] = ''
    config_yaml = yaml.dump(data)

    store = MagicMock()
    store.get_active_name.return_value = 'default'

    node = PipelineActionServer(paths=pipeline_paths, config_store=store)
    node._reload_client.wait_for_service = MagicMock(return_value=True)  # type: ignore[method-assign]

    future = MagicMock()
    future.done.return_value = True
    resp = MagicMock()
    resp.success = True
    resp.message = 'ok'
    future.result.return_value = resp
    future.add_done_callback.side_effect = lambda cb: cb(future)
    node._reload_client.call_async = MagicMock(return_value=future)  # type: ignore[method-assign]

    goal_handle = MagicMock()
    goal_handle.request.mapping_file = ''
    goal_handle.request.boards_to_flash = []
    goal_handle.request.dry_run = False
    goal_handle.request.build_only = False
    goal_handle.request.simulation_only = False
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
        gen.side_effect = lambda **kwargs: _write_outputs(kwargs['output_dir'])

        result = node._execute(goal_handle)

    assert result.success is True
    build.assert_not_called()
    flash.assert_not_called()
    node._reload_client.call_async.assert_called_once()


@pytest.fixture
def pipeline_paths(tmp_path: Path):
    from src.pipeline.models import PipelinePaths

    robot_root = tmp_path / 'so_arm101_urdf'
    (robot_root / 'description' / 'urdf').mkdir(parents=True)
    (robot_root / 'description' / 'ros2_control').mkdir(parents=True)
    (robot_root / 'config' / 'hardware').mkdir(parents=True)
    urdf_xacro = robot_root / 'description' / 'urdf' / 'so_arm.urdf.xacro'
    urdf_xacro.write_text('<robot name="so_arm"/>\n', encoding='utf-8')
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


def _write_outputs(out_dir: Path) -> None:
    (out_dir / 'so_arm101_ros2_control.xacro').write_text('<robot/>', encoding='utf-8')
    (out_dir / 'controllers.yaml').write_text(
        'controller_manager:\n  ros__parameters:\n    update_rate: 100\n',
        encoding='utf-8',
    )
