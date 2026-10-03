"""Unit tests for resolve_robot_description_paths (no ROS init)."""

from pathlib import Path

from src.robot_paths import resolve_robot_description_paths


def test_control_launch_yaml_overrides_urdf(tmp_path: Path):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config' / 'control.launch.yaml').write_text(
        'urdf_path: description/urdf/so_arm101.urdf.xacro\n'
        'base_path: description\n'
        'controllers_yaml: config/controllers.yaml\n',
        encoding='utf-8',
    )
    urdf, base, controllers = resolve_robot_description_paths(tmp_path)
    assert urdf.name == 'so_arm101.urdf.xacro'
    assert base.name == 'description'
    assert controllers.name == 'controllers.yaml'


def test_controllers_basename_override(tmp_path: Path):
    urdf, base, controllers = resolve_robot_description_paths(
        tmp_path, controllers_basename='custom_controllers.yaml'
    )
    assert controllers.name == 'custom_controllers.yaml'
    assert urdf.as_posix().endswith('robot.urdf.xacro')
    assert base.name == 'description'
