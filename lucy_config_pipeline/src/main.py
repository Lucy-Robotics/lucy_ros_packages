from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from lucy_config_generator.schema import GENERATED_FILES_DEFAULTS
from lucy_config_generator.schema import resolve_generated_files
import rclpy
import yaml

from .config_store import ConfigStore
from .pipeline.action_server import PipelineActionServer
from .pipeline.models import PipelinePaths
from .robot_paths import resolve_robot_description_paths
from .services.config_services_node import ConfigServicesNode


def _infer_robot_source_root(robot_package: str, share_dir: Path) -> Path:
    cwd_candidate = Path.cwd() / 'src' / robot_package
    if cwd_candidate.is_dir():
        return cwd_candidate

    for p in share_dir.parents:
        if p.name == 'install':
            src_candidate = p.parent / 'src' / robot_package
            if src_candidate.is_dir():
                return src_candidate
    return share_dir


def _active_generated_files(cfg_dir: Path) -> dict[str, str]:
    """Generated-artifact filenames from the active preset (defaults if unreadable)."""
    active = cfg_dir / 'active.yaml'
    try:
        data = yaml.safe_load(active.read_text(encoding='utf-8'))
        if isinstance(data, dict):
            return resolve_generated_files(data)
    except (OSError, ValueError, yaml.YAMLError):
        pass
    return dict(GENERATED_FILES_DEFAULTS)


def _find_workspace_src(robot_root: Path) -> Path:
    for p in robot_root.parents:
        if p.name == 'src':
            return p
    raise RuntimeError(f'cannot infer workspace src from {robot_root}')


def _resolve_paths(robot_package: str, config_dir: str) -> PipelinePaths:
    share_dir = Path(get_package_share_directory(robot_package))
    robot_root = _infer_robot_source_root(robot_package, share_dir)

    cfg_dir = Path(config_dir).resolve() if config_dir else (robot_root / 'config' / 'hardware')
    names = _active_generated_files(cfg_dir)
    urdf_xacro, base_path, controller_config = resolve_robot_description_paths(
        robot_root,
        controllers_basename=names['controllers_yaml'],
    )

    return PipelinePaths(
        config_dir=cfg_dir,
        urdf_xacro=urdf_xacro,
        base_path=base_path,
        controller_config=controller_config,
        robot_root=robot_root,
        workspace_src=_find_workspace_src(robot_root),
    )


def main() -> None:
    rclpy.init()

    bootstrap = rclpy.create_node('lucy_config_pipeline_bootstrap')
    bootstrap.declare_parameter('robot_package', '')
    bootstrap.declare_parameter('config_dir', '')
    robot_package = (
        bootstrap.get_parameter('robot_package')
        .get_parameter_value()
        .string_value.strip()
    )
    config_dir = bootstrap.get_parameter('config_dir').get_parameter_value().string_value
    if not robot_package:
        bootstrap.destroy_node()
        rclpy.shutdown()
        raise SystemExit(
            'lucy_config_pipeline: robot_package is required '
            '(set by the launcher via lucy.launch → web_ros_api; '
            'do not invent a default package here)'
        )
    paths = _resolve_paths(robot_package, config_dir)
    bootstrap.destroy_node()

    store = ConfigStore(paths.config_dir)
    store.ensure_layout()

    services_node = ConfigServicesNode(
        robot_package=robot_package,
        config_store=store,
        urdf_xacro=paths.urdf_xacro,
        base_path=paths.base_path,
        controller_config=paths.controller_config,
    )
    action_node = PipelineActionServer(paths=paths, config_store=store)

    # 4 threads: 1 for the action's _execute (which blocks for minutes during
    # build/flash and seconds during reload), 1 for the reload service-client
    # response callback that unblocks _execute, and 2 for the config services
    # node so config/get/list/save remain responsive during a pipeline run.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(services_node)
    executor.add_node(action_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        services_node.destroy_node()
        action_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
