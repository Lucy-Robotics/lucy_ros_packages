from pathlib import Path

import pytest
import rclpy
from src.config_store import ConfigStore
from src.services.config_services_node import ConfigServicesNode


@pytest.fixture
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestConfigServicesNode:
    def test_get_config_missing_returns_failure(
        self, rclpy_init_shutdown, tmp_path: Path
    ):
        """Unknown/active config on an empty store fails gracefully."""
        from lucy_msgs.srv import GetConfig

        node = ConfigServicesNode(
            robot_package="test_pkg",
            config_store=ConfigStore(tmp_path / "hardware"),
            urdf_xacro=Path("/dev/null"),
            base_path=tmp_path,
            controller_config=Path("/dev/null"),
        )
        try:
            response = node._on_get_config(GetConfig.Request(), GetConfig.Response())
            assert response.success is False
            assert response.robot_package == "test_pkg"
        finally:
            node.destroy_node()

    def test_get_config_serves_the_specs_of_the_motors_it_uses(
        self, rclpy_init_shutdown, tmp_path: Path
    ):
        from lucy_msgs.srv import GetConfig
        import yaml

        store = ConfigStore(tmp_path / "hardware")
        store.ensure_layout()
        store.write_named_yaml(
            "arm",
            "actuators:\n"
            "  - {id: a, urdf_joint: A, motor: sts3215/12v_345}\n"
            "  - {id: b, urdf_joint: B}\n",
        )
        node = ConfigServicesNode(
            robot_package="test_pkg",
            config_store=store,
            urdf_xacro=Path("/dev/null"),
            base_path=tmp_path,
            controller_config=Path("/dev/null"),
        )
        try:
            response = node._on_get_config(
                GetConfig.Request(config_name="arm"), GetConfig.Response()
            )
            motors = yaml.safe_load(response.motors_yaml)
            assert response.success is True
            assert list(motors) == ["sts3215/12v_345"]
            assert motors["sts3215/12v_345"]["min_volts"] == 6.0
        finally:
            node.destroy_node()
