#!/usr/bin/env python3
"""ROS 2 node exposing hardware discovery services for the Lucy control panel."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger

from lucy_hardware_discovery.device_scanner import (
    dumps_json,
    list_cameras,
    list_realsense_devices,
    list_serial_devices,
    match_boards,
    probe_camera,
)


def _default_active_yaml_path() -> Path:
    cwd = Path.cwd()
    candidates = [
        cwd / "src" / "inmoov_urdf" / "config" / "hardware" / "active.yaml",
        cwd / "src" / "inmoov_urdf" / "config" / "hardware" / "local" / "active.yaml",
    ]
    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]


class HardwareDiscoveryNode(Node):
    """Enumerate host peripherals and return JSON via std_srvs/Trigger responses."""

    def __init__(self) -> None:
        super().__init__("hardware_discovery_node")
        self.declare_parameter("active_yaml_path", str(_default_active_yaml_path()))
        self.declare_parameter("probe_device", "")

        self.create_service(Trigger, "/hardware/list_serial_devices", self._list_serial)
        self.create_service(Trigger, "/hardware/list_cameras", self._list_cameras)
        self.create_service(Trigger, "/hardware/list_realsense", self._list_realsense)
        self.create_service(Trigger, "/hardware/match_boards", self._match_boards)
        self.create_service(Trigger, "/hardware/probe_camera", self._probe_camera)
        self.get_logger().info("Hardware discovery services ready")

    def _list_serial(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = dumps_json({"devices": list_serial_devices()})
        return response

    def _list_cameras(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = dumps_json({"devices": list_cameras()})
        return response

    def _list_realsense(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        devices = list_realsense_devices()
        response.success = True
        response.message = dumps_json({"devices": devices})
        if not devices:
            self.get_logger().debug(
                "No RealSense devices found (rs-enumerate-devices missing or no device attached)"
            )
        return response

    def _load_board_serial_ids(self) -> Dict[str, str]:
        raw = self.get_parameter("active_yaml_path").get_parameter_value().string_value
        path = Path(raw) if raw.strip() else _default_active_yaml_path()
        if not path.is_file():
            self.get_logger().warning(f"active.yaml not found: {path}")
            return {}
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        boards = data.get("boards") or {}
        return {
            board_id: (info or {}).get("serial_id", "")
            for board_id, info in boards.items()
        }

    def _match_boards(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            payload = match_boards(self._load_board_serial_ids())
            response.success = True
            response.message = dumps_json(payload)
        except Exception as exc:  # noqa: BLE001 — return error JSON to LCP
            response.success = False
            response.message = dumps_json({"error": str(exc)})
        return response

    def _probe_camera(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        device = self.get_parameter("probe_device").get_parameter_value().string_value
        if not device:
            response.success = False
            response.message = dumps_json(
                {
                    "error": (
                        "Set probe_device parameter before calling "
                        "/hardware/probe_camera"
                    )
                }
            )
            return response
        result = probe_camera(device)
        response.success = result["success"]
        response.message = dumps_json(result)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareDiscoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
