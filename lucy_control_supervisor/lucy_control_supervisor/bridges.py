"""Host bridge firmwares (lucy_embedded_firmware colcon packages) for mode:=real."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from lucy_config_generator.schema import BOARD_CLASS_I2C_PWM_ONLY
from lucy_config_generator.schema import derive_ros2_node_name
import yaml

# lucy.launch.py bridge:= value -> (colcon package, executable), as for `ros2 run`.
BRIDGE_EXECUTABLES = {
    "linux": ("lucy_embedded_firmware_linux", "lucy-embedded-firmware-linux"),
    "mcu_bridge": (
        "lucy_embedded_firmware_mcu_bridge",
        "lucy-embedded-firmware-mcu-bridge",
    ),
}


def mega_bridges(hardware_yaml: Path) -> list[tuple[str, str]]:
    """Return ``(node_name, serial_id)`` per Mega board that has a serial_id."""
    if not hardware_yaml.is_file():
        return []
    data = yaml.safe_load(hardware_yaml.read_text(encoding="utf-8")) or {}
    out = []
    for board_id, board in (data.get("boards") or {}).items():
        board = board or {}
        serial = str(board.get("serial_id") or "").strip()
        if board.get("board_class") == BOARD_CLASS_I2C_PWM_ONLY and serial:
            out.append((derive_ros2_node_name(board_id), serial))
    return out


def ros2_control_nodes(urdf_xml: str) -> set[str]:
    """``node_name`` of every ros2_control hardware block in the expanded URDF."""
    params = ET.fromstring(urdf_xml).iterfind(
        'ros2_control/hardware/param[@name="node_name"]'
    )
    return {param.text.strip() for param in params if param.text}


def bridge_processes(
    bridge: str, hardware_yaml: Path, urdf_xml: str
) -> list[tuple[str, list[str]]]:
    """Return ``(node_name, argv tail)`` for each bridge process to start.

    ``linux``: one per ros2_control node of the URDF (it maps that node's shared memory).
    ``mcu_bridge``: one per Mega board of the active YAML that the URDF drives.
    """
    nodes = ros2_control_nodes(urdf_xml)
    if bridge == "linux":
        return [(node, [node]) for node in sorted(nodes)]
    if bridge == "mcu_bridge":
        return [
            (node, [node, serial])
            for node, serial in mega_bridges(hardware_yaml)
            if node in nodes
        ]
    raise ValueError(f"unknown bridge {bridge!r} (expected {', '.join(BRIDGE_EXECUTABLES)})")


def bridge_executable(bridge: str) -> str:
    """Path of the bridge executable, looked up the way ``ros2 run`` does.

    The supervisor runs it directly rather than through ``ros2 run``, so stop signals and
    the parent-death signal reach the firmware rather than an intermediate process.
    Raises ``LookupError`` when the package is not built.
    """
    from ros2pkg.api import PackageNotFound
    from ros2run.api import get_executable_path

    package, executable = BRIDGE_EXECUTABLES[bridge]
    try:
        path = get_executable_path(package_name=package, executable_name=executable)
    except PackageNotFound:
        path = None
    if path is None:
        raise LookupError(
            f"{package} is not built (`ros2 run {package} {executable}` would fail): "
            "run `pixi run firmware-setup`, then `pixi run build`"
        )
    return path
