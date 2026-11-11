"""Host bridge firmwares (lucy_embedded_firmware colcon packages) for mode:=real."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

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


def mcu_boards(hardware_yaml: Path) -> list[tuple[str, str]]:
    """Return ``(node_name, serial_id)`` per board of the active YAML (``""``: no serial)."""
    if not hardware_yaml.is_file():
        return []
    data = yaml.safe_load(hardware_yaml.read_text(encoding="utf-8")) or {}
    return [
        (derive_ros2_node_name(board_id), str((board or {}).get("serial_id") or "").strip())
        for board_id, board in (data.get("boards") or {}).items()
    ]


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
    ``mcu_bridge``: one per board of the active YAML that the URDF drives, RP2040 or Mega.
    The bridge finds its board by ``serial_id``, or takes the first RP2040/Mega it sees
    when none is given: fine for a robot with one board (SO-ARM101), ambiguous with
    several, where boards without a ``serial_id`` are left out.
    """
    nodes = ros2_control_nodes(urdf_xml)
    if bridge == "linux":
        return [(node, [node]) for node in sorted(nodes)]
    if bridge == "mcu_bridge":
        boards = [(node, serial) for node, serial in mcu_boards(hardware_yaml) if node in nodes]
        if len(boards) == 1:
            node, serial = boards[0]
            return [(node, [node, serial] if serial else [node])]
        return [(node, [node, serial]) for node, serial in boards if serial]
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
