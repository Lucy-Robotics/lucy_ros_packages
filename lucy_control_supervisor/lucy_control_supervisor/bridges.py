"""Host bridge firmwares (lucy_embedded_firmware colcon packages) for mode:=real."""

from __future__ import annotations

from pathlib import Path
import re
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
# The bridge prints one of these per PCA9685 when its state changes.
_PCA_LINE = re.compile(r"PCA9685 (0x[0-9a-f]+) (not answering|answering)")
# The bridge prints `Board <serial> <state>` when its link to the board changes.
_BOARD_LINE = re.compile(r"Board \S*\s(.+)")
_UNREACHABLE = (
    ("not found", "not found on USB: check its cable and serial_id"),
    ("is being flashed", "being flashed"),
    ("stopped answering", "stopped answering: reconnecting"),
    ("not answering", "on USB but not answering (frozen?): unplug and replug it"),
)


def board_entries(hardware_yaml: Path) -> list[tuple[str, str, str, str]]:
    """Return ``(board_id, node_name, serial_id, pca_addresses)`` per board of the active YAML.

    ``pca_addresses`` is the Mega bridge's argument naming its PCA9685s, e.g. ``0x41,0x40``,
    and ``""`` for other boards.
    """
    if not hardware_yaml.is_file():
        return []
    data = yaml.safe_load(hardware_yaml.read_text(encoding="utf-8")) or {}
    out = []
    for board_id, board in (data.get("boards") or {}).items():
        board = board or {}
        pcas = ""
        if board.get("board_class") == BOARD_CLASS_I2C_PWM_ONLY:
            pcas = ",".join(f"{int(a):#04x}" for a in board.get("pca_addresses") or [])
        serial = str(board.get("serial_id") or "").strip()
        out.append((board_id, derive_ros2_node_name(board_id), serial, pcas))
    return out


def mcu_boards(hardware_yaml: Path) -> list[tuple[str, str]]:
    """Return ``(node_name, serial_id)`` per board of the active YAML (``""``: no serial)."""
    return [(node, serial) for _, node, serial, _ in board_entries(hardware_yaml)]


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
        boards = [
            (node, serial, pcas)
            for _, node, serial, pcas in board_entries(hardware_yaml)
            if node in nodes
        ]
        if len(boards) == 1 and not boards[0][1]:
            node, _, pcas = boards[0]
            return [(node, [node, "", pcas] if pcas else [node])]
        return [
            (node, [node, serial, pcas] if pcas else [node, serial])
            for node, serial, pcas in boards
            if serial
        ]
    raise ValueError(
        f"unknown bridge {bridge!r} (expected {', '.join(BRIDGE_EXECUTABLES)})"
    )


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


def pca_warning(board: str, line: str) -> tuple[str, bool] | None:
    """``(warning, active)`` for a bridge's PCA9685 status line, None for other lines."""
    m = _PCA_LINE.match(line)
    if m is None:
        return None
    message = f"{board} Mega: PCA9685 {m[1]} not answering, its servos cannot move"
    return message, m[2] == "not answering"


def board_unreachable(line: str) -> str | None:
    """Why the board is unreachable after this line, '' once it answers, None if unrelated."""
    m = _BOARD_LINE.match(line)
    if m is None:
        return None
    if m[1] == "answering":
        return ""
    if m[1].startswith("cannot be opened"):
        return m[1].removesuffix(", waiting")
    return next(
        (reason for start, reason in _UNREACHABLE if m[1].startswith(start)), None
    )
