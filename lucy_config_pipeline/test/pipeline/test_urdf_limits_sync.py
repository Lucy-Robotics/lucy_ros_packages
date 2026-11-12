"""Activating a preset moves the shared URDF limits to that preset's."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from lucy_config_generator.calibration import apply_urdf_limits
from lucy_config_generator.calibration import generated_limit_mismatches
import pytest
from src.config_store import ConfigStore
from src.pipeline.generate_phase import sync_urdf_limits

URDF = (
    '<robot><joint name="Rotation" type="revolute">'
    '<limit effort="10" velocity="10" lower="-1.9" upper="1.9"/></joint></robot>'
)
ACTUATOR = {
    "id": "rotation",
    "urdf_joint": "Rotation",
    "offset": 3.14159,
    "direction": 1,
    "scale": 1,
    "limit_min": 1.0,
    "limit_max": 5.0,
}


@pytest.fixture
def server(tmp_path: Path):
    limits_file = tmp_path / "limits.xacro"
    limits_file.write_text(URDF, encoding="utf-8")
    # The entry xacro only includes the file with the limits, as on the SO-ARM101.
    entry = tmp_path / "robot.urdf.xacro"
    entry.write_text(
        '<robot><xacro:include filename="limits.xacro"/></robot>', encoding="utf-8"
    )
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()
    paths = SimpleNamespace(urdf_xacro=entry, robot_root=tmp_path)
    return SimpleNamespace(store=store, paths=paths, urdf=limits_file)


def sync(server, data):
    return sync_urdf_limits(server.store, server.paths, data)


def stock(server):
    server.store.stock_urdf_limits_yaml.write_text(
        "urdf: limits.xacro\njoints:\n  Rotation: [-1.9, 1.9]\n", encoding="utf-8"
    )


def test_calibrated_preset_writes_its_own_limits(server):
    stock(server)
    before = sync(server, {"calibration": {}, "actuators": [ACTUATOR]})
    text = server.urdf.read_text()
    assert before == URDF
    assert 'lower="-2.14159"' in text
    assert 'upper="1.85841"' in text
    assert 'effort="10"' in text


def test_preset_without_calibration_goes_back_to_stock(server):
    stock(server)
    sync(server, {"calibration": {}, "actuators": [ACTUATOR]})
    sync(server, {"actuators": [ACTUATOR]})
    assert 'lower="-1.9" upper="1.9"' in server.urdf.read_text()


def test_robot_without_stock_limits_is_left_alone(server):
    assert sync(server, {"calibration": {}, "actuators": [ACTUATOR]}) is None
    assert server.urdf.read_text() == URDF


def test_so_arm101_profile_points_at_the_file_with_the_limits():
    root = Path(__file__).resolve().parents[4] / "so_arm101_urdf"
    stock_file = root / "config/hardware/urdf_limits_stock.yaml"
    if not stock_file.is_file():
        pytest.skip("so_arm101_urdf is not next to this repository")
    store = ConfigStore(root / "config/hardware")
    relative, limits = store.read_stock_urdf_limits()
    text = (root / relative).read_text(encoding="utf-8")
    assert apply_urdf_limits(text, limits)


def xacro(limit_min=1.0, lower=-2.14159) -> str:
    return (
        '<joint name="Rotation"><param name="offset">3.14159</param>'
        f'<param name="limit_min">{limit_min}</param><param name="limit_max">5</param>'
        f'<command_interface name="position"><param name="min">{lower}</param>'
        '<param name="max">1.85841</param></command_interface></joint>'
    )


def test_generated_xacro_matching_the_preset_passes():
    assert generated_limit_mismatches(xacro(), {"actuators": [ACTUATOR]}) == []


def test_stale_generated_limits_are_caught():
    problems = generated_limit_mismatches(xacro(lower=-1.9), {"actuators": [ACTUATOR]})
    assert len(problems) == 1
    assert "Rotation: min" in problems[0]


def test_joint_missing_from_the_xacro_is_caught():
    problems = generated_limit_mismatches("<robot/>", {"actuators": [ACTUATOR]})
    assert "not in the generated xacro" in problems[0]
