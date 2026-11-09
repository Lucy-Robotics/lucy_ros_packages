"""Applying a finished calibration into a preset, and adopting an existing one."""

from __future__ import annotations

import json

import pytest
from session_support import calibrate_all
from session_support import code_of
from src.calibration.firmware_client import FirmwareUnreachable
from src.calibration.session import APPLIED
from src.calibration.session import HALF_TURN_TICK
from src.calibration.session import REVIEW
from src.calibration.sim_backend import SIM_SERIAL
import yaml


def test_full_session_builds_a_preset_that_lerobot_tools_can_read(session, arm, store):
    calibrate_all(session, arm)
    assert session.phase == REVIEW

    session.command("apply")

    assert session.phase == APPLIED
    name = f"so101_{SIM_SERIAL}"
    preset = yaml.safe_load(store.read_named_yaml(name))
    assert (
        preset["boards"]["rp2040_so_arm"]["serial_id"] == ""
    ), "must not trigger a Pico flash"
    block = preset["calibration"]
    assert block["adapter_serial"] == SIM_SERIAL
    assert block["calibrated_at"] == "NOW"
    assert set(block["homing_offsets"]) == {"rotation", "pitch", "wrist_roll", "jaw"}

    lerobot = json.loads(store.lerobot_json_path(name).read_text())
    assert set(lerobot) == {"shoulder_pan", "shoulder_lift", "wrist_roll", "gripper"}
    assert set(lerobot["gripper"]) == {
        "id",
        "drive_mode",
        "homing_offset",
        "range_min",
        "range_max",
    }
    assert lerobot["wrist_roll"]["range_min"] == 0
    assert lerobot["wrist_roll"]["range_max"] == 4095
    # Ranges are in the homed frame, around the mid pose.
    assert (
        lerobot["gripper"]["range_min"]
        < HALF_TURN_TICK
        < lerobot["gripper"]["range_max"]
    )
    assert block["homing_offsets"]["jaw"] == lerobot["gripper"]["homing_offset"]


def test_limits_are_inset_from_the_stops_except_on_the_full_turn_joint(
    session, arm, store
):
    calibrate_all(session, arm)
    session.command("apply")
    name = f"so101_{SIM_SERIAL}"
    lerobot = json.loads(store.lerobot_json_path(name).read_text())
    actuators = {
        a["id"]: a for a in yaml.safe_load(store.read_named_yaml(name))["actuators"]
    }

    measured = lerobot["gripper"]["range_min"] * 2 * 3.14159265 / 4096
    assert actuators["jaw"]["limit_min"] > measured
    assert actuators["wrist_roll"]["limit_min"] == 0


def test_activating_the_preset_binds_the_adapter(session, arm, store):
    calibrate_all(session, arm)
    session.command("apply")
    preset = session.status()["preset"]
    assert store.bound_config(SIM_SERIAL) == ""
    store.activate(preset)
    assert store.bound_config(SIM_SERIAL) == preset


@pytest.mark.parametrize("name", ["default", "../evil", "has space", "a/b"])
def test_bad_preset_names_are_refused(session, arm, name):
    calibrate_all(session, arm)
    assert code_of(session, "apply", name) == "bad_preset_name"


def test_select_adapter_is_remembered(session, store):
    session.command("select_adapter", "ABC123")
    assert store.selected_adapter() == "ABC123"
    assert code_of(session, "select_adapter", 'bad"serial') == "bad_serial"


def test_unreachable_firmware_is_reported_not_fatal(session, arm):
    def dead():
        raise FirmwareUnreachable("gone")

    arm.status = dead
    session.poll()
    assert next(c for c in session.health() if c["id"] == "firmware")["ok"] is False


def test_adopting_binds_the_active_preset_to_this_adapter(session, store):
    store.write_named_yaml("mine", store.read_named_yaml("default"))
    store.activate("mine")
    session.command("adopt")
    assert store.bound_config(SIM_SERIAL) == "mine"


def test_the_stock_preset_cannot_be_adopted(session, store):
    assert code_of(session, "adopt") == "not_calibrated"
    assert store.bound_config(SIM_SERIAL) == ""
