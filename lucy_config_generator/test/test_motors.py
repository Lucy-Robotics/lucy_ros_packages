"""Motor catalogue: a variant inherits its type's specs, actuators reference it."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.motors import motor_spec
from lucy_config_generator.motors import motor_values_by_joint
from lucy_config_generator.motors import motors_used
from lucy_config_generator.schema import validate_hardware_yaml
import pytest
import yaml

_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "test_mapping.yaml"


def _with_motors(*refs):
    data = yaml.safe_load(_FIXTURE.read_text(encoding="utf-8"))
    for actuator, ref in zip(data["actuators"], refs):
        actuator["motor"] = ref
    return data


def test_variant_inherits_type_specs_and_adds_its_own():
    spec = motor_spec("sts3215/12v_345")
    assert spec["ticks_per_turn"] == 4096
    assert spec["min_volts"] == 6.0
    assert spec["gear_ratio"] == 345
    assert "variants" not in spec


def test_unknown_motor_type_or_variant_is_rejected():
    with pytest.raises(KeyError):
        motor_spec("sts3215/24v_1")
    with pytest.raises(KeyError):
        motor_spec("nope/12v_345")


def test_schema_accepts_known_motor_and_rejects_unknown():
    validate_hardware_yaml(_with_motors("sts3215/12v_345"))
    with pytest.raises(ValueError, match="unknown motor 'sts3215/9v'"):
        validate_hardware_yaml(_with_motors("sts3215/9v"))


def test_motors_used_lists_each_referenced_motor_once():
    data = _with_motors("sts3215/12v_345", "sts3215/12v_345")
    assert list(motors_used(data)) == ["sts3215/12v_345"]


def test_motor_values_by_joint_skips_actuators_without_motor():
    data = _with_motors("sts3215/12v_345")
    first, second = data["actuators"][0], data["actuators"][1]
    values = motor_values_by_joint(data, "min_volts")
    assert values == {first["urdf_joint"]: 6.0}
    assert second["urdf_joint"] not in values
