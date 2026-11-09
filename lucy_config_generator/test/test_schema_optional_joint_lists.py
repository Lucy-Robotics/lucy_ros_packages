"""Optional root lists passive_urdf_joints / ignore_urdf_joints (and synonyms)."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.schema import validate_hardware_yaml

import pytest
import yaml

_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "test_mapping.yaml"


def _fixture_data() -> dict:
    return yaml.safe_load(_FIXTURE.read_text(encoding="utf-8"))


def test_passive_and_ignore_urdf_joints_accepted():
    data = _fixture_data()
    data["passive_urdf_joints"] = ["left_a_joint"]
    data["ignore_urdf_joints"] = ["right_a_joint"]
    validate_hardware_yaml(data)


def test_passive_ignore_synonym_keys_accepted():
    data = _fixture_data()
    data["urdf_passive"] = ["left_a_joint"]
    data["urdf_ignore"] = ["right_a_joint"]
    validate_hardware_yaml(data)


def test_passive_urdf_joints_rejects_empty_string():
    data = _fixture_data()
    data["passive_urdf_joints"] = ["ok", "  "]
    with pytest.raises(ValueError, match="passive_urdf_joints"):
        validate_hardware_yaml(data)
