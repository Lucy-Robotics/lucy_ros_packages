"""Hardware mapping strings cannot inject markup or code into generated files."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.schema import validate_hardware_yaml

import pytest
import yaml

_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "test_mapping.yaml"


def _mapping() -> dict:
    return yaml.safe_load(_FIXTURE.read_text(encoding="utf-8"))


@pytest.mark.parametrize(
    "name",
    [
        'x"><xacro:property name="p" value="${1}"/><r a="',
        "${python.os}",
        "a\nb",
    ],
)
def test_robot_name_with_markup_is_rejected(name):
    data = _mapping()
    data["robot_name"] = name
    with pytest.raises(ValueError, match="robot_name"):
        validate_hardware_yaml(data)


def test_unsafe_keys_are_rejected():
    data = _mapping()
    data["boards"]["<b>"] = {}
    with pytest.raises(ValueError, match="outside"):
        validate_hardware_yaml(data)


def test_the_fixture_is_safe():
    validate_hardware_yaml(_mapping())
