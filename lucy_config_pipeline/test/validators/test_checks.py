from src.validation import parse_yaml_text
from src.validation import ValidationReport


def test_parse_yaml_mapping():
    data = parse_yaml_text("version: 1\nrobot_name: inmoov\n")
    assert data["version"] == 1


def test_parse_yaml_rejects_non_mapping():
    try:
        parse_yaml_text("- 1\n- 2\n")
        assert False, "list root must fail"
    except ValueError:
        pass


def test_validation_report_dataclass():
    r = ValidationReport(errors=["e"], warnings=["w"])
    assert r.errors == ["e"]
    assert r.warnings == ["w"]
