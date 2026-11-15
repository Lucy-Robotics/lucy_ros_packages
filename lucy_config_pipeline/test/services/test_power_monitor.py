from src.services.power_monitor import on_real_hardware


def test_real_hardware_is_the_only_mode_with_a_bridge():
    assert on_real_hardware({"LUCY_BRIDGE": "linux"})
    assert on_real_hardware({"LUCY_BRIDGE": "mcu_bridge"})


def test_mock_and_simulation_skip_the_power_check():
    assert not on_real_hardware({})
    assert not on_real_hardware({"LUCY_BRIDGE": ""})
    assert not on_real_hardware({"LUCY_BRIDGE": "  "})
