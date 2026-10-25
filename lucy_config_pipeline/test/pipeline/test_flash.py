from pathlib import Path

import pytest
from src.pipeline import flash as pipeline_flash


def _sample_data() -> dict:
    return {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {
            'rp2040_right_arm': {
                'firmware_target': 'pico_micro_ros_right_arm',
                'serial_id': 'E6617C93E37A6629',
            },
            'rp2040_left_arm': {
                'firmware_target': 'pico_micro_ros_left_arm',
                'serial_id': None,
            },
        },
    }


def test_run_flash_phase_skips_board_without_serial(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    uf2 = fw_build / 'pico_micro_ros_right_arm.uf2'
    uf2.write_bytes(b'uf2')

    calls: list[list[str]] = []

    def fake_run_command(*, cmd: list[str], **_kwargs):
        calls.append(cmd)

    monkeypatch.setattr(pipeline_flash, '_run_command', fake_run_command)
    monkeypatch.setattr(pipeline_flash, '_wait_for_usb_serial', lambda *_a, **_k: True)

    failed, flashed = pipeline_flash.run_flash_phase(
        data=_sample_data(),
        selected_boards=None,
        boards_built_ok={'rp2040_right_arm', 'rp2040_left_arm'},
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=0,
        node=None,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert flashed == ['rp2040_right_arm']
    assert len(calls) == 1
    assert calls[0][:4] == ['sudo', 'picotool', 'load', str(uf2)]
    assert 'E6617C93E37A6629' in calls[0]


def test_run_flash_phase_skips_board_not_built_ok(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    (fw_build / 'pico_micro_ros_right_arm.uf2').write_bytes(b'uf2')

    calls: list[list[str]] = []

    def fake_run_command(*, cmd: list[str], **_kwargs):
        calls.append(cmd)

    monkeypatch.setattr(pipeline_flash, '_run_command', fake_run_command)

    failed, flashed = pipeline_flash.run_flash_phase(
        data=_sample_data(),
        selected_boards=None,
        boards_built_ok=set(),
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=0,
        node=None,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == []
    assert flashed == []
    assert calls == []


def test_run_flash_phase_missing_uf2(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    fw_src = tmp_path / 'fw'
    (fw_src / 'build').mkdir(parents=True)

    errors: list[str] = []

    monkeypatch.setattr(pipeline_flash, '_run_command', lambda **_k: None)

    failed, flashed = pipeline_flash.run_flash_phase(
        data=_sample_data(),
        selected_boards={'rp2040_right_arm'},
        boards_built_ok={'rp2040_right_arm'},
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=0,
        node=None,
        feedback=lambda **_kwargs: None,
        log_error=lambda msg: errors.append(msg),
    )

    assert failed == ['rp2040_right_arm']
    assert flashed == []
    assert errors


def test_uptime_topic_from_yaml_and_env(monkeypatch: pytest.MonkeyPatch):
    assert pipeline_flash._uptime_topic({'topic_uptime': 'ns/uptime'}) == '/ns/uptime'
    assert pipeline_flash._uptime_topic({'topic_uptime': '/abs/uptime'}) == '/abs/uptime'
    monkeypatch.delenv('LUCY_PIPELINE_UPTIME_TOPIC', raising=False)
    assert pipeline_flash._uptime_topic({}) == '/uptime_publisher'
    monkeypatch.setenv('LUCY_PIPELINE_UPTIME_TOPIC', 'custom/uptime')
    assert pipeline_flash._uptime_topic({}) == '/custom/uptime'


def test_run_flash_phase_uptime_timeout_fails_board(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    uf2 = fw_build / 'pico_micro_ros_right_arm.uf2'
    uf2.write_bytes(b'uf2')

    monkeypatch.setattr(pipeline_flash, '_run_command', lambda **_k: None)
    monkeypatch.setattr(pipeline_flash, '_wait_for_usb_serial', lambda *_a, **_k: True)
    monkeypatch.setattr(pipeline_flash, '_wait_uptime_message', lambda *_a, **_k: False)

    class _DummyNode:
        pass

    failed, flashed = pipeline_flash.run_flash_phase(
        data=_sample_data(),
        selected_boards={'rp2040_right_arm'},
        boards_built_ok={'rp2040_right_arm'},
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=5,
        node=_DummyNode(),
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == ['rp2040_right_arm']
    assert flashed == []


def test_mega_board_is_flashed_with_avrdude_on_its_serial_port(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    artifacts = tmp_path / 'fw' / 'firmwares' / 'arduino_mega' / 'target' / 'lucy'
    artifacts.mkdir(parents=True)
    (artifacts / 'mega_left.elf').write_bytes(b'elf')
    calls: list[list[str]] = []
    monkeypatch.setattr(pipeline_flash, '_run_command', lambda *, cmd, **_k: calls.append(cmd))
    monkeypatch.setattr(pipeline_flash, 'mega_port', lambda serial: f'/dev/tty-{serial}')

    failed, flashed = pipeline_flash.run_flash_phase(
        data={
            'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
            'boards': {
                'mega_left': {
                    'serial_id': 'TESTMEGA0',
                    'board_class': 'i2c_pwm_only',
                    'firmware_target': 'arduino_mega',
                },
            },
        },
        selected_boards=None,
        boards_built_ok={'mega_left'},
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=5,
        node=None,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert (failed, flashed) == ([], ['mega_left'])
    assert calls == [[
        'avrdude', '-p', 'atmega2560', '-c', 'wiring', '-P', '/dev/tty-TESTMEGA0', '-b', '115200',
        '-D', '-U', f"flash:w:{artifacts / 'mega_left.elf'}:e",
    ]]
    # The bridge lock is released once avrdude is done.
    assert not Path('/tmp/.lucy_mcu_flashing_TESTMEGA0').exists()
