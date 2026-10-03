from pathlib import Path

import pytest
from src.pipeline import flash as pipeline_flash


def _sample_data() -> dict:
    return {
        'firmware': {'source_dir': 'fw', 'build_dir': 'build'},
        'boards': {
            'rp2040_right_arm': {
                'firmware_target': 'lucy_right_arm',
                'serial_id': 'E6617C93E37A6629',
            },
            'rp2040_left_arm': {
                'firmware_target': 'lucy_left_arm',
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
    uf2 = fw_build / 'lucy_right_arm.uf2'
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
    (fw_build / 'lucy_right_arm.uf2').write_bytes(b'uf2')

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


def test_run_flash_phase_modbus_verify_timeout_fails_board(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    uf2 = fw_build / 'lucy_right_arm.uf2'
    uf2.write_bytes(b'uf2')

    monkeypatch.setattr(pipeline_flash, '_run_command', lambda **_k: None)
    monkeypatch.setattr(pipeline_flash, '_wait_for_usb_serial', lambda *_a, **_k: True)
    monkeypatch.setattr(pipeline_flash, '_wait_modbus_ready', lambda *_a, **_k: False)

    failed, flashed = pipeline_flash.run_flash_phase(
        data=_sample_data(),
        selected_boards={'rp2040_right_arm'},
        boards_built_ok={'rp2040_right_arm'},
        workspace_src=tmp_path,
        picotool_timeout_seconds=30,
        usb_wait_seconds=1,
        uptime_wait_seconds=5,
        node=None,
        feedback=lambda **_kwargs: None,
        log_error=lambda _msg: None,
    )

    assert failed == ['rp2040_right_arm']
    assert flashed == []
