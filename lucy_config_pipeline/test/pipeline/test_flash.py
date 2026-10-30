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
    monkeypatch.setattr(pipeline_flash, '_mcu_bridge_process_running', lambda *_a, **_k: False)
    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', lambda **_k: [])
    monkeypatch.setattr(pipeline_flash, '_picotool_bin', lambda: 'picotool')
    loc = pipeline_flash._UsbLoc(
        bus=1, address=2, serial='E6617C93E37A6629', product='Servo2040 Serial'
    )
    monkeypatch.setattr(pipeline_flash, '_find_usb_loc_by_serial', lambda _s: loc)
    monkeypatch.setattr(type(loc), 'writable', property(lambda self: True))

    # After force reboot, pretend the volume appears and accept the copy.
    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    seen = {'n': 0}

    def ensure(**_k):
        seen['n'] += 1
        return [volume] if seen['n'] > 1 else []

    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', ensure)

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
    assert calls[0][:2] == ['picotool', 'reboot']
    assert calls[0].index('--bus') < calls[0].index('-f')
    assert 'sudo' not in calls[0]
    assert (volume / uf2.name).exists()


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


def test_flash_uf2_copies_to_bootsel_volume(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    uf2 = tmp_path / 'board.uf2'
    uf2.write_bytes(b'uf2-bytes')
    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2 Bootloader v1\n', encoding='utf-8')

    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [volume])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: ['E0C9125B0D9B'])

    calls: list[list[str]] = []
    monkeypatch.setattr(
        pipeline_flash,
        '_run_command',
        lambda **kwargs: calls.append(kwargs['cmd']),
    )

    pipeline_flash._flash_uf2(
        uf2=uf2,
        configured_serial='CONFIGURED',
        board='rp2040_left_arm',
        cwd=tmp_path,
        timeout_seconds=30,
        feedback=lambda **_k: None,
        stream_progress=0.1,
    )

    assert calls == []
    assert (volume / 'board.uf2').read_bytes() == b'uf2-bytes'


def test_is_bootsel_volume_ignores_unrelated_mounts(tmp_path: Path):
    keys = tmp_path / 'keys'
    keys.mkdir()
    # Mode 0o750 + missing INFO_UF2 must not raise and must not match.
    assert pipeline_flash._is_bootsel_volume(keys) is False
    rp2 = tmp_path / 'RPI-RP2'
    rp2.mkdir()
    (rp2 / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    assert pipeline_flash._is_bootsel_volume(rp2) is True


def test_ensure_bootsel_mounts_uses_udisksctl(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    device = tmp_path / 'sdc1'
    device.write_text('', encoding='utf-8')

    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_block_devices', lambda: [device])
    monkeypatch.setattr(pipeline_flash, '_udisksctl_mount', lambda _dev: volume)

    mounts = pipeline_flash._ensure_bootsel_mounts(
        feedback=lambda **_k: None,
        stream_progress=0.1,
        board='rp2040_left_arm',
    )
    assert mounts == [volume]


def test_force_reflash_reboots_by_bus_then_copies_uf2(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    uf2 = tmp_path / 'board.uf2'
    uf2.write_bytes(b'uf2-bytes')
    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    calls: list[list[str]] = []
    loc = pipeline_flash._UsbLoc(
        bus=5, address=17, serial='E6617C93E3858429', product='Servo2040 Serial'
    )

    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_find_usb_loc_by_serial', lambda _s: loc)
    monkeypatch.setattr(
        type(loc),
        'writable',
        property(lambda self: True),
    )
    monkeypatch.setattr(pipeline_flash, '_picotool_bin', lambda: 'picotool')
    monkeypatch.setattr(
        pipeline_flash,
        '_run_command',
        lambda **kwargs: calls.append(kwargs['cmd']),
    )
    # After reboot, the volume appears.
    seen = {'n': 0}

    def ensure(**_k):
        seen['n'] += 1
        return [volume] if seen['n'] > 1 else []

    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', ensure)

    pipeline_flash._flash_uf2(
        uf2=uf2,
        configured_serial='E6617C93E3858429',
        board='rp2040_left_arm',
        cwd=tmp_path,
        timeout_seconds=30,
        feedback=lambda **_k: None,
        stream_progress=0.1,
    )

    assert len(calls) == 1
    assert calls[0][:2] == ['picotool', 'reboot']
    # Device selection before -f/-u (picotool v2 CLI requirement).
    bus_i = calls[0].index('--bus')
    force_i = calls[0].index('-f')
    assert bus_i < force_i
    assert calls[0][bus_i : bus_i + 2] == ['--bus', '5']
    assert '--address' in calls[0] and '17' in calls[0]
    assert '-u' in calls[0]
    assert 'sudo' not in calls[0]
    assert '--ser' not in calls[0]
    assert (volume / 'board.uf2').read_bytes() == b'uf2-bytes'


def test_force_reflash_errors_when_usb_not_writable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    uf2 = tmp_path / 'board.uf2'
    uf2.write_bytes(b'uf2')
    loc = pipeline_flash._UsbLoc(
        bus=5, address=17, serial='E6617C93E3858429', product='Servo2040 Serial'
    )
    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', lambda **_k: [])
    monkeypatch.setattr(pipeline_flash, '_find_usb_loc_by_serial', lambda _s: loc)
    monkeypatch.setattr(type(loc), 'writable', property(lambda self: False))
    monkeypatch.setattr(pipeline_flash, '_try_make_node_writable', lambda _n: False)

    def _cdc_fail(**_k):
        raise RuntimeError('no CDC tty')

    monkeypatch.setattr(pipeline_flash, '_serial_request_bootsel', _cdc_fail)

    with pytest.raises(RuntimeError, match='setup-udev'):
        pipeline_flash._flash_uf2(
            uf2=uf2,
            configured_serial='E6617C93E3858429',
            board='rp2040_left_arm',
            cwd=tmp_path,
            timeout_seconds=5,
            feedback=lambda **_k: None,
            stream_progress=0.1,
        )


def test_force_reflash_falls_back_to_cdc_when_usb_not_writable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    uf2 = tmp_path / 'board.uf2'
    uf2.write_bytes(b'uf2-bytes')
    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    loc = pipeline_flash._UsbLoc(
        bus=5, address=17, serial='E6617C93E3858429', product='Servo2040 Serial'
    )
    cdc_calls: list[str] = []

    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_find_usb_loc_by_serial', lambda _s: loc)
    monkeypatch.setattr(type(loc), 'writable', property(lambda self: False))
    monkeypatch.setattr(pipeline_flash, '_try_make_node_writable', lambda _n: False)

    def cdc(**kwargs):
        cdc_calls.append(kwargs['configured_serial'])

    monkeypatch.setattr(pipeline_flash, '_serial_request_bootsel', cdc)

    seen = {'n': 0}

    def ensure(**_k):
        seen['n'] += 1
        return [volume] if seen['n'] > 1 else []

    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', ensure)

    pipeline_flash._flash_uf2(
        uf2=uf2,
        configured_serial='E6617C93E3858429',
        board='rp2040_left_arm',
        cwd=tmp_path,
        timeout_seconds=30,
        feedback=lambda **_k: None,
        stream_progress=0.1,
    )

    assert cdc_calls == ['E6617C93E3858429']
    assert (volume / 'board.uf2').read_bytes() == b'uf2-bytes'


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
    monkeypatch.setattr(pipeline_flash, '_mcu_bridge_process_running', lambda *_a, **_k: False)
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


def test_flash_fails_when_bridge_link_stays_dead(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_BRIDGE_LINK_WAIT_SEC', '0.4')
    fw_src = tmp_path / 'fw'
    fw_build = fw_src / 'build'
    fw_build.mkdir(parents=True)
    # Cargo firmware path skips legacy uptime topic wait.
    (fw_src / 'firmwares' / 'rp2040').mkdir(parents=True)
    (fw_src / 'firmwares' / 'rp2040' / 'Cargo.toml').write_text('[package]\n', encoding='utf-8')
    uf2 = fw_build / 'pico_micro_ros_right_arm.uf2'
    uf2.write_bytes(b'uf2')

    volume = tmp_path / 'RPI-RP2'
    volume.mkdir()
    (volume / 'INFO_UF2.TXT').write_text('UF2\n', encoding='utf-8')
    loc = pipeline_flash._UsbLoc(
        bus=1, address=2, serial='E6617C93E37A6629', product='Servo2040 Serial'
    )
    monkeypatch.setattr(pipeline_flash, '_run_command', lambda **_k: None)
    monkeypatch.setattr(pipeline_flash, '_wait_for_usb_serial', lambda *_a, **_k: True)
    monkeypatch.setattr(pipeline_flash, '_mcu_bridge_process_running', lambda *_a, **_k: True)
    monkeypatch.setattr(pipeline_flash, '_wait_for_bridge_link', lambda *_a, **_k: False)
    monkeypatch.setattr(pipeline_flash, '_find_rpi_rp2_mounts', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_list_bootsel_serials', lambda: [])
    monkeypatch.setattr(pipeline_flash, '_picotool_bin', lambda: 'picotool')
    monkeypatch.setattr(pipeline_flash, '_find_usb_loc_by_serial', lambda _s: loc)
    monkeypatch.setattr(type(loc), 'writable', property(lambda self: True))
    seen = {'n': 0}

    def ensure(**_k):
        seen['n'] += 1
        return [volume] if seen['n'] > 1 else []

    monkeypatch.setattr(pipeline_flash, '_ensure_bootsel_mounts', ensure)

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
        log_error=lambda _msg: None,
    )

    assert failed == ['rp2040_right_arm']
    assert flashed == []
