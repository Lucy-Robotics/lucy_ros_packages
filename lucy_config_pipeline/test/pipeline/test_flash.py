from pathlib import Path

import pytest
from src.pipeline import flash as pipeline_flash


@pytest.fixture(autouse=True)
def _fake_picotool_on_path(monkeypatch: pytest.MonkeyPatch):
    """Flash phase requires picotool on PATH; tests do not invoke the real binary."""
    monkeypatch.setattr(
        pipeline_flash.shutil,
        'which',
        lambda name: '/usr/bin/picotool' if name == 'picotool' else None,
    )


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

    failed, flashed, _details = pipeline_flash.run_flash_phase(
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
    assert calls[0][:3] == ['picotool', 'load', str(uf2)]
    assert '-x' in calls[0]
    assert 'E6617C93E37A6629' in calls[0]


def test_run_flash_phase_uses_sudo_when_env_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC', '0')
    monkeypatch.setenv('LUCY_PIPELINE_FLASH_USE_SUDO', '1')
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

    failed, flashed, _details = pipeline_flash.run_flash_phase(
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

    assert failed == []
    assert flashed == ['rp2040_right_arm']
    assert calls[0][:5] == ['sudo', '-n', 'picotool', 'load', str(uf2)]
    assert '-x' in calls[0]
    assert '-f' in calls[0]
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

    failed, flashed, _details = pipeline_flash.run_flash_phase(
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

    failed, flashed, _details = pipeline_flash.run_flash_phase(
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

    failed, flashed, _details = pipeline_flash.run_flash_phase(
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
    assert any('Modbus' in d for d in _details)


def test_bootsel_volume_dev_macos(monkeypatch: pytest.MonkeyPatch):
    import sys

    monkeypatch.setattr(sys, 'platform', 'darwin')

    class FakePath:
        def __init__(self, *args, **_kwargs):
            self._s = str(args[0]) if args else ''

        def is_dir(self) -> bool:
            return self._s == '/Volumes/RPI-RP2'

        def __eq__(self, other: object) -> bool:
            return str(other) == self._s

        def __fspath__(self) -> str:
            return self._s

        def __str__(self) -> str:
            return self._s

    monkeypatch.setattr(pipeline_flash, 'Path', FakePath)
    assert pipeline_flash._bootsel_volume_dev() == FakePath('/Volumes/RPI-RP2')


def test_bootsel_volume_dev_windows(monkeypatch: pytest.MonkeyPatch):
    import sys

    monkeypatch.setattr(sys, 'platform', 'win32')
    monkeypatch.setattr(
        pipeline_flash,
        '_windows_bootsel_mount',
        lambda: Path('E:/'),
    )
    assert pipeline_flash._bootsel_volume_dev() == Path('E:/')


def test_find_mount_point_for_macos_and_windows_are_passthrough(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    import sys

    monkeypatch.setattr(sys, 'platform', 'darwin')
    assert pipeline_flash._find_mount_point_for(tmp_path) == tmp_path
    monkeypatch.setattr(sys, 'platform', 'win32')
    assert pipeline_flash._find_mount_point_for(tmp_path) == tmp_path


def test_copy_uf2_to_bootsel_volume_windows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    import sys

    monkeypatch.setattr(sys, 'platform', 'win32')
    mount = tmp_path / 'E'
    mount.mkdir()
    uf2 = tmp_path / 'board.uf2'
    uf2.write_bytes(b'uf2')
    monkeypatch.setattr(pipeline_flash, '_bootsel_volume_dev', lambda: mount)
    details: list[str] = []

    ok = pipeline_flash._copy_uf2_to_bootsel_volume(
        uf2,
        feedback=lambda **kw: details.append(kw.get('detail', '')),
        board='rp2040_test',
    )
    assert ok is True
    assert (mount / 'board.uf2').read_bytes() == b'uf2'


def test_wait_for_usb_serial_uses_pyserial_on_macos(monkeypatch: pytest.MonkeyPatch):
    import sys
    from types import SimpleNamespace

    monkeypatch.setattr(sys, 'platform', 'darwin')
    calls = {'n': 0}

    def fake_monotonic():
        calls['n'] += 1
        return 0.0 if calls['n'] < 3 else 10.0

    monkeypatch.setattr(pipeline_flash.time, 'monotonic', fake_monotonic)
    monkeypatch.setattr(pipeline_flash.time, 'sleep', lambda _s: None)

    ports = [
        SimpleNamespace(
            device='/dev/tty.usbmodem123',
            serial_number='E6617C93E37A6629',
            description='Pico',
            hwid='USB VID:PID=2E8A:000A SER=E6617C93E37A6629',
        )
    ]

    class FakeSerialTools:
        class list_ports:
            @staticmethod
            def comports():
                return ports

    import sys as _sys

    monkeypatch.setitem(_sys.modules, 'serial', FakeSerialTools)
    monkeypatch.setitem(_sys.modules, 'serial.tools', FakeSerialTools)
    monkeypatch.setitem(_sys.modules, 'serial.tools.list_ports', FakeSerialTools.list_ports)

    assert pipeline_flash._wait_for_usb_serial('E6617C93E37A6629', 1) is True


def test_wait_for_usb_serial_uses_by_id_on_linux(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    import sys

    monkeypatch.setattr(sys, 'platform', 'linux')
    by_id = tmp_path / 'serial' / 'by-id'
    by_id.mkdir(parents=True)
    target = tmp_path / 'ttyACM0'
    target.write_text('')
    (by_id / 'usb-Raspberry_Pi_Pico_E6617C93E37A6629-if00').symlink_to(target)

    calls = {'n': 0}

    def fake_monotonic():
        calls['n'] += 1
        return 0.0 if calls['n'] < 3 else 10.0

    monkeypatch.setattr(pipeline_flash.time, 'monotonic', fake_monotonic)
    monkeypatch.setattr(pipeline_flash.time, 'sleep', lambda _s: None)

    real_path = pipeline_flash.Path

    def path_factory(arg, *args, **kwargs):
        if str(arg) == '/dev/serial/by-id':
            return by_id
        return real_path(arg, *args, **kwargs)

    monkeypatch.setattr(pipeline_flash, 'Path', path_factory)
    # Force ImportError on pyserial so only by-id can succeed.
    import builtins

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name.startswith('serial'):
            raise ImportError('no serial in test')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', fake_import)
    assert pipeline_flash._wait_for_usb_serial('E6617C93E37A6629', 1) is True
