"""The calibration node over ROS: requests must be served while it polls a slow firmware."""

from __future__ import annotations

import threading
import time

from lucy_msgs.srv import CalibrationCommand
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from src.calibration.sim_backend import SimBackend  # noqa: E402
from src.services import calibration_node  # noqa: E402

# The real firmware answers its socket once per 50 ms control tick.
FIRMWARE_TICK_S = 0.05
REPLY_TIMEOUT_S = 10.0


class SlowFirmware(SimBackend):
    """The simulated arm, answering at the real firmware's pace."""

    def __init__(self, _node: str) -> None:
        super().__init__([1, 2, 3, 4, 5, 6])

    @staticmethod
    def _tick() -> None:
        time.sleep(FIRMWARE_TICK_S - time.monotonic() % FIRMWARE_TICK_S)

    def status(self) -> dict:
        self._tick()
        return super().status()

    def read_homing(self) -> dict:
        self._tick()
        return super().read_homing()

    def set_half_turn(self, serial: str) -> dict:
        self._tick()
        return super().set_half_turn(serial)


class AlwaysLock:
    def __init__(self, *_args) -> None:
        pass

    def acquire(self) -> bool:
        return True

    def release(self) -> bool:
        return True


@pytest.fixture
def node(monkeypatch, store, tmp_path):
    profile = store.calibration_profile_yaml
    profile.write_text(
        "host_node: test\njoints:\n"
        + "".join(
            f"  - {{name: J{i}, motor: m{i}, id: {i}, min_span: 10}}\n"
            for i in range(1, 7)
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(calibration_node, "FirmwareBackend", SlowFirmware)
    monkeypatch.setattr(calibration_node, "RegistryLock", AlwaysLock)
    paths = type("P", (), {"config_dir": store.config_dir})
    monkeypatch.setattr(calibration_node, "_resolve_paths", lambda *_a: paths)
    rclpy.init()
    n = calibration_node.CalibrationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(n)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    yield n
    executor.shutdown()
    n.destroy_node()
    rclpy.shutdown()


def call(node, command: str) -> CalibrationCommand.Response:
    client = node.create_client(CalibrationCommand, calibration_node.COMMAND_SERVICE)
    assert client.wait_for_service(timeout_sec=5)
    done = threading.Event()
    future = client.call_async(CalibrationCommand.Request(command=command))
    future.add_done_callback(lambda _f: done.set())
    assert done.wait(
        REPLY_TIMEOUT_S
    ), f"{command} got no reply: requests are being starved"
    return future.result()


def test_commands_are_answered_while_polling_a_slow_firmware(node, store):
    assert call(node, "start").success
    time.sleep(1.0)  # Let the poller run flat out, as it does during a session.
    started = time.monotonic()
    reply = call(node, "capture_mid")
    assert reply.success, reply.message
    assert time.monotonic() - started < 3.0
    assert store.read_journal()["new"]
