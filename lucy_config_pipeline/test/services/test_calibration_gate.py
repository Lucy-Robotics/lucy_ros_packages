from src.calibration.firmware_client import FirmwareUnreachable
from src.calibration.identity import Identity
from src.calibration.session import IDLE
from src.services.calibration_gate import ControlGate


class FakeBackend:
    def __init__(self, unreachable: bool = False) -> None:
        self.unreachable = unreachable
        self.selected: list[str] = []

    def select(self, serial: str) -> None:
        if self.unreachable:
            raise FirmwareUnreachable("offline")
        self.selected.append(serial)


class FakeLock:
    def __init__(self) -> None:
        self.acquired = 0
        self.released = 0

    def acquire(self) -> bool:
        self.acquired += 1
        return True

    def release(self) -> bool:
        self.released += 1
        return True


class FakeStore:
    def __init__(self, selected: str = "") -> None:
        self._selected = selected

    def selected_adapter(self) -> str:
        return self._selected


def make_gate(*, real=True, backend=None, lock=None, store=None, control_allowed=True):
    return ControlGate(
        real=real,
        backend=backend or FakeBackend(),
        lock=lock or FakeLock(),
        store=store or FakeStore(),
        identity_fn=lambda: Identity(state="ok", control_allowed=control_allowed),
    )


def test_restore_selection_sends_the_stored_adapter_once():
    backend = FakeBackend()
    gate = make_gate(backend=backend, store=FakeStore(selected="FT1234"))

    gate.restore_selection()
    gate.restore_selection()

    assert backend.selected == ["FT1234"]


def test_restore_selection_with_no_stored_adapter_does_nothing():
    backend = FakeBackend()
    gate = make_gate(backend=backend, store=FakeStore(selected=""))

    gate.restore_selection()

    assert backend.selected == []


def test_restore_selection_retries_after_firmware_unreachable():
    backend = FakeBackend(unreachable=True)
    gate = make_gate(backend=backend, store=FakeStore(selected="FT1234"))

    gate.restore_selection()
    backend.unreachable = False
    gate.restore_selection()

    assert backend.selected == ["FT1234"]


def test_update_does_nothing_in_simulation():
    lock = FakeLock()
    gate = make_gate(real=False, lock=lock, control_allowed=False)

    gate.update(IDLE)

    assert lock.acquired == 0
    assert lock.released == 0


def test_update_does_nothing_outside_idle():
    lock = FakeLock()
    gate = make_gate(real=True, lock=lock, control_allowed=False)

    gate.update("joints")

    assert lock.acquired == 0
    assert lock.released == 0


def test_update_releases_the_lock_when_control_is_allowed():
    lock = FakeLock()
    gate = make_gate(real=True, lock=lock, control_allowed=True)

    gate.update(IDLE)

    assert lock.released == 1
    assert lock.acquired == 0


def test_update_acquires_the_lock_when_control_is_not_allowed():
    lock = FakeLock()
    gate = make_gate(real=True, lock=lock, control_allowed=False)

    gate.update(IDLE)

    assert lock.acquired == 1
    assert lock.released == 0
