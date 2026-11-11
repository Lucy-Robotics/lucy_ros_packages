"""Keeps real hardware gated until the arm is known to fit the active preset."""

from __future__ import annotations

from collections.abc import Callable

from ..calibration.firmware_client import FirmwareBackend
from ..calibration.firmware_client import FirmwareUnreachable
from ..calibration.identity import Identity
from ..calibration.session import IDLE
from ..calibration.sim_backend import SimBackend
from ..config_store import ConfigStore
from .registry_lock import RegistryLock


class ControlGate:
    def __init__(
        self,
        *,
        real: bool,
        backend: FirmwareBackend | SimBackend,
        lock: RegistryLock,
        store: ConfigStore,
        identity_fn: Callable[[], Identity],
    ) -> None:
        self._real = real
        self._backend = backend
        self._lock = lock
        self._store = store
        self._identity_fn = identity_fn
        self._selected_sent = False

    def restore_selection(self) -> None:
        """Tell a firmware that just started which of several adapters was chosen."""
        if self._selected_sent:
            return
        selected = self._store.selected_adapter()
        try:
            if selected:
                self._backend.select(selected)
            self._selected_sent = True
        except FirmwareUnreachable:
            pass

    def update(self, phase: str) -> None:
        if not self._real or phase != IDLE:
            return
        if self._identity_fn().control_allowed:
            self._lock.release()
        else:
            self._lock.acquire()
