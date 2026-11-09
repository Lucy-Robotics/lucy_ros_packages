"""Commands that only make sense before a calibration session has started."""

from __future__ import annotations

from ..identity import valid_serial
from .constants import IDLE
from .constants import SessionError
from .constants import TEMPLATE_PRESET


class SessionIdleCommandsMixin:
    """Bind an already-calibrated preset to this arm, or pick its USB adapter."""

    def _adopt(self, _argument: str) -> None:
        """Vouch that the active preset fits this arm, calibrated some other way."""
        self._need_phase(IDLE)
        serial = (self._firmware or {}).get("open_serial")
        if not valid_serial(serial):
            raise SessionError(
                "bad_serial", "This USB adapter has no usable serial number."
            )
        name = self._store.get_active_name()
        if name == TEMPLATE_PRESET:
            raise SessionError(
                "not_calibrated", "The stock preset is not calibrated for any arm."
            )
        self._store.bind_adapter(serial, name)
        self._message = f"{name} is now this arm's preset."

    def _select_adapter(self, serial: str) -> None:
        self._need_phase(IDLE)
        if not valid_serial(serial):
            raise SessionError("bad_serial", "Not a usable adapter serial.")
        self._call(self._call_firmware(self._backend.select, serial))
        self._store.select_adapter(serial)
