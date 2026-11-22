"""Which preset each USB adapter last activated, and which adapter is selected."""

from __future__ import annotations

from datetime import datetime

import yaml

from .io import atomic_write_text
from .io import read_mapping


class ConfigStoreBindingsMixin:
    def read_bindings(self) -> dict:
        """Which preset each USB adapter last activated, plus any chosen adapter."""
        return read_mapping(self.bindings_yaml)

    def _write_bindings(self, bindings: dict) -> None:
        self.ensure_layout()
        atomic_write_text(self.bindings_yaml, yaml.safe_dump(bindings, sort_keys=True))

    def bound_config(self, adapter_serial: str) -> str:
        """Return the preset last activated for this adapter, or ''."""
        adapters = self.read_bindings().get("adapters") or {}
        entry = adapters.get(adapter_serial) or {}
        return str(entry.get("config_name") or "")

    def bind_adapter(self, adapter_serial: str, config_name: str) -> None:
        bindings = self.read_bindings()
        bindings.setdefault("adapters", {})[adapter_serial] = {
            "config_name": config_name,
            "activated_at": f"{datetime.utcnow().isoformat()}Z",
        }
        self._write_bindings(bindings)

    def selected_adapter(self) -> str:
        """Return the adapter chosen when several are connected, or ''."""
        return str(self.read_bindings().get("selected_adapter") or "")

    def select_adapter(self, adapter_serial: str) -> None:
        bindings = self.read_bindings()
        bindings["selected_adapter"] = adapter_serial
        self._write_bindings(bindings)
