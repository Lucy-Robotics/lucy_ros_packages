"""Metadata about whichever preset is currently active: its name and timestamps."""

from __future__ import annotations

from datetime import datetime

import yaml

from .io import atomic_write_text


class ConfigStoreActiveMetaMixin:
    def read_active_meta(self) -> dict:
        if not self.active_meta_yaml.is_file():
            return {}
        raw = self.active_meta_yaml.read_text(encoding="utf-8").strip()
        if not raw:
            return {}
        data = yaml.safe_load(raw)
        return data if isinstance(data, dict) else {}

    def _write_active_meta(self, meta: dict) -> None:
        """Persist known keys only; preserves stable key order matching checked-in YAML style."""
        self.ensure_layout()
        lines: list[str] = []
        for key in ("name", "activated_at", "flashed_name", "flashed_at"):
            val = str(meta.get(key, "")).strip()
            if val:
                lines.append(f'{key}: "{val}"')
        atomic_write_text(
            self.active_meta_yaml, "\n".join(lines) + ("\n" if lines else "")
        )

    def _active_field(self, key: str) -> str:
        return str(self.read_active_meta().get(key, "")).strip()

    def get_active_name(self) -> str:
        return self._active_field("name") or "default"

    def get_flashed_name(self) -> str:
        return self._active_field("flashed_name")

    def get_flashed_at(self) -> str:
        return self._active_field("flashed_at")

    def _update_active_meta(self, **fields: str) -> None:
        meta = self.read_active_meta()
        meta.update(fields)
        self._write_active_meta(meta)

    def record_flashed_preset(self, config_name: str) -> None:
        self._update_active_meta(
            flashed_name=config_name, flashed_at=f"{datetime.utcnow().isoformat()}Z"
        )
