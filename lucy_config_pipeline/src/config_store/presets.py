"""Reading, writing, activating and deleting the named presets in a config store."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import shutil

import yaml

from .io import atomic_write_text


class ConfigStorePresetsMixin:
    def list_configs(self) -> list[str]:
        self.ensure_layout()
        return sorted(p.stem for p in self.named_dir.glob("*.yaml"))

    def read_named_yaml(self, config_name: str) -> str:
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        return path.read_text(encoding="utf-8")

    def read_active_yaml(self) -> str:
        if not self.active_yaml.is_file():
            raise FileNotFoundError(self.active_yaml)
        return self.active_yaml.read_text(encoding="utf-8")

    def write_named_yaml(self, config_name: str, config_yaml: str) -> Path:
        self.ensure_layout()
        path = self._named_path(config_name)
        atomic_write_text(path, config_yaml)
        return path

    def read_calibration_block(self, config_name: str) -> dict:
        """Return the preset's ``calibration:`` block, empty if it has none or can't be read."""
        try:
            data = yaml.safe_load(self.read_named_yaml(config_name))
        except (OSError, ValueError, yaml.YAMLError):
            return {}
        block = data.get("calibration") if isinstance(data, dict) else None
        return block if isinstance(block, dict) else {}

    def activate(self, config_name: str) -> str:
        self.ensure_layout()
        src = self._named_path(config_name)
        if not src.is_file():
            raise FileNotFoundError(src)

        backup_name = ""
        if self.active_yaml.is_file():
            old = self.get_active_name()
            ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            backup_name = f"{ts}_{old or 'active'}"
            shutil.copy2(self.active_yaml, self.backups_dir / f"{backup_name}.yaml")

        atomic_write_text(self.active_yaml, src.read_text(encoding="utf-8"))
        meta = self.read_active_meta()
        meta["name"] = config_name
        meta["activated_at"] = f"{datetime.utcnow().isoformat()}Z"
        self._write_active_meta(meta)

        adapter_serial = str(
            self.read_calibration_block(config_name).get("adapter_serial") or ""
        )
        if adapter_serial:
            self.bind_adapter(adapter_serial, config_name)
        return backup_name

    def delete(self, config_name: str) -> None:
        active = self.get_active_name()
        if config_name in {"default", active}:
            raise ValueError("cannot delete default or active config")
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        path.unlink()
        self.lerobot_json_path(config_name).unlink(missing_ok=True)
