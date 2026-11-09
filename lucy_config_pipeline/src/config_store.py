from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import shutil
import tempfile

import yaml


def atomic_write_text(path: Path, text: str) -> None:
    """Replace ``path`` in one step, so a crash leaves the old content or the new."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as out:
            out.write(text)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


@dataclass(frozen=True)
class ConfigStore:
    config_dir: Path

    def __post_init__(self) -> None:
        object.__setattr__(self, "config_dir", self.config_dir.resolve())

    @property
    def active_yaml(self) -> Path:
        return self.config_dir / "active.yaml"

    @property
    def active_meta_yaml(self) -> Path:
        return self.config_dir / "active_meta.yaml"

    @property
    def named_dir(self) -> Path:
        return self.config_dir / "configs"

    @property
    def backups_dir(self) -> Path:
        return self.config_dir / "backups"

    @property
    def bindings_yaml(self) -> Path:
        return self.config_dir / "adapter_bindings.yaml"

    @property
    def session_journal(self) -> Path:
        return self.config_dir / ".calibration_session.yaml"

    @property
    def calibration_profile_yaml(self) -> Path:
        return self.config_dir / "calibration_profile.yaml"

    def lerobot_json_path(self, config_name: str) -> Path:
        return self._named_path(config_name).with_suffix(".lerobot.json")

    def ensure_layout(self) -> None:
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.named_dir.mkdir(parents=True, exist_ok=True)
        self.backups_dir.mkdir(parents=True, exist_ok=True)

    def list_configs(self) -> list[str]:
        self.ensure_layout()
        return sorted(p.stem for p in self.named_dir.glob("*.yaml"))

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

    def get_active_name(self) -> str:
        data = self.read_active_meta()
        name = str(data.get("name", "")).strip()
        return name if name else "default"

    def get_flashed_name(self) -> str:
        return str(self.read_active_meta().get("flashed_name", "")).strip()

    def get_flashed_at(self) -> str:
        return str(self.read_active_meta().get("flashed_at", "")).strip()

    def record_flashed_preset(self, config_name: str) -> None:
        meta = self.read_active_meta()
        meta["flashed_name"] = config_name
        meta["flashed_at"] = f"{datetime.utcnow().isoformat()}Z"
        self._write_active_meta(meta)

    def _named_path(self, config_name: str) -> Path:
        if not config_name or "/" in config_name or ".." in config_name:
            raise ValueError("config_name must be a simple non-empty name")
        return self.named_dir / f"{config_name}.yaml"

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

    def _read_mapping(self, path: Path) -> dict:
        if not path.is_file():
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError:
            return {}
        return data if isinstance(data, dict) else {}

    def read_bindings(self) -> dict:
        """Which preset each USB adapter last activated, plus any chosen adapter."""
        return self._read_mapping(self.bindings_yaml)

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

    def read_journal(self) -> dict:
        """Return the unfinished calibration session, empty if there is none."""
        return self._read_mapping(self.session_journal)

    def write_journal(self, journal: dict) -> None:
        self.ensure_layout()
        atomic_write_text(self.session_journal, yaml.safe_dump(journal, sort_keys=True))

    def clear_journal(self) -> None:
        self.session_journal.unlink(missing_ok=True)

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
