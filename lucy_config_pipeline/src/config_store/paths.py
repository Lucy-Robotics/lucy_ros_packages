"""The file layout inside a config store's directory."""

from __future__ import annotations

from pathlib import Path


class ConfigStorePathsMixin:
    config_dir: Path

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

    def _named_path(self, config_name: str) -> Path:
        if not config_name or "/" in config_name or ".." in config_name:
            raise ValueError("config_name must be a simple non-empty name")
        return self.named_dir / f"{config_name}.yaml"
