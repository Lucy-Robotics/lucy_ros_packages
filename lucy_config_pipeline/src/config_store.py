from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import shutil

import yaml


@dataclass(frozen=True)
class ConfigStore:
    config_dir: Path

    def __post_init__(self) -> None:
        object.__setattr__(self, 'config_dir', self.config_dir.resolve())

    @property
    def active_yaml(self) -> Path:
        return self.config_dir / 'active.yaml'

    @property
    def active_meta_yaml(self) -> Path:
        return self.config_dir / 'active_meta.yaml'

    @property
    def named_dir(self) -> Path:
        return self.config_dir / 'configs'

    @property
    def backups_dir(self) -> Path:
        return self.config_dir / 'backups'

    def ensure_layout(self) -> None:
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.named_dir.mkdir(parents=True, exist_ok=True)
        self.backups_dir.mkdir(parents=True, exist_ok=True)

    def list_configs(self) -> list[str]:
        self.ensure_layout()
        return sorted(p.stem for p in self.named_dir.glob('*.yaml'))

    def read_active_meta(self) -> dict:
        if not self.active_meta_yaml.is_file():
            return {}
        raw = self.active_meta_yaml.read_text(encoding='utf-8').strip()
        if not raw:
            return {}
        data = yaml.safe_load(raw)
        return data if isinstance(data, dict) else {}

    def _write_active_meta(self, meta: dict) -> None:
        """Persist known keys only; preserves stable key order matching checked-in YAML style."""
        self.ensure_layout()
        lines: list[str] = []
        for key in ('name', 'activated_at', 'flashed_name', 'flashed_at'):
            val = str(meta.get(key, '')).strip()
            if val:
                lines.append(f'{key}: "{val}"')
        self.active_meta_yaml.write_text(
            '\n'.join(lines) + ('\n' if lines else ''),
            encoding='utf-8',
        )

    def get_active_name(self) -> str:
        data = self.read_active_meta()
        name = str(data.get('name', '')).strip()
        return name if name else 'default'

    def get_flashed_name(self) -> str:
        return str(self.read_active_meta().get('flashed_name', '')).strip()

    def get_flashed_at(self) -> str:
        return str(self.read_active_meta().get('flashed_at', '')).strip()

    def record_flashed_preset(self, config_name: str) -> None:
        meta = self.read_active_meta()
        meta['flashed_name'] = config_name
        meta['flashed_at'] = f'{datetime.utcnow().isoformat()}Z'
        self._write_active_meta(meta)

    def _named_path(self, config_name: str) -> Path:
        if not config_name or '/' in config_name or '..' in config_name:
            raise ValueError('config_name must be a simple non-empty name')
        return self.named_dir / f'{config_name}.yaml'

    def read_named_yaml(self, config_name: str) -> str:
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        return path.read_text(encoding='utf-8')

    def read_active_yaml(self) -> str:
        if not self.active_yaml.is_file():
            raise FileNotFoundError(self.active_yaml)
        return self.active_yaml.read_text(encoding='utf-8')

    def write_named_yaml(self, config_name: str, config_yaml: str) -> Path:
        self.ensure_layout()
        path = self._named_path(config_name)
        path.write_text(config_yaml, encoding='utf-8')
        return path

    def activate(self, config_name: str) -> str:
        self.ensure_layout()
        src = self._named_path(config_name)
        if not src.is_file():
            raise FileNotFoundError(src)

        backup_name = ''
        if self.active_yaml.is_file():
            old = self.get_active_name()
            ts = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
            backup_name = f"{ts}_{old or 'active'}"
            shutil.copy2(self.active_yaml, self.backups_dir / f'{backup_name}.yaml')

        shutil.copy2(src, self.active_yaml)
        meta = self.read_active_meta()
        meta['name'] = config_name
        meta['activated_at'] = f'{datetime.utcnow().isoformat()}Z'
        self._write_active_meta(meta)
        return backup_name

    def delete(self, config_name: str) -> None:
        active = self.get_active_name()
        if config_name in {'default', active}:
            raise ValueError('cannot delete default or active config')
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        path.unlink()
