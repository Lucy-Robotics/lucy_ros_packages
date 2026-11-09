"""Where calibration presets, bindings and session state live on disk."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .active_meta import ConfigStoreActiveMetaMixin
from .bindings import ConfigStoreBindingsMixin
from .io import atomic_write_text
from .journal import ConfigStoreJournalMixin
from .paths import ConfigStorePathsMixin
from .presets import ConfigStorePresetsMixin

__all__ = ["ConfigStore", "atomic_write_text"]


@dataclass(frozen=True)
class ConfigStore(
    ConfigStorePathsMixin,
    ConfigStoreActiveMetaMixin,
    ConfigStoreBindingsMixin,
    ConfigStoreJournalMixin,
    ConfigStorePresetsMixin,
):
    config_dir: Path

    def __post_init__(self) -> None:
        object.__setattr__(self, "config_dir", self.config_dir.resolve())
