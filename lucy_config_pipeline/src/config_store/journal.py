"""The unfinished calibration session a crash or restart might need to recover."""

from __future__ import annotations

import yaml

from .io import atomic_write_text
from .io import read_mapping


class ConfigStoreJournalMixin:
    def read_journal(self) -> dict:
        """Return the unfinished calibration session, empty if there is none."""
        return read_mapping(self.session_journal)

    def write_journal(self, journal: dict) -> None:
        self.ensure_layout()
        atomic_write_text(self.session_journal, yaml.safe_dump(journal, sort_keys=True))

    def clear_journal(self) -> None:
        self.session_journal.unlink(missing_ok=True)
