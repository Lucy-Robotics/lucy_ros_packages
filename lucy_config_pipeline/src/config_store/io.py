"""Reading and writing the YAML/text files a config store is made of."""

from __future__ import annotations

import os
from pathlib import Path
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


def read_mapping(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError:
        return {}
    return data if isinstance(data, dict) else {}
