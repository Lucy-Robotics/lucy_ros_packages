from __future__ import annotations

import hashlib
from pathlib import Path

MESH_CACHE_DIR = Path.home() / ".cache" / "lucy" / "meshes"
MESH_QUALITIES = frozenset({"full", "balanced", "light"})


def cached_glb(
    source: Path, quality: str, cache_dir: Path = MESH_CACHE_DIR
) -> Path | None:
    if quality not in MESH_QUALITIES:
        return None
    key = hashlib.sha1(str(source).encode()).hexdigest()[:20]
    glb = cache_dir / f"{key}.{quality}.glb"
    try:
        return glb if glb.stat().st_mtime >= source.stat().st_mtime else None
    except OSError:
        return None
