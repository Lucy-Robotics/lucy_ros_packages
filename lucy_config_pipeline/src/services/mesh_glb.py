from __future__ import annotations

from pathlib import Path

MESH_QUALITIES = frozenset({"full", "balanced", "light"})


def glb_beside(source: Path, quality: str) -> Path | None:
    if quality not in MESH_QUALITIES:
        return None
    glb = source.with_name(f"{source.name}.{quality}.glb")
    return glb if glb.is_file() else None
