import base64
import hashlib
import os
from pathlib import Path

import pytest
import rclpy
from src.config_store import ConfigStore
from src.services import config_services_node as csn
from src.services.mesh_glb_cache import cached_glb

GLB = b"glTF\x02\x00\x00\x00binary\xff\x00payload"


@pytest.fixture
def env(tmp_path: Path, monkeypatch):
    from lucy_msgs.srv import GetMesh

    cache = tmp_path / "cache"
    cache.mkdir()
    real = csn.cached_glb
    monkeypatch.setattr(csn, "cached_glb", lambda s, q: real(s, q, cache))
    desc = tmp_path / "description"
    mesh = desc / "meshes" / "part.dae"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("<COLLADA/>", encoding="utf-8")
    rclpy.init()
    node = csn.ConfigServicesNode(
        robot_package="test_pkg",
        config_store=ConfigStore(tmp_path / "hardware"),
        urdf_xacro=tmp_path / "missing.urdf.xacro",
        base_path=desc,
        controller_config=tmp_path / "controllers.yaml",
    )

    def get(path: Path, quality: str):
        req = GetMesh.Request()
        req.path = str(path)
        req.quality = quality
        return node._on_get_mesh(req, GetMesh.Response())

    def put_glb(source: Path, quality: str, mtime: float | None = None) -> Path:
        glb = (
            cache
            / f"{hashlib.sha1(str(source).encode()).hexdigest()[:20]}.{quality}.glb"
        )
        glb.write_bytes(GLB)
        if mtime is not None:
            os.utime(glb, (mtime, mtime))
        return glb

    yield mesh.resolve(), cache, get, put_glb
    node.destroy_node()
    rclpy.shutdown()


def _served_source(res):
    return (
        res.success
        and res.encoding == csn.MESH_ENCODING_UTF8
        and res.data == "<COLLADA/>"
    )


@pytest.mark.parametrize("quality", ["full", "balanced", "light"])
def test_fresh_cached_glb_is_served_as_glb_base64(env, quality):
    mesh, _, get, put_glb = env
    put_glb(mesh, quality)
    res = get(mesh, quality)
    assert res.success and res.encoding == "glb_base64"
    assert base64.b64decode(res.data) == GLB


def test_glb_older_than_source_mesh_is_ignored(env):
    mesh, _, get, put_glb = env
    put_glb(mesh, "full", mtime=mesh.stat().st_mtime - 100)
    assert _served_source(get(mesh, "full"))


def test_glb_as_new_as_source_mesh_is_served(env):
    mesh, _, get, put_glb = env
    put_glb(mesh, "full", mtime=mesh.stat().st_mtime)
    assert get(mesh, "full").encoding == "glb_base64"


def test_missing_glb_serves_the_source(env):
    mesh, _, get, _ = env
    assert _served_source(get(mesh, "full"))


@pytest.mark.parametrize("quality", ["", "ultra", "FULL", "../x", "full/../light"])
def test_unknown_or_empty_quality_serves_the_source(env, quality):
    mesh, cache, get, put_glb = env
    put_glb(mesh, "full")
    put_glb(mesh, "balanced")
    put_glb(mesh, "light")
    if "/" not in quality:
        put_glb(mesh, quality)
    (cache / "x.glb").write_bytes(GLB)
    assert _served_source(get(mesh, quality))


def test_path_outside_description_is_refused_even_with_cached_glb(env, tmp_path):
    _, _, get, put_glb = env
    outside = tmp_path / "secret.dae"
    outside.write_text("nope", encoding="utf-8")
    put_glb(outside.resolve(), "full")
    res = get(outside.resolve(), "full")
    assert res.success is False and "outside" in res.message and res.data == ""


def test_missing_source_is_not_found_even_with_cached_glb(env):
    mesh, _, get, put_glb = env
    ghost = mesh.parent / "ghost.dae"
    put_glb(ghost, "full")
    res = get(ghost, "full")
    assert res.success is False and "not found" in res.message


def test_cache_file_name_matches_the_panel_key_for_a_fixed_path(tmp_path):
    class Source:
        def __str__(self):
            return "/opt/robot/meshes/arm.stl"

        def stat(self):
            return os.stat_result((0,) * 10)

    expected = tmp_path / "999073b40db2f90f9b6a.balanced.glb"
    assert cached_glb(Source(), "balanced", tmp_path) is None
    expected.write_bytes(GLB)
    assert cached_glb(Source(), "balanced", tmp_path) == expected
