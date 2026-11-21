import base64
from pathlib import Path
import zlib

import pytest
import rclpy
from src.config_store import ConfigStore
from src.services import config_services_node as csn

GLB = b"glTF\x02\x00\x00\x00binary\xff\x00payload"
STL = b"solid part\nendsolid part\n"


@pytest.fixture
def env(tmp_path: Path):
    from lucy_msgs.srv import GetMesh

    desc = tmp_path / "description"
    mesh = desc / "meshes" / "part.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(STL)
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

    def put_glb(name: str, folder: Path = mesh.parent) -> Path:
        glb = folder / name
        glb.parent.mkdir(parents=True, exist_ok=True)
        glb.write_bytes(GLB)
        return glb

    yield mesh.resolve(), get, put_glb
    node.destroy_node()
    rclpy.shutdown()


def _served_source(res):
    return (
        res.success
        and res.encoding == csn.MESH_ENCODING_ZLIB_BASE64
        and zlib.decompress(base64.b64decode(res.data)) == STL
    )


@pytest.mark.parametrize("quality", ["full", "balanced", "light"])
def test_glb_beside_the_mesh_is_served_as_glb_base64(env, quality):
    mesh, get, put_glb = env
    put_glb(f"part.stl.{quality}.glb")
    res = get(mesh, quality)
    assert res.success and res.encoding == "glb_base64"
    assert base64.b64decode(res.data) == GLB


def test_missing_sibling_glb_serves_the_source(env):
    mesh, get, _ = env
    assert _served_source(get(mesh, "full"))


def test_only_the_requested_quality_is_served(env):
    mesh, get, put_glb = env
    put_glb("part.stl.light.glb")
    assert _served_source(get(mesh, "full"))


@pytest.mark.parametrize("quality", ["", "ultra", "FULL", "../x", "full/../light"])
def test_unknown_or_empty_quality_serves_the_source(env, quality):
    mesh, get, put_glb = env
    for name in ("full", "balanced", "light"):
        put_glb(f"part.stl.{name}.glb")
    put_glb(f"part.stl.{quality}.glb")
    put_glb("x.glb", mesh.parent)
    assert _served_source(get(mesh, quality))


def test_path_outside_description_is_refused_even_with_sibling_glb(env, tmp_path):
    _, get, put_glb = env
    outside = tmp_path / "secret.stl"
    outside.write_bytes(STL)
    put_glb("secret.stl.full.glb", tmp_path)
    res = get(outside.resolve(), "full")
    assert res.success is False and "outside" in res.message and res.data == ""


def test_missing_source_is_not_found_even_with_sibling_glb(env):
    mesh, get, put_glb = env
    put_glb("ghost.stl.full.glb")
    res = get(mesh.parent / "ghost.stl", "full")
    assert res.success is False and "not found" in res.message


@pytest.mark.parametrize("name", ["other.stl.full.glb", "part.dae.full.glb"])
def test_glb_beside_a_different_mesh_is_not_served(env, name):
    mesh, get, put_glb = env
    put_glb(name)
    assert _served_source(get(mesh, "full"))
