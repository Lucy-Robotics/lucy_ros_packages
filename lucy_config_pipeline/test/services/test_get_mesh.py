from pathlib import Path
import struct
import zlib

import pytest
import rclpy
from src.config_store import ConfigStore
from src.services.config_services_node import (
    ConfigServicesNode,
    MESH_ENCODING_UTF8,
    MESH_ENCODING_ZLIB_BASE64,
)


@pytest.fixture
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def _write_binary_stl(path: Path, triangles: int = 2) -> bytes:
    """Minimal valid binary STL (80-byte header + count + 50 bytes/tri)."""
    header = b'\0' * 80
    body = b''
    for i in range(triangles):
        # normal (3f) + 3 vertices (9f) + attribute (H)
        body += struct.pack('<12fH', *([float(i)] * 12), 0)
    raw = header + struct.pack('<I', triangles) + body
    path.write_bytes(raw)
    return raw


class TestGetMesh:
    def test_stl_returns_zlib_base64(self, rclpy_init_shutdown, tmp_path: Path):
        from lucy_msgs.srv import GetMesh

        desc = tmp_path / 'description'
        mesh = desc / 'robot_description' / 'meshes' / 'stl' / 'part.stl'
        mesh.parent.mkdir(parents=True)
        raw = _write_binary_stl(mesh)

        node = ConfigServicesNode(
            robot_package='test_pkg',
            config_store=ConfigStore(tmp_path / 'hardware'),
            urdf_xacro=tmp_path / 'missing.urdf.xacro',
            base_path=desc,
            controller_config=tmp_path / 'controllers.yaml',
        )
        try:
            req = GetMesh.Request()
            req.path = mesh.resolve().as_uri()
            res = node._on_get_mesh(req, GetMesh.Response())
            assert res.success is True
            assert res.encoding == MESH_ENCODING_ZLIB_BASE64
            import base64
            inflated = zlib.decompress(base64.b64decode(res.data))
            assert inflated == raw
        finally:
            node.destroy_node()

    def test_dae_returns_utf8(self, rclpy_init_shutdown, tmp_path: Path):
        from lucy_msgs.srv import GetMesh

        desc = tmp_path / 'description'
        mesh = desc / 'meshes' / 'part.dae'
        mesh.parent.mkdir(parents=True)
        mesh.write_text('<COLLADA/>', encoding='utf-8')

        node = ConfigServicesNode(
            robot_package='test_pkg',
            config_store=ConfigStore(tmp_path / 'hardware'),
            urdf_xacro=tmp_path / 'missing.urdf.xacro',
            base_path=desc,
            controller_config=tmp_path / 'controllers.yaml',
        )
        try:
            req = GetMesh.Request()
            req.path = str(mesh.resolve())
            res = node._on_get_mesh(req, GetMesh.Response())
            assert res.success is True
            assert res.encoding == MESH_ENCODING_UTF8
            assert res.data == '<COLLADA/>'
        finally:
            node.destroy_node()

    def test_path_outside_description_rejected(self, rclpy_init_shutdown, tmp_path: Path):
        from lucy_msgs.srv import GetMesh

        desc = tmp_path / 'description'
        desc.mkdir()
        outside = tmp_path / 'secret.txt'
        outside.write_text('nope', encoding='utf-8')

        node = ConfigServicesNode(
            robot_package='test_pkg',
            config_store=ConfigStore(tmp_path / 'hardware'),
            urdf_xacro=tmp_path / 'missing.urdf.xacro',
            base_path=desc,
            controller_config=tmp_path / 'controllers.yaml',
        )
        try:
            req = GetMesh.Request()
            req.path = str(outside.resolve())
            res = node._on_get_mesh(req, GetMesh.Response())
            assert res.success is False
            assert 'outside' in res.message
        finally:
            node.destroy_node()
