from __future__ import annotations

import base64
from pathlib import Path
import zlib

from ament_index_python.packages import get_package_share_directory
from lucy_msgs.srv import ActivateConfig
from lucy_msgs.srv import DeleteConfig
from lucy_msgs.srv import GetConfig
from lucy_msgs.srv import GetMesh
from lucy_msgs.srv import ListConfigs
from lucy_msgs.srv import SaveConfig
import rclpy
from rclpy.node import Node

from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import urdf_crosscheck
from ..validation import validate_schema

# Wire encodings for GetMesh.data (must match lucy_msgs/srv/GetMesh.srv comments).
MESH_ENCODING_UTF8 = 'utf8'
MESH_ENCODING_ZLIB_BASE64 = 'zlib_base64'


class ConfigServicesNode(Node):
    def __init__(
        self,
        *,
        robot_package: str,
        config_store: ConfigStore,
        urdf_xacro: Path,
        base_path: Path,
        controller_config: Path,
    ):
        super().__init__('lucy_config_services')
        self._robot_package = robot_package
        self._store = config_store
        self._urdf_xacro = urdf_xacro
        self._base_path = base_path
        self._controller_config = controller_config

        self.create_service(ListConfigs, 'config/list', self._on_list_configs)
        self.create_service(GetConfig, 'config/get', self._on_get_config)
        self.create_service(SaveConfig, 'config/save', self._on_save_config)
        self.create_service(ActivateConfig, 'config/activate', self._on_activate_config)
        self.create_service(DeleteConfig, 'config/delete', self._on_delete_config)
        self.create_service(GetMesh, 'mesh/get', self._on_get_mesh)

    def _on_list_configs(
        self, _req: ListConfigs.Request, res: ListConfigs.Response
    ) -> ListConfigs.Response:
        try:
            res.config_names = self._store.list_configs()
            res.active_config = self._store.get_active_name()
            res.success = True
            res.message = 'ok'
        except Exception as e:  # pragma: no cover - defensive wrapper
            res.success = False
            res.message = str(e)
        return res

    def _on_get_config(
        self, req: GetConfig.Request, res: GetConfig.Response
    ) -> GetConfig.Response:
        try:
            if req.config_name:
                res.config_name = req.config_name
                res.config_yaml = self._store.read_named_yaml(req.config_name)
            else:
                res.config_name = self._store.get_active_name()
                res.config_yaml = self._store.read_active_yaml()
            res.robot_package = self._robot_package
            res.flashed_config_name = self._store.get_flashed_name()
            res.flashed_at = self._store.get_flashed_at()
            res.success = True
            res.message = 'ok'
        except FileNotFoundError:
            res.success = False
            res.message = 'Configuration not found'
            res.robot_package = self._robot_package
            res.config_name = ''
            res.config_yaml = ''
            res.flashed_config_name = ''
            res.flashed_at = ''
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.robot_package = self._robot_package
            res.config_name = ''
            res.config_yaml = ''
            res.flashed_config_name = ''
            res.flashed_at = ''
        return res

    def _on_save_config(
        self, req: SaveConfig.Request, res: SaveConfig.Response
    ) -> SaveConfig.Response:
        try:
            data = validate_schema(req.config_yaml)
            report = urdf_crosscheck(
                data,
                self._urdf_xacro,
                self._base_path,
                self._controller_config,
            )
            res.urdf_warnings = report.warnings
            if report.errors:
                res.validation_errors = report.errors
                res.success = False
                res.message = 'validation failed'
                return res

            self._store.write_named_yaml(req.config_name, req.config_yaml)
            if req.activate:
                self._store.activate(req.config_name)
            res.success = True
            res.message = 'saved'
        except ValueError as e:
            res.success = False
            res.message = 'validation failed'
            lines = [line for line in str(e).splitlines() if line.strip()]
            res.validation_errors = format_error_lines(lines)
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_activate_config(
        self, req: ActivateConfig.Request, res: ActivateConfig.Response
    ) -> ActivateConfig.Response:
        try:
            res.backup_name = self._store.activate(req.config_name)
            res.success = True
            res.message = 'activated'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_delete_config(
        self, req: DeleteConfig.Request, res: DeleteConfig.Response
    ) -> DeleteConfig.Response:
        try:
            self._store.delete(req.config_name)
            res.success = True
            res.message = 'deleted'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _mesh_allowed_roots(self) -> list[Path]:
        """Directories mesh reads may come from (src + install description trees)."""
        roots = [self._base_path.resolve()]
        try:
            share_desc = (
                Path(get_package_share_directory(self._robot_package)) / 'description'
            ).resolve()
            if share_desc not in roots:
                roots.append(share_desc)
        except Exception:  # pragma: no cover - package may be missing in unit tests
            pass
        return roots

    def _path_allowed(self, candidate: Path) -> bool:
        for root in self._mesh_allowed_roots():
            if candidate == root or root in candidate.parents:
                return True
        return False

    def _on_get_mesh(self, req: GetMesh.Request, res: GetMesh.Response) -> GetMesh.Response:
        # Serves the mesh files referenced by the URDF (file:// / package:// /
        # absolute) so the web viewer can fetch them over ROS instead of the
        # filesystem. Reads are confined to the robot's description tree(s).
        try:
            candidate = self._mesh_path_from_ref(req.path)
            if not self._path_allowed(candidate):
                res.success = False
                res.message = f'path outside robot description: {candidate}'
                res.encoding = ''
                res.data = ''
                return res
            if not candidate.is_file():
                res.success = False
                res.message = f'mesh not found: {candidate}'
                res.encoding = ''
                res.data = ''
                return res
            # COLLADA/DAE as UTF-8. Binary STL as zlib+base64 so rosbridge can
            # carry large meshes in a string field without locale corruption.
            suffix = candidate.suffix.lower()
            if suffix == '.stl':
                compressed = zlib.compress(candidate.read_bytes(), 6)
                res.data = base64.b64encode(compressed).decode('ascii')
                res.encoding = MESH_ENCODING_ZLIB_BASE64
            else:
                res.data = candidate.read_text(encoding='utf-8')
                res.encoding = MESH_ENCODING_UTF8
            res.success = True
            res.message = 'ok'
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.encoding = ''
            res.data = ''
        return res

    def _mesh_path_from_ref(self, ref: str) -> Path:
        """Resolve a URDF mesh reference to an absolute filesystem path."""
        if ref.startswith('file://'):
            return Path(ref[len('file://'):]).resolve()
        if ref.startswith('package://'):
            # package://<pkg>/<rel> — prefer ament share, fall back to robot root.
            rest = ref[len('package://'):]
            pkg, _, rel = rest.partition('/')
            if not pkg or not rel:
                raise ValueError(f'invalid package:// URI: {ref}')
            try:
                share = Path(get_package_share_directory(pkg))
                return (share / rel).resolve()
            except Exception:
                if pkg == self._robot_package:
                    return (self._base_path.parent / rel).resolve()
                raise
        return Path(ref).resolve()


def main() -> None:  # pragma: no cover
    from ..robot_paths import resolve_robot_description_paths

    rclpy.init()
    robot_root = Path('.')
    urdf, base, controllers = resolve_robot_description_paths(robot_root)
    node = ConfigServicesNode(
        robot_package='',
        config_store=ConfigStore(Path('.')),
        urdf_xacro=urdf,
        base_path=base,
        controller_config=controllers,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
