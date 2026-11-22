from __future__ import annotations

import base64
import json
from pathlib import Path
import zlib

from ament_index_python.packages import get_package_share_directory
from lucy_config_generator.motors import motors_used
from lucy_msgs.srv import ActivateConfig
from lucy_msgs.srv import DeleteConfig
from lucy_msgs.srv import GetConfig
from lucy_msgs.srv import GetMesh
from lucy_msgs.srv import ListConfigs
from lucy_msgs.srv import SaveConfig
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import yaml

from .mesh_glb import glb_beside
from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import parse_yaml_text
from ..validation import urdf_crosscheck
from ..validation import validate_schema

# Wire encodings for GetMesh.data (must match lucy_msgs/srv/GetMesh.srv comments).
MESH_ENCODING_UTF8 = "utf8"
MESH_ENCODING_ZLIB_BASE64 = "zlib_base64"
MESH_ENCODING_GLB_BASE64 = "glb_base64"


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
        super().__init__("lucy_config_services")
        self._robot_package = robot_package
        self._store = config_store
        self._urdf_xacro = urdf_xacro
        self._base_path = base_path
        self._controller_config = controller_config

        self.create_service(ListConfigs, "config/list", self._on_list_configs)
        self.create_service(GetConfig, "config/get", self._on_get_config)
        self.create_service(SaveConfig, "config/save", self._on_save_config)
        self.create_service(ActivateConfig, "config/activate", self._on_activate_config)
        self.create_service(DeleteConfig, "config/delete", self._on_delete_config)
        # Reentrant: mesh reads are independent and the viewer fires many of
        # them concurrently, so they must not queue behind the default
        # mutually exclusive group shared by the other config services.
        self.create_service(
            GetMesh,
            "mesh/get",
            self._on_get_mesh,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_list_configs(
        self, _req: ListConfigs.Request, res: ListConfigs.Response
    ) -> ListConfigs.Response:
        try:
            res.config_names = self._store.list_configs()
            res.active_config = self._store.get_active_name()
            res.success = True
            res.message = "ok"
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
            res.motors_yaml = yaml.safe_dump(
                motors_used(parse_yaml_text(res.config_yaml)), sort_keys=False
            )
            res.success = True
            res.message = "ok"
        except FileNotFoundError:
            res.success = False
            res.message = "Configuration not found"
            res.robot_package = self._robot_package
            res.config_name = ""
            res.config_yaml = ""
            res.flashed_config_name = ""
            res.flashed_at = ""
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.robot_package = self._robot_package
            res.config_name = ""
            res.config_yaml = ""
            res.flashed_config_name = ""
            res.flashed_at = ""
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
                res.message = "validation failed"
                return res

            # Persist schema-normalized labels (e.g. derived board_class).
            self._store.write_named_yaml(
                req.config_name,
                yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
            )
            if req.activate:
                self._store.activate(req.config_name)
            res.success = True
            res.message = "saved"
        except ValueError as e:
            res.success = False
            res.message = "validation failed"
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
            res.message = "activated"
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
            res.message = "deleted"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _mesh_allowed_roots(self) -> list[Path]:
        """Directories mesh reads may come from (src + install description trees)."""
        roots = [self._base_path.resolve()]
        try:
            share_desc = (
                Path(get_package_share_directory(self._robot_package)) / "description"
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

    def _on_get_mesh(
        self, req: GetMesh.Request, res: GetMesh.Response
    ) -> GetMesh.Response:
        # TEMP (tmp/mesh-load-bench): a JSON list in `path` answers every mesh in one call.
        if req.path.startswith("["):
            out = {}
            for ref in json.loads(req.path):
                ok, message, encoding, data = self._load_mesh(ref, req.quality)
                out[ref] = {"success": ok, "message": message, "encoding": encoding, "data": data}
            res.success, res.message, res.encoding = True, "ok", "batch_json"
            res.data = json.dumps(out)
            return res
        res.success, res.message, res.encoding, res.data = self._load_mesh(req.path, req.quality)
        return res

    def _load_mesh(self, ref: str, quality: str) -> tuple[bool, str, str, str]:
        # Serves the mesh files referenced by the URDF (file:// / package:// /
        # absolute) so the web viewer can fetch them over ROS instead of the
        # filesystem. Reads are confined to the robot's description tree(s).
        try:
            candidate = self._mesh_path_from_ref(ref)
            if not self._path_allowed(candidate):
                return False, f"path outside robot description: {candidate}", "", ""
            if not candidate.is_file():
                return False, f"mesh not found: {candidate}", "", ""
            # COLLADA/DAE as UTF-8. Binary STL as zlib+base64 so rosbridge can
            # carry large meshes in a string field without locale corruption.
            glb = glb_beside(candidate, quality)
            if glb is not None:
                return True, "ok", MESH_ENCODING_GLB_BASE64, base64.b64encode(glb.read_bytes()).decode("ascii")
            if candidate.suffix.lower() == ".stl":
                compressed = zlib.compress(candidate.read_bytes(), 6)
                return True, "ok", MESH_ENCODING_ZLIB_BASE64, base64.b64encode(compressed).decode("ascii")
            return True, "ok", MESH_ENCODING_UTF8, candidate.read_text(encoding="utf-8")
        except Exception as e:
            return False, str(e), "", ""

    def _mesh_path_from_ref(self, ref: str) -> Path:
        """Resolve a URDF mesh reference to an absolute filesystem path."""
        if ref.startswith("file://"):
            return Path(ref.removeprefix("file://")).resolve()
        if ref.startswith("package://"):
            # package://<pkg>/<rel> — prefer ament share, fall back to robot root.
            rest = ref.removeprefix("package://")
            pkg, _, rel = rest.partition("/")
            if not pkg or not rel:
                raise ValueError(f"invalid package:// URI: {ref}")
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
    robot_root = Path(".")
    urdf, base, controllers = resolve_robot_description_paths(robot_root)
    node = ConfigServicesNode(
        robot_package="thais_urdf",
        config_store=ConfigStore(Path(".")),
        urdf_xacro=urdf,
        base_path=base,
        controller_config=controllers,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
