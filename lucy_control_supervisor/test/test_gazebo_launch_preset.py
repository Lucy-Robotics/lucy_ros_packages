from pathlib import Path

from lucy_control_supervisor import gazebo_launch as gl


def _share_with_active(tmp_path: Path, text: str) -> Path:
    share = tmp_path / "install" / "robot" / "share" / "robot"
    (share / "config" / "hardware").mkdir(parents=True)
    (share / "config" / "hardware" / "active.yaml").write_text(text, encoding="utf-8")
    return share


def test_workspace_source_copy_wins_over_the_install_share(tmp_path, monkeypatch):
    share = _share_with_active(tmp_path, "from: share\n")
    src = tmp_path / "src" / "robot" / "config" / "hardware"
    src.mkdir(parents=True)
    (src / "active.yaml").write_text("from: src\n", encoding="utf-8")
    monkeypatch.chdir(tmp_path / "install")

    assert (
        gl._infer_robot_source_root("robot", str(share)) == tmp_path / "src" / "robot"
    )
    assert gl._active_preset("robot", str(share)) == {"from": "src"}


def test_install_share_is_read_when_there_is_no_source_copy(tmp_path, monkeypatch):
    share = _share_with_active(tmp_path, "from: share\n")
    monkeypatch.chdir(tmp_path)
    assert gl._infer_robot_source_root("robot", str(share)) is None
    assert gl._active_preset("robot", str(share)) == {"from": "share"}


def test_broken_or_non_mapping_active_yaml_counts_as_empty(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert (
        gl._active_preset("robot", str(_share_with_active(tmp_path / "a", "a: [\n")))
        == {}
    )
    assert (
        gl._active_preset("robot", str(_share_with_active(tmp_path / "b", "- 1\n")))
        == {}
    )
    assert gl._active_preset("robot", str(tmp_path / "nowhere")) == {}


def test_generated_files_only_rename_plain_basenames():
    active = {
        "generated_files": {
            "ros2_control_xacro": " arm.ros2_control.xacro ",
            "controllers_yaml": "../escape.yaml",
        }
    }
    assert gl._active_generated_files(active, "default.xacro") == {
        "ros2_control_xacro": "arm.ros2_control.xacro",
        "controllers_yaml": "controllers.yaml",
    }
    assert gl._active_generated_files({"generated_files": "x"}, "d.xacro") == {
        "ros2_control_xacro": "d.xacro",
        "controllers_yaml": "controllers.yaml",
    }


def test_controllers_yaml_prefers_the_writable_source_copy(tmp_path, monkeypatch):
    share = _share_with_active(tmp_path, "{}\n")
    monkeypatch.chdir(tmp_path)
    assert gl._default_controllers_yaml("robot", str(share), "c.yaml") == str(
        share / "config" / "c.yaml"
    )
    src = tmp_path / "src" / "robot" / "config"
    src.mkdir(parents=True)
    (src / "c.yaml").write_text("{}\n", encoding="utf-8")
    assert gl._default_controllers_yaml("robot", str(share), "c.yaml") == str(
        (src / "c.yaml").resolve()
    )


def test_only_simulated_cameras_with_both_topics_get_a_compressor():
    cameras = [
        {"topic": " /head/image ", "compressed_topic": "/head/jpeg", "link": "head"},
        {
            "topic": "/world/image",
            "compressed_topic": "/world/jpeg",
            "external": True,
            "sim_gz_topic": "/gz/world",
        },
        {"topic": "/ext/image", "compressed_topic": "/ext/jpeg", "external": True},
        {"topic": "/nolink/image", "compressed_topic": "/nolink/jpeg"},
        {"topic": "/raw/only", "link": "x"},
        {"topic": " ", "compressed_topic": "/blank", "link": "x"},
        "not a camera",
    ]
    assert gl._sim_camera_topics({"cameras": cameras}) == [
        ("/head/image", "/head/jpeg"),
        ("/world/image", "/world/jpeg"),
    ]
    assert gl._sim_camera_topics({"cameras": "none"}) == []


def test_plugin_path_joins_env_gz_ros2_control_and_mock_sensor(tmp_path, monkeypatch):
    (tmp_path / "robot" / "lib" / "mock_sensor").mkdir(parents=True)
    monkeypatch.setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", "/env/plugins")
    monkeypatch.setattr(gl, "get_package_prefix", lambda pkg: str(tmp_path / pkg))
    assert gl._gz_plugin_path("robot").split(gl.os.pathsep) == [
        "/env/plugins",
        str(tmp_path / "gz_ros2_control" / "lib"),
        str(tmp_path / "robot" / "lib" / "mock_sensor"),
    ]


def test_plugin_path_falls_back_to_the_share_dir_then_to_nothing(tmp_path, monkeypatch):
    def prefix(pkg):
        if pkg == "gz_ros2_control":
            raise LookupError(pkg)
        return str(tmp_path / pkg)

    monkeypatch.delenv("GZ_SIM_SYSTEM_PLUGIN_PATH", raising=False)
    monkeypatch.setattr(gl, "get_package_prefix", prefix)
    monkeypatch.setattr(
        gl, "get_package_share_directory", lambda pkg: "/opt/ros/share/gz_ros2_control"
    )
    assert gl._gz_plugin_path("robot") == "/opt/ros/share/lib"

    def no_share(pkg):
        raise LookupError(pkg)

    monkeypatch.setattr(gl, "get_package_share_directory", no_share)
    assert gl._gz_plugin_path("robot") == ""
