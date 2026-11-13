# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from lucy_control_supervisor.gazebo_cameras import camera_compressor
from lucy_control_supervisor.gazebo_cameras import sim_camera_topics
from lucy_control_supervisor.gazebo_preset import active_preset
from lucy_control_supervisor.gazebo_preset import generated_files
from lucy_control_supervisor.gazebo_world import gz_args

WORLD = "/w/default.sdf"


def test_headless_runs_server_only_with_headless_rendering():
    assert gz_args(WORLD, True, False) == f"-s -r --headless-rendering {WORLD}"


def test_headless_wins_over_macos_server_only_mode():
    assert gz_args(WORLD, True, True) == f"-s -r --headless-rendering {WORLD}"


def test_macos_gui_runs_server_only_so_the_gui_starts_separately():
    assert gz_args(WORLD, False, True) == f"-s -r {WORLD}"


def test_linux_gui_runs_server_and_gui_together():
    assert gz_args(WORLD, False, False) == f"-r {WORLD}"


def test_generated_files_default_to_launch_arguments_without_preset():
    assert generated_files({}, "robot_ros2_control.xacro") == {
        "ros2_control_xacro": "robot_ros2_control.xacro",
        "controllers_yaml": "controllers.yaml",
    }


def test_generated_files_use_preset_basenames():
    active = {
        "generated_files": {
            "ros2_control_xacro": " custom.xacro ",
            "controllers_yaml": "custom.yaml",
        }
    }
    assert generated_files(active, "x.xacro") == {
        "ros2_control_xacro": "custom.xacro",
        "controllers_yaml": "custom.yaml",
    }


def test_generated_files_reject_paths_and_blank_values():
    active = {
        "generated_files": {
            "ros2_control_xacro": "../escape.xacro",
            "controllers_yaml": "  ",
        }
    }
    assert generated_files(active, "x.xacro") == {
        "ros2_control_xacro": "x.xacro",
        "controllers_yaml": "controllers.yaml",
    }


def test_active_preset_reads_the_share_directory_file(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    share = tmp_path / "share" / "pkg"
    (share / "config" / "hardware").mkdir(parents=True)
    (share / "config" / "hardware" / "active.yaml").write_text("cameras: []\nname: a\n")
    assert active_preset("pkg", str(share)) == {"cameras": [], "name": "a"}


def test_active_preset_prefers_the_source_tree_over_the_share_directory(
    tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    source = tmp_path / "src" / "pkg" / "config" / "hardware"
    source.mkdir(parents=True)
    (source / "active.yaml").write_text("name: source\n")
    share = tmp_path / "share" / "pkg"
    (share / "config" / "hardware").mkdir(parents=True)
    (share / "config" / "hardware" / "active.yaml").write_text("name: installed\n")
    assert active_preset("pkg", str(share)) == {"name": "source"}


def test_active_preset_is_empty_for_invalid_yaml(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    share = tmp_path / "share" / "pkg"
    (share / "config" / "hardware").mkdir(parents=True)
    (share / "config" / "hardware" / "active.yaml").write_text("a: [unclosed\n")
    assert active_preset("pkg", str(share)) == {}


def test_active_preset_is_empty_when_no_file_exists(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert active_preset("pkg", str(tmp_path / "share" / "pkg")) == {}


def test_sim_camera_topics_keep_link_cameras_and_external_sim_topics():
    active = {
        "cameras": [
            {"topic": "/a", "compressed_topic": "/a/c", "link": "head"},
            {"topic": "/b", "compressed_topic": "/b/c", "external": True},
            {
                "topic": "/c",
                "compressed_topic": "/c/c",
                "external": True,
                "sim_gz_topic": "/gz/c",
            },
            {"topic": "/d", "compressed_topic": "/d/c"},
            {"topic": "/e", "link": "head"},
        ]
    }
    assert sim_camera_topics(active) == [("/a", "/a/c"), ("/c", "/c/c")]


def test_sim_camera_topics_are_empty_without_a_camera_list():
    assert sim_camera_topics({}) == []
    assert sim_camera_topics({"cameras": "none"}) == []


def test_humble_compressor_passes_transports_as_arguments():
    node = camera_compressor("humble", "/a", "/a/compressed")
    assert node._Node__arguments == ["raw", "compressed"]


def test_jazzy_compressor_passes_transports_as_parameters():
    node = camera_compressor("jazzy", "/a", "/a/compressed")
    assert node._Node__arguments is None
