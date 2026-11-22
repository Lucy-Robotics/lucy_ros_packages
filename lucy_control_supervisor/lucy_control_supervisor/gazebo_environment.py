# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

NVIDIA_GPU_MODES = ("jetson", "nvidia")
NVIDIA_EGL_VENDOR = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
NIXOS_MESA_EGL_VENDOR = "/run/opengl-driver/share/glvnd/egl_vendor.d/50_mesa.json"
HEADLESS_RUNTIME_DIR = "/tmp/runtime-root"
LOOPBACK_GZ_IP = "127.0.0.1"


def is_headless(context) -> bool:
    value = LaunchConfiguration("headless").perform(context)
    return value.lower().strip() in ("true", "1", "yes")


def gz_ros2_control_lib() -> str:
    try:
        return os.path.join(get_package_prefix("gz_ros2_control"), "lib")
    except Exception:
        try:
            share = get_package_share_directory("gz_ros2_control")
        except Exception:
            return ""
        return os.path.join(os.path.dirname(share), "lib")


def gz_plugin_path(package: str) -> str:
    mock_sensor = os.path.join(get_package_prefix(package), "lib", "mock_sensor")
    paths = [os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), gz_ros2_control_lib()]
    if os.path.isdir(mock_sensor):
        paths.append(mock_sensor)
    return os.pathsep.join(path for path in paths if path)


def gpu_environment_actions(context, *args, **kwargs):
    gpu_mode = os.environ.get("LUCY_GPU_MODE", "").lower()
    nvidia = gpu_mode in NVIDIA_GPU_MODES
    actions = []
    if is_headless(context) or nvidia:
        actions.append(SetEnvironmentVariable("XDG_RUNTIME_DIR", HEADLESS_RUNTIME_DIR))
    if nvidia:
        actions.append(
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", NVIDIA_EGL_VENDOR)
        )
        actions.append(SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"))
    elif os.path.isfile(NIXOS_MESA_EGL_VENDOR):
        actions.append(
            SetEnvironmentVariable(
                "__EGL_VENDOR_LIBRARY_FILENAMES", NIXOS_MESA_EGL_VENDOR
            )
        )
    if not os.environ.get("GZ_IP"):
        actions.append(SetEnvironmentVariable("GZ_IP", LOOPBACK_GZ_IP))
    return actions
