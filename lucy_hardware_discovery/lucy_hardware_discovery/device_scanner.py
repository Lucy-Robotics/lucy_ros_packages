"""Host device enumeration helpers for lucy_hardware_discovery."""

from __future__ import annotations

import glob
import json
import os
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional


def _readlink(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _usb_identity_for_tty(tty_path: str) -> Dict[str, str]:
    """Read USB vendor/product/serial from sysfs for a tty node (/dev/ttyACM0, etc.)."""
    out = {
        "usb_serial": "",
        "usb_vendor_id": "",
        "usb_product_id": "",
        "usb_manufacturer": "",
        "usb_product": "",
    }
    tty_name = os.path.basename(tty_path)
    device_link = Path("/sys/class/tty") / tty_name / "device"
    if not device_link.exists():
        return out
    try:
        current = device_link.resolve()
    except OSError:
        return out

    for _ in range(10):
        vendor = current / "idVendor"
        product = current / "idProduct"
        if vendor.is_file() and product.is_file():
            try:
                out["usb_vendor_id"] = vendor.read_text(encoding="utf-8").strip()
                out["usb_product_id"] = product.read_text(encoding="utf-8").strip()
            except OSError:
                pass
            for key, fname in (
                ("usb_manufacturer", "manufacturer"),
                ("usb_product", "product"),
            ):
                attr = current / fname
                if attr.is_file():
                    try:
                        out[key] = attr.read_text(encoding="utf-8").strip()
                    except OSError:
                        pass
            serial = current / "serial"
            if serial.is_file():
                try:
                    out["usb_serial"] = serial.read_text(encoding="utf-8").strip()
                except OSError:
                    pass
            if out["usb_serial"] and out["usb_serial"] not in ("0", "00000000"):
                break
        parent = current.parent
        if parent == current:
            break
        current = parent
    return out


def _enrich_serial_device(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Attach USB sysfs identity; prefer resolved tty path for micro-ROS agents."""
    resolved = entry.get("resolved") or entry.get("path", "")
    if resolved.startswith("/dev/tty"):
        identity = _usb_identity_for_tty(resolved)
        entry.update({k: v for k, v in identity.items() if v})
        if identity.get("usb_serial") and not entry.get("serial_hint"):
            entry["serial_hint"] = identity["usb_serial"]
    return entry


def _serial_id_matches_device(serial_id: str, dev: Dict[str, Any]) -> bool:
    """True when configured board serial_id matches a discovered serial device."""
    needle = (serial_id or "").strip().upper()
    if not needle:
        return False
    candidates = [
        dev.get("usb_serial", ""),
        dev.get("serial_hint", ""),
        dev.get("name", ""),
        dev.get("path", ""),
    ]
    for value in candidates:
        if value and needle == str(value).strip().upper():
            return True
    haystack = " ".join(str(v) for v in candidates if v).upper()
    return needle in haystack


def _device_path_for_agent(dev: Dict[str, Any]) -> str:
    """Prefer stable by-id symlink when present; else the tty device node."""
    path = dev.get("path", "")
    if "by-id" in path:
        return path
    return dev.get("resolved") or path


def list_serial_devices() -> List[Dict[str, Any]]:
    """List USB serial devices under /dev/serial/by-id when available."""
    devices: List[Dict[str, Any]] = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path):
                continue
            name = os.path.basename(path)
            serial_hint = ""
            if "by-id" in path:
                parts = name.split("_")
                if len(parts) >= 2:
                    serial_hint = parts[-1]
            devices.append(
                {
                    "path": path,
                    "name": name,
                    "serial_hint": serial_hint,
                    "resolved": _readlink(path),
                }
            )
    # Deduplicate by resolved path (by-id + ttyACM may alias same device).
    seen = set()
    unique: List[Dict[str, Any]] = []
    for entry in devices:
        key = entry["resolved"]
        if key in seen:
            continue
        seen.add(key)
        unique.append(_enrich_serial_device(entry))
    return unique


INTEL_REALSENSE_VENDOR = "8086"


def _list_realsense_usb_sysfs() -> List[Dict[str, Any]]:
    """List Intel RealSense via USB sysfs (works when libuvc owns or shares the device)."""
    base = Path("/sys/bus/usb/devices")
    if not base.is_dir():
        return []

    devices: List[Dict[str, Any]] = []
    for dev_dir in sorted(base.iterdir()):
        # Top-level USB devices only (skip interfaces like 1-4.2.2.1:1.0).
        if ":" in dev_dir.name:
            continue
        vendor_file = dev_dir / "idVendor"
        if not vendor_file.is_file():
            continue
        try:
            vendor = vendor_file.read_text(encoding="utf-8").strip().lower()
        except OSError:
            continue
        if vendor != INTEL_REALSENSE_VENDOR:
            continue

        def _read_attr(name: str) -> str:
            path = dev_dir / name
            if not path.is_file():
                return ""
            try:
                return path.read_text(encoding="utf-8").strip()
            except OSError:
                return ""

        product_id = _read_attr("idProduct")
        product = _read_attr("product")
        manufacturer = _read_attr("manufacturer")
        serial = _read_attr("serial")
        busnum = _read_attr("busnum")
        devnum = _read_attr("devnum")
        name = product or manufacturer or f"Intel RealSense ({product_id})"
        usb_path = ""
        if busnum.isdigit() and devnum.isdigit():
            usb_path = f"/dev/bus/usb/{int(busnum):03d}/{int(devnum):03d}"

        entry: Dict[str, Any] = {
            "name": name,
            "driver": "realsense2",
            "product_id": product_id,
            "access": "usb_sysfs",
        }
        if serial:
            entry["serial_number"] = serial
        if usb_path:
            entry["usb_path"] = usb_path
        devices.append(entry)
    return devices


def _is_realsense_v4l2_name(name: str) -> bool:
    return "realsense" in name.lower()


def _parse_v4l2_list_devices(text: str) -> List[Dict[str, Any]]:
    devices: List[Dict[str, Any]] = []
    current_name: Optional[str] = None
    for line in text.splitlines():
        if not line.strip():
            continue
        if not line.startswith("\t") and not line.startswith(" "):
            current_name = line.strip().rstrip(":")
            continue
        dev_path = line.strip()
        if dev_path.startswith("/dev/video") and current_name:
            driver = (
                "realsense2"
                if _is_realsense_v4l2_name(current_name)
                else "camera_ros"
            )
            devices.append(
                {
                    "name": current_name,
                    "device": dev_path,
                    "driver": driver,
                }
            )
    return devices


def list_cameras() -> List[Dict[str, Any]]:
    """List V4L2 capture devices via v4l2-ctl when installed."""
    try:
        proc = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0:
        return []
    return _parse_v4l2_list_devices(proc.stdout)


def _list_realsense_rs_enumerate() -> List[Dict[str, Any]]:
    """List Intel RealSense devices via rs-enumerate-devices (librealsense USB path)."""
    try:
        proc = subprocess.run(
            ["rs-enumerate-devices", "-s"],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0:
        return []

    devices: List[Dict[str, Any]] = []
    current: Dict[str, Any] = {}
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("Device Name"):
            if current:
                devices.append(current)
            current = {"name": line.split(":", 1)[-1].strip(), "driver": "realsense2"}
        elif line.startswith("Serial Number") and current:
            current["serial_number"] = line.split(":", 1)[-1].strip()
        elif line.startswith("Firmware Version") and current:
            current["firmware"] = line.split(":", 1)[-1].strip()
    if current:
        devices.append(current)
    for entry in devices:
        entry.setdefault("access", "librealsense_usb")
    return devices


def _list_realsense_v4l2_grouped() -> List[Dict[str, Any]]:
    """Fallback when kernel uvcvideo still owns RealSense (libuvc cannot enumerate USB)."""
    groups: Dict[str, Dict[str, Any]] = {}
    for cam in list_cameras():
        if cam.get("driver") != "realsense2":
            continue
        key = cam["name"].split("(")[0].strip()
        if key not in groups:
            groups[key] = {
                "name": key,
                "driver": "realsense2",
                "v4l2_devices": [],
                "access": "kernel_uvcvideo",
            }
        groups[key]["v4l2_devices"].append(cam["device"])
    return list(groups.values())


def list_realsense_devices() -> List[Dict[str, Any]]:
    """List Intel RealSense devices (librealsense USB, V4L2, or USB sysfs)."""
    devices = _list_realsense_rs_enumerate()
    if devices:
        return devices
    devices = _list_realsense_v4l2_grouped()
    if devices:
        return devices
    return _list_realsense_usb_sysfs()


def probe_camera(device: str) -> Dict[str, Any]:
    """Check whether a V4L2 device node exists and is readable."""
    path = Path(device)
    result: Dict[str, Any] = {
        "device": device,
        "success": False,
        "message": "",
        "compatible_camera_ros": False,
    }
    if not path.exists():
        result["message"] = f"Device not found: {device}"
        return result
    if not os.access(device, os.R_OK | os.W_OK):
        result["message"] = f"Device not readable/writable: {device}"
        return result
    try:
        proc = subprocess.run(
            ["v4l2-ctl", "-d", device, "--all"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        result["message"] = str(exc)
        return result
    if proc.returncode != 0:
        result["message"] = proc.stderr.strip() or "v4l2-ctl failed"
        return result
    result["success"] = True
    result["message"] = "Device opened successfully"
    # MJPEG formats are what camera_ros expects.
    result["compatible_camera_ros"] = bool(
        re.search(r"MJPG|Motion-JPEG|image/jpeg", proc.stdout, re.IGNORECASE)
    )
    return result


def match_boards(board_serial_ids: Dict[str, str]) -> Dict[str, Any]:
    """Match configured board serial_id values to discovered serial devices."""
    serial_devices = list_serial_devices()
    matches: Dict[str, Any] = {}
    for board_id, serial_id in board_serial_ids.items():
        serial_id = (serial_id or "").strip()
        if not serial_id:
            matches[board_id] = {
                "serial_id": "",
                "device": None,
                "matched": False,
                "reason": "empty serial_id in config",
            }
            continue
        found = None
        for dev in serial_devices:
            if _serial_id_matches_device(serial_id, dev):
                found = _device_path_for_agent(dev)
                break
        matches[board_id] = {
            "serial_id": serial_id,
            "device": found,
            "matched": found is not None,
            "reason": "matched" if found else "no device with matching serial_id",
        }
    return {"boards": matches, "serial_devices": serial_devices}


def dumps_json(payload: Any) -> str:
    return json.dumps(payload, indent=2, sort_keys=True)
