"""POSIX SHM helpers matching LucySystemHardware layout (Unix only).

Layout contract (lucy_ros2_control RegisterHeader / SharedRegisters):

* Register table: uint16_t[256] little-endian (512 bytes).
* Dirty header: uint8_t header[32] + uint16_t iterator (34 bytes).
  Bit for register r lives in header[r // 8], MSB-first:
  (header[i] >> (7 - (r % 8))) & 1.
* Names (after shm_node_name_for): /{shm}.lucy_reg_table,
  /{shm}.lucy_reg_header, semaphore /{shm}.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import mmap
import os
import struct
from dataclasses import dataclass

REGISTER_COUNT = 256
REG_TABLE_SIZE = REGISTER_COUNT * 2  # uint16_t[256]

# Matches sizeof(RegisterHeader): uint8_t header[32] + uint16_t iterator.
HEADER_DIRTY_BYTES = 32
HEADER_SIZE = HEADER_DIRTY_BYTES + 2  # 34

REG_TABLE_SUFFIX = ".lucy_reg_table"
REG_HEADER_SUFFIX = ".lucy_reg_header"

# Darwin PSHMNAMLEN; C++ always applies this budget on every platform.
_MAX_SHM_NAME = 31


def shm_node_name_for(node_name: str) -> str:
    """Mirror lucy_ros2_control anonymous shm_node_name_for.

    Sanitises to [A-Za-z0-9_.-], then keeps the **tail** so it fits
    /<name>.lucy_reg_header under a 31-char POSIX name cap
    (budget = 31 - 1 - len(".lucy_reg_header") = 14).
    """
    budget = _MAX_SHM_NAME - 1 - len(REG_HEADER_SUFFIX)
    sanitised = "".join(
        c if (c.isalnum() or c in "_.-") else "_" for c in node_name
    )
    if len(sanitised) > budget:
        sanitised = sanitised[-budget:]
    return sanitised


def shm_object_names(node_name: str) -> tuple[str, str, str]:
    """Return (reg_table, reg_header, sem) paths for a logical node_name."""
    shm = shm_node_name_for(node_name)
    return (
        f"/{shm}{REG_TABLE_SUFFIX}",
        f"/{shm}{REG_HEADER_SUFFIX}",
        f"/{shm}",
    )


@dataclass
class ShmMaps:
    reg_name: str
    header_name: str
    sem_name: str
    shm_node_name: str
    reg_mm: mmap.mmap
    header_mm: mmap.mmap
    sem: ctypes.c_void_p


def _libc():
    path = ctypes.util.find_library("c") or ctypes.util.find_library("rt")
    if not path:
        raise RuntimeError("libc not found")
    return ctypes.CDLL(path, use_errno=True)


def open_board_shm(node_name: str) -> ShmMaps:
    """Open SHM segments created by LucySystemHardware for node_name.

    node_name is the logical ros2_control hardware parameter; truncation
    to the POSIX shm stem is applied here the same way as in C++.
    """
    if os.name == "nt":
        raise NotImplementedError(
            "POSIX SHM bridge is not available on Windows yet; "
            "migrate LucySystemHardware to Boost.Interprocess first"
        )

    libc = _libc()
    libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
    libc.shm_open.restype = ctypes.c_int
    libc.sem_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
    libc.sem_open.restype = ctypes.c_void_p
    libc.sem_wait.argtypes = [ctypes.c_void_p]
    libc.sem_wait.restype = ctypes.c_int
    libc.sem_post.argtypes = [ctypes.c_void_p]
    libc.sem_post.restype = ctypes.c_int

    shm = shm_node_name_for(node_name)
    reg_name, header_name, sem_name = shm_object_names(node_name)

    O_RDWR = os.O_RDWR
    reg_fd = libc.shm_open(reg_name.encode(), O_RDWR, 0o666)
    if reg_fd < 0:
        raise OSError(ctypes.get_errno(), f"shm_open failed for {reg_name}")
    hdr_fd = libc.shm_open(header_name.encode(), O_RDWR, 0o666)
    if hdr_fd < 0:
        os.close(reg_fd)
        raise OSError(ctypes.get_errno(), f"shm_open failed for {header_name}")

    reg_mm = mmap.mmap(reg_fd, REG_TABLE_SIZE)
    header_mm = mmap.mmap(hdr_fd, HEADER_SIZE)
    os.close(reg_fd)
    os.close(hdr_fd)

    # Attach to an existing semaphore (do not create).
    SEM_FAILED = ctypes.c_void_p(-1).value
    sem = libc.sem_open(sem_name.encode(), 0)
    if not sem or sem == SEM_FAILED:
        raise OSError(ctypes.get_errno(), f"sem_open failed for {sem_name}")

    return ShmMaps(
        reg_name, header_name, sem_name, shm, reg_mm, header_mm, sem
    )


def wait_sem(shm: ShmMaps) -> None:
    libc = _libc()
    libc.sem_wait.argtypes = [ctypes.c_void_p]
    libc.sem_wait.restype = ctypes.c_int
    if libc.sem_wait(shm.sem) != 0:
        raise OSError(ctypes.get_errno(), "sem_wait failed")


def post_sem(shm: ShmMaps) -> None:
    libc = _libc()
    libc.sem_post.argtypes = [ctypes.c_void_p]
    libc.sem_post.restype = ctypes.c_int
    if libc.sem_post(shm.sem) != 0:
        raise OSError(ctypes.get_errno(), "sem_post failed")


def _dirty_byte_index(reg: int) -> tuple[int, int]:
    return reg // 8, reg % 8


def get_dirty(header_mm, reg: int) -> bool:
    """Match RegisterHeader::get_register_status (uint8_t bitfield)."""
    index, index2 = _dirty_byte_index(reg)
    return ((header_mm[index] >> (7 - index2)) & 0b1) != 0


def set_dirty(header_mm, reg: int) -> None:
    """Match RegisterHeader::set_dirty."""
    if get_dirty(header_mm, reg):
        return
    index, index2 = _dirty_byte_index(reg)
    header_mm[index] = header_mm[index] ^ (1 << (7 - index2))


def set_clean(header_mm, reg: int) -> None:
    """Match RegisterHeader::set_clean."""
    if not get_dirty(header_mm, reg):
        return
    index, index2 = _dirty_byte_index(reg)
    header_mm[index] = header_mm[index] ^ (1 << (7 - index2))


def read_register(reg_mm, reg: int) -> int:
    return struct.unpack_from("<H", reg_mm, reg * 2)[0]


def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, "little")


def build_write_single(slave: int, addr: int, value: int) -> bytes:
    frame = bytearray([slave & 0xFF, 0x06])
    frame.extend(addr.to_bytes(2, "big"))
    frame.extend(value.to_bytes(2, "big"))
    frame.extend(modbus_crc(frame))
    return bytes(frame)
