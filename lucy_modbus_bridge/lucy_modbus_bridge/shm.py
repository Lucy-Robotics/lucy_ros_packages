"""POSIX SHM helpers matching LucySystemHardware layout (Unix only)."""

from __future__ import annotations

import ctypes
import ctypes.util
import mmap
import os
import struct
from dataclasses import dataclass

REGISTER_COUNT = 256
HEADER_WORDS = 32
REG_TABLE_SIZE = REGISTER_COUNT * 2  # uint16
HEADER_SIZE = HEADER_WORDS * 4  # uint32


@dataclass
class ShmMaps:
    reg_name: str
    header_name: str
    sem_name: str
    reg_mm: mmap.mmap
    header_mm: mmap.mmap
    sem: ctypes.c_void_p


def _libc():
    path = ctypes.util.find_library('c') or ctypes.util.find_library('rt')
    if not path:
        raise RuntimeError('libc not found')
    return ctypes.CDLL(path, use_errno=True)


def open_board_shm(node_name: str) -> ShmMaps:
    """Open SHM segments created by LucySystemHardware for ``node_name``."""
    if os.name == 'nt':
        raise NotImplementedError(
            'POSIX SHM bridge is not available on Windows yet; '
            'migrate LucySystemHardware to Boost.Interprocess first'
        )

    libc = _libc()
    libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
    libc.shm_open.restype = ctypes.c_int
    libc.sem_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    libc.sem_open.restype = ctypes.c_void_p
    libc.sem_wait.argtypes = [ctypes.c_void_p]
    libc.sem_wait.restype = ctypes.c_int
    libc.sem_post.argtypes = [ctypes.c_void_p]
    libc.sem_post.restype = ctypes.c_int

    reg_name = f'/{node_name}.lucy_reg_table'
    header_name = f'/{node_name}.lucy_reg_header'
    sem_name = f'/{node_name}'

    O_RDWR = os.O_RDWR
    reg_fd = libc.shm_open(reg_name.encode(), O_RDWR, 0o666)
    if reg_fd < 0:
        raise OSError(ctypes.get_errno(), f'shm_open failed for {reg_name}')
    hdr_fd = libc.shm_open(header_name.encode(), O_RDWR, 0o666)
    if hdr_fd < 0:
        raise OSError(ctypes.get_errno(), f'shm_open failed for {header_name}')

    reg_mm = mmap.mmap(reg_fd, REG_TABLE_SIZE)
    header_mm = mmap.mmap(hdr_fd, HEADER_SIZE)
    os.close(reg_fd)
    os.close(hdr_fd)

    sem = libc.sem_open(sem_name.encode(), 0, 0o644, 1)
    if not sem:
        raise OSError(ctypes.get_errno(), f'sem_open failed for {sem_name}')

    return ShmMaps(reg_name, header_name, sem_name, reg_mm, header_mm, sem)


def wait_sem(shm: ShmMaps) -> None:
    libc = _libc()
    libc.sem_wait.argtypes = [ctypes.c_void_p]
    libc.sem_wait.restype = ctypes.c_int
    if libc.sem_wait(shm.sem) != 0:
        raise OSError(ctypes.get_errno(), 'sem_wait failed')


def post_sem(shm: ShmMaps) -> None:
    libc = _libc()
    libc.sem_post.argtypes = [ctypes.c_void_p]
    libc.sem_post.restype = ctypes.c_int
    if libc.sem_post(shm.sem) != 0:
        raise OSError(ctypes.get_errno(), 'sem_post failed')


def get_dirty(header_mm: mmap.mmap, reg: int) -> bool:
    # Matches LucySystemHardware: word = reg/8, bit = 7 - (reg%8) within that word's low byte view.
    index = reg // 8
    index2 = reg % 8
    words = memoryview(header_mm).cast('I')
    return ((words[index] >> (7 - index2)) & 0b1) != 0


def set_clean(header_mm: mmap.mmap, reg: int) -> None:
    if not get_dirty(header_mm, reg):
        return
    index = reg // 8
    index2 = reg % 8
    words = memoryview(header_mm).cast('I')
    words[index] = words[index] ^ (1 << (7 - index2))


def read_register(reg_mm: mmap.mmap, reg: int) -> int:
    return struct.unpack_from('<H', reg_mm, reg * 2)[0]


def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')


def build_write_single(slave: int, addr: int, value: int) -> bytes:
    frame = bytearray([slave & 0xFF, 0x06])
    frame.extend(addr.to_bytes(2, 'big'))
    frame.extend(value.to_bytes(2, 'big'))
    frame.extend(modbus_crc(frame))
    return bytes(frame)
