"""Unit tests for Modbus frame helpers and SHM dirty-bit layout."""

from __future__ import annotations

import struct

from lucy_modbus_bridge.shm import (
    HEADER_WORDS,
    REGISTER_COUNT,
    build_write_single,
    get_dirty,
    modbus_crc,
    set_clean,
)


def test_build_write_single_millirad_value():
    # 90° ≈ π/2 rad → ~1571 millirad (firmware encoding contract).
    frame = build_write_single(1, 1, 1571)
    assert frame[0] == 1
    assert frame[1] == 0x06
    assert frame[2:4] == (1).to_bytes(2, 'big')
    assert frame[4:6] == (1571).to_bytes(2, 'big')
    assert frame[6:] == modbus_crc(frame[:6])


def test_dirty_bit_roundtrip_in_memory():
    header_mm = bytearray(HEADER_WORDS * 4)
    assert REGISTER_COUNT == 256
    for reg in (0, 1, 7, 8, 15, 16, 255):
        assert get_dirty(header_mm, reg) is False
        index = reg // 8
        index2 = reg % 8
        words = memoryview(header_mm).cast('I')
        words[index] = words[index] | (1 << (7 - index2))
        assert get_dirty(header_mm, reg) is True
        set_clean(header_mm, reg)
        assert get_dirty(header_mm, reg) is False


def test_register_pack_little_endian():
    buf = bytearray(4)
    struct.pack_into('<H', buf, 0, 1)  # cmd
    struct.pack_into('<H', buf, 2, 1571)  # millirad
    assert buf[0:2] == (1).to_bytes(2, 'little')
    assert struct.unpack_from('<H', buf, 2)[0] == 1571
