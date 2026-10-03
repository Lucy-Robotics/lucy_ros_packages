"""Unit tests for Modbus frame helpers and SHM dirty-bit layout."""

from __future__ import annotations

import struct

from lucy_modbus_bridge.shm import (
    HEADER_DIRTY_BYTES,
    HEADER_SIZE,
    REGISTER_COUNT,
    build_write_single,
    get_dirty,
    modbus_crc,
    set_clean,
    set_dirty,
    shm_node_name_for,
    shm_object_names,
)


def test_build_write_single_millirad_value():
    # 90° ≈ π/2 rad → ~1571 millirad (firmware encoding contract).
    frame = build_write_single(1, 1, 1571)
    assert frame[0] == 1
    assert frame[1] == 0x06
    assert frame[2:4] == (1).to_bytes(2, "big")
    assert frame[4:6] == (1571).to_bytes(2, "big")
    assert frame[6:] == modbus_crc(frame[:6])


def test_header_size_matches_cpp_register_header():
    # sizeof(RegisterHeader) = uint8_t[32] + uint16_t = 34
    assert HEADER_DIRTY_BYTES == 32
    assert HEADER_SIZE == 34


def test_dirty_bit_roundtrip_byte_layout():
    """Dirty bits are per-byte (not uint32 words), matching C++ RegisterHeader."""
    header_mm = bytearray(HEADER_SIZE)
    assert REGISTER_COUNT == 256
    for reg in (0, 1, 7, 8, 15, 16, 255):
        assert get_dirty(header_mm, reg) is False
        set_dirty(header_mm, reg)
        index = reg // 8
        index2 = reg % 8
        assert header_mm[index] & (1 << (7 - index2))
        assert get_dirty(header_mm, reg) is True
        set_clean(header_mm, reg)
        assert get_dirty(header_mm, reg) is False


def test_dirty_bits_do_not_alias_across_bytes():
    header_mm = bytearray(HEADER_SIZE)
    set_dirty(header_mm, 0)   # byte 0
    set_dirty(header_mm, 8)   # byte 1
    set_dirty(header_mm, 16)  # byte 2
    assert header_mm[0] != 0
    assert header_mm[1] != 0
    assert header_mm[2] != 0
    assert get_dirty(header_mm, 0) and get_dirty(header_mm, 8) and get_dirty(header_mm, 16)
    set_clean(header_mm, 8)
    assert get_dirty(header_mm, 0) is True
    assert get_dirty(header_mm, 8) is False
    assert get_dirty(header_mm, 16) is True


def test_shm_node_name_truncates_like_cpp():
    full = "lucy_hardware_interface_left_arm"
    assert shm_node_name_for(full) == "rface_left_arm"
    assert len(shm_node_name_for(full)) == 14
    reg, hdr, sem = shm_object_names(full)
    assert reg == "/rface_left_arm.lucy_reg_table"
    assert hdr == "/rface_left_arm.lucy_reg_header"
    assert sem == "/rface_left_arm"
    # Short names pass through unchanged.
    assert shm_node_name_for("lucy") == "lucy"
    # Sanitise non-POSIX characters.
    assert shm_node_name_for("a/b c") == "a_b_c"


def test_register_pack_little_endian():
    buf = bytearray(4)
    struct.pack_into("<H", buf, 0, 1)  # cmd
    struct.pack_into("<H", buf, 2, 1571)  # millirad
    assert buf[0:2] == (1).to_bytes(2, "little")
    assert struct.unpack_from("<H", buf, 2)[0] == 1571
