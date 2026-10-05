from lucy_modbus_bridge.shm import build_write_single, modbus_crc


def test_modbus_crc_known_vector():
    # Write single: slave=1 addr=0 value=1
    payload = bytes([0x01, 0x06, 0x00, 0x00, 0x00, 0x01])
    crc = modbus_crc(payload)
    assert len(crc) == 2
    frame = build_write_single(1, 0, 1)
    assert frame[:6] == payload
    assert frame[6:] == crc
