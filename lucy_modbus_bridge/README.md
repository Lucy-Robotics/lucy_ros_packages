# lucy_modbus_bridge

Relays dirty holding registers from `LucySystemHardware` POSIX shared memory to
Modbus RTU over USB CDC (RP2040 Rust firmware).

## Parameters

| Name | Default | Description |
|------|---------|-------------|
| `node_name` | `lucy_hardware_interface` | Must match ros2_control hardware `node_name` |
| `serial_id` | `""` | Substring matched against USB serial / hwid |
| `slave_address` | `1` | Modbus slave address |
| `baud` | `115200` | Serial baud rate |
| `vid` / `pid` | `0x16C0` / `0x27DD` | RP2040 USB identifiers |

## Launch

```bash
ros2 launch lucy_modbus_bridge modbus_bridge.launch.py serial_id:=E6617C93E3858429
```

## Platform notes

POSIX `shm_open` / `sem_open` only (Linux/macOS). Windows requires migrating the
hardware interface to Boost.Interprocess first.
