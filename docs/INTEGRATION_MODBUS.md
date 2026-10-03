# Integration checklist — Rust Modbus firmware path

Manual / CI-adjacent checks for the VALIDATE → ACTIVATE → GENERATE → BUILD → FLASH → RELOAD flow with `lucy_embedded_firmware` and `thais_urdf`.

## Prerequisites

```bash
cd lucy_ws
pixi run firmware-setup          # rustup target + elf2uf2-rs
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select lucy_config_generator lucy_config_pipeline \
    lucy_ros2_control lucy_modbus_bridge lucy_bringup
source install/setup.bash
```

## Unit (no hardware)

```bash
# Firmware core
pixi run firmware-test

# Generator / pipeline / bridge
colcon test --packages-select lucy_config_generator lucy_config_pipeline lucy_modbus_bridge \
  --event-handlers console_direct+
```

Expect:

- `config_<board>.yaml` emitted (not `.c`)
- Cargo build mocked tests pass
- Flash phase uses Modbus verify (not `/uptime_publisher`)
- Bridge CRC / dirty-bit tests pass

## Functional (thais hardware YAML)

1. Ensure `thais_urdf/config/hardware/active.yaml` has:
   - `firmware.source_dir: lucy_embedded_firmware`
   - board `firmware_target` / `serial_id` filled for boards under test
2. From the control panel (or action client): run pipeline **simulation_only** → confirm xacro/controllers install.
3. Run full pipeline for one board with serial:
   - GENERATE installs `lucy_embedded_firmware/config/config_<board>.yaml`
   - BUILD produces `build/firmware/<target>.uf2`
   - FLASH loads via picotool; Modbus FC03 succeeds
4. Bringup:

```bash
ros2 launch lucy_bringup lucy.launch.py real:=true robot_package:=thais_urdf
```

5. Confirm nodes:

```bash
ros2 node list | grep -E 'modbus_bridge|lucy_hardware_interface|lucy_config_pipeline'
```

6. Command a joint via the panel / trajectory controller; servo should move.
   Encoding check: SHM angle register ≈ `servo_rad * 1000` (milliradians).

## Failure triage

| Symptom | Likely cause |
|---------|----------------|
| `cargo not found` | `pixi run firmware-setup` |
| `elf2uf2-rs not found` | same |
| Bridge cannot open SHM | HI not up yet / `node_name` mismatch |
| Flash Modbus verify timeout | wrong baud, cable, or firmware not answering FC03 |
| Motion in degrees off by ~57× | host still writing degrees instead of milliradians |
