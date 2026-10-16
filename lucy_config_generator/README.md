# lucy_config_generator

> Package charts: [`docs/architecture/pipeline_shm.md`](../docs/architecture/pipeline_shm.md).  
> Workspace overview (under `lucy_ws/src/`): [`lucy_control_panel/docs/architecture/overview.md`](../../lucy_control_panel/docs/architecture/overview.md).

Python tool that reads the hardware mapping YAML (`config/hardware/active.yaml`) and generates:

1. **Firmware** — per-board firmware config for `lucy_embedded_firmware`. Actuator angles are **radians** end-to-end (YAML, hardware interface, f64 shared memory). Sensors (e.g. pressure) are included when the selected firmware template emits them. Set `boards.*.firmware_target` to the Cargo package to build (for example `rp2040`).
2. **ros2_control** — `inmoov_ros2_control.xacro` with one `<ros2_control>` block per board in **`boards:`** document order. `hardware_name` / `node_name` / `publisher_topic` are derived and validated from each board id and YAML fields (see `schema.py`). Shoulder Y joints stay on whichever board the YAML assigns them to (typically `rp2040_torso_head`). Each actuated joint also gets `<command_interface name="position"><param name="min/max"/></command_interface>` from the URDF `<limit lower upper>` (radians); `LucySystemHardware` clamps to those bounds in `write()`. Calibration params on each joint are `offset` / `limit_min` / `limit_max` / `default` in **radians**.
3. **Controllers** — `controllers.yaml` with `joint_state_broadcaster` / trajectory controllers and **`extra_joints`** as URDF joints that are not any YAML actuator (passive / unmapped only; all actuator rows are trajectory-controlled regardless of `enabled`).

Schema validation lives in `lucy_config_generator/schema.py` for reuse by **`lucy_config_pipeline`**.

## Generated filenames (`generated_files`)

The ros2_control xacro and `controllers.yaml` basenames are centralized in the hardware YAML so the generator, **`lucy_config_pipeline`**, `inmoov.urdf.xacro`, and the **`thais_urdf`** launches all agree. The section is **optional**; omitting it (or any key) falls back to `schema.GENERATED_FILES_DEFAULTS`.

```yaml
generated_files:
  ros2_control_xacro: inmoov_ros2_control.xacro   # written to description/ros2_control/
  controllers_yaml: controllers.yaml              # written to config/
```

Values are **bare filenames** (no path separators); the directories are fixed by repo convention. `inmoov.urdf.xacro` reads the basename via its `ros2_control_file` xacro arg, and the launches/supervisor forward the same value so the running stack loads the file the pipeline just installed. Resolve them in code with `schema.resolve_generated_files(data)`.

## CLI

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

generate_config \
  --input src/thais_urdf/config/hardware/active.yaml \
  --urdf src/thais_urdf/description/urdf/inmoov.urdf.xacro \
  --base-path src/thais_urdf/description \
  --controller-config src/thais_urdf/config/controllers.yaml \
  --output-dir /tmp/gen_out \
  --targets all
```

Options:

| Flag | Meaning |
|------|---------|
| `--targets` | `all` (default), `firmware`, `ros2_control`, or `controllers` |
| `--boards` | Optional comma-separated board ids (e.g. `rp2040_left_arm,rp2040_torso_head`) |

`--urdf`, `--base-path`, and `--controller-config` are passed to **`xacro`** the same way as robot-package tests so joint discovery matches the deployed robot.

## Actuator calibration fields

Required per actuator in the hardware YAML (radians):

| Field | Role |
|-------|------|
| `offset` | Servo-frame zero shift used by the hardware interface mapping |
| `limit_min` / `limit_max` | Servo-frame travel bounds |
| `default` | Servo-frame default pose (must lie in `[limit_min, limit_max]`) |

Also required: `direction` (`-1` or `1`) and `scale` (non-zero). Other angle field names are rejected by schema validation.

## Templates

Jinja2 sources live under `lucy_config_generator/lucy_config_generator/templates/`:

| Template | Output |
|----------|--------|
| `config_internal_only_board.c.j2` | Firmware config for `internal_servo_only` boards (radians) |
| `config_internal_i2c_board.c.j2` | Firmware config for `internal_servo_i2c_pwm` boards (radians) |
| `config_bus_servo_board.rs.j2` | Rust register layout for `bus_servo_only` boards |
| `ros2_control.xacro.j2` | One `<ros2_control>` block per board; emits `offset` / `limit_min` / `limit_max` / `default`. Mock hardware sets `<param name="publish_actuators">false</param>` |
| `controllers.yaml.j2` | `controller_manager` + per-board joint lists + `extra_joints` |

### Board class and firmware target

| `board_class` | Firmware template family |
|---------------|--------------------------|
| `internal_servo_only` | `config_internal_only_board.c.j2` |
| `internal_servo_i2c_pwm` | `config_internal_i2c_board.c.j2` |
| `bus_servo_only` | `config_bus_servo_board.rs.j2` |

The Cargo package to build and flash is `boards.*.firmware_target` in the hardware YAML (for example `rp2040` under `lucy_embedded_firmware/firmwares/`).

**Disabled actuators** (`enabled: false`): omitted from firmware device tables only (no device row on the MCU for that joint). They remain in **ros2_control** and in the per-board trajectory controller joint list so the stack stays uniform; firmware ignores commands at indices it does not own. **`extra_joints`** lists only URDF joints that are not any actuator row (typically passive links).

## Tests

```bash
colcon test --packages-select lucy_config_generator --event-handlers console_direct+
```

Golden files live in `test/fixtures/` alongside `test_mapping.yaml`. `test/test_position_limits_unified.py` checks that URDF position limits land on every actuator's `command_interface` and stay outside any `<xacro:if>` so every plugin path receives them identically.
