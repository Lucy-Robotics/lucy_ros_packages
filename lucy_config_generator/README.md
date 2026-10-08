# lucy_config_generator

> Package charts: [`docs/architecture/pipeline_shm.md`](../docs/architecture/pipeline_shm.md).  
> Workspace overview (under `lucy_ws/src/`): [`lucy_control_panel/docs/architecture/overview.md`](../../lucy_control_panel/docs/architecture/overview.md).

Python tool that reads the hardware mapping YAML from an **URDF** (`config/hardware/active.yaml`) and generates:

1. **Firmware** - one architecture-shaped `config_<board_id>.yaml` per RP2040 board for the Rust builder (`lucy_embedded_firmware`). Actuator angles are **radians (float)** end-to-end (YAML, HI, f64 SHM). Sensors (e.g. pressure) are included. Device channels (`ServoN`, `ADC0`, `UART0:N`, …) are assigned by the board-crate builder. Crate selection follows `boards.*.board_class` (`internal_servo_only` / `internal_servo_i2c_pwm` / `bus_servo_only` → `rp2040_servo2040`).
2. **ros2_control** - `inmoov_ros2_control.xacro` with one `<ros2_control>` block per board in **`boards:`** document order; `hardware_name` / `node_name` / `publisher_topic` are **derived and validated** from each board id and YAML fields (see `schema.py`). Shoulder Y joints stay on whichever board the YAML assigns them to (typically `rp2040_torso_head`). Each actuated joint also gets `<command_interface name="position"><param name="min/max"/></command_interface>` populated from the URDF `<limit lower upper>` (radians) - these are the values `LucySystemHardware` clamps to in `write()`. Calibration params use `offset_rad` / `servo_*_rad` (degrees are rejected by schema).
3. **Controllers** - `controllers.yaml` with `joint_state_broadcaster` / trajectory controllers and **`extra_joints`** as *URDF joints that are not any YAML actuator* (passive / unmapped only; all actuator rows are trajectory-controlled regardless of `enabled`).

Schema validation lives in `lucy_config_generator/schema.py` for reuse by **`lucy_config_pipeline`**.

## Generated filenames (`generated_files`)

The ros2_control xacro and `controllers.yaml` basenames are centralized in the hardware YAML so the generator, **`lucy_config_pipeline`**, `inmoov.urdf.xacro` and the **`thais_urdf`** launches all agree. The section is **optional**; omitting it (or any key) falls back to `schema.GENERATED_FILES_DEFAULTS`.

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

`--urdf`, `--base-path`, and `--controller-config` are passed to **`xacro`** the same way as **`thais_urdf`** tests so joint discovery matches the real robot.

## Templates

Jinja2 sources are under the Python package at `lucy_config_generator/lucy_config_generator/templates/`:

| Template | Output |
|----------|--------|
| `config_board.yaml.j2` | **Primary** (rad / Rust tips) - architecture-shaped per-board YAML for `rp2040_servo2040`. Angles in **radians**. |
| `config_internal_only_board.c.j2` | **Legacy** - C `dump_config()` golden fixtures for `internal_servo_only` (migration / tests only). |
| `config_internal_i2c_board.c.j2` | **Legacy** - C scaffold for `internal_servo_i2c_pwm` (migration / tests only). |
| `ros2_control.xacro.j2` | One `<ros2_control>` block per board (names derived from board id). Selects `gz_ros2_control/GazeboSimSystem` when `use_gazebo_sim:=true`, otherwise `lucy_ros2_control/LucySystemHardware` for both real and mock (mock sets `<param name="publish_actuators">false</param>` so optional JointState debug traffic is off; URDF clamping and SHM path still apply as configured). |
| `controllers.yaml.j2` | `controller_manager` + per-board joint lists + `extra_joints` |

### Board class → firmware crate

| `board_class` | Crate under `lucy_embedded_firmware/firmwares/` |
|---------------|--------------------------------------------------|
| `internal_servo_only` | `rp2040_servo2040` |
| `internal_servo_i2c_pwm` | `rp2040_servo2040` (same binary; I2C/PWM banks gated by YAML) |
| `bus_servo_only` | `rp2040_servo2040` (`HAS_BUS` → UART0 Feetech) |

**Disabled actuators** (`enabled: false`): **omitted from firmware YAML device tables** only (no device row on the Pico for that joint). They **remain** in **`ros2_control`** and in the per-board **trajectory controller** joint list so the stack is uniform; firmware ignores commands at indices it does not own. **`extra_joints`** lists only URDF joints that are **not** any actuator row (typically passive links).

## Tests

```bash
colcon test --packages-select lucy_config_generator --event-handlers console_direct+
```

Golden files live in `test/fixtures/` alongside `test_mapping.yaml`. `test/test_position_limits_unified.py` pins that URDF position limits land on every actuator's `command_interface` and stay outside any `<xacro:if>` so every plugin path receives them identically.