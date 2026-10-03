# Config pipeline and SHM / Modbus

Package-level detail for **`lucy_ros_packages`**.  
**System schematic:** [`lucy_ws/docs/architecture/overview.md`](../../../../docs/architecture/overview.md)  
**Index:** [`lucy_ws/docs/architecture/README.md`](../../../../docs/architecture/README.md)  
**Conventions:** [`lucy_ws/docs/architecture/GUIDE.md`](../../../../docs/architecture/GUIDE.md)

## Pipeline phases

**UML Activity (config pipeline)** - VALIDATE through RELOAD.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart LR
  V["VALIDATE"] --> G["GENERATE"]
  G --> B["BUILD"]
  B --> F["FLASH"]
  F --> R["RELOAD"]
  linkStyle default stroke:#00FF41,stroke-width:2px
```

| Phase | Always? | Output |
|-------|---------|--------|
| VALIDATE | yes | schema + URDF cross-check |
| GENERATE | yes (incl. sim-only) | ros2_control xacro, `controllers.yaml`, per-board firmware YAML |
| BUILD | skipped in sim-only | UF2 per selected board crate |
| FLASH | skipped in sim-only / build_only | `picotool load` by `serial_id` |
| RELOAD | yes | restart control so URDF + controllers reload |

Toolchain readiness (`pixi run firmware-setup`) gates ACTIVATE / BUILD / FLASH.

## Generator outputs

**UML Component (generator)** - `active.yaml` to ROS and firmware artifacts.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  Active["active.yaml"] --> Gen["lucy_config_generator"]
  Gen -.-> Xacro["ros2_control_xacro"]
  Gen -.-> Ctrl["controllers.yaml"]
  Gen -.-> Fw["config_board_id.yaml"]
  Active -->|"board_class"| Map["BOARD_CLASS_TO_CRATE"]
  Map -.-> Crate["firmware_crate"]
  Fw -.-> Crate
  linkStyle default stroke:#00FF41,stroke-width:2px
```

| `board_class` | Firmware crate |
|---------------|----------------|
| `internal_servo_only` | `firmwares/rp2040_servo2040` |
| `internal_servo_i2c_pwm` | `firmwares/rp2040_servo2040` |
| `bus_servo_only` | `firmwares/rp2040_servo2040` |

## Host ↔ MCU (real robot)

**UML Sequence (actuation)** - trajectory clients to RP2040 Modbus.

```mermaid
sequenceDiagram
  participant Clients
  participant CM as controller_manager
  participant HI as LucySystemHardware
  participant SHM as POSIX SHM
  participant Bridge as lucy_modbus_bridge
  participant MCU as RP2040 firmware
  
  Clients->>+CM: trajectory
  CM->>+HI: write cycle
  HI->>SHM: millirad + dirty bits
  deactivate HI
  deactivate CM
  
  Bridge->>SHM: poll dirty registers
  Bridge->>MCU: Modbus FC06 (USB CDC)
  
  CM-->>Clients: joint_states (broadcaster)
```

**UML Component (actuation path)** - same path as boxes.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  Clients["Clients"]
  CM["controller_manager"]
  HI["LucySystemHardware"]
  SHM["POSIX_SHM"]
  Bridge["lucy_modbus_bridge"]
  FW["RP2040_Modbus"]
  JS["joint_states"]

  Clients -->|"trajectory"| CM
  CM --> HI
  CM --> JS
  HI -->|"write millirad"| SHM
  SHM --> Bridge
  Bridge -->|"FC06"| FW
  linkStyle default stroke:#00FF41,stroke-width:2px
```

Actuation does **not** depend on JointState debug topics. `/joint_states`
comes from `joint_state_broadcaster` for TF / RViz.

## Related

- Workspace overview: [`lucy_ws/docs/architecture/overview.md`](../../../../docs/architecture/overview.md)
- Architecture guide: [`lucy_ws/docs/architecture/GUIDE.md`](../../../../docs/architecture/GUIDE.md)
- Package README: [`lucy_config_pipeline/README.md`](../../lucy_config_pipeline/README.md)
- Generator README: [`lucy_config_generator/README.md`](../../lucy_config_generator/README.md)
