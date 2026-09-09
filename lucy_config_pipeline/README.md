# lucy_config_pipeline

Service/action layer for the hardware config workflow.

- Config store operations on `config/hardware/` (`active.yaml`, `configs/*.yaml`, backups)
- Validation (schema + URDF cross-check)
- `ConfigurePipeline` action phases: **validate → generate → build → flash → reload**

### Phases

| # | Phase | Always runs? | What it does |
|---|-------|--------------|--------------|
| 1 | **VALIDATE** | yes | schema + URDF cross-check on the YAML |
| 2 | **GENERATE** | **yes — including `simulation_only`** | runs `lucy_config_generator` and installs the resulting `inmoov_ros2_control.xacro` and `controllers.yaml` into `thais_urdf` so URDF limits and joint topology are up to date even when no firmware build runs |
| 3 | **BUILD** | skipped in `simulation_only` / `build_only=false` | Cargo build (`thumbv6m-none-eabi`) + `elf2uf2-rs` per selected board |
| 4 | **FLASH** | skipped in `simulation_only` / `build_only=true` | `sudo picotool load` per board (see below) |
| 5 | **RELOAD** | yes | calls `/lucy_control/restart` so `robot_state_publisher` + `ros2_control` re-read URDF + controller YAML; Gazebo topology changes still require a relaunch |

Decoupling **GENERATE** from **BUILD** is what lets the LCP "SIMULATION ONLY" toggle update URDF limits and ros2_control wiring without a firmware build.

## Flash phase (RP2040)

After a successful build, each selected board with a non-empty `serial_id` is flashed using:

- `sudo picotool load <absolute_path_to_uf2> -f --ser <serial_id>` — the pipeline passes the UF2 under `firmware.build_dir`. **`picotool load` reboots the Pico into application mode**; a separate `picotool reboot` is **not** run (it races USB re-enumeration and often fails with exit **249**).
- Optional **1 s** pause after load (env `LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC`, set `0` to disable) before polling USB.
- Wait (default **5 s**, env `LUCY_PIPELINE_FLASH_WAIT_SEC`) until USB serial reappears (case-insensitive serial substring match).
- Verify the board answers a Modbus FC03 read of holding register 0 (up to **30 s**, env `LUCY_PIPELINE_FLASH_UPTIME_WAIT_SEC`).

Shell aliases must use a **full path** to the `.uf2` or **`cd`** to the firmware `build/` directory first.

### Passwordless sudo for picotool

`picotool` runs under **sudo**. On Ubuntu 24.04, for unattended flashing install a **sudoers drop-in** that whitelists the **`picotool` binary only** (use the path from `command -v picotool` / `which picotool`):

```bash
PICOTOOL="$(command -v picotool)"
test -n "$PICOTOOL" && test -x "$PICOTOOL" || { echo "install picotool first"; exit 1; }

echo "$USER ALL=(ALL) NOPASSWD: $PICOTOOL" | sudo tee /etc/sudoers.d/99-lucy-picotool
sudo chmod 0440 /etc/sudoers.d/99-lucy-picotool
sudo visudo -cf /etc/sudoers.d/99-lucy-picotool
```

If `visudo -cf` prints **syntax OK**, the rule is valid. Replace **`$USER`** with the account that runs the ROS node if you run this from a root shell where `$USER` is `root`. If `picotool` moves to another path, update the rule or re-run the block.

Env: `LUCY_PIPELINE_FLASH_TIMEOUT_SEC` (default **120**) caps each `sudo picotool …` subprocess.

## Services

- `config/list` (`lucy_msgs/srv/ListConfigs`)
- `config/get` (`lucy_msgs/srv/GetConfig`)
- `config/save` (`lucy_msgs/srv/SaveConfig`)
- `config/activate` (`lucy_msgs/srv/ActivateConfig`)
- `config/delete` (`lucy_msgs/srv/DeleteConfig`)

## Action

- `configure_pipeline` (`lucy_msgs/action/ConfigurePipeline`)

## Launch

```bash
ros2 launch lucy_config_pipeline config_pipeline.launch.py robot_package:=thais_urdf
```

Optional: pass `config_dir:=/abs/path/to/config/hardware` to override source-path detection.
