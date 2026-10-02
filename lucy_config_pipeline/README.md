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
| 4 | **FLASH** | skipped in `simulation_only` / `build_only=true` | `picotool load` (no sudo by default) per board (see below) |
| 5 | **RELOAD** | yes | calls `/lucy_control/restart` so `robot_state_publisher` + `ros2_control` re-read URDF + controller YAML; Gazebo topology changes still require a relaunch |

Decoupling **GENERATE** from **BUILD** is what lets the LCP "SIMULATION ONLY" toggle update URDF limits and ros2_control wiring without a firmware build.

## Flash phase (RP2040)

After a successful build, each selected board with a non-empty `serial_id` is flashed using:

- `picotool load <uf2> -x -f --ser <serial_id>` — **no sudo by default**.
  `-x` executes the image after load (required when flashing from BOOTSEL so the
  board leaves RPI-RP2 and enumerates CDC). Install USB access once via
  `pixi run firmware-setup` (udev + optional passwordless sudoers). Force sudo
  only if needed: `LUCY_PIPELINE_FLASH_USE_SUDO=1` (uses `sudo -n`).
- Firmware uses Raspberry Pi USB VID ``0x2e8a`` with a picotool Reset
  interface so **reflash does not need BOOTSEL** (`picotool load -x -f --ser …`).
  Boards still on legacy Custom VID ``16c0`` need **one** BOOTSEL flash to
  upgrade, then subsequent flashes are hands-free.
- **`serial_id` in YAML is the Pico flash unique id** (same as `picotool info`
  `flash id`, e.g. `E6617C93E3858429`). Firmware exposes that string as the USB
  CDC serial; flash success is **not** “picotool printed 100%” alone.
- Optional pause after load (default **2 s**, env `LUCY_PIPELINE_FLASH_POST_LOAD_DELAY_SEC`,
  set `0` to disable) before polling USB.
- Wait (default **30 s**, env `LUCY_PIPELINE_FLASH_WAIT_SEC`) until USB CDC whose
  by-id / port metadata contains `serial_id` reappears.
- Verify the board answers a Modbus FC03 read of holding register 0 (up to **30 s**,
  env `LUCY_PIPELINE_FLASH_UPTIME_WAIT_SEC`).

Shell aliases must use a **full path** to the `.uf2` or **`cd`** to the firmware `build/` directory first.

### Pico USB access (no interactive sudo during flash)

Run once (prompts for your password during setup only):

```bash
pixi run firmware-setup
# or explicitly:
python scripts/install_pico_usb_access.py
```

That installs (Linux):

1. **udev** rules (`/etc/udev/rules.d/99-lucy-pico.rules`) for vendors `2e8a`
   (Pico BOOTSEL) and `16c0` (Lucy Custom Servo2040 CDC) so `picotool` and
   `ttyACM*` are usable without root.
2. **dialout** membership when that group exists (log out/in afterward).
3. Optional **passwordless sudoers** for the `picotool` on your `PATH` (fallback;
   pipeline prefers non-sudo).

On Windows / macOS the same Python entrypoint documents local tooling (no udev);
pipeline flash already runs `picotool` without sudo there.

Then check:

```bash
# Pico in BOOTSEL:
picotool info -a
# After firmware is running:
ls /dev/serial/by-id/
```

NixOS / read-only `/etc`: the installer writes the same rules to
**`/run/udev/rules.d/99-lucy-pico.rules`** (tmpfs — lasts until reboot). No
`configuration.nix` change required; re-run `python scripts/install_pico_usb_access.py`
after reboot (one sudo). Optional persistent Nix snippet is printed by the script
if you later want `services.udev.extraRules`.

Env: `LUCY_PIPELINE_FLASH_TIMEOUT_SEC` (default **120**) caps each `picotool …` subprocess.
`LUCY_FIRMWARE_SETUP_SKIP_UDEV=1` skips udev during firmware-setup (CI).
`LUCY_FIRMWARE_SETUP_INSTALL_UDEV=1` forces udev install in non-interactive shells.

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
