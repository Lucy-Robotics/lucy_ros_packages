"""
Entry point for the Lucy CLI TUI actuator node: connects the ROS layer
(ros_interface.py) to the terminal UI (tui.py) and runs the menu loops.
"""
import argparse
from collections import deque
import math
import os
import sys
import time

import rclpy
import yaml

from . import tui
from .ros_interface import LucyROSInterface
from .tui import clear_screen
from .tui import display_category_menu
from .tui import display_control_status
from .tui import display_control_taken_popup
from .tui import display_help_screen
from .tui import display_joint_menu
from .tui import display_main_menu
from .tui import display_sensor_menu
from .tui import get_new_joint_value
from .tui import get_user_input
from .tui import render_sensor_graph

GRAPH_WIDTH = 60

# Shared between the on_control_change callback (background ROS thread) and the TUI loop.
# 'by' is the id of the client to announce in the takeover popup, or None when there is nothing to announce.
# 'was_ours' is whether we held control at the previous change.
_control_takeover = {'by': None, 'was_ours': False}

def _consume_control_takeover() -> bool:
    """Show the takeover popup if another client just grabbed control.

    Returns True if a popup was shown (the caller should restart its loop so the
    menu redraws with the updated state).
    """
    controller_id = _control_takeover['by']
    if controller_id is None:
        return False
    _control_takeover['by'] = None
    display_control_taken_popup(controller_id)
    return True

def _actuator_deg_to_joint_rad(actuator_deg: float, mapping: dict) -> float:
    """Servo degrees -> URDF joint radians. Mirrors LCP's actuatorDegToJointRad
    so commands from the CLI land where LCP would put them."""
    joint_deg = (actuator_deg - mapping['offset_deg']) * mapping['direction'] * mapping['scale']
    return math.radians(joint_deg)

def _joint_rad_to_actuator_deg(joint_rad: float, mapping: dict) -> float:
    """URDF joint radians -> servo degrees. Mirrors LCP's jointRadToActuatorDeg
    so /joint_states feedback reads in the same units the menu displays."""
    denom = mapping['direction'] * mapping['scale']
    if denom == 0:
        return mapping['offset_deg']
    return math.degrees(joint_rad) / denom + mapping['offset_deg']

def _build_state(ros: LucyROSInterface, actuators: dict, autorefresh: bool) -> dict:
    """Snapshots the current UI state passed to the rendering functions.

    Overlays each joint's displayed value with its actual measured position from
    /joint_states (URDF radians), converted to servo degrees via the joint's
    per-actuator calibration, so the menus show the robot's live state in the
    same units LCP uses regardless of which client moved it. Joints without
    feedback yet keep their config default.
    """
    live_positions_rad = ros.get_joint_positions_rad()
    for group in actuators.values():
        for joint in group['joints']:
            position_rad = live_positions_rad.get(joint['name'])
            if position_rad is not None:
                joint['value'] = _joint_rad_to_actuator_deg(position_rad, joint['mapping'])
    return {
        'autorefresh': autorefresh,
        'client_count': ros.client_count,
        'has_control': ros.has_control(),
        'active_controller': ros.active_controller,
        'actuator_groups': list(actuators.keys()),
        'actuators': actuators,
    }

def _build_sensor_state(ros: LucyROSInterface, sensors: dict, autorefresh: bool) -> dict:
    """Snapshots the UI state passed to the sensor menu rendering function.

    No live sensor values yet (ros.get_sensor_values() isn't wired up); this
    just exposes the static, parsed sensor descriptors plus the common control
    state shared with the other menus.
    """
    return {
        'autorefresh': autorefresh,
        'client_count': ros.client_count,
        'has_control': ros.has_control(),
        'active_controller': ros.active_controller,
        'sensors': sensors,
    }

def _prompt_choice(autorefresh: bool) -> str | None:
    """Reads one menu choice, returning None when the user typed nothing.

    The short timeout under auto-refresh lets the screen repaint periodically;
    otherwise we wait essentially indefinitely (a background event still wakes
    the input via tui.notify_event()).
    """
    timeout = 1.0 if autorefresh else 3600.0
    choice = get_user_input('> ', timeout=timeout)
    if choice is None or choice == '':
        return None
    return choice.lower()

def _toggle_control(ros: LucyROSInterface, has_control: bool):
    """Takes or releases control, whichever the current state calls for.

    Shared by the menus and by the sensor monitor screen so that the 'c' key
    advertised by tui.display_control_status() works everywhere it is shown.
    """
    if has_control:
        ros.release_control()
        print('Releasing control...')
    else:
        ros.take_control()
        print('Requesting control...')
    time.sleep(0.5)

def _handle_common_command(ros: LucyROSInterface, choice: str, autorefresh: bool,
                           has_control: bool) -> tuple[bool, bool, bool]:
    """Handles keys shared by every menu (quit / help / auto-refresh / control).

    Returns (handled, autorefresh, quit): `handled` is False when `choice` is not
    a common command and the caller should apply its menu-specific logic.
    """
    if choice == 'q':
        return True, autorefresh, True
    if choice == 'h':
        display_help_screen()
        return True, autorefresh, False
    if choice == 'a':
        autorefresh = not autorefresh
        print(f"Auto-refresh {'enabled' if autorefresh else 'disabled'}.")
        time.sleep(0.7)
        return True, autorefresh, False
    if choice == 'c':
        _toggle_control(ros, has_control)
        return True, autorefresh, False
    return False, autorefresh, False

def run_tui(ros: LucyROSInterface, actuators: dict, sensors: dict, autorefresh: bool):
    """
    The main application loop.

    Args:
        ros: The initialized LucyROSInterface instance.
        actuators: The dynamic actuator configuration dictionary.
        sensors: The dynamic sensor configuration dictionary.
        autorefresh: Boolean indicating if auto-refresh is enabled by default.
    """
    while rclpy.ok():
        if _consume_control_takeover():
            continue

        clear_screen()
        state = _build_state(ros, actuators, autorefresh)
        display_main_menu(state)

        choice = _prompt_choice(autorefresh)
        if choice is None:
            continue

        handled, autorefresh, quit_app = _handle_common_command(
            ros, choice, autorefresh, state['has_control'])
        if quit_app:
            break
        if handled:
            continue

        # Menu-specific: select an actuator group by number.
        try:
            idx = int(choice) - 1
            if not (0 <= idx < len(state['actuator_groups'])):
                raise ValueError
        except ValueError:
            print('Invalid input. Refreshing...')
            time.sleep(1)
            continue

        group_name = state['actuator_groups'][idx]
        autorefresh, quit_app = handle_category_menu(ros, actuators, sensors, group_name, autorefresh)
        if quit_app:
            break

def handle_category_menu(ros: LucyROSInterface, actuators: dict, sensors: dict, group_name: str,
                         autorefresh: bool) -> tuple[bool, bool]:
    """Handles the category sub-menu for a board group: choose actuators or sensors.

    Returns (autorefresh, quit): `quit` is True when the user asked to quit the
    whole application, so the caller can unwind to main()'s cleanup.
    """
    while rclpy.ok():
        if _consume_control_takeover():
            continue

        clear_screen()
        state = _build_state(ros, actuators, autorefresh)
        display_category_menu(state, group_name)

        choice = _prompt_choice(autorefresh)
        if choice is None:
            continue

        if choice == 'b':
            return autorefresh, False

        handled, autorefresh, quit_app = _handle_common_command(
            ros, choice, autorefresh, state['has_control'])
        if quit_app:
            return autorefresh, True
        if handled:
            continue

        if choice == '1':
            autorefresh, quit_app = handle_actuator_menu(ros, actuators, group_name, autorefresh)
            if quit_app:
                return autorefresh, True
        elif choice == '2':
            autorefresh, quit_app = handle_sensor_menu(ros, sensors, group_name, autorefresh)
            if quit_app:
                return autorefresh, True
        else:
            print('Invalid input. Refreshing...')
            time.sleep(1)

    return autorefresh, False

def handle_sensor_menu(ros: LucyROSInterface, sensors: dict, group_name: str,
                       autorefresh: bool) -> tuple[bool, bool]:
    """Handles the sub-menu for viewing a board group's sensors.

    Read-only listing (no live values yet, no editing): just numbers the
    sensors 1..n the same way handle_actuator_menu numbers joints.

    Returns (autorefresh, quit): `quit` is True when the user asked to quit the
    whole application, so the caller can unwind to main()'s cleanup.
    """
    while rclpy.ok():
        if _consume_control_takeover():
            continue

        clear_screen()
        state = _build_sensor_state(ros, sensors, autorefresh)
        display_sensor_menu(state, group_name)

        choice = _prompt_choice(autorefresh)
        if choice is None:
            continue

        if choice == 'b':
            return autorefresh, False

        handled, autorefresh, quit_app = _handle_common_command(
            ros, choice, autorefresh, state['has_control'])
        if quit_app:
            return autorefresh, True
        if handled:
            continue

        # Menu-specific: monitor a sensor's live value by number.
        _monitor_sensor(ros, sensors, group_name, choice, autorefresh)

    return autorefresh, False

def _monitor_sensor(ros: LucyROSInterface, sensors: dict, group_name: str, choice: str,
                    autorefresh: bool):
    """Continuously displays one sensor's live value until the user goes back.

    Read-only: unlike _edit_joint, there's nothing to write, so this loops
    polling ros.get_sensor_values() and redrawing instead of doing a single
    blocking input() round-trip.
    """
    sensor_list = sensors.get(group_name, {}).get('sensors', [])
    try:
        idx = int(choice) - 1
        if not (0 <= idx < len(sensor_list)):
            raise ValueError
    except ValueError:
        print('Invalid input.')
        time.sleep(1)
        return

    sensor = sensor_list[idx]
    history = deque(maxlen=GRAPH_WIDTH)
    while rclpy.ok():
        clear_screen()
        state = _build_sensor_state(ros, sensors, autorefresh)
        value = ros.get_sensor_values().get(sensor['name'])
        history.append(value)
        value_str = f'{value:.3f}' if value is not None else 'N/A (no data yet)'
        print(f"--- Monitoring: {sensor['name']} ---\n")
        display_control_status(state)
        print()
        print(f"Type: {sensor['type']}")
        print(f"Associated actuator: {sensor.get('associated_actuator') or 'N/A'}")
        print(f'Value: {value_str}\n')
        print(render_sensor_graph(history))
        print("\nPress 'b' to go back.")

        choice = get_user_input('> ', timeout=0.5)
        if choice is None:
            continue
        choice = choice.lower()
        if choice == 'b':
            return
        if choice == 'c':
            _toggle_control(ros, state['has_control'])

def handle_actuator_menu(ros: LucyROSInterface, actuators: dict, group_name: str,
                      autorefresh: bool) -> tuple[bool, bool]:
    """Handles the sub-menu for a specific actuator group.

    Returns (autorefresh, quit): `quit` is True when the user asked to quit the
    whole application, so the caller can unwind to main()'s cleanup.
    """
    while rclpy.ok():
        if _consume_control_takeover():
            continue

        clear_screen()
        state = _build_state(ros, actuators, autorefresh)
        display_joint_menu(state, group_name)

        choice = _prompt_choice(autorefresh)
        if choice is None:
            continue

        if choice == 'b':
            return autorefresh, False

        handled, autorefresh, quit_app = _handle_common_command(
            ros, choice, autorefresh, state['has_control'])
        if quit_app:
            return autorefresh, True
        if handled:
            continue

        # Menu-specific: edit a joint by number (only when we hold control).
        if state['has_control']:
            _edit_joint(ros, state, group_name, choice)

    return autorefresh, False

def _edit_joint(ros: LucyROSInterface, state: dict, group_name: str, choice: str):
    """Prompts for and publishes a new value for the chosen joint, if valid."""
    joints = state['actuators'][group_name]['joints']
    try:
        joint_idx = int(choice) - 1
        if not (0 <= joint_idx < len(joints)):
            raise ValueError
    except ValueError:
        print('Invalid input.')
        time.sleep(1)
        return

    selected_joint = joints[joint_idx]
    new_val = get_new_joint_value(selected_joint['name'])
    if new_val is None:
        return  # get_new_joint_value already reported the error.

    if not (selected_joint['min'] <= new_val <= selected_joint['max']):
        print(f"Value out of range ({selected_joint['min']} - {selected_joint['max']}).")
        time.sleep(1)
        return

    selected_joint['value'] = new_val
    names = [j['name'] for j in joints]
    positions = [_actuator_deg_to_joint_rad(j['value'], j['mapping']) for j in joints]
    topic = state['actuators'][group_name]['topic']
    ros.publish_joint_trajectory(topic, names, positions)
    print('Value updated successfully. Refreshing...')
    time.sleep(1)

def _yaml_angle_deg(actuator: dict, rad_key: str, deg_key: str, default_deg: float) -> float:
    """Read one actuator angle as degrees.

    Current hardware YAML stores radians (`offset`, `limit_min`, `limit_max`,
    `default`). Older files used `*_deg` keys. The TUI menu still displays
    servo degrees.
    """
    if actuator.get(rad_key) is not None:
        return math.degrees(float(actuator[rad_key]))
    if actuator.get(deg_key) is not None:
        return float(actuator[deg_key])
    return default_deg

def parse_actuators_yaml(yaml_string: str) -> dict:
    """Parses the hardware config YAML and builds the ACTUATORS dictionary."""
    if not yaml_string: return {}
    try:
        doc = yaml.safe_load(yaml_string)
        if not isinstance(doc, dict): return {}
        boards, actuators_list = doc.get('boards', {}), doc.get('actuators', [])
        parsed_actuators = {}
        for board_id, bdef in boards.items():
            if not isinstance(bdef, dict): continue
            ctrl_name = bdef.get('controller', {}).get('name')
            if not ctrl_name: continue
            board_actuators = sorted([a for a in actuators_list if isinstance(a, dict) and a.get('board') == board_id], key=lambda a: a.get('virtual_pin', 0))
            joints = []
            for actuator in board_actuators:
                urdf_joint = actuator.get('urdf_joint')
                if not urdf_joint: continue
                joints.append({
                    'name': urdf_joint,
                    'min': _yaml_angle_deg(actuator, 'limit_min', 'servo_min_deg', 0.0),
                    'max': _yaml_angle_deg(actuator, 'limit_max', 'servo_max_deg', 180.0),
                    'value': _yaml_angle_deg(actuator, 'default', 'servo_default_deg', 90.0),
                    # Servo<->URDF calibration (defaults match LCP: identity map).
                    # The menu still speaks degrees; hardware YAML stores radians.
                    'mapping': {
                        'offset_deg': _yaml_angle_deg(actuator, 'offset', 'offset_deg', 0.0),
                        'direction': float(actuator.get('direction', 1.0)),
                        'scale': float(actuator.get('scale', 1.0)),
                    },
                })
            if joints:
                group_name = board_id.replace('rp2040_', '').replace('_', ' ').title()
                parsed_actuators[group_name] = {'topic': f'/{ctrl_name}/joint_trajectory', 'joints': joints}
        return parsed_actuators
    except yaml.YAMLError as e:
        print(f'Error parsing YAML: {e}')
        return {}

def parse_sensors_yaml(yaml_string: str) -> dict:
    """Parses the hardware config YAML and builds the SENSORS dictionary."""
    if not yaml_string: return {}
    try:
        doc = yaml.safe_load(yaml_string)
        if not isinstance(doc, dict): return {}
        boards, sensors_list = doc.get('boards', {}), doc.get('sensors', [])
        parsed_sensors = {}
        for board_id, bdef in boards.items():
            if not isinstance(bdef, dict): continue
            topic_sensor = bdef.get('topic_sensors', {})
            if not topic_sensor: continue
            board_sensors = sorted([s for s in sensors_list if isinstance(s, dict) and s.get('board') == board_id], key=lambda s: s.get('virtual_pin', 0))
            sensors = []
            for sensor in board_sensors:
                sensor_type = sensor.get('type')
                if not sensor_type: continue
                min_value = sensor.get('min_value')
                max_value = sensor.get('max_value')
                enabled = sensor.get('enabled')
                sensors.append({
                    'name': sensor.get('id', 'Unknown'),
                    'type': sensor_type,
                    'associated_actuator': sensor.get('associated_actuator', None),
                    'min_value': float(min_value) if min_value is not None else None,
                    'max_value': float(max_value) if max_value is not None else None,
                    'enabled': bool(enabled) if enabled is not None else None,
                })
            if sensors:
                group_name = board_id.replace('rp2040_', '').replace('_', ' ').title()
                parsed_sensors[group_name] = {'topic': topic_sensor, 'sensors': sensors}
        return parsed_sensors
    except yaml.YAMLError as e:
        print(f'Error parsing YAML: {e}')
        return {}


# The file the launcher's readiness_check tests for: created only once the TUI is up, so the launcher reports LOADING during startup and RUNNING from the main menu on.
READY_MARKER = '/tmp/lucy_cli.ready'

def _set_ready_marker():
    try:
        with open(READY_MARKER, 'w') as f:
            f.write(str(os.getpid()))
    except OSError:
        pass

def _clear_ready_marker():
    try:
        os.remove(READY_MARKER)
    except OSError:
        pass

def _fail(ros: LucyROSInterface, message: str):
    """Reports a startup failure, tears down ROS, and exits non-zero."""
    _clear_ready_marker()
    print(message)
    ros.shutdown()
    rclpy.try_shutdown()
    sys.exit(1)

def main(args=None):
    parser = argparse.ArgumentParser(description='Lucy TUI Actuator Node')
    parser.add_argument('-a', '--autorefresh', action='store_true', help='Enable auto-refresh for the TUI screen')
    
    sys_args = sys.argv[1:] if args is None else args
    parsed_args, ros_args = parser.parse_known_args(sys_args)
    
    # Drop any stale marker from a previous run before we start fetching, so the launcher shows LOADING until this run reaches the main menu.
    _clear_ready_marker()

    rclpy.init(args=ros_args)
    ros_interface = LucyROSInterface()

    # Wake the (possibly blocked) TUI input loop on background state changes so the screen refreshes immediately instead of only after the next keypress.
    def on_client_count_change(_count):
        tui.notify_event()

    # The interface already drains joint-state changes at a fixed rate, so just wake the input loop to redraw.
    def on_joint_state_change():
        tui.notify_event()

    def on_control_change(controller_id):
        # Show the takeover popup only when we held control and another client now holds it.
        # An empty controller_id means control was released.
        now_ours = (controller_id == ros_interface.client_id)
        taken_by_other = bool(controller_id) and not now_ours
        if _control_takeover['was_ours'] and taken_by_other:
            _control_takeover['by'] = controller_id
        _control_takeover['was_ours'] = now_ours
        tui.notify_event()

    ros_interface.register_on_client_count_change(on_client_count_change)
    ros_interface.register_on_control_change(on_control_change)
    ros_interface.register_on_joint_state_change(on_joint_state_change)

    print('Fetching active hardware configuration...')
    config_yaml = ros_interface.get_hardware_config_yaml()
    if not config_yaml:
        _fail(ros_interface, 'Could not retrieve hardware configuration. Is lucy_config_pipeline running?')

    actuators = parse_actuators_yaml(config_yaml)
    sensors = parse_sensors_yaml(config_yaml)
    if not actuators:
        _fail(ros_interface, 'Failed to parse hardware configuration or no actuators found.')

    ros_interface.setup_sensor_subscriptions(sensors)

    print('Fetching initial client count...')
    ros_interface.get_initial_client_count()

    # Config and client count are in; the main menu is about to render.
    _set_ready_marker()

    try:
        run_tui(ros_interface, actuators, sensors, parsed_args.autorefresh)
    except KeyboardInterrupt:
        pass
    finally:
        _clear_ready_marker()
        print('\nShutting down...')
        ros_interface.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()