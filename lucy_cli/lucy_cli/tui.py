"""
This module contains all the logic for rendering the Text-based User Interface (TUI).

It is designed to be completely decoupled from ROS or any other backend logic.
It receives data structures to display and uses callbacks to notify the main
application of user input.

Responsibilities:
- Clearing the screen.
- Drawing menus and lists.
- Handling raw keyboard input with timeouts.
- Maintaining a persistent input buffer across screen refreshes.
"""

import os
import select
import sys
import time

# Global buffer to hold unsubmitted user input, allowing it to persist
# across screen refreshes in auto-refresh mode.
INPUT_BUFFER = ''

# Self-pipe that lets a background thread wake get_user_input() from its select() to force a redraw.
# Only effective on POSIX, where select() watches it.
_wake_r, _wake_w = os.pipe()


def notify_event():
    """Wake a blocked get_user_input() so the TUI can redraw on a state change.

    Safe to call from a background (ROS executor) thread.
    """
    try:
        os.write(_wake_w, b'x')
    except OSError:
        pass


def clear_screen():
    """Clears the terminal screen using ANSI escape codes for a flicker-free update."""
    if os.name == 'posix':
        sys.stdout.write('\033[H\033[J')
        sys.stdout.flush()
    else:
        os.system('cls')


def render_sensor_graph(history, height: int = 12, width: int = 60) -> str:
    """Renders a list of recent sensor values as an ASCII line graph.

    The vertical axis is scaled to the min/max of the visible window, with
    the max value labeled on the top row and the min value on the bottom row.
    """
    data = list(history)[-width:]
    values = [v for v in data if v is not None]
    if not values:
        return 'No data yet.'

    vmax = max(values)
    vmin = min(values)
    cols = width
    pad = width - len(data)

    rows_idx = [None] * pad
    for v in data:
        if v is None:
            rows_idx.append(None)
            continue
        if vmax == vmin or height <= 1:
            rows_idx.append(height - 1)
        else:
            frac = (v - vmin) / (vmax - vmin)
            rows_idx.append(round((1 - frac) * (height - 1)))

    grid = [[' '] * cols for _ in range(height)]
    for col, row in enumerate(rows_idx):
        if row is not None:
            grid[row][col] = '*'

    label_width = 9
    lines = [' ' * label_width + ' +' + '-' * cols + '+']
    for row in range(height):
        if row == 0:
            label = f'{vmax:>{label_width}.3f}'
        elif row == height - 1:
            label = f'{vmin:>{label_width}.3f}'
        else:
            label = ' ' * label_width
        lines.append(label + ' |' + ''.join(grid[row]) + '|')
    lines.append(' ' * label_width + ' +' + '-' * cols + '+')
    return '\n'.join(lines)


def get_user_input(prompt: str, timeout: float = 1.0) -> str | None:
    """
    Waits for user input with a timeout, preserving typed characters.

    This function uses non-blocking, character-by-character input on POSIX
    systems to provide a smooth TUI experience, even when the screen is
    auto-refreshing.

    Args:
        prompt: The input prompt to display to the user.
        timeout: The time in seconds to wait for input before returning.

    Returns:
        - The user's full line of input if they press Enter.
        - An empty string ("") if the user typed but did not press Enter.
        - None if the timeout was reached with no user input.
    """
    global INPUT_BUFFER
    sys.stdout.write(prompt + INPUT_BUFFER)
    sys.stdout.flush()

    if os.name != 'posix':
        # Fallback for non-POSIX systems (like Windows) which lacks tty/termios.
        # This will not have the non-blocking, char-by-char benefits.
        res = input()
        return res

    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ready = select.select([sys.stdin, _wake_r], [], [], timeout)[0]
        if _wake_r in ready:
            # A background state change asked for a redraw. Drain the pipe.
            try:
                os.read(_wake_r, 65536)
            except OSError:
                pass
            # If the user wasn't also typing, redraw without touching the buffer.
            if sys.stdin not in ready:
                return None
        if sys.stdin in ready:
            while True:
                char = sys.stdin.read(1)
                if char == '\n' or char == '\r':  # Enter pressed
                    result = INPUT_BUFFER
                    INPUT_BUFFER = ''
                    sys.stdout.write('\n')
                    return result
                elif char == '\x7f' or char == '\b':  # Backspace
                    if INPUT_BUFFER:
                        INPUT_BUFFER = INPUT_BUFFER[:-1]
                        # Move cursor back, write space, move back again
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif char == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
                elif char.isprintable():
                    INPUT_BUFFER += char
                    sys.stdout.write(char)
                    sys.stdout.flush()

                # Check if more characters are available to be read immediately
                if not select.select([sys.stdin], [], [], 0)[0]:
                    break
            # User typed, but didn't press Enter. Redraw will handle the buffer.
            return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # Timeout occurred
    return None


def display_control_status(state: dict):
    """Prints the control-status banner shown on every TUI screen.

    Always states who holds control (us, another client, or nobody) and always
    reminds the user of the 'c' key, so the shortcut stays discoverable from any
    screen rather than only the main menu.

    Args:
        state: The current UI state; reads 'has_control' and 'active_controller'.
    """
    if state.get('has_control'):
        print('>> YOU ARE IN CONTROL of the robot <<')
        print("Type 'c' to release control.")
        return

    controller = state.get('active_controller')
    if controller:
        print(f'!! READ-ONLY - CONTROLLED BY: {controller} !!')
    else:
        print('No client has control.')
    print("Type 'c' to take control.")


def display_help_screen():
    """Displays a static help message and waits for user confirmation."""
    clear_screen()
    print('--- TUI Help ---')
    print('\n[General Commands]')
    print("  'a'         - Toggle auto-refresh mode on/off.")
    print("  'h'         - Display this help screen.")
    print("  'q'         - Quit the application.")
    print('\n[Main Menu]')
    print('  <number>    - Select an actuator group to view and edit its joints.')
    print(
        "  'c'         - Toggle control: take control of the robot, or release it if you already hold it."
    )
    print('\n[Joint Menu]')
    print('  <number>    - Select a joint to modify its angle.')
    print("  'b'         - Go back to the main menu.")
    print('\n[Sensor Menu]')
    print('  <number>    - Select a sensor to view its live value.')
    print("  'b'         - Go back to the main menu.")
    print('\nPress Enter to return...')
    input()


def display_control_taken_popup(controller_id: str):
    """Full-screen notice shown when another client takes control from us.

    Blocks until the user acknowledges, mirroring the front-end popup behaviour.
    """
    global INPUT_BUFFER
    INPUT_BUFFER = ''  # Discard anything typed before the takeover.
    clear_screen()
    print('=' * 50)
    print('  CONTROL TAKEN')
    print('=' * 50)
    print(f"\n  '{controller_id}' has taken control of the robot.")
    print('  You are now in read-only mode.\n')
    print('  Press ENTER to continue...')
    input()


def display_main_menu(state: dict):
    """
    Renders the main menu screen.

    Args:
        state: A dictionary containing the current UI state, including:
               'client_count', 'autorefresh', 'has_control', 'active_controller',
               and 'actuator_groups'.
    """
    print('--- Robot Monitoring TUI ---\n')
    print(f"Connected Clients: {state.get('client_count', 'N/A')}")
    if state.get('autorefresh'):
        print('[Auto-Refresh: ON]')

    display_control_status(state)

    print('\nSelect an actuator group:')
    for i, name in enumerate(state.get('actuator_groups', [])):
        print(f'{i+1}. {name}')
    print("\nEnter 'q' to quit or 'h' for help.")


def display_category_menu(state: dict, group_name: str):
    """
    Renders the category selection menu for a board group: actuators vs sensors.

    Args:
        state: A dictionary containing the current UI state.
        group_name: The name of the board group being displayed.
    """
    print(f'--- {group_name} ---\n')
    display_control_status(state)
    print()

    print('1. Actuators')
    print('2. Sensors')
    print(
        "\nEnter a number to select a category, 'b' to go back, 'h' for help, or 'q' to quit."
    )


def display_sensor_menu(state: dict, group_name: str):
    """
    Renders the menu listing sensors within a specific group.

    Args:
        state: A dictionary containing the current UI state.
        group_name: The name of the board group being displayed.
    """
    print(f'--- {group_name} (Sensors) ---')
    display_control_status(state)
    print()

    sensors = state.get('sensors', {}).get(group_name, {}).get('sensors', [])
    for i, sensor in enumerate(sensors):
        min_value = sensor.get('min_value')
        max_value = sensor.get('max_value')
        range_str = (
            f'{min_value} - {max_value}'
            if min_value is not None or max_value is not None
            else 'N/A'
        )
        print(f"{i+1}. {sensor['name']} ({sensor['type']}) - (Range: {range_str})")

    print(
        "\nEnter sensor number to view details, 'b' to go back, 'h' for help, or 'q' to quit."
    )


def display_joint_menu(state: dict, group_name: str):
    """
    Renders the menu for editing joints within a specific group.

    Args:
        state: A dictionary containing the current UI state.
        group_name: The name of the actuator group being displayed.
    """
    print(f'--- {group_name} ---')
    display_control_status(state)
    print()

    joints = state.get('actuators', {}).get(group_name, {}).get('joints', [])
    for i, joint in enumerate(joints):
        print(
            f"{i+1}. {joint['name']}: {joint['value']:.1f}° (Min: {joint['min']:.0f}, Max: {joint['max']:.0f})"
        )

    if state.get('has_control'):
        print(
            "\nEnter joint number to edit, 'b' to go back, 'h' for help, or 'q' to quit."
        )
    else:
        print("\nEnter 'b' to go back, 'h' for help, or 'q' to quit.")


def get_new_joint_value(joint_name: str) -> float | None:
    """
    Prompts the user for a new joint value.

    This uses standard blocking input as it's a specific data entry task.

    Args:
        joint_name: The name of the joint being edited.

    Returns:
        The new value as a float, or None if input is invalid.
    """
    global INPUT_BUFFER
    INPUT_BUFFER = ''  # Clear buffer before this prompt
    try:
        value_str = input(f'Enter new value for {joint_name} (in degrees): ')
        return float(value_str)
    except (ValueError, TypeError):
        print('Invalid input. Please enter a number.')
        time.sleep(1)
        return None
