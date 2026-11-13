# Lucy CLI

This package provides a command-line interface for interacting with Lucy.

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Lucy-Robotics/lucy_ros_packages
   ```
2. **Build the package:**
   Navigate to your ROS 2 workspace and build the package using `colcon`:
   ```bash
   colcon build --packages-select lucy_cli
   ```
3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

The main command provided by this package is `tui`, which launches a textual user interface.

To run the TUI, use the following command:
```bash
ros2 run lucy_cli tui
```

## For Developers

For more information on the development of this package, please see the [developer documentation](developer.md).

## License

This project is licensed under the GPL-3.0 License. See the `LICENSE` file for more details.
