# Developer Guide: Creating a New Lucy Interface

This guide explains how to build a new client application (e.g., a web UI, a different command-line tool, a Python script for automation) that can interact with and control a Lucy-enabled robot.

The core of the Lucy control system is a set of ROS 2 topics and services. By interacting with these, any application can view robot state, get its hardware configuration, and send commands.

The recommended way to interface with Lucy is to use the `lucy_cli.ros_interface.LucyROSInterface` class as a reference. It provides a high-level, well-documented Python API that handles all the underlying ROS 2 complexity. This guide will walk through the key concepts and provide code snippets based on that reference implementation.

> Please note that the LucyROSInterface class is a simple example. You will find that more topics & services are available if needed, with for example a complete configuration workflow. If you need more feature or want to write your interface in another language, please check the differents packages available in this repository. The [Lucy Control Panel](https://github.com/Lucy-Robotics/lucy_control_panel/tree/master/) is a good example of a more in-depth implementation.

## 1. Core Concepts

### Presence and Control Arbitration (the client registry)

Presence (the connected-client count) and control arbitration are owned by a single transport-agnostic node, `lucy_client_registry`.
Every interface — the web control panel over rosbridge, the native CLI directly over DDS, your own script — participates in exactly the same way, and **nothing here depends on a rosbridge connection**.
The registry is the only writer of the count and the active controller; clients register by heartbeat and change control through a service.

A good client ID is one that is both unique and descriptive, for example: `my_app_1678886400_a3f7`.

**Registering presence (heartbeat).** Pulse your client ID on `/lucy/client_heartbeat` (`std_msgs/msg/String`, volatile) roughly once per second.
The registry counts you as connected while your heartbeats keep arriving and prunes you automatically about three seconds after they stop (three missed beats at the 1 Hz cadence) — so a crash or a closed tab is handled for free, with no explicit "disconnect" step.

```python
# A unique identifier for this specific client instance.
CLIENT_ID = f"my_app_{int(time.time())}_{random.randint(1000, 9999)}"

# ... inside your ROS Node class ...
# self.heartbeat_publisher = self.create_publisher(String, '/lucy/client_heartbeat', qos_profile=VOLATILE_QOS)
# self.create_timer(1.0, lambda: self.heartbeat_publisher.publish(String(data=CLIENT_ID)))
```

**Taking and releasing control.** Call the `/lucy/control` service (`lucy_msgs/srv/ClientControl`) with your client ID and `acquire=True` to take control, or `acquire=False` to release it.
Control is preemptive (the latest acquirer wins); if the controlling client's heartbeat lapses, the registry releases control automatically so the robot is never stuck owned by a dead client.

**Monitoring control.** Subscribe to `/lucy/active_client` (`std_msgs/msg/String`, latching).
If you receive a non-empty client ID that is *not* your own, enter a "read-only" mode and disable your command publishing.

```python
def take_control(self):
    self._send_control_request(acquire=True)

def release_control(self):
    self._send_control_request(acquire=False)

def _send_control_request(self, acquire: bool):
    """The resulting controller comes back on /lucy/active_client."""
    if not self._control_client.wait_for_service(timeout_sec=2.0):
        self.get_logger().warn("'/lucy/control' service not available.")
        return
    req = ClientControl.Request(client_id=CLIENT_ID, acquire=acquire)
    self._control_client.call_async(req)  # fire-and-forget
```

### Dynamic Hardware Configuration

You should **never** hardcode joint names, limits, or controller topics in your application. Lucy provides a service to fetch the currently active hardware configuration from the robot.

- **Service:** `/config/get`
- **Service Type:** `lucy_msgs/srv/GetConfig`

To get the currently active configuration, call this service with an empty `config_name`. The service will return a YAML string containing all the necessary information.

```python
# From lucy_cli/ros_interface.py

def get_hardware_config_yaml(self) -> str | None:
    """
    Synchronously calls the /config/get service to fetch the active config.
    """
    try:
        # Wait for the service to be available
        if not self._get_config_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'/config/get' service not available.")
            return None
            
        # Call the service with an empty name to get the active config
        req = GetConfig.Request(config_name="")
        future = self._get_config_client.call_async(req)
        
        # This requires spinning a temporary node to wait for the result
        temp_node = Node('temp_service_caller')
        rclpy.spin_until_future_complete(temp_node, future, timeout_sec=5.0)
        temp_node.destroy_node()

        response = future.result()
        if not response or not response.success:
            self.get_logger().error(f"Failed to get active config: {response.message if response else 'timeout'}")
            return None
            
        self.get_logger().info("Successfully fetched active hardware configuration.")
        return response.config_yaml
    except Exception as e:
        self.get_logger().error(f"Exception in get_hardware_config_yaml: {e}")
        return None
```

The returned YAML needs to be parsed to build your UI and command structures. See `tui_node.py`'s `parse_config_yaml` function for a reference implementation.

## 2. Sending Commands

### Commanding Joint Positions

To move the robot's joints, you must publish to the appropriate [`ros2_control`](https://control.ros.org/rolling/index.html) controller topic. These topics are discovered from the hardware configuration YAML.

- **Topic:** Dynamically determined (e.g., `/left_arm_controller/joint_trajectory`)
- **Message Type:** `trajectory_msgs/msg/JointTrajectory`
- **QoS:** Volatile (`VOLATILE` durability)

A `JointTrajectory` message contains a list of joint names and one or more `JointTrajectoryPoint`s. For direct position control, you only need to send one point.

**Important:** The `joint_names` list **must** be in the exact order expected by the controller, which is the order you get from parsing the hardware config YAML.

```python
# From lucy_cli/ros_interface.py

def publish_joint_trajectory(self, controller_topic: str, joint_names: list[str], positions_rad: list[float]):
    """
    Publishes a JointTrajectory message to a specified controller.
    """
    # Create a publisher for this topic if it doesn't exist yet
    if controller_topic not in self._joint_publishers:
        self.get_logger().warn(f"Creating new publisher for an unknown topic: {controller_topic}")
        self._joint_publishers[controller_topic] = self.create_publisher(
            JointTrajectory, controller_topic, qos_profile=VOLATILE_QOS)

    publisher = self._joint_publishers[controller_topic]
    
    msg = JointTrajectory()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.joint_names = joint_names # Must be in the correct order
    
    point = JointTrajectoryPoint()
    point.positions = positions_rad # Must be in radians
    # A small non-zero time is required for the controller to accept the goal
    point.time_from_start = Duration(nanoseconds=200000000).to_msg()  # 0.2s
    
    msg.points.append(point)
    publisher.publish(msg)
```

## 3. Subscribing to State

### Monitoring Connected Clients

You can monitor how many clients are connected to the control system.
This is the universal count maintained by the registry from live heartbeats — it includes *every* interface (web and native alike), not just rosbridge WebSocket clients.

- **Topic:** `/lucy/client_count`
- **Message Type:** `std_msgs/msg/Int32`
- **QoS:** Latching (`TRANSIENT_LOCAL` durability)

- **Service:** `/lucy/get_client_count`
- **Service Type:** `lucy_msgs/srv/GetInt`

**Protocol:**
1.  **On Startup:** Call the `/lucy/get_client_count` service to get the number of clients already connected.
2.  **Continuously:** Subscribe to the `/lucy/client_count` topic to receive live updates whenever a client connects or disconnects.

```python
# From lucy_cli/ros_interface.py

# In __init__:
# self.create_subscription(Int32, '/lucy/client_count', self._client_count_callback, qos_profile=LATCHING_QOS)

def _client_count_callback(self, msg: Int32):
    """Handles incoming messages on the client count topic."""
    new_count = msg.data
    # ... (update internal state and notify application) ...

# --- In your startup logic ---
def get_initial_client_count(self) -> int | None:
    """
    Synchronously calls the /lucy/get_client_count service.
    """
    # ... (implementation similar to get_hardware_config_yaml) ...
```

### Other Important Topics

- **`/joint_states`** (`sensor_msgs/msg/JointState`): This topic is published by `ros2_control` and contains the *actual*, measured position of every joint on the robot. You should subscribe to this to display the robot's real-time state, which may differ from your last commanded position.

## Summary of Endpoints

| Endpoint | Type | Message/Service Type | QoS Durability | Purpose |
|---|---|---|---|---|
| `/lucy/client_heartbeat` | Topic | `std_msgs/msg/String` | `VOLATILE` | Pulse your client ID ~1 Hz to register as connected. |
| `/lucy/active_client` | Topic | `std_msgs/msg/String` | `TRANSIENT_LOCAL` | Monitor which client currently holds control. |
| `/lucy/client_count` | Topic | `std_msgs/msg/Int32` | `TRANSIENT_LOCAL` | Get live updates on the number of connected clients. |
| `/lucy/control` | Service | `lucy_msgs/srv/ClientControl` | N/A | Take or release control (`acquire` flag). |
| `/lucy/get_client_count` | Service | `lucy_msgs/srv/GetInt` | N/A | Get the initial number of connected clients on startup. |
| `/config/get` | Service | `lucy_msgs/srv/GetConfig` | N/A | Fetch the active robot hardware configuration (YAML). |
| `/{controller_name}/joint_trajectory` | Topic | `trajectory_msgs/msg/JointTrajectory` | `VOLATILE` | Send joint position commands to a specific controller. |
| `/joint_states` | Topic | `sensor_msgs/msg/JointState` | `VOLATILE` | Read the actual, measured state of all robot joints. |

By using these endpoints, you can build any application to reliably and safely interact with the Lucy robot ecosystem.