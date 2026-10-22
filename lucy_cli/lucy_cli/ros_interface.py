"""
This module provides a high-level, reusable Python interface for interacting
with Lucy's core ROS 2 services and topics.

It is designed to be agnostic of any specific user interface (TUI, GUI, web),
allowing developers to build their own control applications by using this class
as a foundation.

The LucyROSInterface class encapsulates all ROS 2-specific logic, including:
- Node and executor management.
- Service clients for fetching configuration and state.
- Publishers for sending commands.
- Subscribers for receiving state updates.
- Correct QoS (Quality of Service) profile management.

Example Usage:
    import rclpy
    from ros_interface import LucyROSInterface

    def on_client_count_change(count):
        print(f"Client count updated: {count}")

    def on_control_change(controller_id):
        print(f"Control changed to: {controller_id}")

    rclpy.init()
    ros_interface = LucyROSInterface()
    ros_interface.register_on_client_count_change(on_client_count_change)
    ros_interface.register_on_control_change(on_control_change)

    # The interface handles spinning in a background thread.
    # You can now use its methods to interact with the robot.

    try:
        config = ros_interface.get_hardware_config()
        if config:
            # ... parse config and build UI ...
            pass
        # Your application logic here...
    finally:
        ros_interface.shutdown()
        rclpy.shutdown()
"""
import random
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from ros_gz_interfaces.msg import Float32Array
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from lucy_msgs.srv import ClientControl
from lucy_msgs.srv import GetConfig
from lucy_msgs.srv import GetInt

# --- Unique Client ID ---
# A unique identifier for this specific client instance.
CLIENT_ID = f'cli_{int(time.time())}_{random.randint(1000, 9999)}'

# --- Topic & Service Names ---
# Presence and control go through the lucy_client_registry node; see lucy_cli/developer.md.
HEARTBEAT_TOPIC = '/lucy/client_heartbeat'
CLIENT_COUNT_TOPIC = '/lucy/client_count'
ACTIVE_CLIENT_TOPIC = '/lucy/active_client'
# Actual, measured joint positions published by ros2_control (URDF radians).
JOINT_STATES_TOPIC = '/joint_states'
GET_CLIENT_COUNT_SERVICE = '/lucy/get_client_count'
CONTROL_SERVICE = '/lucy/control'

# How often we pulse our heartbeat so the registry keeps counting us as connected.
HEARTBEAT_PERIOD_S = 1.0

# Joint moves smaller than this (radians) don't count as a change.
JOINT_STATE_EPSILON_RAD = 0.001
# Store /joint_states at full rate; drain changes to listeners at this period.
JOINT_STATE_DRAIN_PERIOD_S = 1.0

# --- QoS Profiles ---
# For state topics that should be "latched" (i.e., the last published message
# is saved for new subscribers). This is crucial for getting the current
# controller and client count as soon as a client connects.
LATCHING_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
# For command topics where you only care about the latest command and don't
# need persistence. This is the standard for ros2_control joint trajectories.
VOLATILE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)


class LucyROSInterface(Node):
    """
    A ROS 2 node providing a high-level API to interact with Lucy's systems.

    This class manages all ROS 2 publishers, subscribers, and service clients,
    exposing simple methods and callbacks for use in an application. It handles
    its own ROS executor in a background thread, so the user does not need to
    manually call `rclpy.spin()`.
    """

    def __init__(self):
        super().__init__('lucy_ros_interface_node')

        # --- Internal State ---
        self._active_controller_id = ''
        self._client_count = 0
        self._has_control = False
        self._control_lock = threading.Lock()
        self._is_shutting_down = False

        # Latest measured joint positions in URDF radians, keyed by joint name.
        # Written at full rate by the subscription, read by the drain timer / UI.
        self._joint_positions_rad = {}
        # Positions at the last drain; changes are measured against this so slow, sub-epsilon-per-message motion still accumulates past the threshold.
        self._notify_baseline_rad = {}
        self._joint_state_lock = threading.Lock()

        self._sensor_subscriptions = {}     # topic -> rclpy subscription handle
        self._sensor_sources_by_topic = {}  # topic -> [(sensor_id, array_index), ...]
        self._sensor_values = {}            # sensor_id -> float (latest value)

        # --- Callbacks ---
        # These sets will hold functions to be called when state changes.
        self._on_control_change_callbacks = set()
        self._on_client_count_change_callbacks = set()
        self._on_joint_state_change_callbacks = set()

        # --- Publishers ---
        self.heartbeat_publisher = self.create_publisher(
            String, HEARTBEAT_TOPIC, qos_profile=VOLATILE_QOS)
        # Joint publishers are created dynamically after config is loaded.
        self._joint_publishers = {}

        # --- Subscribers ---
        self.create_subscription(
            String, ACTIVE_CLIENT_TOPIC, self._control_callback, qos_profile=LATCHING_QOS)
        self.create_subscription(
            Int32, CLIENT_COUNT_TOPIC, self._client_count_callback, qos_profile=LATCHING_QOS)
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._joint_state_callback, qos_profile=VOLATILE_QOS)

        # --- Service Clients ---
        self._get_config_client = self.create_client(GetConfig, '/config/get')
        self._get_count_client = self.create_client(GetInt, GET_CLIENT_COUNT_SERVICE)
        self._control_client = self.create_client(ClientControl, CONTROL_SERVICE)

        # --- Heartbeat ---
        self._heartbeat_timer = self.create_timer(HEARTBEAT_PERIOD_S, self._publish_heartbeat)

        # --- Joint-state drain ---
        self._joint_state_timer = self.create_timer(
            JOINT_STATE_DRAIN_PERIOD_S, self._drain_joint_state_changes)

        # --- Executor ---
        # Spin the node in a background thread to handle callbacks automatically.
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(f'LucyROSInterface started with Client ID: {CLIENT_ID}')
        # Register as connected. Control is opt-in (the user takes it from the TUI).
        self._publish_heartbeat()

    # --- Public Methods: Callbacks ---

    def register_on_control_change(self, callback):
        """
        Register a function to be called when the active controller changes.

        The callback will receive one argument:
        - active_client_id (str): The ID of the client now in control, or an
          empty string if control has been released.
        """
        self._on_control_change_callbacks.add(callback)

    def register_on_client_count_change(self, callback):
        """
        Register a function to be called when the client count changes.

        The callback will receive one argument:
        - count (int): The new number of connected clients.
        """
        self._on_client_count_change_callbacks.add(callback)

    def register_on_joint_state_change(self, callback):
        """
        Register a function to be called when measured joint positions change.

        The callback receives no arguments; call get_joint_positions_rad() to
        read the latest state. Fired only when at least one joint moved beyond
        JOINT_STATE_EPSILON_RAD, so an idle robot doesn't spam callbacks.
        """
        self._on_joint_state_change_callbacks.add(callback)

    # --- Public Methods: State & Commands ---

    def get_joint_positions_rad(self) -> dict:
        """Returns a snapshot of the latest measured joint positions (radians),
        keyed by joint name."""
        with self._joint_state_lock:
            return dict(self._joint_positions_rad)

    def setup_sensor_subscriptions(self, sensors: dict):
        """
        Creates one subscription per unique sensor topic found in `sensors`
        (the dict returned by tui_node.parse_sensors_yaml), fanning out to every
        sensor on that board by its position in the board's sensor list (which
        is already sorted by virtual_pin).

        Call once at startup after fetching and parsing the hardware config.
        Safe to call again: topics already subscribed to are left in place.
        """
        if self._is_shutting_down: return
        for group in sensors.values():
            topic = group.get('topic')
            if not topic: continue
            topic = topic if topic.startswith('/') else f'/{topic}'

            sensor_list = group.get('sensors', [])
            self._sensor_sources_by_topic.setdefault(topic, [])
            for array_index, sensor in enumerate(sensor_list):
                self._sensor_sources_by_topic[topic].append((sensor['name'], array_index))

            if topic not in self._sensor_subscriptions:
                self._sensor_subscriptions[topic] = self.create_subscription(
                    Float32Array, topic, self._make_sensor_callback(topic), qos_profile=VOLATILE_QOS)
                self.get_logger().info(f'Subscribed to pressure-sensor topic: {topic}')

    def _make_sensor_callback(self, topic: str):
        """Returns a closure bound to `topic` so one generic callback can serve
        every dynamically-created subscription while still knowing its own topic."""
        def _callback(msg: Float32Array):
            for sensor_id, array_index in self._sensor_sources_by_topic.get(topic, []):
                if 0 <= array_index < len(msg.data):
                    self._sensor_values[sensor_id] = float(msg.data[array_index])
        return _callback

    def get_sensor_values(self) -> dict:
        """Returns a snapshot of the latest pressure-sensor values, keyed by sensor id."""
        return dict(self._sensor_values)

    @property
    def client_id(self) -> str:
        """Returns the unique ID for this client instance."""
        return CLIENT_ID

    def has_control(self) -> bool:
        """Returns True if this client currently has control, False otherwise."""
        with self._control_lock:
            return self._has_control

    @property
    def client_count(self) -> int:
        """Returns the most recently known number of connected clients."""
        return self._client_count

    @property
    def active_controller(self) -> str:
        """Returns the id of the client currently in control, or '' if none."""
        return self._active_controller_id

    def take_control(self):
        """Requests control of the robot from the client registry."""
        if self._is_shutting_down: return
        self.get_logger().info('Requesting control...')
        self._send_control_request(acquire=True)

    def release_control(self):
        """Releases control of the robot via the client registry."""
        if self._is_shutting_down: return
        self.get_logger().info('Releasing control...')
        self._send_control_request(acquire=False)

    def _send_control_request(self, acquire: bool):
        # The resulting controller comes back on ACTIVE_CLIENT_TOPIC.
        try:
            if not self._control_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"'{CONTROL_SERVICE}' service not available.")
                return
            req = ClientControl.Request(client_id=CLIENT_ID, acquire=acquire)
            self._control_client.call_async(req)
        except Exception as e:
            self.get_logger().error(f'Control request failed: {e}')

    def _publish_heartbeat(self):
        if self._is_shutting_down: return
        try:
            self.heartbeat_publisher.publish(String(data=CLIENT_ID))
        except Exception:
            pass

    def publish_joint_trajectory(self, controller_topic: str, joint_names: list[str], positions_rad: list[float]):
        """
        Publishes a JointTrajectory message to a specified controller.

        Args:
            controller_topic: The full topic name for the joint trajectory
                              controller (e.g., '/left_arm_controller/joint_trajectory').
            joint_names: A list of joint names in the correct order for the controller.
            positions_rad: A list of target joint positions in radians, matching
                           the order of joint_names.
        """
        if self._is_shutting_down: return
        if controller_topic not in self._joint_publishers:
            self.get_logger().warn(f'Creating new publisher for an unknown topic: {controller_topic}')
            self._joint_publishers[controller_topic] = self.create_publisher(
                JointTrajectory, controller_topic, qos_profile=VOLATILE_QOS)

        publisher = self._joint_publishers[controller_topic]
        msg = JointTrajectory()
        # msg.header.stamp = 0  # Empty because get_clock() gives wall time.
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions_rad
        point.time_from_start = Duration(nanoseconds=200000000).to_msg()  # 0.2s
        msg.points.append(point)
        try:
            publisher.publish(msg)
            self.get_logger().info(f'Published JointTrajectory to {controller_topic}')
        except Exception:
             pass

    # --- Public Methods: Initial State Fetching ---

    def get_hardware_config_yaml(self) -> str | None:
        """
        Synchronously calls the /config/get service to fetch the active config.

        This is a blocking call and should be used during application startup.

        Returns:
            The active hardware configuration as a YAML string, or None if
            the service call fails or times out.
        """
        try:
            if not self._get_config_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("'/config/get' service not available.")
                return None
            req = GetConfig.Request(config_name='')
            future = self._get_config_client.call_async(req)
            
            # Create a temporary node solely to wait for THIS specific future.
            # This avoids interfering with the main executor.
            temp_node = Node('temp_service_caller')
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=5.0)
            temp_node.destroy_node()

            response = future.result()
            if not response or not response.success:
                self.get_logger().error(f"Failed to get active config: {response.message if response else 'timeout'}")
                return None
            self.get_logger().info('Successfully fetched active hardware configuration.')
            return response.config_yaml
        except Exception as e:
            self.get_logger().error(f'Exception in get_hardware_config_yaml: {e}')
            return None

    def get_initial_client_count(self) -> int | None:
        """
        Synchronously calls the registry's get-client-count service.

        This is a blocking call and should be used during application startup.

        Returns:
            The current client count as an integer, or None on failure.
        """
        try:
            if not self._get_count_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f"'{GET_CLIENT_COUNT_SERVICE}' service not available.")
                return None
            req = GetInt.Request()
            future = self._get_count_client.call_async(req)
            
            # Create a temporary node solely to wait for THIS specific future.
            # This avoids interfering with the main executor.
            temp_node = Node('temp_service_caller_2')
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=3.0)
            temp_node.destroy_node()

            response = future.result()
            if response:
                self.get_logger().info(f'Fetched initial client count: {response.value}')
                # Manually update internal state and call callbacks
                self._client_count = response.value
                for cb in self._on_client_count_change_callbacks:
                    cb(self._client_count)
                return response.value
            return None
        except Exception as e:
            self.get_logger().error(f'Exception in get_initial_client_count: {e}')
            return None

    # --- Internal ROS Callbacks ---

    def _control_callback(self, msg: String):
        """Handles incoming messages on the control topic."""
        with self._control_lock:
            new_controller_id = msg.data
            
            # Only react if the controller has actually changed
            if self._active_controller_id == new_controller_id:
                return

            self._active_controller_id = new_controller_id
            
            # We only lose control if someone else took it.
            # We do NOT lose control if the message is from ourselves,
            # nor do we gain control if the message is empty (released).
            if new_controller_id == CLIENT_ID:
                self._has_control = True
                self.get_logger().info('This client successfully took control.')
            elif new_controller_id == '':
                self._has_control = False
                self.get_logger().info('Control was released by previous client.')
            else:
                self._has_control = False
                self.get_logger().info(f"Control taken by another client: '{new_controller_id}'")
            
            # Notify all registered listeners of the change
            for cb in self._on_control_change_callbacks:
                cb(new_controller_id)

    def _joint_state_callback(self, msg: JointState):
        """Stores the latest measured joint positions at the full ROS rate."""
        with self._joint_state_lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions_rad[name] = position

    def _drain_joint_state_changes(self):
        """Notifies listeners at most once per drain, and only when a joint has
        moved past the threshold since the last drain. Comparing against the
        last-drained baseline (not the previous sample) lets slow motion
        accumulate past it; an idle robot stays below and produces no redraws.
        """
        if self._is_shutting_down:
            return
        changed = False
        with self._joint_state_lock:
            for name, position in self._joint_positions_rad.items():
                baseline = self._notify_baseline_rad.get(name)
                if baseline is None or abs(baseline - position) > JOINT_STATE_EPSILON_RAD:
                    changed = True
                    break
            if changed:
                self._notify_baseline_rad.update(self._joint_positions_rad)
        if changed:
            for cb in self._on_joint_state_change_callbacks:
                cb()

    def _client_count_callback(self, msg: Int32):
        """Handles incoming messages on the client count topic."""
        with self._control_lock:
            new_count = msg.data
            if self._client_count != new_count:
                self._client_count = new_count
                self.get_logger().info(f'Client count updated: {new_count}')
                # Notify all registered listeners
                for cb in self._on_client_count_change_callbacks:
                    cb(new_count)

    # --- Lifecycle ---

    def shutdown(self):
        """Gracefully shuts down the node and its background thread."""
        if self._is_shutting_down: return

        self.get_logger().info('Shutting down LucyROSInterface.')
        # Release before flipping the guard, else release_control() short-circuits.
        self.release_control()
        self._is_shutting_down = True
        self._heartbeat_timer.cancel()
        self._joint_state_timer.cancel()
        time.sleep(0.1)  # let the release request go out
        self._executor.shutdown()
        self.destroy_node()
