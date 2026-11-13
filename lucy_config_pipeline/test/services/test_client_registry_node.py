import time

import pytest
import rclpy
from src.services.client_registry_node import ClientRegistryNode
from src.services.client_registry_node import NO_SERVO_POWER
from src.services.client_registry_node import SOCKET_DROP_GRACE_S
from std_msgs.msg import Int32
from std_msgs.msg import String


class TestClientRegistryNode:
    def test_heartbeat_registers_and_counts(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data="cli_1"))
        node._on_heartbeat(String(data="cp_2"))
        node._on_heartbeat(String(data="cli_1"))  # duplicate refresh, not a new client

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 2

    def test_empty_heartbeat_ignored(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data=""))

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 0

    def test_take_then_preempt_then_release_control(self, node):
        from lucy_msgs.srv import ClientControl

        def control(client_id, acquire):
            return node._on_control(
                ClientControl.Request(client_id=client_id, acquire=acquire),
                ClientControl.Response(),
            )

        res = control("cli_1", True)
        assert res.success is True
        assert res.active_client == "cli_1"

        # Control is preemptive: the latest acquirer wins.
        assert control("cp_2", True).active_client == "cp_2"

        # A non-owner releasing is a no-op.
        assert control("cli_1", False).active_client == "cp_2"

        # The owner releasing frees control.
        assert control("cp_2", False).active_client == ""

    def test_empty_client_id_rejected(self, node):
        from lucy_msgs.srv import ClientControl

        res = node._on_control(
            ClientControl.Request(client_id="", acquire=True), ClientControl.Response()
        )
        assert res.success is False
        assert res.active_client == ""

    def test_control_request_also_registers_client(self, node):
        from lucy_msgs.srv import ClientControl
        from lucy_msgs.srv import GetInt

        node._on_control(
            ClientControl.Request(client_id="cli_1", acquire=True),
            ClientControl.Response(),
        )

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 1

    def test_goodbye_drops_the_client_and_its_control(self, node):
        from lucy_msgs.srv import ClientControl
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data="cli_1"))
        node._on_control(
            ClientControl.Request(client_id="cp_2", acquire=True),
            ClientControl.Response(),
        )
        node._on_goodbye(String(data="cp_2"))

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 1
        assert node._active_client == ""

    def test_goodbye_from_a_non_controller_keeps_control(self, node):
        from lucy_msgs.srv import ClientControl

        node._on_heartbeat(String(data="cp_2"))
        node._on_control(
            ClientControl.Request(client_id="cli_1", acquire=True),
            ClientControl.Response(),
        )
        node._on_goodbye(String(data="cp_2"))
        node._on_goodbye(String(data="unknown"))

        assert node._active_client == "cli_1"

    def test_tick_prunes_stale_clients(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data="cli_1"))
        # Backdate the heartbeat well past the TTL.
        node._last_seen["cli_1"] = time.monotonic() - 999
        node._on_tick()

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 0

    def test_tick_releases_control_when_controller_expires(self, node):
        from lucy_msgs.srv import ClientControl

        node._on_control(
            ClientControl.Request(client_id="cli_1", acquire=True),
            ClientControl.Response(),
        )
        assert node._active_client == "cli_1"

        node._last_seen["cli_1"] = time.monotonic() - 999
        node._on_tick()

        assert node._active_client == ""

    def _spin_past_grace(self, node, beating):
        deadline = time.monotonic() + SOCKET_DROP_GRACE_S + 1.0
        while time.monotonic() < deadline:
            for client_id in beating:
                node._on_heartbeat(String(data=client_id))
            rclpy.spin_once(node, timeout_sec=0.1)

    def test_socket_drop_removes_only_the_client_that_stopped_beating(self, node):
        node._on_heartbeat(String(data="cp_closed"))
        node._on_heartbeat(String(data="cp_open"))
        node._on_bridge_client_count(Int32(data=2))
        node._on_bridge_client_count(Int32(data=1))
        self._spin_past_grace(node, ["cp_open"])
        assert "cp_closed" not in node._last_seen
        assert "cp_open" in node._last_seen

    def test_first_bridge_count_is_not_a_drop(self, node):
        node._on_heartbeat(String(data="cp_quiet"))
        node._on_bridge_client_count(Int32(data=0))
        self._spin_past_grace(node, [])
        assert "cp_quiet" in node._last_seen


def _supply(node, *volts):
    from control_msgs.msg import DynamicJointState
    from control_msgs.msg import InterfaceValue

    node._on_dynamic_joint_states(
        DynamicJointState(
            joint_names=[f"j{i}" for i in range(len(volts))],
            interface_values=[
                InterfaceValue(interface_names=["position", "voltage"], values=[0.0, v])
                for v in volts
            ],
        )
    )


def _control(node, client_id, acquire):
    from lucy_msgs.srv import ClientControl

    return node._on_control(
        ClientControl.Request(client_id=client_id, acquire=acquire),
        ClientControl.Response(),
    )


MOTORS_ON_J0_J1 = (
    "actuators:\n"
    "  - {id: a, urdf_joint: j0, motor: sts3215/12v_345}\n"
    "  - {id: b, urdf_joint: j1, motor: sts3215/12v_345}\n"
    "  - {id: c, urdf_joint: j2}\n"
)


@pytest.fixture
def powered_node(rclpy_init_shutdown):
    n = ClientRegistryNode(read_active_config=lambda: MOTORS_ON_J0_J1)
    yield n
    n.destroy_node()


class TestServoPower:
    def test_power_loss_releases_control(self, powered_node):
        node = powered_node
        _supply(node, 12.2, 12.2)
        assert _control(node, "cp_1", True).active_client == "cp_1"

        _supply(node, 12.2, 0.0)

        assert node._active_client == ""

    def test_acquire_refused_until_power_returns(self, powered_node):
        node = powered_node
        _supply(node, 0.0, 0.0)
        res = _control(node, "cp_1", True)
        assert res.success is False
        assert res.message == NO_SERVO_POWER
        assert res.active_client == ""

        _supply(node, 12.1, 12.2)

        assert _control(node, "cp_1", True).active_client == "cp_1"

    def test_threshold_comes_from_each_joints_motor(self, powered_node):
        node = powered_node
        _supply(node, 6.1, 6.0, 0.0)

        assert _control(node, "cp_1", True).active_client == "cp_1"

        _supply(node, 5.9, 12.0, 12.0)

        assert node._active_client == ""

    def test_no_motor_specs_means_no_power_check(self, node):
        _supply(node, 0.0, 0.0)

        assert _control(node, "cp_1", True).active_client == "cp_1"

    def test_unreadable_config_turns_the_power_check_off(self, rclpy_init_shutdown):
        def missing():
            raise FileNotFoundError("active.yaml")

        node = ClientRegistryNode(read_active_config=missing)
        try:
            _supply(node, 0.0, 0.0)
            assert _control(node, "cp_1", True).active_client == "cp_1"
        finally:
            node.destroy_node()

    def test_states_without_voltage_leave_control_alone(self, powered_node):
        node = powered_node
        from control_msgs.msg import DynamicJointState
        from control_msgs.msg import InterfaceValue

        _control(node, "cp_1", True)
        node._on_dynamic_joint_states(
            DynamicJointState(
                joint_names=["j0"],
                interface_values=[
                    InterfaceValue(interface_names=["position"], values=[0.0])
                ],
            )
        )

        assert node._active_client == "cp_1"
