import time

import pytest
import rclpy
from src.services.client_registry_node import ClientRegistryNode
from std_msgs.msg import String


@pytest.fixture
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init_shutdown):
    n = ClientRegistryNode()
    yield n
    n.destroy_node()


class TestClientRegistryNode:
    def test_heartbeat_registers_and_counts(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data='cli_1'))
        node._on_heartbeat(String(data='cp_2'))
        node._on_heartbeat(String(data='cli_1'))  # duplicate refresh, not a new client

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 2

    def test_empty_heartbeat_ignored(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data=''))

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 0

    def test_take_then_preempt_then_release_control(self, node):
        from lucy_msgs.srv import ClientControl

        def control(client_id, acquire):
            return node._on_control(
                ClientControl.Request(client_id=client_id, acquire=acquire),
                ClientControl.Response(),
            )

        res = control('cli_1', True)
        assert res.success is True
        assert res.active_client == 'cli_1'

        # Control is preemptive: the latest acquirer wins.
        assert control('cp_2', True).active_client == 'cp_2'

        # A non-owner releasing is a no-op.
        assert control('cli_1', False).active_client == 'cp_2'

        # The owner releasing frees control.
        assert control('cp_2', False).active_client == ''

    def test_empty_client_id_rejected(self, node):
        from lucy_msgs.srv import ClientControl

        res = node._on_control(
            ClientControl.Request(client_id='', acquire=True), ClientControl.Response()
        )
        assert res.success is False
        assert res.active_client == ''

    def test_control_request_also_registers_client(self, node):
        from lucy_msgs.srv import ClientControl
        from lucy_msgs.srv import GetInt

        node._on_control(
            ClientControl.Request(client_id='cli_1', acquire=True), ClientControl.Response()
        )

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 1

    def test_goodbye_drops_the_client_and_its_control(self, node):
        from lucy_msgs.srv import ClientControl
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data='cli_1'))
        node._on_control(
            ClientControl.Request(client_id='cp_2', acquire=True), ClientControl.Response()
        )
        node._on_goodbye(String(data='cp_2'))

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 1
        assert node._active_client == ''

    def test_goodbye_from_a_non_controller_keeps_control(self, node):
        from lucy_msgs.srv import ClientControl

        node._on_heartbeat(String(data='cp_2'))
        node._on_control(
            ClientControl.Request(client_id='cli_1', acquire=True), ClientControl.Response()
        )
        node._on_goodbye(String(data='cp_2'))
        node._on_goodbye(String(data='unknown'))

        assert node._active_client == 'cli_1'

    def test_tick_prunes_stale_clients(self, node):
        from lucy_msgs.srv import GetInt

        node._on_heartbeat(String(data='cli_1'))
        # Backdate the heartbeat well past the TTL.
        node._last_seen['cli_1'] = time.monotonic() - 999
        node._on_tick()

        res = node._on_get_client_count(GetInt.Request(), GetInt.Response())
        assert res.value == 0

    def test_tick_releases_control_when_controller_expires(self, node):
        from lucy_msgs.srv import ClientControl

        node._on_control(
            ClientControl.Request(client_id='cli_1', acquire=True), ClientControl.Response()
        )
        assert node._active_client == 'cli_1'

        node._last_seen['cli_1'] = time.monotonic() - 999
        node._on_tick()

        assert node._active_client == ''
