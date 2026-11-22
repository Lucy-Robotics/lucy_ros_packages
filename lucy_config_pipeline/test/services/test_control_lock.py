from lucy_msgs.srv import ClientControl
from lucy_msgs.srv import ControlLock


class TestControlLock:
    def _lock(self, node, owner="calibration", acquire=True):
        return node._on_control_lock(
            ControlLock.Request(owner=owner, acquire=acquire), ControlLock.Response()
        )

    def _control(self, node, client="cp_1", acquire=True):
        return node._on_control(
            ClientControl.Request(client_id=client, acquire=acquire),
            ClientControl.Response(),
        )

    def test_locked_control_is_refused(self, node):
        assert self._lock(node).success
        res = self._control(node)
        assert not res.success
        assert res.active_client == ""
        assert "locked by calibration" in res.message

    def test_lock_fails_while_someone_has_control(self, node):
        assert self._control(node).success
        res = self._lock(node)
        assert not res.success
        assert res.active_client == "cp_1"

    def test_release_frees_control(self, node):
        self._lock(node)
        assert self._lock(node, acquire=False).success
        assert self._control(node).success

    def test_lock_is_idempotent_for_its_owner_only(self, node):
        assert self._lock(node).success
        assert self._lock(node).success
        assert not self._lock(node, owner="other").success
        assert not self._lock(node, owner="other", acquire=False).success
        assert not self._control(node).success

    def test_releasing_control_still_works_while_locked(self, node):
        self._control(node)
        node._lock_owner = "calibration"
        assert self._control(node, acquire=False).success

    def test_empty_owner_is_refused(self, node):
        assert not self._lock(node, owner="").success
