"""Pure decision logic for the client registry's single-writer control arbitration."""

from __future__ import annotations


def decide_control(
    *,
    client_id: str,
    acquire: bool,
    active_client: str,
    lock_owner: str,
    servo_powered: bool,
) -> tuple[str, bool, str]:
    """
    Decide the next active client for `_on_control`.

    Returns (new_active_client, refused, locked_by).
    """
    refused = acquire and not servo_powered
    locked_by = lock_owner if acquire else ""
    if acquire and not refused and not locked_by:
        active_client = client_id
    elif not acquire and active_client == client_id:
        active_client = ""
    return active_client, refused, locked_by


def decide_control_lock(
    *, owner: str, acquire: bool, lock_owner: str, active_client: str
) -> tuple[str, bool, str]:
    """
    Decide the next lock owner for `_on_control_lock`.

    Returns (new_lock_owner, success, message).
    """
    owns_or_free = lock_owner in ("", owner)
    if acquire:
        if not owns_or_free:
            return lock_owner, False, f"already locked by {lock_owner}"
        if active_client:
            return lock_owner, False, f"{active_client} has control"
        return owner, True, "locked"
    if owns_or_free:
        return "", True, "released"
    return lock_owner, False, f"locked by {lock_owner}"
