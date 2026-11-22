from __future__ import annotations

import threading

from rclpy.task import Future


def wait_for_future(future: Future, timeout_s: float) -> bool:
    """Block until `future` completes or `timeout_s` elapses, without re-entering the executor."""
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return done.wait(timeout_s) and future.done()
