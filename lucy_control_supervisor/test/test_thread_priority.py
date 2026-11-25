from lucy_control_supervisor.thread_priority import controller_manager_priority_args


def test_windows_asks_for_time_critical_priority():
    assert controller_manager_priority_args("win32", {}) == [
        "-p",
        "thread_priority:=15",
    ]


def test_linux_and_macos_keep_the_upstream_priority():
    assert controller_manager_priority_args("linux", {}) == []
    assert controller_manager_priority_args("darwin", {}) == []


def test_lucy_realtime_0_keeps_the_upstream_priority_on_windows():
    assert controller_manager_priority_args("win32", {"LUCY_REALTIME": "0"}) == []


def test_other_lucy_realtime_values_keep_time_critical_on_windows():
    assert controller_manager_priority_args("win32", {"LUCY_REALTIME": "1"}) == [
        "-p",
        "thread_priority:=15",
    ]
