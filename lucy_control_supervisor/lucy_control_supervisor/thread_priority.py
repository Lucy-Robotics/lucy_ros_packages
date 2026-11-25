from __future__ import annotations

import os
import sys
from typing import List, Mapping

WINDOWS_TIME_CRITICAL_PRIORITY = 15


def controller_manager_priority_args(
    platform: str = sys.platform, environ: Mapping[str, str] = os.environ
) -> List[str]:
    if platform != "win32" or environ.get("LUCY_REALTIME") == "0":
        return []
    return ["-p", f"thread_priority:={WINDOWS_TIME_CRITICAL_PRIORITY}"]
