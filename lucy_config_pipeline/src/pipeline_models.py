"""Backward-compatible import wrapper."""

from .pipeline.models import FirmwarePaths
from .pipeline.models import PipelinePaths

__all__ = ["PipelinePaths", "FirmwarePaths"]
