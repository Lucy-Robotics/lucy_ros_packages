"""Backward-compatible import wrapper."""

from .services.config_services_node import ConfigServicesNode
from .services.config_services_node import main

__all__ = ["ConfigServicesNode", "main"]
