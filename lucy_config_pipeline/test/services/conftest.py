"""Fixtures shared by the client registry and control lock test suites."""

from __future__ import annotations

import pytest
import rclpy
from src.services.client_registry_node import ClientRegistryNode


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
