"""Gemm contract tests for UnitreeAdapter (runs without real hardware).

Inherits the 8 shared contract tests from gemm.testing.AdapterContractTests.
The adapter fixture is provided by conftest.py and uses a mocked WebRTC
connection, so no Unitree robot is required.
"""

from gemm.testing import AdapterContractTests


class TestUnitreeAdapterContract(AdapterContractTests):
    """All 8 inherited tests run against the mocked adapter."""

    # `adapter` fixture comes from conftest.py
