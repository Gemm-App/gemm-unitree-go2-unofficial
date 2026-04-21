"""Run Gemm's reusable adapter-contract test suite against UnitreeAdapter."""

from __future__ import annotations

import pytest
from gemm.testing import AdapterContractTests
from unitree_webrtc_connect import RTC_TOPIC

from gemm_unitree_go2_unofficial import UnitreeAdapter

from .conftest import (
    LIDAR_PAYLOAD,
    LOW_STATE_PAYLOAD,
    ROBOTODOM_PAYLOAD,
    SPORT_STATE_PAYLOAD,
    FakeConnection,
)


class TestUnitreeAdapterContract(AdapterContractTests):
    """All 11 contract tests must pass against the Go2 adapter."""

    @pytest.fixture
    def adapter(self, fake_conn: FakeConnection) -> UnitreeAdapter:
        adapter = UnitreeAdapter(name="go2-contract", ip="192.168.12.1")

        # Populate sensor caches after connect so get_sensor doesn't raise
        # SensorError from "no data yet" during contract probing.
        original_connect = adapter.connect

        async def connect_and_prime() -> None:
            await original_connect()
            fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
            fake_conn.fire(RTC_TOPIC["LF_SPORT_MOD_STATE"], SPORT_STATE_PAYLOAD)
            fake_conn.fire(RTC_TOPIC["ROBOTODOM"], ROBOTODOM_PAYLOAD)
            fake_conn.fire(RTC_TOPIC["ULIDAR_ARRAY"], LIDAR_PAYLOAD)

        adapter.connect = connect_and_prime  # type: ignore[method-assign]
        return adapter
