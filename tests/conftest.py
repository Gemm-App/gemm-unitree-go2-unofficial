"""Shared test fixtures for gemm-unitree-go2-unofficial."""

from __future__ import annotations

import asyncio
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from gemm_unitree_go2_unofficial import UnitreeAdapter


def _make_mock_connection(
    *,
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    battery_pct: float = 80.0,
) -> MagicMock:
    """Build a fake UnitreeWebRTCConnection that stores callbacks and fires them."""
    conn = MagicMock()
    conn.connect = AsyncMock()

    callbacks: dict[str, Any] = {}

    def subscribe(topic: str, cb: Any) -> None:
        callbacks[topic] = cb

    def publish_request_new(topic: str, data: Any) -> asyncio.Future[None]:
        # If it's a Move command, update the stored pose slightly so
        # the controller can observe "movement" in tests
        fut: asyncio.Future[None] = asyncio.get_event_loop().create_future()
        fut.set_result(None)
        return fut

    conn.datachannel.pub_sub.subscribe.side_effect = subscribe
    conn.datachannel.pub_sub.publish_request_new.side_effect = publish_request_new
    conn._callbacks = callbacks
    conn._pose = list(pose)
    conn._battery_pct = battery_pct
    return conn


@pytest.fixture
def mock_conn() -> MagicMock:
    return _make_mock_connection()


@pytest.fixture
def adapter(mock_conn: MagicMock, monkeypatch: pytest.MonkeyPatch) -> UnitreeAdapter:
    """UnitreeAdapter wired to a fake WebRTC connection — no real robot needed."""
    import gemm_unitree_go2_unofficial.adapter as adapter_mod

    # Patch the constructor so it returns our mock
    monkeypatch.setattr(
        adapter_mod,
        "UnitreeWebRTCConnection",
        lambda *args, **kwargs: mock_conn,
    )

    a = UnitreeAdapter(name="test-go2", ip="192.168.12.1")

    # After connect(), fire fake ROBOTODOM and LOW_STATE messages
    # so the adapter has a valid initial state
    original_connect = a.connect

    async def patched_connect() -> None:
        await original_connect()
        cb = mock_conn._callbacks
        if "rt/utlidar/robot_pose" in cb:
            cb["rt/utlidar/robot_pose"](
                {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "imu_state": {"rpy": [0.0, 0.0, 0.0]},
                }
            )
        if "rt/lowstate" in cb:
            cb["rt/lowstate"]({"battery_state": {"percent": 80.0}})

    a.connect = patched_connect  # type: ignore[method-assign]
    return a
