"""Shared test fixtures for gemm-unitree-go2-unofficial."""

from __future__ import annotations

import asyncio
import struct
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest
from unitree_webrtc_connect import RTC_TOPIC

from gemm_unitree_go2_unofficial import UnitreeAdapter

# ---- Fake sensor payloads ------------------------------------------------- #

_FAKE_ROBOTODOM = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "linear_velocity": [0.0, 0.0, 0.0],
    "angular_velocity": [0.0, 0.0, 0.0],
}

_FAKE_SPORT_STATE = {
    "imu_state": {
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "gyroscope": [0.0, 0.0, 0.0],
        "accelerometer": [0.0, 0.0, 9.81],
        "rpy": [0.0, 0.0, 0.0],
        "temperature": 35.0,
    },
    "position": [0.0, 0.0, 0.0],
}

_FAKE_LOW_STATE = {
    "bms_state": {
        "soc": 80.0,
        "power_v": 25.2,
        "power_a": -2.0,
        "temperature": 28.0,
    },
    "motor_state": [{"q": 0.0, "dq": 0.0, "tau_est": 0.0, "temperature": 25.0} for _ in range(12)],
}

# Raw binary LiDAR: 2 points (x=1, y=2, z=3) and (x=4, y=5, z=6)
_FAKE_LIDAR_BYTES = struct.pack("<6f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


# ---- Mock connection factory ---------------------------------------------- #


def _make_mock_connection(
    *,
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    battery_pct: float = 80.0,
) -> MagicMock:
    """Build a fake UnitreeWebRTCConnection that stores callbacks and fires them."""
    conn = MagicMock()
    conn.connect = AsyncMock()
    conn.disableTrafficSaving = MagicMock()

    callbacks: dict[str, Any] = {}

    def subscribe(topic: str, cb: Any) -> None:
        callbacks[topic] = cb

    def publish_request_new(topic: str, data: Any) -> asyncio.Future[None]:
        fut: asyncio.Future[None] = asyncio.get_event_loop().create_future()
        fut.set_result(None)
        return fut

    def publish_without_callback(topic: str, data: Any) -> None:
        pass  # no-op in tests

    conn.datachannel.pub_sub.subscribe.side_effect = subscribe
    conn.datachannel.pub_sub.publish_request_new.side_effect = publish_request_new
    conn.datachannel.pub_sub.publish_without_callback.side_effect = publish_without_callback
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

    # After connect(), fire all fake sensor messages so the adapter has
    # a valid initial state for every sensor.
    original_connect = a.connect

    async def patched_connect() -> None:
        await original_connect()
        cb = mock_conn._callbacks

        # ROBOTODOM → odometry cache
        robodom_topic = RTC_TOPIC["ROBOTODOM"]
        if robodom_topic in cb:
            cb[robodom_topic](_FAKE_ROBOTODOM)

        # LF_SPORT_MOD_STATE → IMU cache
        sport_topic = RTC_TOPIC["LF_SPORT_MOD_STATE"]
        if sport_topic in cb:
            cb[sport_topic](_FAKE_SPORT_STATE)

        # LOW_STATE → battery + motor caches
        low_topic = RTC_TOPIC["LOW_STATE"]
        if low_topic in cb:
            cb[low_topic](_FAKE_LOW_STATE)

        # ULIDAR_ARRAY → lidar cache
        lidar_topic = RTC_TOPIC["ULIDAR_ARRAY"]
        if lidar_topic in cb:
            cb[lidar_topic](_FAKE_LIDAR_BYTES)

    a.connect = patched_connect  # type: ignore[method-assign]
    return a
