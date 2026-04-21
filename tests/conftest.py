"""Shared test fixtures.

Builds a :class:`FakeConnection` that stands in for
:class:`unitree_webrtc_connect.UnitreeWebRTCConnection` so the whole adapter
can be exercised offline. The fake preserves the two things the adapter
actually uses:

* ``conn.datachannel.pub_sub.subscribe(topic, callback)``
* ``conn.datachannel.pub_sub.publish_request_new(topic, payload)``
* ``conn.datachannel.pub_sub.publish_without_callback(topic, payload)``
* ``conn.datachannel.disableTrafficSaving(bool)``
* ``conn.datachannel.set_decoder(str)``

Tests drive the adapter by popping values out of ``fake.sent_requests`` and
pushing fake messages back in via ``fake.fire(topic, payload)``.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field
from typing import Any

import pytest
import pytest_asyncio
from unitree_webrtc_connect import RTC_TOPIC

import gemm_unitree_go2_unofficial.adapter as adapter_mod
from gemm_unitree_go2_unofficial import UnitreeAdapter


@dataclass
class FakePubSub:
    subscriptions: dict[str, Callable[[Any], None]] = field(default_factory=dict)
    sent_requests: list[tuple[str, dict[str, Any]]] = field(default_factory=list)
    sent_raw: list[tuple[str, Any]] = field(default_factory=list)
    next_response: Any = None

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.subscriptions[topic] = callback

    def unsubscribe(self, topic: str) -> None:
        self.subscriptions.pop(topic, None)

    async def publish_request_new(self, topic: str, options: dict[str, Any]) -> Any:
        self.sent_requests.append((topic, options))
        return self.next_response

    def publish_without_callback(
        self, topic: str, data: Any = None, msg_type: Any = None
    ) -> None:
        self.sent_raw.append((topic, data))

    async def publish(self, topic: str, data: Any = None, msg_type: Any = None) -> Any:
        return self.next_response


@dataclass
class FakeDataChannel:
    pub_sub: FakePubSub = field(default_factory=FakePubSub)
    decoder_name: str = "libvoxel"
    traffic_saving_disabled: bool = False

    def set_decoder(self, name: str) -> None:
        if name not in {"libvoxel", "native"}:
            raise ValueError(f"bad decoder: {name}")
        self.decoder_name = name

    async def disableTrafficSaving(self, switch: bool) -> bool:
        self.traffic_saving_disabled = bool(switch)
        return True


class FakeConnection:
    def __init__(self, **_: Any) -> None:
        self.datachannel = FakeDataChannel()
        self.connected = False
        self.disconnected = False

    async def connect(self) -> None:
        self.connected = True

    async def disconnect(self) -> None:
        self.disconnected = True

    def fire(self, topic: str, payload: Any) -> None:
        cb = self.datachannel.pub_sub.subscriptions.get(topic)
        if cb is None:
            raise AssertionError(f"no subscriber registered for {topic!r}")
        cb(payload)


# ---- fake payloads --------------------------------------------------------- #


LOW_STATE_PAYLOAD = {
    "data": {
        "bms_state": {
            "soc": 80.0,
            "power_v": 25.2,
            "power_a": -2.0,
            "temperature": 28.0,
            "cycle_count": 3,
        },
        "motor_state": [
            {"q": 0.1 * i, "dq": 0.0, "tau_est": 0.5, "temperature": 25.0 + i}
            for i in range(12)
        ],
    }
}

SPORT_STATE_PAYLOAD = {
    "data": {
        "imu_state": {
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "gyroscope": [0.01, 0.02, 0.03],
            "accelerometer": [0.0, 0.0, 9.81],
            "rpy": [0.0, 0.0, 0.25],
            "temperature": 35.5,
        }
    }
}

ROBOTODOM_PAYLOAD = {
    "data": {
        "pose": {
            "position": {"x": 1.5, "y": -0.5, "z": 0.0},
            "orientation": {"w": 0.9689, "x": 0.0, "y": 0.0, "z": 0.2474},
        },
        "linear_velocity": [0.1, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.05],
    }
}

# Decoded LiDAR payload mirroring what UnifiedLidarDecoder(native) emits:
# ``data["data"]`` is a dict with a "points" (N, 3) numpy-like array.
LIDAR_PAYLOAD = {
    "data": {
        "origin": [0.0, 0.0, 0.0],
        "resolution": 0.05,
        "data": {
            "points": [
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [-1.0, 0.0, 0.5],
            ]
        },
    }
}


# ---- fixtures -------------------------------------------------------------- #


@pytest.fixture
def fake_conn(monkeypatch: pytest.MonkeyPatch) -> FakeConnection:
    conn = FakeConnection()
    monkeypatch.setattr(
        adapter_mod, "UnitreeWebRTCConnection", lambda **kwargs: conn
    )
    return conn


@pytest.fixture
def adapter(fake_conn: FakeConnection) -> UnitreeAdapter:
    return UnitreeAdapter(name="go2-test", ip="192.168.12.1")


@pytest_asyncio.fixture
async def connected_adapter(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> AsyncIterator[UnitreeAdapter]:
    """Connected adapter with every sensor pre-populated."""
    await adapter.connect()
    fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
    fake_conn.fire(RTC_TOPIC["LF_SPORT_MOD_STATE"], SPORT_STATE_PAYLOAD)
    fake_conn.fire(RTC_TOPIC["ROBOTODOM"], ROBOTODOM_PAYLOAD)
    fake_conn.fire(RTC_TOPIC["ULIDAR_ARRAY"], LIDAR_PAYLOAD)
    yield adapter
    await adapter.disconnect()


# Allow test modules to silence unused-import warnings on the shared topics
__all__ = [
    "LIDAR_PAYLOAD",
    "LOW_STATE_PAYLOAD",
    "ROBOTODOM_PAYLOAD",
    "SPORT_STATE_PAYLOAD",
    "FakeConnection",
    "FakeDataChannel",
    "FakePubSub",
]


# pytest-asyncio's strict mode needs an explicit event_loop_policy for async
# fixtures; pytest-asyncio >= 0.23 auto-uses the default so nothing extra here.
_ = asyncio  # re-export reference for optional downstream use
