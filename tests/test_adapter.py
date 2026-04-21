"""Adapter-level behaviour tests: construction, state, lifecycle, entry point."""

from __future__ import annotations

from importlib.metadata import entry_points

import pytest
from gemm.errors import AdapterConnectionError
from gemm.types import RobotState
from unitree_webrtc_connect import RTC_TOPIC, WebRTCConnectionMethod

from gemm_unitree_go2_unofficial import UnitreeAdapter

from .conftest import LOW_STATE_PAYLOAD, ROBOTODOM_PAYLOAD, FakeConnection


def test_requires_ip_or_serial() -> None:
    with pytest.raises(ValueError, match="ip"):
        UnitreeAdapter(name="x")


def test_accepts_serial_only() -> None:
    adapter = UnitreeAdapter(
        name="go2",
        serial_number="B42D12345",
        connection_method=WebRTCConnectionMethod.Remote,
    )
    assert adapter.name == "go2"


def test_name_attribute_is_set() -> None:
    adapter = UnitreeAdapter(name="my-robot", ip="1.2.3.4")
    assert adapter.name == "my-robot"


@pytest.mark.asyncio
async def test_get_state_before_connect_raises(adapter: UnitreeAdapter) -> None:
    with pytest.raises(AdapterConnectionError):
        await adapter.get_state()


@pytest.mark.asyncio
async def test_get_state_returns_robot_state(
    connected_adapter: UnitreeAdapter,
) -> None:
    state = await connected_adapter.get_state()
    assert isinstance(state, RobotState)
    assert 0.0 <= state.battery <= 1.0
    # Pose pulled from ROBOTODOM_PAYLOAD
    assert state.pose.x == pytest.approx(1.5)
    assert state.pose.y == pytest.approx(-0.5)


@pytest.mark.asyncio
async def test_connect_subscribes_to_core_topics(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        subs = fake_conn.datachannel.pub_sub.subscriptions
        for topic_key in ("LOW_STATE", "LF_SPORT_MOD_STATE", "ROBOTODOM", "ULIDAR_ARRAY"):
            assert RTC_TOPIC[topic_key] in subs, topic_key
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_lidar_false_skips_lidar_subscription(
    fake_conn: FakeConnection,
) -> None:
    adapter = UnitreeAdapter(name="no-lidar", ip="1.2.3.4", lidar=False)
    await adapter.connect()
    try:
        subs = fake_conn.datachannel.pub_sub.subscriptions
        assert RTC_TOPIC["ULIDAR_ARRAY"] not in subs
        assert not fake_conn.datachannel.traffic_saving_disabled
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_lidar_true_switches_decoder_and_turns_on_traffic(
    fake_conn: FakeConnection,
) -> None:
    adapter = UnitreeAdapter(name="lidar", ip="1.2.3.4", lidar=True)
    await adapter.connect()
    try:
        assert fake_conn.datachannel.decoder_name == "native"
        assert fake_conn.datachannel.traffic_saving_disabled is True
        assert (RTC_TOPIC["ULIDAR_SWITCH"], "ON") in fake_conn.datachannel.pub_sub.sent_raw
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_disconnect_calls_sdk_disconnect(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    await adapter.disconnect()
    assert fake_conn.disconnected


@pytest.mark.asyncio
async def test_disconnect_without_connect_is_noop(adapter: UnitreeAdapter) -> None:
    # Must not raise
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_callbacks_update_pose_and_battery(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        state = await adapter.get_state()
        assert state.pose.x == 0.0  # no data yet

        fake_conn.fire(RTC_TOPIC["ROBOTODOM"], ROBOTODOM_PAYLOAD)
        fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)

        state = await adapter.get_state()
        assert state.pose.x == pytest.approx(1.5)
        assert state.battery == pytest.approx(0.8)
    finally:
        await adapter.disconnect()


def test_entry_point_discovers_adapter() -> None:
    eps = list(entry_points(group="gemm.adapters"))
    names = {ep.name: ep.value for ep in eps}
    assert names["unitree-go2"] == "gemm_unitree_go2_unofficial:UnitreeAdapter"
    loaded = next(ep for ep in eps if ep.name == "unitree-go2").load()
    assert loaded is UnitreeAdapter
