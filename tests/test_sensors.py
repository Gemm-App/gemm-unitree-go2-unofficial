"""Tests for ``get_sensor()`` and ``subscribe()``."""

from __future__ import annotations

import pytest
from gemm.errors import AdapterConnectionError, SensorError, SensorNotAvailable
from gemm.types import BatteryState, IMUData, LiDARScan, MotorState, RobotOdometry
from unitree_webrtc_connect import RTC_TOPIC

from gemm_unitree_go2_unofficial import UnitreeAdapter

from .conftest import (
    LIDAR_PAYLOAD,
    LOW_STATE_PAYLOAD,
    ROBOTODOM_PAYLOAD,
    SPORT_STATE_PAYLOAD,
    FakeConnection,
)

# --------------------------------------------------------------------------- #
# get_sensor                                                                  #
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
async def test_get_sensor_before_connect_raises(adapter: UnitreeAdapter) -> None:
    with pytest.raises(AdapterConnectionError):
        await adapter.get_sensor("battery")


@pytest.mark.asyncio
async def test_get_sensor_unknown_raises(connected_adapter: UnitreeAdapter) -> None:
    with pytest.raises(SensorNotAvailable):
        await connected_adapter.get_sensor("warp_drive")


@pytest.mark.asyncio
async def test_get_sensor_before_data_raises_sensor_error(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        with pytest.raises(SensorError):
            await adapter.get_sensor("battery")
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_battery_soc_normalised_to_0_1(
    connected_adapter: UnitreeAdapter,
) -> None:
    reading = await connected_adapter.get_sensor("battery")
    assert isinstance(reading, BatteryState)
    assert reading.soc == pytest.approx(0.80)
    assert reading.voltage == pytest.approx(25.2)
    assert reading.current == pytest.approx(-2.0)
    assert reading.cycle_count == 3


@pytest.mark.asyncio
async def test_imu_reading(connected_adapter: UnitreeAdapter) -> None:
    reading = await connected_adapter.get_sensor("imu")
    assert isinstance(reading, IMUData)
    assert reading.quaternion == (1.0, 0.0, 0.0, 0.0)
    assert reading.rpy == (0.0, 0.0, 0.25)
    assert reading.temperature == pytest.approx(35.5)


@pytest.mark.asyncio
async def test_odometry_reading(connected_adapter: UnitreeAdapter) -> None:
    reading = await connected_adapter.get_sensor("odometry")
    assert isinstance(reading, RobotOdometry)
    assert reading.position == pytest.approx((1.5, -0.5, 0.0))
    # orientation quaternion passes through
    assert reading.orientation[0] == pytest.approx(0.9689, abs=1e-4)


@pytest.mark.asyncio
async def test_lidar_points_parsed(connected_adapter: UnitreeAdapter) -> None:
    reading = await connected_adapter.get_sensor("lidar")
    assert isinstance(reading, LiDARScan)
    assert len(reading) == 3
    assert reading.points[0] == pytest.approx((1.0, 2.0, 3.0))
    assert reading.points[2] == pytest.approx((-1.0, 0.0, 0.5))


@pytest.mark.asyncio
async def test_motor_reading(connected_adapter: UnitreeAdapter) -> None:
    reading = await connected_adapter.get_sensor("motor.3")
    assert isinstance(reading, MotorState)
    assert reading.index == 3
    assert reading.position == pytest.approx(0.3)
    assert reading.temperature == pytest.approx(28.0)  # 25 + 3


@pytest.mark.asyncio
async def test_motor_out_of_range_raises(
    connected_adapter: UnitreeAdapter,
) -> None:
    with pytest.raises(SensorNotAvailable):
        await connected_adapter.get_sensor("motor.99")


@pytest.mark.asyncio
async def test_motor_non_numeric_index_raises(
    connected_adapter: UnitreeAdapter,
) -> None:
    with pytest.raises(SensorNotAvailable):
        await connected_adapter.get_sensor("motor.abc")


# --------------------------------------------------------------------------- #
# subscribe                                                                   #
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
async def test_subscribe_before_connect_raises(adapter: UnitreeAdapter) -> None:
    with pytest.raises(AdapterConnectionError):
        adapter.subscribe("battery", lambda _: None)


@pytest.mark.asyncio
async def test_subscribe_unknown_sensor_raises(
    connected_adapter: UnitreeAdapter,
) -> None:
    with pytest.raises(SensorNotAvailable):
        connected_adapter.subscribe("warp_drive", lambda _: None)


@pytest.mark.asyncio
async def test_subscribe_battery_fires_on_low_state(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[BatteryState] = []
        adapter.subscribe("battery", readings.append)
        fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
        assert len(readings) == 1
        assert isinstance(readings[0], BatteryState)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_subscribe_imu_fires_on_sport_state(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[IMUData] = []
        adapter.subscribe("imu", readings.append)
        fake_conn.fire(RTC_TOPIC["LF_SPORT_MOD_STATE"], SPORT_STATE_PAYLOAD)
        assert len(readings) == 1
        assert isinstance(readings[0], IMUData)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_subscribe_odometry_fires_on_robotodom(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[RobotOdometry] = []
        adapter.subscribe("odometry", readings.append)
        fake_conn.fire(RTC_TOPIC["ROBOTODOM"], ROBOTODOM_PAYLOAD)
        assert len(readings) == 1
        assert readings[0].position == pytest.approx((1.5, -0.5, 0.0))
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_subscribe_lidar_fires_on_ulidar_array(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[LiDARScan] = []
        adapter.subscribe("lidar", readings.append)
        fake_conn.fire(RTC_TOPIC["ULIDAR_ARRAY"], LIDAR_PAYLOAD)
        assert len(readings) == 1
        assert len(readings[0]) == 3
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_subscribe_motor_fires_on_low_state(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[MotorState] = []
        adapter.subscribe("motor.5", readings.append)
        fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
        assert len(readings) == 1
        assert readings[0].index == 5
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_unsubscribe_stops_callbacks(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    try:
        readings: list[BatteryState] = []
        unsub = adapter.subscribe("battery", readings.append)
        unsub()
        fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
        assert readings == []
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_disconnect_clears_subscriptions(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    await adapter.connect()
    readings: list[BatteryState] = []
    adapter.subscribe("battery", readings.append)
    await adapter.disconnect()

    # Reconnect; old subscription must not survive.
    await adapter.connect()
    fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
    assert readings == []
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_subscriber_exception_does_not_break_adapter(
    adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    """A misbehaving callback must not prevent other subscribers from firing."""
    await adapter.connect()
    try:
        good: list[BatteryState] = []
        adapter.subscribe("battery", lambda _: (_ for _ in ()).throw(RuntimeError("bad")))
        adapter.subscribe("battery", good.append)
        fake_conn.fire(RTC_TOPIC["LOW_STATE"], LOW_STATE_PAYLOAD)
        assert len(good) == 1
    finally:
        await adapter.disconnect()
