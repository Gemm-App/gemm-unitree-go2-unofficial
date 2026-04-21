"""Unit tests for UnitreeAdapter (no hardware required)."""

from __future__ import annotations

import struct

import pytest
from gemm.errors import AdapterConnectionError, SensorNotAvailable
from gemm.types import (
    BatteryState,
    IMUData,
    MotorState,
    RobotOdometry,
    RobotState,
    TaskResult,
)

from gemm_unitree_go2_unofficial import UnitreeAdapter
from gemm_unitree_go2_unofficial.adapter import LiDARScan


class TestUnitreeAdapterActions:
    @pytest.mark.asyncio
    async def test_get_state_returns_robot_state(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            state = await adapter.get_state()
            assert isinstance(state, RobotState)
            assert 0.0 <= state.battery <= 1.0
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_move_velocity_returns_success(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            result = await adapter.execute("move_velocity", {"x": 0.2, "y": 0.0, "z": 0.0})
            assert isinstance(result, TaskResult)
            assert result.ok
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_stand_up_returns_success(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            result = await adapter.execute("stand_up", {})
            assert result.ok
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_stand_down_returns_success(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            result = await adapter.execute("stand_down", {})
            assert result.ok
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_stop_returns_success(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            result = await adapter.execute("stop", {})
            assert result.ok
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_unknown_action_returns_failure(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            result = await adapter.execute("do_backflip_please", {})
            assert not result.ok
            assert "unsupported" in (result.error or "").lower()
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_raises_before_connect(self, adapter: UnitreeAdapter) -> None:
        with pytest.raises(AdapterConnectionError):
            await adapter.get_state()

    def test_requires_ip_or_serial(self) -> None:
        with pytest.raises(ValueError, match="ip"):
            UnitreeAdapter(name="x")

    @pytest.mark.asyncio
    async def test_move_to_at_goal_returns_success(self, adapter: UnitreeAdapter) -> None:
        """Already at (0,0) — should complete immediately."""
        await adapter.connect()
        try:
            result = await adapter.execute("move_to", {"x": 0.0, "y": 0.0})
            assert result.ok
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_move_to_timeout_returns_failure(self, adapter: UnitreeAdapter) -> None:
        """Unreachable goal with very short timeout -> failure result, not exception."""
        await adapter.connect()
        try:
            result = await adapter.execute("move_to", {"x": 999.0, "y": 999.0, "timeout": 0.05})
            assert not result.ok
            assert "timed out" in (result.error or "").lower()
        finally:
            await adapter.disconnect()


# ------------------------------------------------------------------ #
# Sensor: get_sensor tests                                            #
# ------------------------------------------------------------------ #


class TestGetSensor:
    @pytest.mark.asyncio
    async def test_get_sensor_before_connect_raises(self, adapter: UnitreeAdapter) -> None:
        with pytest.raises(AdapterConnectionError):
            await adapter.get_sensor("battery")

    @pytest.mark.asyncio
    async def test_get_sensor_unknown_raises_sensor_not_available(
        self, adapter: UnitreeAdapter
    ) -> None:
        await adapter.connect()
        try:
            with pytest.raises(SensorNotAvailable):
                await adapter.get_sensor("flux_capacitor")
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_imu_returns_imu_data(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("imu")
            assert isinstance(reading, IMUData)
            assert len(reading.quaternion) == 4
            assert len(reading.rpy) == 3
            assert len(reading.angular_velocity) == 3
            assert len(reading.linear_acceleration) == 3
            assert reading.temperature == pytest.approx(35.0)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_battery_returns_battery_state(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("battery")
            assert isinstance(reading, BatteryState)
            assert 0.0 <= reading.soc <= 1.0
            assert reading.voltage > 0.0
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_battery_soc_is_normalised(self, adapter: UnitreeAdapter) -> None:
        """bms_state.soc arrives as 0-100; adapter should normalise to 0.0-1.0."""
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("battery")
            assert isinstance(reading, BatteryState)
            assert reading.soc == pytest.approx(0.8)  # 80 / 100
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_odometry_returns_robot_odometry(
        self, adapter: UnitreeAdapter
    ) -> None:
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("odometry")
            assert isinstance(reading, RobotOdometry)
            assert len(reading.position) == 3
            assert len(reading.orientation) == 4
            assert len(reading.linear_velocity) == 3
            assert len(reading.angular_velocity) == 3
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_lidar_returns_lidar_scan(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("lidar")
            assert isinstance(reading, LiDARScan)
            # conftest fires 2 packed float32 XYZ points
            assert len(reading) == 2
            assert reading.points[0] == pytest.approx((1.0, 2.0, 3.0))
            assert reading.points[1] == pytest.approx((4.0, 5.0, 6.0))
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_motor_returns_motor_state(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            reading = await adapter.get_sensor("motor.0")
            assert isinstance(reading, MotorState)
            assert reading.index == 0
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_motor_index_out_of_range_raises(
        self, adapter: UnitreeAdapter
    ) -> None:
        await adapter.connect()
        try:
            with pytest.raises(SensorNotAvailable):
                await adapter.get_sensor("motor.99")
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_get_sensor_motor_beyond_reported_count_raises(
        self, adapter: UnitreeAdapter
    ) -> None:
        """Motor index within the 0-19 range but robot only reported 12."""
        await adapter.connect()
        try:
            with pytest.raises(SensorNotAvailable):
                await adapter.get_sensor("motor.15")
        finally:
            await adapter.disconnect()


# ------------------------------------------------------------------ #
# Sensor: subscribe / unsubscribe tests                               #
# ------------------------------------------------------------------ #


class TestSubscribe:
    @pytest.mark.asyncio
    async def test_subscribe_before_connect_raises(self, adapter: UnitreeAdapter) -> None:
        with pytest.raises(AdapterConnectionError):
            adapter.subscribe("battery", lambda _: None)

    @pytest.mark.asyncio
    async def test_subscribe_unknown_sensor_raises(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            with pytest.raises(SensorNotAvailable):
                adapter.subscribe("warp_drive", lambda _: None)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_returns_callable(self, adapter: UnitreeAdapter) -> None:
        await adapter.connect()
        try:
            unsub = adapter.subscribe("battery", lambda _: None)
            assert callable(unsub)
            unsub()
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_imu_fires_on_sport_state(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[IMUData] = []
            adapter.subscribe("imu", received.append)

            # Fire a fresh SPORT_STATE message manually
            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["LF_SPORT_MOD_STATE"]](
                {
                    "imu_state": {
                        "quaternion": [0.9998, 0.0, 0.0, 0.02],
                        "gyroscope": [0.01, 0.0, -0.01],
                        "accelerometer": [0.1, 0.0, 9.8],
                        "rpy": [0.0, 0.01, 0.04],
                        "temperature": 38.0,
                    }
                }
            )
            assert len(received) == 1
            assert isinstance(received[0], IMUData)
            assert received[0].temperature == pytest.approx(38.0)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_battery_fires_on_low_state(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[BatteryState] = []
            adapter.subscribe("battery", received.append)

            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["LOW_STATE"]](
                {
                    "bms_state": {
                        "soc": 60.0,
                        "power_v": 24.0,
                        "power_a": -3.0,
                        "temperature": 30.0,
                    },
                    "motor_state": [],
                }
            )
            assert len(received) == 1
            assert isinstance(received[0], BatteryState)
            assert received[0].soc == pytest.approx(0.6)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_odometry_fires_on_robotodom(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[RobotOdometry] = []
            adapter.subscribe("odometry", received.append)

            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["ROBOTODOM"]](
                {
                    "position": {"x": 1.5, "y": 0.3, "z": 0.0},
                    "orientation": {"w": 0.99, "x": 0.0, "y": 0.0, "z": 0.14},
                }
            )
            assert len(received) == 1
            assert isinstance(received[0], RobotOdometry)
            assert received[0].position[0] == pytest.approx(1.5)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_lidar_fires_on_ulidar_array(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[LiDARScan] = []
            adapter.subscribe("lidar", received.append)

            raw = struct.pack("<3f", 7.0, 8.0, 9.0)  # single point
            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["ULIDAR_ARRAY"]](raw)
            assert len(received) == 1
            assert isinstance(received[0], LiDARScan)
            assert received[0].points[0] == pytest.approx((7.0, 8.0, 9.0))
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_subscribe_motor_fires_on_low_state(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[MotorState] = []
            adapter.subscribe("motor.2", received.append)

            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["LOW_STATE"]](
                {
                    "bms_state": {"soc": 75.0, "power_v": 24.0, "power_a": 0.0},
                    "motor_state": [
                        {"q": 0.1 * i, "dq": 0.0, "tau_est": 0.0, "temperature": 25.0}
                        for i in range(5)
                    ],
                }
            )
            assert len(received) == 1
            assert isinstance(received[0], MotorState)
            assert received[0].index == 2
            assert received[0].position == pytest.approx(0.2)
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_unsubscribe_stops_callbacks(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        try:
            received: list[BatteryState] = []
            unsub = adapter.subscribe("battery", received.append)
            unsub()

            cb = mock_conn._callbacks  # type: ignore[attr-defined]
            cb[RTC_TOPIC["LOW_STATE"]](
                {
                    "bms_state": {"soc": 50.0, "power_v": 23.0, "power_a": -1.0},
                    "motor_state": [],
                }
            )
            assert received == []
        finally:
            await adapter.disconnect()

    @pytest.mark.asyncio
    async def test_disconnect_clears_subscriptions(
        self, adapter: UnitreeAdapter, mock_conn: object
    ) -> None:
        from unitree_webrtc_connect import RTC_TOPIC

        await adapter.connect()
        received: list[BatteryState] = []
        adapter.subscribe("battery", received.append)
        await adapter.disconnect()

        # After reconnect, the old subscription must be gone
        await adapter.connect()
        cb = mock_conn._callbacks  # type: ignore[attr-defined]
        cb[RTC_TOPIC["LOW_STATE"]](
            {
                "bms_state": {"soc": 90.0, "power_v": 25.0, "power_a": 0.0},
                "motor_state": [],
            }
        )
        assert received == []
        await adapter.disconnect()
