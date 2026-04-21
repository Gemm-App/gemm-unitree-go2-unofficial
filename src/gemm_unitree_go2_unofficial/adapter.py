"""Gemm adapter for Unitree Go2 via ``unitree_webrtc_connect``.

Maps the SDK's ``SPORT_CMD`` commands directly onto Gemm actions and the SDK's
built-in data-channel topics onto Gemm sensors. The adapter does not invent
any behaviour on top of the SDK — every action is a one-to-one passthrough to
``publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD[<cmd>], ...})``.

Actions
-------
Action names are snake_case versions of the SDK's ``SPORT_CMD`` keys. See
``SPORT_ACTIONS`` below for the full list.

Sensors
-------
    battery     — :class:`~gemm.types.BatteryState`  from ``LOW_STATE``
    imu         — :class:`~gemm.types.IMUData`       from ``LF_SPORT_MOD_STATE``
    odometry    — :class:`~gemm.types.RobotOdometry` from ``ROBOTODOM``
    lidar       — :class:`~gemm.types.LiDARScan`     from ``ULIDAR_ARRAY``
    motor.{i}   — :class:`~gemm.types.MotorState`    from ``LOW_STATE`` (i = 0..11)
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, TypeVar

import numpy as np
from gemm.errors import AdapterConnectionError, SensorError, SensorNotAvailable
from gemm.types import (
    BatteryState,
    IMUData,
    LiDARScan,
    MotorState,
    Pose,
    RobotOdometry,
    RobotState,
    SensorReading,
    TaskResult,
)
from unitree_webrtc_connect import (
    RTC_TOPIC,
    SPORT_CMD,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

logger = logging.getLogger(__name__)

_T = TypeVar("_T")


# Gemm action name → ``SPORT_CMD`` key in unitree_webrtc_connect.
# Every entry is a direct 1:1 passthrough: execute(name, params) forwards
# params as the ``parameter`` field of a SPORT_MOD request.
SPORT_ACTIONS: dict[str, str] = {
    "damp": "Damp",
    "balance_stand": "BalanceStand",
    "stop_move": "StopMove",
    "stand_up": "StandUp",
    "stand_down": "StandDown",
    "recovery_stand": "RecoveryStand",
    "euler": "Euler",
    "move": "Move",
    "sit": "Sit",
    "rise_sit": "RiseSit",
    "switch_gait": "SwitchGait",
    "trigger": "Trigger",
    "body_height": "BodyHeight",
    "foot_raise_height": "FootRaiseHeight",
    "speed_level": "SpeedLevel",
    "hello": "Hello",
    "stretch": "Stretch",
    "trajectory_follow": "TrajectoryFollow",
    "continuous_gait": "ContinuousGait",
    "content": "Content",
    "wallow": "Wallow",
    "dance1": "Dance1",
    "dance2": "Dance2",
    "get_body_height": "GetBodyHeight",
    "get_foot_raise_height": "GetFootRaiseHeight",
    "get_speed_level": "GetSpeedLevel",
    "switch_joystick": "SwitchJoystick",
    "pose": "Pose",
    "scrape": "Scrape",
    "front_flip": "FrontFlip",
    "left_flip": "LeftFlip",
    "right_flip": "RightFlip",
    "back_flip": "BackFlip",
    "front_jump": "FrontJump",
    "front_pounce": "FrontPounce",
    "wiggle_hips": "WiggleHips",
    "get_sport_state": "GetState",
    "economic_gait": "EconomicGait",
    "lead_follow": "LeadFollow",
    "finger_heart": "FingerHeart",
    "bound": "Bound",
    "moon_walk": "MoonWalk",
    "onesided_step": "OnesidedStep",
    "cross_step": "CrossStep",
    "handstand": "Handstand",
    "stand_out": "StandOut",
    "free_walk": "FreeWalk",
    "standup_extra": "Standup",
    "cross_walk": "CrossWalk",
}

_MOTOR_COUNT = 12  # Go2 has 12 joint motors (3 per leg, 4 legs).
_NAMED_SENSORS = frozenset({"battery", "imu", "odometry", "lidar"})


def _is_supported_sensor(sensor: str) -> bool:
    if sensor in _NAMED_SENSORS:
        return True
    if sensor.startswith("motor."):
        try:
            idx = int(sensor[len("motor.") :])
        except ValueError:
            return False
        return 0 <= idx < _MOTOR_COUNT
    return False


@dataclass
class _Cache:
    """Latest cached sensor readings, updated by WebRTC callbacks."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    battery_soc: float = 1.0

    imu: IMUData | None = field(default=None, repr=False)
    battery: BatteryState | None = field(default=None, repr=False)
    odometry: RobotOdometry | None = field(default=None, repr=False)
    motors: list[MotorState] = field(default_factory=list, repr=False)
    lidar: LiDARScan | None = field(default=None, repr=False)


class UnitreeAdapter:
    """Gemm adapter for a Unitree Go2 robot connected over WebRTC.

    Does not inherit from :class:`gemm.Adapter` — structural typing via the
    Protocol is enough.

    Parameters
    ----------
    name
        Adapter identifier used by the Gemm engine.
    ip
        Robot IP (required for ``LocalAP`` and a faster path for ``LocalSTA``).
    serial_number
        Unitree serial number; required for ``Remote`` and an alternative to
        ``ip`` for ``LocalSTA`` (multicast discovery).
    connection_method
        One of :class:`unitree_webrtc_connect.WebRTCConnectionMethod`. Default
        is :attr:`WebRTCConnectionMethod.LocalSTA`.
    username / password
        Unitree account credentials, only needed for ``Remote``.
    lidar
        If ``True`` (default), subscribe to the LiDAR stream on connect. Set
        to ``False`` to skip LiDAR setup when you don't need it.
    """

    def __init__(
        self,
        name: str,
        *,
        ip: str | None = None,
        serial_number: str | None = None,
        connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA,
        username: str | None = None,
        password: str | None = None,
        lidar: bool = True,
    ) -> None:
        if ip is None and serial_number is None:
            raise ValueError("provide at least one of: ip, serial_number")
        self.name = name
        self._ip = ip
        self._serial = serial_number
        self._method = connection_method
        self._username = username
        self._password = password
        self._lidar_enabled = lidar

        self._conn: UnitreeWebRTCConnection | None = None
        self._cache = _Cache()
        self._lock = asyncio.Lock()
        self._subscribers: dict[str, list[Callable[[SensorReading], None]]] = {}

    # ------------------------------------------------------------------ #
    # Adapter protocol — lifecycle                                        #
    # ------------------------------------------------------------------ #

    async def connect(self) -> None:
        conn = UnitreeWebRTCConnection(
            connectionMethod=self._method,
            serialNumber=self._serial,
            ip=self._ip,
            username=self._username,
            password=self._password,
        )
        await conn.connect()

        pub_sub = conn.datachannel.pub_sub
        pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], self._on_low_state)
        pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport_state)
        pub_sub.subscribe(RTC_TOPIC["ROBOTODOM"], self._on_robotodom)

        if self._lidar_enabled:
            await self._enable_lidar(conn)

        self._conn = conn
        logger.info("UnitreeAdapter %r connected via %s", self.name, self._method.name)

    async def disconnect(self) -> None:
        if self._conn is None:
            return
        try:
            await self._conn.disconnect()
        except Exception:
            logger.debug("error during disconnect", exc_info=True)
        self._conn = None
        self._subscribers.clear()
        logger.info("UnitreeAdapter %r disconnected", self.name)

    # ------------------------------------------------------------------ #
    # Adapter protocol — state                                            #
    # ------------------------------------------------------------------ #

    async def get_state(self) -> RobotState:
        self._require_connected()
        async with self._lock:
            return RobotState(
                pose=Pose(x=self._cache.x, y=self._cache.y, yaw=self._cache.yaw),
                battery=self._cache.battery_soc,
            )

    # ------------------------------------------------------------------ #
    # Adapter protocol — actions                                          #
    # ------------------------------------------------------------------ #

    async def execute(self, action: str, params: dict[str, Any]) -> TaskResult:
        self._require_connected()

        cmd = SPORT_ACTIONS.get(action)
        if cmd is None:
            return TaskResult.failure(f"unsupported action: {action!r}")

        try:
            response = await self._send_sport(cmd, params)
        except Exception as exc:
            logger.exception("sport command %s failed", cmd)
            return TaskResult.failure(f"{cmd} failed: {exc}")

        data = _extract_response_data(response)
        return TaskResult.success(**data) if data else TaskResult.success()

    # ------------------------------------------------------------------ #
    # Adapter protocol — sensors                                          #
    # ------------------------------------------------------------------ #

    async def get_sensor(self, sensor: str) -> SensorReading:
        """Return the latest cached reading for *sensor*.

        Raises
        ------
        AdapterConnectionError
            If called before :meth:`connect`.
        SensorNotAvailable
            If *sensor* is not a recognised sensor name.
        SensorError
            If no data has arrived yet — give it a moment after ``connect()``.
        """
        self._require_connected()

        if sensor == "battery":
            return _require_cached(self._cache.battery, "battery", "LOW_STATE")
        if sensor == "imu":
            return _require_cached(self._cache.imu, "imu", "LF_SPORT_MOD_STATE")
        if sensor == "odometry":
            return _require_cached(self._cache.odometry, "odometry", "ROBOTODOM")
        if sensor == "lidar":
            return _require_cached(self._cache.lidar, "lidar", "ULIDAR_ARRAY")
        if sensor.startswith("motor."):
            return self._get_motor(sensor)

        raise SensorNotAvailable(
            f"UnitreeAdapter does not support sensor {sensor!r}. "
            f"Supported: battery, imu, odometry, lidar, motor.0 .. motor.{_MOTOR_COUNT - 1}"
        )

    def subscribe(
        self,
        sensor: str,
        callback: Callable[[SensorReading], None],
    ) -> Callable[[], None]:
        """Register *callback* for new readings of *sensor*.

        The callback runs on the WebRTC event thread; it must not block.

        Returns an unsubscribe callable — call it (no args) to stop.
        """
        self._require_connected()
        if not _is_supported_sensor(sensor):
            raise SensorNotAvailable(
                f"UnitreeAdapter does not support sensor {sensor!r}. "
                f"Supported: battery, imu, odometry, lidar, motor.0 .. motor.{_MOTOR_COUNT - 1}"
            )
        listeners = self._subscribers.setdefault(sensor, [])
        listeners.append(callback)

        def unsubscribe() -> None:
            with contextlib.suppress(ValueError):
                listeners.remove(callback)

        return unsubscribe

    # ------------------------------------------------------------------ #
    # SDK plumbing                                                        #
    # ------------------------------------------------------------------ #

    async def _send_sport(
        self, cmd_name: str, parameter: dict[str, Any] | None
    ) -> Any:
        conn = self._require_connected()
        return await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD[cmd_name], "parameter": parameter or {}},
        )

    async def _enable_lidar(self, conn: UnitreeWebRTCConnection) -> None:
        # Switch to the native decoder: libvoxel returns a triangulated mesh;
        # native returns an (N, 3) point array that maps cleanly to LiDARScan.
        try:
            conn.datachannel.set_decoder("native")
        except Exception:
            logger.debug("set_decoder('native') failed", exc_info=True)

        try:
            await conn.datachannel.disableTrafficSaving(True)
        except Exception:
            logger.debug("disableTrafficSaving failed", exc_info=True)

        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], self._on_lidar)

        try:
            conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["ULIDAR_SWITCH"], "ON"
            )
        except Exception:
            logger.debug("LiDAR switch-on failed; stream may not start", exc_info=True)

    # ------------------------------------------------------------------ #
    # Subscriber callbacks (sync — called on the WebRTC event thread)     #
    # ------------------------------------------------------------------ #

    def _on_low_state(self, msg: dict[str, Any]) -> None:
        data = _payload(msg)
        try:
            bms = data.get("bms_state") or data.get("battery_state") or {}
            raw_soc = float(bms.get("soc", bms.get("percent", 0.0)))
            soc = raw_soc / 100.0 if raw_soc > 1.0 else raw_soc
            battery = BatteryState(
                soc=soc,
                voltage=float(bms.get("power_v", bms.get("voltage", 0.0))),
                current=float(bms.get("power_a", bms.get("current", 0.0))),
                temperature=float(bms.get("temperature", 0.0)),
                cycle_count=int(bms.get("cycle_count", 0)),
            )
            self._cache.battery = battery
            self._cache.battery_soc = soc
            self._fire("battery", battery)
        except (TypeError, ValueError):
            logger.debug("bad LOW_STATE/bms: %s", data, exc_info=True)

        motor_list = data.get("motor_state") or []
        motors: list[MotorState] = []
        for i, m in enumerate(motor_list):
            if i >= _MOTOR_COUNT or not isinstance(m, dict):
                break
            try:
                motor = MotorState(
                    index=i,
                    position=float(m.get("q", 0.0)),
                    velocity=float(m.get("dq", 0.0)),
                    torque=float(m.get("tau_est", 0.0)),
                    temperature=float(m.get("temperature", 0.0)),
                )
            except (TypeError, ValueError):
                continue
            motors.append(motor)
            self._fire(f"motor.{i}", motor)
        if motors:
            self._cache.motors = motors

    def _on_sport_state(self, msg: dict[str, Any]) -> None:
        data = _payload(msg)
        imu_raw = data.get("imu_state")
        if not isinstance(imu_raw, dict):
            return
        try:
            quat = imu_raw.get("quaternion") or [1.0, 0.0, 0.0, 0.0]
            gyro = imu_raw.get("gyroscope") or [0.0, 0.0, 0.0]
            accel = imu_raw.get("accelerometer") or [0.0, 0.0, 0.0]
            rpy = imu_raw.get("rpy") or [0.0, 0.0, 0.0]
            imu = IMUData(
                quaternion=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
                angular_velocity=(float(gyro[0]), float(gyro[1]), float(gyro[2])),
                linear_acceleration=(float(accel[0]), float(accel[1]), float(accel[2])),
                rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
                temperature=float(imu_raw.get("temperature", 0.0)),
            )
            self._cache.imu = imu
            self._fire("imu", imu)
            self._cache.yaw = imu.rpy[2]
        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("bad LF_SPORT_MOD_STATE: %s", data, exc_info=True)

    def _on_robotodom(self, msg: dict[str, Any]) -> None:
        data = _payload(msg)
        try:
            pose_obj = data.get("pose", data)
            if not isinstance(pose_obj, dict):
                pose_obj = data
            pos = pose_obj.get("position") or data.get("position") or {}
            px, py, pz = _vec3(pos)

            ori = pose_obj.get("orientation") or data.get("orientation") or {}
            ow, ox, oy, oz = _quat(ori)

            lv = _vec3(data.get("linear_velocity", [0.0, 0.0, 0.0]))
            av = _vec3(data.get("angular_velocity", [0.0, 0.0, 0.0]))

            odom = RobotOdometry(
                position=(px, py, pz),
                orientation=(ow, ox, oy, oz),
                linear_velocity=lv,
                angular_velocity=av,
            )
            self._cache.odometry = odom
            self._cache.x = px
            self._cache.y = py
            self._cache.yaw = _quat_to_yaw(ow, ox, oy, oz)
            self._fire("odometry", odom)
        except (KeyError, TypeError, ValueError):
            logger.debug("bad ROBOTODOM: %s", data, exc_info=True)

    def _on_lidar(self, msg: dict[str, Any]) -> None:
        data = _payload(msg)
        try:
            scan = _parse_lidar(data)
        except Exception:
            logger.debug("bad LiDAR frame", exc_info=True)
            return
        self._cache.lidar = scan
        self._fire("lidar", scan)

    # ------------------------------------------------------------------ #
    # Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _get_motor(self, sensor: str) -> MotorState:
        try:
            idx = int(sensor[len("motor.") :])
        except ValueError:
            raise SensorNotAvailable(f"invalid motor sensor name {sensor!r}") from None
        if not 0 <= idx < _MOTOR_COUNT:
            raise SensorNotAvailable(
                f"motor index {idx} out of range (0..{_MOTOR_COUNT - 1})"
            )
        if not self._cache.motors:
            raise SensorError(
                "no motor data yet — wait a moment after connect() for LOW_STATE"
            )
        if idx >= len(self._cache.motors):
            raise SensorNotAvailable(
                f"motor.{idx} not reported by robot "
                f"(only {len(self._cache.motors)} motors seen)"
            )
        return self._cache.motors[idx]

    def _fire(self, sensor: str, reading: SensorReading) -> None:
        for cb in list(self._subscribers.get(sensor, [])):
            try:
                cb(reading)
            except Exception:
                logger.exception("subscriber for %s raised", sensor)

    def _require_connected(self) -> UnitreeWebRTCConnection:
        if self._conn is None:
            raise AdapterConnectionError(f"adapter {self.name!r} is not connected")
        return self._conn

# --------------------------------------------------------------------------- #
# Module-level parsing helpers                                                #
# --------------------------------------------------------------------------- #


def _require_cached(value: _T | None, sensor: str, topic_hint: str) -> _T:
    if value is None:
        raise SensorError(
            f"no {sensor} data yet — wait a moment after connect() for {topic_hint}"
        )
    return value


def _payload(msg: Any) -> dict[str, Any]:
    """Return the ``data`` sub-dict of an RTC message, or the message itself."""
    if isinstance(msg, dict):
        inner = msg.get("data")
        if isinstance(inner, dict):
            return inner
        return msg
    return {}


def _vec3(v: Any) -> tuple[float, float, float]:
    if isinstance(v, dict):
        return (
            float(v.get("x", 0.0)),
            float(v.get("y", 0.0)),
            float(v.get("z", 0.0)),
        )
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return float(v[0]), float(v[1]), float(v[2])
    return 0.0, 0.0, 0.0


def _quat(q: Any) -> tuple[float, float, float, float]:
    if isinstance(q, dict):
        return (
            float(q.get("w", 1.0)),
            float(q.get("x", 0.0)),
            float(q.get("y", 0.0)),
            float(q.get("z", 0.0)),
        )
    if isinstance(q, (list, tuple)) and len(q) >= 4:
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return 1.0, 0.0, 0.0, 0.0


def _quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _parse_lidar(data: dict[str, Any]) -> LiDARScan:
    """Build a :class:`LiDARScan` from a decoded ``ULIDAR_ARRAY`` payload.

    After ``set_decoder("native")`` the SDK puts the decoded result in
    ``data["data"]`` as ``{"points": np.ndarray (N, 3)}``. A couple of
    fallback shapes are accepted so the adapter keeps working if a user
    switches decoders or the firmware returns raw bytes.
    """
    ts = time.time()
    origin = _vec3(data.get("origin", [0.0, 0.0, 0.0]))
    resolution = float(data.get("resolution", 0.05))

    inner = data.get("data")
    points: list[tuple[float, float, float]] = []

    if isinstance(inner, dict) and "points" in inner:
        pts = np.asarray(inner["points"], dtype=np.float64)
        if pts.ndim == 2 and pts.shape[1] == 3:
            points = [(float(p[0]), float(p[1]), float(p[2])) for p in pts]
    elif isinstance(inner, dict) and "positions" in inner:
        raw = np.asarray(inner["positions"], dtype=np.uint8).tobytes()
        n = len(raw) // 12
        if n:
            floats = np.frombuffer(raw[: n * 12], dtype=np.float32).reshape(n, 3)
            points = [(float(p[0]), float(p[1]), float(p[2])) for p in floats]
    elif isinstance(inner, (bytes, bytearray)):
        n = len(inner) // 12
        if n:
            floats = np.frombuffer(bytes(inner)[: n * 12], dtype=np.float32).reshape(n, 3)
            points = [(float(p[0]), float(p[1]), float(p[2])) for p in floats]

    return LiDARScan(points=points, origin=origin, resolution=resolution, timestamp=ts)


def _extract_response_data(response: Any) -> dict[str, Any]:
    """Pull a usable dict out of a ``publish_request_new`` response.

    Go2 sport responses typically arrive as ``{"info": {...}, "data": {...}}``;
    getter commands put their value in ``data``.
    """
    if not isinstance(response, dict):
        return {}
    data = response.get("data")
    if isinstance(data, dict):
        return data
    if data is not None:
        return {"value": data}
    return {}
