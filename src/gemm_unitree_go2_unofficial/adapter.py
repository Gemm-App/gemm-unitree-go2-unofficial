"""Gemm adapter for Unitree Go2 robots via unitree_webrtc_connect.

Supports
--------
Actions
~~~~~~~
    move_to          — navigate to (x, y) in the current SLAM odometry frame
    move_velocity    — direct body-frame velocity (vx, vy, vz)
    stand_up         — StandUp sport command
    stand_down       — StandDown sport command
    balance_stand    — BalanceStand sport command
    stop             — StopMove sport command

Sensors (get_sensor / subscribe)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    battery          — :class:`~gemm.types.BatteryState` from LOW_STATE / bms_state
    imu              — :class:`~gemm.types.IMUData` from LF_SPORT_MOD_STATE
    odometry         — :class:`~gemm.types.RobotOdometry` from ROBOTODOM
    lidar            — :class:`~gemm.types.LiDARScan` from ULIDAR_ARRAY
    motor.{i}        — :class:`~gemm.types.MotorState` for joint i (0-based) from LOW_STATE

Connection example::

    from gemm_unitree_go2_unofficial import UnitreeAdapter
    from unitree_webrtc_connect import WebRTCConnectionMethod

    adapter = UnitreeAdapter(
        name="go2",
        ip="192.168.12.1",                          # robot AP address
        connection_method=WebRTCConnectionMethod.LocalAP,
    )

SLAM frame note
---------------
(x, y) in move_to are coordinates in the LiDAR SLAM odometry frame whose
origin is wherever the robot initialised its SLAM session (power-on position).
The frame does not persist across power cycles.

Message format note
-------------------
Field names in ROBOTODOM / SPORT_MOD_STATE vary by firmware version.
Run with DEBUG logging to inspect live payloads and adjust the _on_* methods
below if needed::

    import logging
    logging.basicConfig(level=logging.DEBUG)
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

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

from gemm_unitree_go2_unofficial._controller import DEFAULT_TIMEOUT, PositionController

logger = logging.getLogger(__name__)

# Sensors natively supported by UnitreeAdapter
_NAMED_SENSORS = frozenset({"battery", "imu", "odometry", "lidar"})
_MOTOR_COUNT = 20  # Go2 has up to 12 joint motors; reserve indices 0-19


def _is_supported_sensor(sensor: str) -> bool:
    if sensor in _NAMED_SENSORS:
        return True
    if sensor.startswith("motor."):
        try:
            idx = int(sensor[6:])
            return 0 <= idx < _MOTOR_COUNT
        except ValueError:
            return False
    return False


@dataclass
class _RobotState:
    """Mutable snapshot updated by subscriber callbacks."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    battery: float = 1.0
    # Lock so move_to reads a consistent (x, y, yaw) triple
    _lock: asyncio.Lock = field(default_factory=asyncio.Lock, repr=False)

    # Sensor caches — None until the first message arrives
    imu: IMUData | None = field(default=None, repr=False)
    battery_state: BatteryState | None = field(default=None, repr=False)
    odometry: RobotOdometry | None = field(default=None, repr=False)
    motors: list[MotorState] = field(default_factory=list, repr=False)
    lidar: LiDARScan | None = field(default=None, repr=False)


class UnitreeAdapter:
    """Gemm adapter for Unitree Go2 / G1 via unitree_webrtc_connect.

    Does not inherit from any Gemm base class — structural typing via the
    ``Adapter`` protocol is sufficient.
    """

    def __init__(
        self,
        name: str,
        *,
        ip: str | None = None,
        serial_number: str | None = None,
        connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA,
    ) -> None:
        if ip is None and serial_number is None:
            raise ValueError("provide at least one of: ip, serial_number")
        self.name = name
        self._ip = ip
        self._serial = serial_number
        self._method = connection_method
        self._conn: UnitreeWebRTCConnection | None = None
        self._state = _RobotState()
        # Sensor subscription registry: sensor name -> list of callbacks
        self._sensor_callbacks: dict[str, list[Callable[[SensorReading], None]]] = {}

    # ------------------------------------------------------------------ #
    # Adapter protocol                                                     #
    # ------------------------------------------------------------------ #

    async def connect(self) -> None:
        kwargs: dict[str, Any] = {}
        if self._ip is not None:
            kwargs["ip"] = self._ip
        if self._serial is not None:
            kwargs["serialNumber"] = self._serial

        conn = UnitreeWebRTCConnection(self._method, **kwargs)
        await conn.connect()

        # Subscribe to odometry — LiDAR SLAM pose (x, y, yaw in SLAM frame)
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ROBOTODOM"], self._on_robotodom)
        # Subscribe to sport state — IMU + fallback pose
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport_state)
        # Subscribe to low state — battery + motor states
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], self._on_low_state)

        # Enable LiDAR streaming:
        # 1. Disable traffic-saving so the robot sends raw point-cloud data
        try:
            conn.disableTrafficSaving(True)
        except (AttributeError, Exception) as exc:
            logger.debug("disableTrafficSaving not available: %s", exc)

        # 2. Subscribe to the ULIDAR_ARRAY topic before turning on the switch
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], self._on_lidar)

        # 3. Send the switch-on command
        try:
            conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "on")
        except (AttributeError, Exception) as exc:
            logger.debug("LiDAR switch-on failed: %s — LiDAR may be unavailable", exc)

        self._conn = conn
        logger.info("UnitreeAdapter %r connected via %s", self.name, self._method)

    async def disconnect(self) -> None:
        if self._conn is not None:
            # unitree_webrtc_connect does not expose an explicit close();
            # dropping the reference lets the WebRTC stack clean up.
            self._conn = None
            self._sensor_callbacks.clear()
            logger.info("UnitreeAdapter %r disconnected", self.name)

    async def get_state(self) -> RobotState:
        self._require_connected()
        async with self._state._lock:
            return RobotState(
                pose=Pose(x=self._state.x, y=self._state.y, yaw=self._state.yaw),
                battery=self._state.battery,
            )

    async def execute(self, action: str, params: dict[str, Any]) -> TaskResult:
        self._require_connected()

        if action == "move_to":
            return await self._action_move_to(params)

        if action == "move_velocity":
            return await self._action_move_velocity(params)

        if action == "stand_up":
            await self._send_sport_cmd("StandUp")
            return TaskResult.success()

        if action == "stand_down":
            await self._send_sport_cmd("StandDown")
            return TaskResult.success()

        if action == "balance_stand":
            await self._send_sport_cmd("BalanceStand")
            return TaskResult.success()

        if action == "stop":
            await self._send_sport_cmd("StopMove")
            return TaskResult.success()

        return TaskResult.failure(f"unsupported action: {action!r}")

    # ------------------------------------------------------------------ #
    # Sensor interface                                                     #
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
            If no data has been received for the sensor yet — call
            :meth:`connect` and wait a moment before reading.
        """
        self._require_connected()

        if sensor == "imu":
            if self._state.imu is None:
                raise SensorError(
                    "no IMU data yet — wait a moment after connect() "
                    "for the first LF_SPORT_MOD_STATE message"
                )
            return self._state.imu

        if sensor == "battery":
            if self._state.battery_state is None:
                raise SensorError(
                    "no battery data yet — wait a moment after connect() "
                    "for the first LOW_STATE message"
                )
            return self._state.battery_state

        if sensor == "odometry":
            if self._state.odometry is None:
                raise SensorError(
                    "no odometry data yet — wait a moment after connect() "
                    "for the first ROBOTODOM message"
                )
            return self._state.odometry

        if sensor == "lidar":
            if self._state.lidar is None:
                raise SensorError(
                    "no LiDAR scan yet — LiDAR needs a moment to start streaming " "after connect()"
                )
            return self._state.lidar

        if sensor.startswith("motor."):
            try:
                idx = int(sensor[6:])
            except ValueError:
                raise SensorNotAvailable(f"invalid motor sensor name {sensor!r}") from None
            if not (0 <= idx < _MOTOR_COUNT):
                raise SensorNotAvailable(f"motor index {idx} out of range (0-{_MOTOR_COUNT - 1})")
            if not self._state.motors:
                raise SensorError(
                    "no motor data yet — wait a moment after connect() "
                    "for the first LOW_STATE message"
                )
            if idx >= len(self._state.motors):
                raise SensorNotAvailable(
                    f"motor.{idx} not reported by robot "
                    f"(only {len(self._state.motors)} motors seen)"
                )
            return self._state.motors[idx]

        raise SensorNotAvailable(
            f"UnitreeAdapter does not support sensor {sensor!r}. "
            f"Supported: battery, imu, odometry, lidar, motor.0 - motor.{_MOTOR_COUNT - 1}"
        )

    def subscribe(
        self,
        sensor: str,
        callback: Callable[[SensorReading], None],
    ) -> Callable[[], None]:
        """Register *callback* to be called whenever *sensor* produces a new reading.

        The callback is invoked synchronously from the WebRTC event thread — it
        must not block. Use :func:`asyncio.run_coroutine_threadsafe` if you need
        async behaviour from a callback.

        Returns an *unsubscribe* callable — call it (no args) to stop receiving
        callbacks for this registration.

        Raises
        ------
        AdapterConnectionError
            If called before :meth:`connect`.
        SensorNotAvailable
            If *sensor* is not a recognised sensor name.
        """
        self._require_connected()
        if not _is_supported_sensor(sensor):
            raise SensorNotAvailable(
                f"UnitreeAdapter does not support sensor {sensor!r}. "
                f"Supported: battery, imu, odometry, lidar, motor.0 - motor.{_MOTOR_COUNT - 1}"
            )
        listeners = self._sensor_callbacks.setdefault(sensor, [])
        listeners.append(callback)

        def unsubscribe() -> None:
            with contextlib.suppress(ValueError):
                listeners.remove(callback)

        return unsubscribe

    # ------------------------------------------------------------------ #
    # Action handlers                                                      #
    # ------------------------------------------------------------------ #

    async def _action_move_to(self, params: dict[str, Any]) -> TaskResult:
        goal_x = float(params["x"])
        goal_y = float(params["y"])
        timeout = float(params.get("timeout", DEFAULT_TIMEOUT))

        controller = PositionController(
            get_pose=self._get_pose_tuple,
            send_move=self._send_move,
        )

        try:
            x, y, yaw = await controller.move_to(goal_x, goal_y, timeout=timeout)
        except TimeoutError as exc:
            return TaskResult.failure(str(exc))

        return TaskResult.success(pose={"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)})

    async def _action_move_velocity(self, params: dict[str, Any]) -> TaskResult:
        vx = float(params.get("x", 0.0))
        vy = float(params.get("y", 0.0))
        vz = float(params.get("z", 0.0))
        await self._send_move(vx, vy, vz)
        return TaskResult.success()

    # ------------------------------------------------------------------ #
    # Low-level SDK helpers                                                #
    # ------------------------------------------------------------------ #

    async def _get_pose_tuple(self) -> tuple[float, float, float]:
        async with self._state._lock:
            return self._state.x, self._state.y, self._state.yaw

    async def _send_move(self, vx: float, vy: float, vz: float) -> None:
        conn = self._require_conn()
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Move"], "parameter": {"x": vx, "y": vy, "z": vz}},
        )

    async def _send_sport_cmd(self, cmd: str, parameter: dict[str, Any] | None = None) -> None:
        conn = self._require_conn()
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD[cmd], "parameter": parameter or {}},
        )

    # ------------------------------------------------------------------ #
    # Subscriber callbacks (sync — called from WebRTC event loop)         #
    # ------------------------------------------------------------------ #

    def _on_robotodom(self, msg: dict[str, Any]) -> None:
        """Parse rt/utlidar/robot_pose — primary source for SLAM position."""
        logger.debug("ROBOTODOM: %s", msg)
        try:
            data = msg.get("data", msg)  # some firmware versions wrap in "data"

            # Position
            pos = data["position"]
            if isinstance(pos, dict):
                px = float(pos["x"])
                py = float(pos["y"])
                pz = float(pos.get("z", 0.0))
            else:
                px = float(pos[0])
                py = float(pos[1])
                pz = float(pos[2]) if len(pos) > 2 else 0.0

            # Orientation quaternion (w, x, y, z)
            ori = data.get("orientation", {})
            if isinstance(ori, dict):
                ow = float(ori.get("w", 1.0))
                ox = float(ori.get("x", 0.0))
                oy = float(ori.get("y", 0.0))
                oz = float(ori.get("z", 0.0))
            elif isinstance(ori, (list, tuple)) and len(ori) >= 4:
                ow, ox, oy, oz = (float(ori[i]) for i in range(4))
            else:
                ow, ox, oy, oz = 1.0, 0.0, 0.0, 0.0

            # Yaw — prefer explicit field, then IMU rpy, then derive from quaternion
            if "pose" in data and "yaw" in data["pose"]:
                yaw = float(data["pose"]["yaw"])
            elif "imu_state" in data:
                yaw = float(data["imu_state"]["rpy"][2])
            else:
                yaw = _quat_to_yaw({"w": ow, "x": ox, "y": oy, "z": oz})

            # Linear / angular velocity (optional fields)
            lv_raw = data.get("linear_velocity", [0.0, 0.0, 0.0])
            av_raw = data.get("angular_velocity", [0.0, 0.0, 0.0])
            lv = _parse_vec3(lv_raw)
            av = _parse_vec3(av_raw)

            # Update motion state
            self._state.x = px
            self._state.y = py
            self._state.yaw = yaw

            # Update odometry cache and fire sensor callbacks
            odometry = RobotOdometry(
                position=(px, py, pz),
                orientation=(ow, ox, oy, oz),
                linear_velocity=lv,
                angular_velocity=av,
            )
            self._state.odometry = odometry
            self._fire("odometry", odometry)

        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("unexpected ROBOTODOM format — raw msg: %s", msg)

    def _on_sport_state(self, msg: dict[str, Any]) -> None:
        """Parse LF_SPORT_MOD_STATE — IMU data + fallback pose."""
        logger.debug("SPORT_STATE: %s", msg)
        try:
            data = msg.get("data", msg)
            imu_raw = data["imu_state"]

            quat = imu_raw.get("quaternion", [1.0, 0.0, 0.0, 0.0])
            gyro = imu_raw.get("gyroscope", [0.0, 0.0, 0.0])
            accel = imu_raw.get("accelerometer", [0.0, 0.0, 9.81])
            rpy_raw = imu_raw.get("rpy", [0.0, 0.0, 0.0])
            temp_imu = float(imu_raw.get("temperature", 0.0))

            imu = IMUData(
                quaternion=(
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                ),
                angular_velocity=(float(gyro[0]), float(gyro[1]), float(gyro[2])),
                linear_acceleration=(float(accel[0]), float(accel[1]), float(accel[2])),
                rpy=(float(rpy_raw[0]), float(rpy_raw[1]), float(rpy_raw[2])),
                temperature=temp_imu,
            )
            self._state.imu = imu
            self._fire("imu", imu)

            # Keep yaw in sync
            self._state.yaw = float(rpy_raw[2])

            # Fallback pose — only applied if ROBOTODOM has not yet populated (x, y)
            pos = data.get("position")
            if pos is not None and self._state.x == 0.0 and self._state.y == 0.0:
                x = float(pos[0] if isinstance(pos, list) else pos["x"])
                y = float(pos[1] if isinstance(pos, list) else pos["y"])
                self._state.x = x
                self._state.y = y

        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("unexpected SPORT_STATE format — raw msg: %s", msg)

    def _on_low_state(self, msg: dict[str, Any]) -> None:
        """Parse LOW_STATE — battery level and per-joint motor states."""
        logger.debug("LOW_STATE: %s", msg)
        try:
            data = msg.get("data", msg)

            # Battery — try bms_state first, fall back to battery_state
            bms = data.get("bms_state") or data.get("battery_state") or {}
            raw_soc = float(bms.get("soc", bms.get("percent", 0.0)))
            soc = raw_soc / 100.0 if raw_soc > 1.0 else raw_soc
            voltage = float(bms.get("power_v", bms.get("voltage", 0.0)))
            current = float(bms.get("power_a", bms.get("current", 0.0)))
            temp_b = float(bms.get("temperature", 0.0))
            cycle = int(bms.get("cycle_count", 0))

            battery_state = BatteryState(
                soc=soc,
                voltage=voltage,
                current=current,
                temperature=temp_b,
                cycle_count=cycle,
            )
            self._state.battery = soc
            self._state.battery_state = battery_state
            self._fire("battery", battery_state)

            # Motor states — list of dicts with q (position), dq (velocity),
            # tau_est (torque), temperature
            motor_list = data.get("motor_state", [])
            motors: list[MotorState] = []
            for i, m in enumerate(motor_list):
                if not isinstance(m, dict):
                    continue
                motor = MotorState(
                    index=i,
                    position=float(m.get("q", 0.0)),
                    velocity=float(m.get("dq", 0.0)),
                    torque=float(m.get("tau_est", 0.0)),
                    temperature=float(m.get("temperature", 0.0)),
                )
                motors.append(motor)
                self._fire(f"motor.{i}", motor)

            if motors:
                self._state.motors = motors

        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("unexpected LOW_STATE format — raw msg: %s", msg)

    def _on_lidar(self, msg: Any) -> None:
        """Parse ULIDAR_ARRAY — binary point-cloud data."""
        logger.debug("LIDAR frame received, type=%s len=%s", type(msg).__name__, _msg_len(msg))
        try:
            scan = _parse_lidar_msg(msg)
            self._state.lidar = scan
            self._fire("lidar", scan)
        except Exception:
            logger.debug("failed to parse LiDAR message", exc_info=True)

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _fire(self, sensor: str, data: SensorReading) -> None:
        """Invoke all registered callbacks for *sensor*."""
        for cb in list(self._sensor_callbacks.get(sensor, [])):
            cb(data)

    # ------------------------------------------------------------------ #
    # Guards                                                               #
    # ------------------------------------------------------------------ #

    def _require_conn(self) -> UnitreeWebRTCConnection:
        if self._conn is None:
            raise AdapterConnectionError(f"adapter {self.name!r} is not connected")
        return self._conn

    def _require_connected(self) -> None:
        self._require_conn()


# --------------------------------------------------------------------------- #
# Module-level utilities                                                        #
# --------------------------------------------------------------------------- #


def _quat_to_yaw(q: dict[str, float] | list[float]) -> float:
    """Extract yaw (rotation about Z) from a unit quaternion.

    Accepts either ``{"w": w, "x": x, "y": y, "z": z}`` or ``[w, x, y, z]``.
    """
    if isinstance(q, dict):
        w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    else:
        w, x, y, z = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _parse_vec3(v: Any) -> tuple[float, float, float]:
    """Parse a 3-vector from a dict or list."""
    if isinstance(v, dict):
        return float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0))
    try:
        return float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0
    except (IndexError, TypeError):
        return 0.0, 0.0, 0.0


def _msg_len(msg: Any) -> int:
    """Best-effort size of a message, for debug logging."""
    try:
        return len(msg)
    except TypeError:
        return -1


def _parse_lidar_msg(msg: Any) -> LiDARScan:
    """Parse a ULIDAR_ARRAY message into a :class:`~gemm.types.LiDARScan`.

    The message may arrive as:

    * Raw ``bytes`` — packed little-endian float32 XYZ triplets.
    * A ``dict`` with a ``"data"`` key containing bytes or a ``"points"`` key
      containing a list of ``[x, y, z]`` values.
    * A list of dicts ``[{"x": ..., "y": ..., "z": ...}, ...]``.

    Unknown formats produce an empty scan rather than raising.
    """
    ts = time.time()

    # --- Case 1: raw bytes ---
    if isinstance(msg, (bytes, bytearray)):
        return _bytes_to_scan(bytes(msg), ts)

    # --- Case 2: dict wrapper ---
    if isinstance(msg, dict):
        payload = msg.get("data") or msg.get("points")
        if isinstance(payload, (bytes, bytearray)):
            return _bytes_to_scan(bytes(payload), ts)
        if isinstance(payload, list):
            return _list_to_scan(payload, ts)
        # Fallback: try to parse the dict values as a point list
        return LiDARScan(points=[], origin=(0.0, 0.0, 0.0), resolution=0.05, timestamp=ts)

    # --- Case 3: list of points ---
    if isinstance(msg, list):
        return _list_to_scan(msg, ts)

    return LiDARScan(points=[], origin=(0.0, 0.0, 0.0), resolution=0.05, timestamp=ts)


def _bytes_to_scan(raw: bytes, ts: float) -> LiDARScan:
    """Convert a flat array of little-endian float32 XYZ triplets to a LiDARScan."""
    n = len(raw) // 12  # 3 * 4 bytes per point
    if n == 0:
        return LiDARScan(points=[], origin=(0.0, 0.0, 0.0), resolution=0.05, timestamp=ts)
    floats = struct.unpack_from(f"<{n * 3}f", raw)
    points = [(floats[i * 3], floats[i * 3 + 1], floats[i * 3 + 2]) for i in range(n)]
    return LiDARScan(points=points, origin=(0.0, 0.0, 0.0), resolution=0.05, timestamp=ts)


def _list_to_scan(lst: list[Any], ts: float) -> LiDARScan:
    """Convert a list of point dicts / tuples to a LiDARScan."""
    points: list[tuple[float, float, float]] = []
    for p in lst:
        try:
            if isinstance(p, dict):
                points.append((float(p["x"]), float(p["y"]), float(p["z"])))
            else:
                points.append((float(p[0]), float(p[1]), float(p[2])))
        except (KeyError, IndexError, TypeError, ValueError):
            continue
    return LiDARScan(points=points, origin=(0.0, 0.0, 0.0), resolution=0.05, timestamp=ts)
