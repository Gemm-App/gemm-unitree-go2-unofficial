"""Gemm adapter for Unitree Go2 robots via unitree_webrtc_connect.

Supports:
    move_to          — navigate to (x, y) in the current SLAM odometry frame
    move_velocity    — direct body-frame velocity (vx, vy, vz)
    stand_up         — StandUp sport command
    stand_down       — StandDown sport command
    balance_stand    — BalanceStand sport command
    stop             — StopMove sport command

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
The frame does not persist across power cycles — if the robot is moved
before powering on, the origin shifts accordingly.

Message format note
-------------------
The exact field names in ROBOTODOM / SPORT_MOD_STATE messages depend on
robot firmware version. Run with DEBUG logging to inspect live payloads and
adjust the _parse_* methods below if needed::

    import logging
    logging.basicConfig(level=logging.DEBUG)
"""

from __future__ import annotations

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Any

from gemm.errors import AdapterConnectionError
from gemm.types import Pose, RobotState, TaskResult
from unitree_webrtc_connect import (
    RTC_TOPIC,
    SPORT_CMD,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

from gemm_unitree_go2_unofficial._controller import DEFAULT_TIMEOUT, PositionController

logger = logging.getLogger(__name__)


@dataclass
class _RobotState:
    """Mutable snapshot updated by subscriber callbacks."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    battery: float = 1.0
    # Lock so move_to reads a consistent (x, y, yaw) triple
    _lock: asyncio.Lock = field(default_factory=asyncio.Lock, repr=False)


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
        # Subscribe to sport state — fallback pose + IMU
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport_state)
        # Subscribe to low state — battery
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], self._on_low_state)

        self._conn = conn
        logger.info("UnitreeAdapter %r connected via %s", self.name, self._method)

    async def disconnect(self) -> None:
        if self._conn is not None:
            # unitree_webrtc_connect does not expose an explicit close();
            # dropping the reference lets the WebRTC stack clean up.
            self._conn = None
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
            x = float(pos["x"] if isinstance(pos, dict) else pos[0])
            y = float(pos["y"] if isinstance(pos, dict) else pos[1])

            # Yaw — may be direct field or derived from quaternion
            if "pose" in data and "yaw" in data["pose"]:
                yaw = float(data["pose"]["yaw"])
            elif "imu_state" in data:
                yaw = float(data["imu_state"]["rpy"][2])
            elif "orientation" in data:
                yaw = _quat_to_yaw(data["orientation"])
            else:
                yaw = self._state.yaw  # keep last known

            self._state.x = x
            self._state.y = y
            self._state.yaw = yaw
        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("unexpected ROBOTODOM format — raw msg: %s", msg)

    def _on_sport_state(self, msg: dict[str, Any]) -> None:
        """Parse LF_SPORT_MOD_STATE — fallback pose source + IMU yaw."""
        logger.debug("SPORT_STATE: %s", msg)
        try:
            data = msg.get("data", msg)
            pos = data["position"]  # [x, y, z] array
            x = float(pos[0] if isinstance(pos, list) else pos["x"])
            y = float(pos[1] if isinstance(pos, list) else pos["y"])
            yaw = float(data["imu_state"]["rpy"][2])

            # Only use if ROBOTODOM has not provided a value yet
            if self._state.x == 0.0 and self._state.y == 0.0:
                self._state.x = x
                self._state.y = y
            self._state.yaw = yaw
        except (KeyError, IndexError, TypeError, ValueError):
            logger.debug("unexpected SPORT_STATE format — raw msg: %s", msg)

    def _on_low_state(self, msg: dict[str, Any]) -> None:
        """Parse LOW_STATE — battery level."""
        logger.debug("LOW_STATE: %s", msg)
        try:
            data = msg.get("data", msg)
            pct = data["battery_state"]["percent"]
            self._state.battery = float(pct) / 100.0
        except (KeyError, TypeError, ValueError):
            logger.debug("unexpected LOW_STATE format — raw msg: %s", msg)

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
# Utilities                                                                     #
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
