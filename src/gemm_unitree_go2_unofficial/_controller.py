"""Proportional position controller for move_to navigation.

Converts a (goal_x, goal_y) in the SLAM odometry frame into a sequence of
body-frame velocity commands sent to the robot's sport mode.

Theory of operation
-------------------
The robot exposes:
- Current pose  (x, y, yaw)  from ROBOTODOM / SPORT_MOD_STATE
- Velocity sink (vx, vy, vz) via SPORT_CMD["Move"]  — body frame

The controller runs a simple proportional loop:

    error      = goal - current_position          (world frame)
    speed      = clamp(Kp * |error|, 0, MAX_VEL)
    vel_world  = speed * (error / |error|)
    vel_body   = rotate(vel_world, -yaw)           (into robot frame)
    send Move(vel_body)
    sleep DT
    repeat until |error| < TOLERANCE or timeout
"""

from __future__ import annotations

import asyncio
import logging
import math
from collections.abc import Awaitable, Callable

logger = logging.getLogger(__name__)

# Tunable constants — adjust for your floor surface and robot speed setting
TOLERANCE: float = 0.15  # metres — goal reached when closer than this
MAX_VEL: float = 0.5  # m/s   — cap on forward/lateral speed
KP: float = 0.8  # proportional gain
DT: float = 0.1  # control loop interval (seconds)
DEFAULT_TIMEOUT: float = 60.0  # seconds before giving up


class PositionController:
    """Runs the move_to control loop.

    Args:
        get_pose:   async callable that returns (x, y, yaw) of current pose.
        send_move:  async callable that sends body-frame (vx, vy, vz=0).
    """

    def __init__(
        self,
        get_pose: Callable[[], Awaitable[tuple[float, float, float]]],
        send_move: Callable[[float, float, float], Awaitable[None]],
    ) -> None:
        self._get_pose = get_pose
        self._send_move = send_move

    async def move_to(
        self,
        goal_x: float,
        goal_y: float,
        *,
        timeout: float = DEFAULT_TIMEOUT,
        tolerance: float = TOLERANCE,
        max_vel: float = MAX_VEL,
        kp: float = KP,
    ) -> tuple[float, float, float]:
        """Drive the robot to (goal_x, goal_y) in the SLAM frame.

        Returns the final (x, y, yaw) pose when the goal is reached.
        Raises TimeoutError if the robot does not arrive within *timeout* seconds.
        Always sends a stop command before returning or raising.
        """
        deadline = asyncio.get_event_loop().time() + timeout

        try:
            while True:
                if asyncio.get_event_loop().time() > deadline:
                    raise TimeoutError(
                        f"move_to({goal_x:.2f}, {goal_y:.2f}) timed out after {timeout}s"
                    )

                x, y, yaw = await self._get_pose()
                dx = goal_x - x
                dy = goal_y - y
                dist = math.hypot(dx, dy)

                logger.debug(
                    "move_to goal=(%.2f,%.2f) current=(%.2f,%.2f) dist=%.3f",
                    goal_x,
                    goal_y,
                    x,
                    y,
                    dist,
                )

                if dist < tolerance:
                    return x, y, yaw

                # Proportional speed, capped
                speed = min(kp * dist, max_vel)
                # Unit vector in world frame
                ux, uy = dx / dist, dy / dist
                vx_w = speed * ux
                vy_w = speed * uy

                # Rotate world-frame velocity into robot body frame
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                vx_b = vx_w * cos_y + vy_w * sin_y
                vy_b = -vx_w * sin_y + vy_w * cos_y

                await self._send_move(vx_b, vy_b, 0.0)
                await asyncio.sleep(DT)

        finally:
            # Always stop the robot when the loop exits, regardless of outcome
            try:
                await self._send_move(0.0, 0.0, 0.0)
            except Exception:
                logger.warning("failed to send stop command after move_to", exc_info=True)
