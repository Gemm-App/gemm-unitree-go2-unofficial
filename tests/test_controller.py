"""Unit tests for the position controller — no hardware needed."""

from __future__ import annotations

import math

import pytest

from gemm_unitree_go2_unofficial._controller import TOLERANCE, PositionController


class TestPositionController:
    """Tests the proportional controller in isolation."""

    def _make_robot(
        self,
        start_x: float = 0.0,
        start_y: float = 0.0,
        yaw: float = 0.0,
        speed: float = 0.5,
    ) -> tuple[PositionController, list[tuple[float, float, float]]]:
        """Returns a controller and a log of move commands sent."""
        pos = [start_x, start_y, yaw]
        moves: list[tuple[float, float, float]] = []

        async def get_pose() -> tuple[float, float, float]:
            return pos[0], pos[1], pos[2]

        async def send_move(vx: float, vy: float, vz: float) -> None:
            moves.append((vx, vy, vz))
            # Simulate robot moving in body frame, convert to world frame
            cos_y = math.cos(pos[2])
            sin_y = math.sin(pos[2])
            wx = vx * cos_y - vy * sin_y
            wy = vx * sin_y + vy * cos_y
            pos[0] += wx * 0.1 * speed
            pos[1] += wy * 0.1 * speed

        return PositionController(get_pose=get_pose, send_move=send_move), moves

    @pytest.mark.asyncio
    async def test_reaches_nearby_goal(self) -> None:
        controller, moves = self._make_robot()
        x, y, _ = await controller.move_to(1.0, 0.0)
        assert math.hypot(1.0 - x, 0.0 - y) < TOLERANCE
        assert len(moves) > 0

    @pytest.mark.asyncio
    async def test_sends_stop_on_arrival(self) -> None:
        controller, moves = self._make_robot()
        await controller.move_to(0.5, 0.0)
        # Last command must be a stop (all zeros)
        assert moves[-1] == (0.0, 0.0, 0.0)

    @pytest.mark.asyncio
    async def test_already_at_goal_sends_one_stop(self) -> None:
        controller, moves = self._make_robot(start_x=0.0, start_y=0.0)
        await controller.move_to(0.0, 0.0)
        assert moves == [(0.0, 0.0, 0.0)]

    @pytest.mark.asyncio
    async def test_timeout_raises_and_stops(self) -> None:
        # Immobile robot — will never reach goal
        async def get_pose() -> tuple[float, float, float]:
            return 0.0, 0.0, 0.0

        stops: list[bool] = []

        async def send_move(vx: float, vy: float, vz: float) -> None:
            if vx == 0.0 and vy == 0.0 and vz == 0.0:
                stops.append(True)

        controller = PositionController(get_pose=get_pose, send_move=send_move)
        with pytest.raises(TimeoutError):
            await controller.move_to(10.0, 0.0, timeout=0.05)

        assert stops, "stop command must be sent after timeout"

    @pytest.mark.asyncio
    async def test_diagonal_goal(self) -> None:
        controller, _ = self._make_robot()
        x, y, _ = await controller.move_to(1.0, 1.0)
        assert math.hypot(1.0 - x, 1.0 - y) < TOLERANCE

    @pytest.mark.asyncio
    async def test_negative_coordinates(self) -> None:
        controller, _ = self._make_robot(start_x=2.0, start_y=2.0)
        x, y, _ = await controller.move_to(0.0, 0.0)
        assert math.hypot(x, y) < TOLERANCE
