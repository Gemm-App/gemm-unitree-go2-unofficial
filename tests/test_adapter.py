"""Unit tests for UnitreeAdapter (no hardware required)."""

from __future__ import annotations

import pytest
from gemm.errors import AdapterConnectionError
from gemm.types import RobotState, TaskResult

from gemm_unitree_go2_unofficial import UnitreeAdapter


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
        """Unreachable goal with very short timeout → failure result, not exception."""
        await adapter.connect()
        try:
            # Immovable mock + tiny timeout
            result = await adapter.execute("move_to", {"x": 999.0, "y": 999.0, "timeout": 0.05})
            assert not result.ok
            assert "timed out" in (result.error or "").lower()
        finally:
            await adapter.disconnect()
