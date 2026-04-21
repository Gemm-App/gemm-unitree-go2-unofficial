"""Tests for ``execute()`` — the SPORT_CMD action passthrough."""

from __future__ import annotations

from typing import Any

import pytest
from gemm.errors import AdapterConnectionError
from gemm.types import TaskResult
from unitree_webrtc_connect import RTC_TOPIC, SPORT_CMD

from gemm_unitree_go2_unofficial import SPORT_ACTIONS, UnitreeAdapter

from .conftest import FakeConnection


@pytest.mark.asyncio
async def test_execute_before_connect_raises(adapter: UnitreeAdapter) -> None:
    with pytest.raises(AdapterConnectionError):
        await adapter.execute("stand_up", {})


@pytest.mark.asyncio
async def test_unknown_action_returns_failure(
    connected_adapter: UnitreeAdapter,
) -> None:
    result = await connected_adapter.execute("fly_to_mars", {})
    assert isinstance(result, TaskResult)
    assert not result.ok
    assert "unsupported" in (result.error or "").lower()


@pytest.mark.asyncio
async def test_every_sport_action_publishes_to_sport_mod(
    connected_adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    """Every entry in SPORT_ACTIONS must translate to a SPORT_MOD request
    carrying the matching SPORT_CMD api_id."""
    for action, cmd_name in SPORT_ACTIONS.items():
        fake_conn.datachannel.pub_sub.sent_requests.clear()
        result = await connected_adapter.execute(action, {})
        assert result.ok, f"{action} did not return success"

        sent = fake_conn.datachannel.pub_sub.sent_requests
        assert len(sent) == 1, f"{action} sent {len(sent)} requests"
        topic, payload = sent[0]
        assert topic == RTC_TOPIC["SPORT_MOD"], action
        assert payload["api_id"] == SPORT_CMD[cmd_name], action


@pytest.mark.asyncio
async def test_move_forwards_parameters(
    connected_adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    fake_conn.datachannel.pub_sub.sent_requests.clear()
    result = await connected_adapter.execute("move", {"x": 0.3, "y": 0.0, "z": 0.1})
    assert result.ok

    topic, payload = fake_conn.datachannel.pub_sub.sent_requests[-1]
    assert topic == RTC_TOPIC["SPORT_MOD"]
    assert payload["api_id"] == SPORT_CMD["Move"]
    assert payload["parameter"] == {"x": 0.3, "y": 0.0, "z": 0.1}


@pytest.mark.asyncio
async def test_empty_params_send_empty_parameter_dict(
    connected_adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    fake_conn.datachannel.pub_sub.sent_requests.clear()
    await connected_adapter.execute("stand_up", {})
    _, payload = fake_conn.datachannel.pub_sub.sent_requests[-1]
    assert payload["parameter"] == {}


@pytest.mark.asyncio
async def test_getter_response_data_flows_into_task_result(
    connected_adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    fake_conn.datachannel.pub_sub.next_response = {
        "info": {"execution": "ok"},
        "data": {"height": 0.32},
    }
    result = await connected_adapter.execute("get_body_height", {})
    assert result.ok
    assert result.data == {"height": 0.32}


@pytest.mark.asyncio
async def test_sdk_exception_returns_failure(
    connected_adapter: UnitreeAdapter, fake_conn: FakeConnection
) -> None:
    async def boom(*_: Any, **__: Any) -> Any:
        raise RuntimeError("boom")

    fake_conn.datachannel.pub_sub.publish_request_new = boom  # type: ignore[method-assign]
    result = await connected_adapter.execute("stand_up", {})
    assert not result.ok
    assert "boom" in (result.error or "")
