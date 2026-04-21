# Getting started

## Install

```bash
pip install gemm-unitree-go2-unofficial
```

Installing the package registers the `unitree-go2` entry point under the
`gemm.adapters` group — Gemm-based tools that enumerate entry points will
discover the adapter without extra wiring.

## Connection methods

The adapter forwards its connection arguments straight to
`unitree_webrtc_connect.UnitreeWebRTCConnection`, so every scenario the SDK
supports is available here.

| Scenario                          | `connection_method`                   | Required argument                          |
| --------------------------------- | ------------------------------------- | ------------------------------------------ |
| Robot AP hotspot (192.168.12.1)   | `WebRTCConnectionMethod.LocalAP`      | `ip="192.168.12.1"`                        |
| Same LAN, known IP                | `WebRTCConnectionMethod.LocalSTA`     | `ip="<robot-ip>"`                          |
| Same LAN, serial-number discovery | `WebRTCConnectionMethod.LocalSTA`     | `serial_number="B42D..."`                  |
| Remote via Unitree TURN           | `WebRTCConnectionMethod.Remote`       | `serial_number`, `username`, `password`    |

```python
from gemm_unitree_go2_unofficial import UnitreeAdapter
from unitree_webrtc_connect import WebRTCConnectionMethod

# Robot's own Wi-Fi AP
UnitreeAdapter(
    name="go2",
    ip="192.168.12.1",
    connection_method=WebRTCConnectionMethod.LocalAP,
)

# Robot on your LAN by serial number
UnitreeAdapter(
    name="go2",
    serial_number="B42D12345",
    connection_method=WebRTCConnectionMethod.LocalSTA,
)

# Remote, via Unitree's TURN servers
UnitreeAdapter(
    name="go2",
    serial_number="B42D12345",
    username="you@example.com",
    password="…",
    connection_method=WebRTCConnectionMethod.Remote,
)
```

The constructor requires **at least one** of `ip` or `serial_number`;
otherwise it raises `ValueError`.

## Your first session

```python
import asyncio
from gemm_unitree_go2_unofficial import UnitreeAdapter


async def main() -> None:
    adapter = UnitreeAdapter(name="go2", ip="192.168.12.1")
    await adapter.connect()

    # Give the robot a moment to publish its first state messages.
    await asyncio.sleep(0.5)

    state = await adapter.get_state()
    print(f"Battery: {state.battery * 100:.0f}%")
    print(f"Pose: x={state.pose.x:.2f} y={state.pose.y:.2f} yaw={state.pose.yaw:.2f}")

    await adapter.execute("stand_up", {})
    await asyncio.sleep(2)
    await adapter.execute("stand_down", {})

    await adapter.disconnect()


asyncio.run(main())
```

## Using with the Gemm engine

```python
from gemm import Engine
from gemm_unitree_go2_unofficial import UnitreeAdapter


async def main() -> None:
    async with Engine() as engine:
        await engine.register(UnitreeAdapter(name="go2", ip="192.168.12.1"))

        result = await engine.submit("go2", "move", {"x": 0.3, "y": 0, "z": 0})
        assert result.ok

        await engine.submit("go2", "stop_move", {})
```

## LiDAR setup

LiDAR streaming is enabled by default on `connect()`:

1. The SDK's decoder is switched from `libvoxel` (triangulated mesh) to
   `native`, which produces an `(N, 3)` float array.
2. `disableTrafficSaving(True)` is called so the robot sends full data.
3. The adapter subscribes to `ULIDAR_ARRAY` and publishes `ULIDAR_SWITCH "ON"`.

The first scan typically arrives 1–3 s after connect. Set `lidar=False` to
skip the entire setup if you don't need point clouds:

```python
UnitreeAdapter(name="go2", ip="192.168.12.1", lidar=False)
```
