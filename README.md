# gemm-unitree-go2-unofficial

Unofficial [Gemm](https://github.com/Gemm-App/Gemm-Engine) adapter for
**Unitree Go2** robots, powered by
[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect).

No ROS. No firmware modifications. Pure Python over WebRTC.

> **Unofficial** — not affiliated with Unitree Robotics. Use at your own risk.

## Install

```bash
pip install gemm-unitree-go2-unofficial
```

## Usage

```python
import asyncio
from gemm import Engine
from gemm_unitree_go2_unofficial import UnitreeAdapter
from unitree_webrtc_connect import WebRTCConnectionMethod

async def main():
    async with Engine() as engine:
        await engine.register(
            UnitreeAdapter(
                name="go2",
                ip="192.168.12.1",                        # robot AP hotspot
                connection_method=WebRTCConnectionMethod.LocalAP,
            )
        )

        # Navigate to a point in the current SLAM session's coordinate frame
        task = await engine.submit("go2", "move_to", {"x": 2.0, "y": 0.0})
        result = await task.wait()
        print(result.data)   # {"pose": {"x": ..., "y": ..., "yaw": ...}}

        # Direct velocity control
        await engine.submit("go2", "move_velocity", {"x": 0.3, "y": 0.0, "z": 0.0})

        # Stance commands
        await engine.submit("go2", "stand_up", {})
        await engine.submit("go2", "stand_down", {})
        await engine.submit("go2", "stop", {})

asyncio.run(main())
```

## Connection methods

| Scenario | `connection_method` | Required arg |
|---|---|---|
| Robot AP hotspot (192.168.12.1) | `WebRTCConnectionMethod.LocalAP` | `ip="192.168.12.1"` |
| Same LAN, known IP | `WebRTCConnectionMethod.LocalSTA` | `ip="<robot-ip>"` |
| Same LAN, serial number | `WebRTCConnectionMethod.LocalSTA` | `serial_number="B42D..."` |
| Remote via Unitree TURN | `WebRTCConnectionMethod.Remote` | `serial_number`, Unitree account |

## Supported actions

| Action | Params | Description |
|---|---|---|
| `move_to` | `x`, `y`, `timeout` (opt, default 60s) | Navigate to (x, y) in SLAM frame |
| `move_velocity` | `x`, `y`, `z` | Body-frame velocity (m/s) |
| `stand_up` | — | StandUp sport command |
| `stand_down` | — | StandDown sport command |
| `balance_stand` | — | BalanceStand sport command |
| `stop` | — | StopMove sport command |

## SLAM frame note

`move_to` coordinates are relative to the SLAM origin — the position where
the robot initialised its LiDAR SLAM session (usually at power-on). The frame
does **not** persist across reboots. Always start the robot from the same
physical spot, or re-map after each power cycle.

## Firmware compatibility

Tested against Go2 EDU firmware. — run with `DEBUG` logging to inspect live message payloads:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## Development

```bash
git clone https://github.com/Gemm-App/gemm-unitree-go2-unofficial
cd gemm-unitree-go2-unofficial
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest          # 8 contract tests + unit tests, no hardware needed
```
