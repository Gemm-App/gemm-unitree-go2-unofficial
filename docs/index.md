# gemm-unitree-go2-unofficial

Unofficial [Gemm](https://github.com/Gemm-App/Gemm-Engine) adapter for
**Unitree Go2** robots, backed by
[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect).

No ROS. No firmware modifications. Pure Python over WebRTC.

!!! warning "Unofficial"
    Not affiliated with Unitree Robotics. Use at your own risk — acrobatic
    actions (flips, jumps, handstands) can damage the robot or injure people
    if triggered without enough clear space.

## What you get

- **49 sport actions** — every `SPORT_CMD` from the SDK, mapped 1:1 to a
  snake_case action name. Full table: [Actions](actions.md).
- **Five sensor families** — `battery`, `imu`, `odometry`, `lidar`, and
  `motor.0`…`motor.11`. See [Sensors](sensors.md).
- **Entry-point discovery** — registers `unitree-go2` under the
  `gemm.adapters` group, so any Gemm-based tool enumerating entry points
  picks the adapter up automatically.
- **Typed + tested** — ships a `py.typed` marker and a 50-test suite that
  runs fully offline against a fake WebRTC connection.

## Install

```bash
pip install gemm-unitree-go2-unofficial
```

## 60-second example

```python
import asyncio
from gemm import Engine
from gemm_unitree_go2_unofficial import UnitreeAdapter
from unitree_webrtc_connect import WebRTCConnectionMethod


async def main() -> None:
    async with Engine() as engine:
        await engine.register(
            UnitreeAdapter(
                name="go2",
                ip="192.168.12.1",
                connection_method=WebRTCConnectionMethod.LocalAP,
            )
        )

        await engine.submit("go2", "stand_up", {})
        await engine.submit("go2", "move", {"x": 0.3, "y": 0.0, "z": 0.0})
        await engine.submit("go2", "stop_move", {})


asyncio.run(main())
```

## Where next

<div class="grid cards" markdown>

-   :material-rocket-launch: **[Getting started](getting-started.md)**

    Connection methods, first connect, LiDAR setup.

-   :material-run-fast: **[Actions](actions.md)**

    Every `SPORT_CMD` the Go2 understands, grouped by purpose.

-   :material-radar: **[Sensors](sensors.md)**

    One-shot reads, subscriptions, error handling.

-   :material-book-open-page-variant: **[API reference](api.md)**

    Auto-generated from docstrings.

</div>
