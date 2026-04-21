# gemm-unitree-go2-unofficial

Unofficial [Gemm](https://github.com/Gemm-App/Gemm-Engine) adapter for
**Unitree Go2** robots, backed by
[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect).

No ROS. No firmware modifications. Pure Python over WebRTC.

> **Unofficial** — not affiliated with Unitree Robotics. Use at your own risk.

**Full docs:** <https://gemm-app.github.io/gemm-unitree-go2-unofficial/>

## Install

```bash
pip install gemm-unitree-go2-unofficial
```

Installing the package automatically registers the `unitree-go2` entry point
under the `gemm.adapters` group, so any Gemm-based tool that enumerates
entry points will discover the adapter without extra wiring.

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
                ip="192.168.12.1",
                connection_method=WebRTCConnectionMethod.LocalAP,
            )
        )

        await engine.submit("go2", "stand_up", {})
        await engine.submit("go2", "move", {"x": 0.3, "y": 0.0, "z": 0.0})
        await engine.submit("go2", "stop_move", {})

asyncio.run(main())
```

## Connection methods

| Scenario | `connection_method` | Required arg |
|---|---|---|
| Robot AP hotspot (192.168.12.1) | `WebRTCConnectionMethod.LocalAP` | `ip="192.168.12.1"` |
| Same LAN, known IP | `WebRTCConnectionMethod.LocalSTA` | `ip="<robot-ip>"` |
| Same LAN, serial number | `WebRTCConnectionMethod.LocalSTA` | `serial_number="B42D..."` |
| Remote via Unitree TURN | `WebRTCConnectionMethod.Remote` | `serial_number`, `username`, `password` |

## Actions

Every action is a **1:1 passthrough** to the SDK's `SPORT_CMD` table — the
adapter publishes a `SPORT_MOD` request with the matching `api_id` and the
caller-supplied `parameter` dict. The full mapping is exported as
`gemm_unitree_go2_unofficial.SPORT_ACTIONS`.

### Locomotion / stance

| Action | Maps to `SPORT_CMD` | Typical params |
|---|---|---|
| `stand_up` | `StandUp` | — |
| `stand_down` | `StandDown` | — |
| `balance_stand` | `BalanceStand` | — |
| `recovery_stand` | `RecoveryStand` | — |
| `damp` | `Damp` | — |
| `stop_move` | `StopMove` | — |
| `move` | `Move` | `{"x": vx, "y": vy, "z": vz}` (m/s, body frame) |
| `sit` | `Sit` | — |
| `rise_sit` | `RiseSit` | — |
| `stand_out` | `StandOut` | — |
| `standup_extra` | `Standup` *(SPORT_CMD 1050 — different from `StandUp`)* | — |

### Gaits & tuning

| Action | Maps to | Typical params |
|---|---|---|
| `switch_gait` | `SwitchGait` | `{"data": <gait_id>}` |
| `continuous_gait` | `ContinuousGait` | `{"data": true/false}` |
| `economic_gait` | `EconomicGait` | `{"data": true/false}` |
| `speed_level` | `SpeedLevel` | `{"data": -1..1}` |
| `body_height` | `BodyHeight` | `{"data": <metres>}` |
| `foot_raise_height` | `FootRaiseHeight` | `{"data": <metres>}` |
| `euler` | `Euler` | `{"x": roll, "y": pitch, "z": yaw}` (radians) |
| `switch_joystick` | `SwitchJoystick` | `{"data": true/false}` |
| `trajectory_follow` | `TrajectoryFollow` | see SDK |

### Getters

Getter actions forward the SDK's response payload through to the
`TaskResult.data` dict, so callers can read the returned value.

| Action | Maps to |
|---|---|
| `get_body_height` | `GetBodyHeight` |
| `get_foot_raise_height` | `GetFootRaiseHeight` |
| `get_speed_level` | `GetSpeedLevel` |
| `get_sport_state` | `GetState` |

### Tricks & stunts

| Action | Maps to | Action | Maps to |
|---|---|---|---|
| `hello` | `Hello` | `front_flip` | `FrontFlip` |
| `stretch` | `Stretch` | `back_flip` | `BackFlip` |
| `pose` | `Pose` | `left_flip` | `LeftFlip` |
| `scrape` | `Scrape` | `right_flip` | `RightFlip` |
| `content` | `Content` | `front_jump` | `FrontJump` |
| `wallow` | `Wallow` | `front_pounce` | `FrontPounce` |
| `dance1` | `Dance1` | `handstand` | `Handstand` |
| `dance2` | `Dance2` | `bound` | `Bound` |
| `wiggle_hips` | `WiggleHips` | `moon_walk` | `MoonWalk` |
| `finger_heart` | `FingerHeart` | `onesided_step` | `OnesidedStep` |
| `lead_follow` | `LeadFollow` | `cross_step` | `CrossStep` |
| `free_walk` | `FreeWalk` | `cross_walk` | `CrossWalk` |
| `trigger` | `Trigger` | | |

> **Danger:** Acrobatic commands (`front_flip`, `back_flip`, `handstand`,
> `front_jump`, etc.) can damage the robot if triggered without enough clear
> space. Only run them with the proper safety conditions.

### Unknown actions

An unknown action name returns a failed `TaskResult` — it does not raise —
which is the behaviour required by the Gemm adapter contract.

```python
result = await adapter.execute("do_a_barrel_roll", {})
assert not result.ok
assert "unsupported" in result.error
```

## Sensors

The adapter implements Gemm's sensor interface: one-shot reads with
`get_sensor()` and real-time callbacks with `subscribe()`.

| Sensor | Type | Source topic |
|---|---|---|
| `"battery"` | `BatteryState` | `LOW_STATE` / `bms_state` — `soc` normalised to 0.0–1.0 |
| `"imu"` | `IMUData` | `LF_SPORT_MOD_STATE` / `imu_state` |
| `"odometry"` | `RobotOdometry` | `ROBOTODOM` (LiDAR-SLAM pose) |
| `"lidar"` | `LiDARScan` | `ULIDAR_ARRAY` (decoded via the native decoder) |
| `"motor.{i}"` | `MotorState` | `LOW_STATE` / `motor_state[i]` — `i = 0..11` |

### One-shot read

```python
import asyncio
from gemm_unitree_go2_unofficial import UnitreeAdapter

async def main():
    adapter = UnitreeAdapter(name="go2", ip="192.168.12.1")
    await adapter.connect()

    # Give the robot a moment to publish its first messages.
    await asyncio.sleep(0.5)

    battery = await adapter.get_sensor("battery")
    print(f"SOC: {battery.soc * 100:.0f}%  ({battery.voltage:.1f} V)")

    imu = await adapter.get_sensor("imu")
    print(f"RPY: {imu.rpy}")

    odom = await adapter.get_sensor("odometry")
    print(f"Position: {odom.position}")

    motor0 = await adapter.get_sensor("motor.0")
    print(f"Joint 0: {motor0.position:.3f} rad")

    await adapter.disconnect()

asyncio.run(main())
```

### Subscriptions

```python
async def main():
    adapter = UnitreeAdapter(name="go2", ip="192.168.12.1")
    await adapter.connect()

    def on_battery(reading):
        print(f"{reading.soc * 100:.0f}%")

    unsubscribe = adapter.subscribe("battery", on_battery)

    await asyncio.sleep(5)
    unsubscribe()
    await adapter.disconnect()
```

Callbacks run on the WebRTC thread — **do not block** or await inside them.
Fan work out to an executor or schedule onto the main loop with
`asyncio.run_coroutine_threadsafe` if you need async behaviour.

### Error handling

```python
from gemm.errors import SensorError, SensorNotAvailable

try:
    reading = await adapter.get_sensor("lidar")
except SensorError:
    print("No scan yet — try again shortly")
except SensorNotAvailable:
    print("Sensor name not recognised")
```

### LiDAR

LiDAR streaming is enabled automatically on `connect()`:

1. The SDK's decoder is switched from `libvoxel` (mesh) to `native`, which
   produces an `(N, 3)` point array.
2. `disableTrafficSaving(True)` is called so the robot sends full data.
3. The adapter subscribes to `ULIDAR_ARRAY` and publishes `ULIDAR_SWITCH "ON"`.

The first scan typically arrives 1–3 s after `connect()`. Pass
`lidar=False` to `UnitreeAdapter(...)` to skip the LiDAR setup entirely.

## Coordinate frame

`odometry` and the pose in `get_state()` are in the **LiDAR SLAM odometry
frame**, whose origin is wherever the robot initialised its SLAM session
(usually power-on). The frame does **not** persist across reboots.

## Firmware compatibility

Tested against Go2 EDU firmware. Go2 Air/Pro may work but has not been
verified. Enable DEBUG logging to inspect live message payloads:

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
pytest          # 50 tests, no hardware needed
ruff check .
mypy src
```

## License

MIT
