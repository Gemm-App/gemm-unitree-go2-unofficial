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
| `move_to` | `x`, `y`, `timeout` (opt, default 60 s) | Navigate to (x, y) in the SLAM frame using a proportional position controller |
| `move_velocity` | `x`, `y`, `z` | Body-frame velocity in m/s (continuous, no auto-stop) |
| `stand_up` | — | StandUp sport command |
| `stand_down` | — | StandDown sport command |
| `balance_stand` | — | BalanceStand sport command |
| `stop` | — | StopMove sport command |

## Sensors

The adapter implements the full Gemm sensor interface: one-shot reads via
`get_sensor()` and real-time callbacks via `subscribe()`.

### Supported sensors

| Sensor name | Return type | Source topic | Notes |
|---|---|---|---|
| `"battery"` | `BatteryState` | `LOW_STATE` / `bms_state` | `soc` normalised to 0.0–1.0 |
| `"imu"` | `IMUData` | `LF_SPORT_MOD_STATE` | quaternion, gyro, accel, rpy |
| `"odometry"` | `RobotOdometry` | `ROBOTODOM` | SLAM frame, origin at power-on |
| `"lidar"` | `LiDARScan` | `ULIDAR_ARRAY` | requires LiDAR init (auto on connect) |
| `"motor.{i}"` | `MotorState` | `LOW_STATE` | i = 0–11 (Go2 has 12 joints) |

### One-shot read

```python
import asyncio
from gemm_unitree_go2_unofficial import UnitreeAdapter
from unitree_webrtc_connect import WebRTCConnectionMethod

async def main():
    adapter = UnitreeAdapter(
        name="go2",
        ip="192.168.12.1",
        connection_method=WebRTCConnectionMethod.LocalAP,
    )
    await adapter.connect()

    # Wait a moment for the first messages to arrive
    await asyncio.sleep(0.5)

    battery = await adapter.get_sensor("battery")
    print(f"Battery: {battery.soc * 100:.0f}%  {battery.voltage:.1f} V")

    imu = await adapter.get_sensor("imu")
    print(f"Roll/Pitch/Yaw: {imu.rpy}")

    odom = await adapter.get_sensor("odometry")
    print(f"Position: {odom.position}")

    lidar = await adapter.get_sensor("lidar")
    print(f"LiDAR points: {len(lidar)}")

    motor0 = await adapter.get_sensor("motor.0")
    print(f"Joint 0 angle: {motor0.position:.3f} rad")

    await adapter.disconnect()

asyncio.run(main())
```

### Real-time subscriptions

```python
async def main():
    adapter = UnitreeAdapter(name="go2", ip="192.168.12.1")
    await adapter.connect()

    # Battery updates fire every time a LOW_STATE message arrives
    def on_battery(b):
        print(f"SOC: {b.soc * 100:.0f}%")

    unsubscribe = adapter.subscribe("battery", on_battery)

    await asyncio.sleep(5)   # listen for 5 seconds

    unsubscribe()            # stop receiving callbacks
    await adapter.disconnect()
```

### LiDAR note

LiDAR streaming is enabled automatically on `connect()`:

1. `disableTrafficSaving(True)` — tells the robot to send full point-cloud data
2. Subscribe to `ULIDAR_ARRAY`
3. Publish `ULIDAR_SWITCH "on"` — activates the LiDAR stream

The first scan typically arrives 1–3 seconds after connecting. Until it does,
`get_sensor("lidar")` raises `SensorError`. Use `subscribe("lidar", cb)` to
react as soon as the first scan lands.

### Error handling

```python
from gemm.errors import SensorError, SensorNotAvailable

try:
    reading = await adapter.get_sensor("lidar")
except SensorError:
    print("No scan yet — try again in a moment")
except SensorNotAvailable:
    print("Sensor name not recognised")
```

## SLAM frame note

`move_to` and `odometry` coordinates are relative to the SLAM origin — the
position where the robot initialised its LiDAR SLAM session (usually at
power-on). The frame does **not** persist across reboots. Always start the
robot from the same physical spot, or re-map after each power cycle.

## Firmware compatibility

Tested against Go2 EDU firmware. Run with `DEBUG` logging to inspect live
message payloads and diagnose field-name differences across firmware versions:

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
pytest          # 47 tests, no hardware needed
```
