# Sensors

The adapter implements Gemm's sensor interface: one-shot reads with
`get_sensor()` and real-time callbacks with `subscribe()`.

| Sensor          | Type              | Source topic                                                                 |
| --------------- | ----------------- | ---------------------------------------------------------------------------- |
| `"battery"`     | `BatteryState`    | `LOW_STATE` / `bms_state` — `soc` normalised to 0.0–1.0                      |
| `"imu"`         | `IMUData`         | `LF_SPORT_MOD_STATE` / `imu_state`                                           |
| `"odometry"`    | `RobotOdometry`   | `ROBOTODOM` (LiDAR-SLAM pose)                                                |
| `"lidar"`       | `LiDARScan`       | `ULIDAR_ARRAY` (decoded via the native decoder)                              |
| `"motor.{i}"`   | `MotorState`      | `LOW_STATE` / `motor_state[i]` — `i = 0..11`                                 |

## One-shot reads

```python
import asyncio
from gemm_unitree_go2_unofficial import UnitreeAdapter


async def main() -> None:
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

### Battery normalisation

The Go2 firmware reports `soc` as a 0–100 percentage in the BMS payload.
The adapter normalises it to the 0.0–1.0 range Gemm expects, so
`BatteryState.soc` is always a fraction.

## Subscriptions

```python
async def main() -> None:
    adapter = UnitreeAdapter(name="go2", ip="192.168.12.1")
    await adapter.connect()

    def on_battery(reading):
        print(f"{reading.soc * 100:.0f}%")

    unsubscribe = adapter.subscribe("battery", on_battery)

    await asyncio.sleep(5)
    unsubscribe()
    await adapter.disconnect()
```

!!! warning "Callbacks run on the WebRTC thread"
    Subscribers fire on the SDK's event thread — **do not block** and do
    not `await` inside them. If you need async behaviour, hand work off to
    an executor or schedule onto the main loop with
    `asyncio.run_coroutine_threadsafe`.

A subscriber that raises is isolated from the others — the adapter logs
the exception and keeps fanning the reading out to the remaining
listeners.

## Error handling

```python
from gemm.errors import AdapterConnectionError, SensorError, SensorNotAvailable

try:
    reading = await adapter.get_sensor("lidar")
except AdapterConnectionError:
    print("Call connect() first.")
except SensorError:
    print("No scan yet — try again shortly.")
except SensorNotAvailable:
    print("Sensor name not recognised.")
```

| Error                        | Raised when                                                               |
| ---------------------------- | ------------------------------------------------------------------------- |
| `AdapterConnectionError`     | `get_sensor` / `subscribe` called before `connect()`                      |
| `SensorNotAvailable`         | Unknown sensor name or out-of-range motor index (e.g. `"motor.99"`)       |
| `SensorError`                | Sensor is known but no data has arrived yet — wait a moment after connect |

## LiDAR

Enabled by default; pass `lidar=False` to skip setup.

On `connect()` with LiDAR enabled, the adapter:

1. Switches the SDK's decoder to `native` — returns an `(N, 3)` float
   array of points.
2. Calls `disableTrafficSaving(True)` so the robot emits full data.
3. Subscribes to `ULIDAR_ARRAY` and publishes `ULIDAR_SWITCH "ON"`.

The first frame typically lands 1–3 s after connect. If you `get_sensor("lidar")`
before that, you'll get a `SensorError`.

### Point cloud layout

`LiDARScan.points` is a list of `(x, y, z)` tuples in the LiDAR-SLAM frame
(same frame used by `odometry`). `origin` and `resolution` are passed
through from the SDK payload; `timestamp` is filled in by the adapter at
decode time.

## Coordinate frame

`odometry` and the pose embedded in `get_state()` are in the **LiDAR-SLAM
odometry frame**, whose origin is wherever the robot initialised its SLAM
session (usually power-on). The frame does **not** persist across reboots
— consumers should treat it as session-relative.
