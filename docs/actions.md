# Actions

Every action is a **1:1 passthrough** to the SDK's `SPORT_CMD` table — the
adapter publishes a `SPORT_MOD` request with the matching `api_id` and the
caller-supplied `parameter` dict. The full mapping is exported as
`gemm_unitree_go2_unofficial.SPORT_ACTIONS`.

```python
from gemm_unitree_go2_unofficial import SPORT_ACTIONS

print(SPORT_ACTIONS["move"])  # -> "Move"
```

## How `execute()` dispatches

```python
result = await adapter.execute("move", {"x": 0.3, "y": 0.0, "z": 0.0})
```

Under the hood this becomes roughly:

```python
conn.datachannel.pub_sub.publish_request_new(
    RTC_TOPIC["SPORT_MOD"],
    {"api_id": SPORT_CMD["Move"], "parameter": {"x": 0.3, "y": 0.0, "z": 0.0}},
)
```

An unknown action name returns a **failed `TaskResult`** — it does not raise
— which is the behaviour required by the Gemm adapter contract:

```python
result = await adapter.execute("do_a_barrel_roll", {})
assert not result.ok
assert "unsupported" in result.error
```

## Locomotion & stance

| Action             | Maps to `SPORT_CMD`                          | Typical params                                    |
| ------------------ | -------------------------------------------- | ------------------------------------------------- |
| `stand_up`         | `StandUp`                                    | —                                                 |
| `stand_down`       | `StandDown`                                  | —                                                 |
| `balance_stand`    | `BalanceStand`                               | —                                                 |
| `recovery_stand`   | `RecoveryStand`                              | —                                                 |
| `damp`             | `Damp`                                       | —                                                 |
| `stop_move`        | `StopMove`                                   | —                                                 |
| `move`             | `Move`                                       | `{"x": vx, "y": vy, "z": vz}` (m/s, body frame)   |
| `sit`              | `Sit`                                        | —                                                 |
| `rise_sit`         | `RiseSit`                                    | —                                                 |
| `stand_out`        | `StandOut`                                   | —                                                 |
| `standup_extra`    | `Standup` *(SPORT_CMD 1050, not `StandUp`)*  | —                                                 |

## Gaits & tuning

| Action               | Maps to             | Typical params                           |
| -------------------- | ------------------- | ---------------------------------------- |
| `switch_gait`        | `SwitchGait`        | `{"data": <gait_id>}`                    |
| `continuous_gait`    | `ContinuousGait`    | `{"data": true/false}`                   |
| `economic_gait`      | `EconomicGait`      | `{"data": true/false}`                   |
| `speed_level`        | `SpeedLevel`        | `{"data": -1..1}`                        |
| `body_height`        | `BodyHeight`        | `{"data": <metres>}`                     |
| `foot_raise_height`  | `FootRaiseHeight`   | `{"data": <metres>}`                     |
| `euler`              | `Euler`             | `{"x": roll, "y": pitch, "z": yaw}` rad  |
| `switch_joystick`    | `SwitchJoystick`    | `{"data": true/false}`                   |
| `trajectory_follow`  | `TrajectoryFollow`  | see SDK                                  |

## Getters

Getter actions forward the SDK's response payload through to the
`TaskResult.data` dict, so callers can read the returned value.

| Action                   | Maps to               |
| ------------------------ | --------------------- |
| `get_body_height`        | `GetBodyHeight`       |
| `get_foot_raise_height`  | `GetFootRaiseHeight`  |
| `get_speed_level`        | `GetSpeedLevel`       |
| `get_sport_state`        | `GetState`            |

```python
result = await adapter.execute("get_body_height", {})
if result.ok:
    print(result.data)   # e.g. {"value": 0.28}
```

## Tricks & stunts

!!! danger "Acrobatics can damage the robot"
    Commands like `front_flip`, `back_flip`, `handstand`, `front_jump`, and
    `front_pounce` can break joints or hurt people if there isn't enough
    clear space. Only run them with proper safety conditions.

| Action          | Maps to       | Action           | Maps to         |
| --------------- | ------------- | ---------------- | --------------- |
| `hello`         | `Hello`       | `front_flip`     | `FrontFlip`     |
| `stretch`       | `Stretch`     | `back_flip`      | `BackFlip`      |
| `pose`          | `Pose`        | `left_flip`      | `LeftFlip`      |
| `scrape`        | `Scrape`      | `right_flip`     | `RightFlip`     |
| `content`       | `Content`     | `front_jump`     | `FrontJump`     |
| `wallow`        | `Wallow`      | `front_pounce`   | `FrontPounce`   |
| `dance1`        | `Dance1`      | `handstand`      | `Handstand`     |
| `dance2`        | `Dance2`      | `bound`          | `Bound`         |
| `wiggle_hips`   | `WiggleHips`  | `moon_walk`      | `MoonWalk`      |
| `finger_heart`  | `FingerHeart` | `onesided_step`  | `OnesidedStep`  |
| `lead_follow`   | `LeadFollow`  | `cross_step`     | `CrossStep`     |
| `free_walk`     | `FreeWalk`    | `cross_walk`     | `CrossWalk`     |
| `trigger`       | `Trigger`     |                  |                 |
