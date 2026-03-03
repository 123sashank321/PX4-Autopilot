# Striker Module

The `striker` module is a PX4 custom flight mode that enables **fixed-wing guided strike maneuvers**. It receives target designation commands via MAVLink (`MAV_CMD_USER_1`), manages the strike state machine, and issues guidance commands to the vehicle.

---

## Architecture Overview

```
QGroundControl / Mission
        │
        │  MAVLink: MAV_CMD_USER_1 (31010)
        ▼
  ┌─────────────┐      strike_target (uORB)     ┌─────────────────┐
  │   striker   │ ────────────────────────────►  │   Commander     │
  │  (this mod) │                                │  nav_state =    │
  └─────────────┘                                │  STRIKE (7)     │
        │                                        └────────┬────────┘
        │  VEHICLE_CMD_DO_REPOSITION                      │ vehicle_control_mode
        │  (on Guided Abort)                              ▼
        │                                        ┌─────────────────────────────┐
        │                                        │  FixedwingPositionControl   │
        │                                        │  control_strike()  (Pro-Nav)│
        │                                        │  FW_POSCTRL_MODE_STRIKE     │
        └───────────────────────────────────────►│  → attitude_setpoint        │
                                                 └─────────────────────────────┘
```

---

## MAVLink Command Protocol

The module listens for **`MAV_CMD_USER_1` (ID: 31010)** on the `vehicle_command` uORB topic.

### Strike Command (Action = 0)

| Parameter | Field   | Type   | Description                            |
|-----------|---------|--------|----------------------------------------|
| `param1`  | Action  | uint8  | `0` = Strike                           |
| `param5`  | Lat     | double | Target latitude (degrees)              |
| `param6`  | Lon     | double | Target longitude (degrees)             |
| `param7`  | Alt     | float  | Target altitude AMSL (meters)          |

### Guided Abort Command (Action = 1)

| Parameter | Field   | Type   | Description                            |
|-----------|---------|--------|----------------------------------------|
| `param1`  | Action  | uint8  | `1` = Abort                            |
| `param5`  | Lat     | double | Safe recovery waypoint latitude        |
| `param6`  | Lon     | double | Safe recovery waypoint longitude       |

> If `param5`/`param6` are non-zero, a **Guided Abort** is triggered: the vehicle climbs to `STR_REC_ALT` and repositions to the given coordinates. If zero, strike is simply cancelled (Hold/Loiter).

---

## uORB Topics

| Topic                  | Direction | Description                               |
|------------------------|-----------|-------------------------------------------|
| `vehicle_command`      | Subscribe | Receives MAVLink commands                 |
| `vehicle_status`       | Subscribe | Monitors nav_state for external aborts    |
| `vehicle_global_position` | Subscribe | Reads current altitude for logging     |
| `home_position`        | Subscribe | Used for LLA→NED coordinate conversion   |
| `strike_target`        | **Publish** | Notifies Commander of strike state      |
| `vehicle_command`      | **Publish** | Issues `DO_REPOSITION` on guided abort  |

### `strike_target` Message Format

| Field         | Type   | Description                       |
|---------------|--------|-----------------------------------|
| `active`      | bool   | Whether a strike is ongoing       |
| `action_type` | uint8  | `0` = Strike, `1` = Abort         |
| `x`, `y`, `z`| float  | Target in local NED frame (meters)|

---

## Parameters

| Parameter     | Default | Range    | Description                                   |
|---------------|---------|----------|-----------------------------------------------|
| `STR_REC_ALT` | 100.0 m | 10–500 m | Altitude (AMSL) to climb to during Guided Abort before repositioning |

Set this to a value high enough to safely clear terrain and obstacles near the strike zone.

---

## Strike State Machine

```
        [IDLE]
           │  Receive MAV_CMD_USER_1 (action=0, valid lat/lon)
           ▼
       [STRIKING]  ◄──── Commander sets nav_state = STRIKE (7)
           │              FixedwingPositionControl runs Pro-Nav guidance
           │
           ├──── Receive Abort (action=1, with lat/lon) ────►  [GUIDED ABORT]
           │                                                      Climb to STR_REC_ALT
           │                                                      Reposition to safe coords
           │
           ├──── External mode change (user switches flight mode)
           │     Watchdog detects nav_state ≠ STRIKE
           ▼
        [IDLE]  (returns to loiter)
```

---

## Navigation State Integration

Strike mode uses a **dedicated navigation state: `NAVIGATION_STATE_STRIKE = 7`**.

When `strike_target.active == true`, Commander switches to this state, which configures:

```cpp
// src/modules/commander/ModeUtil/control_mode.cpp
case vehicle_status_s::NAVIGATION_STATE_STRIKE:
    vehicle_control_mode.flag_control_auto_enabled     = true;
    vehicle_control_mode.flag_control_attitude_enabled = true;
    vehicle_control_mode.flag_control_rates_enabled    = true;
    vehicle_control_mode.flag_control_allocation_enabled = true;
    break;
```

The actual guidance is implemented in `FixedwingPositionControl::control_strike()` using **Proportional Navigation (Pro-Nav)** with N=4.

---

## Abort Behavior

There are two distinct abort mechanisms:

### 1. Active Abort (QGC Button / MAVLink)
- Triggered by `MAV_CMD_USER_1` with `param1 = 1`
- If recovery coordinates (lat/lon) are provided → **Guided Abort**
  - Issues `VEHICLE_CMD_DO_REPOSITION` with altitude = `STR_REC_ALT`
  - Vehicle climbs first, then flies to safe zone
- If no coordinates → Strike cancelled, vehicle enters Loiter

### 2. Passive Abort (Watchdog)
- Triggered automatically if `nav_state` changes away from `STRIKE` (e.g., user flips RC switch to Loiter)
- Updates internal `_strike_active` flag and publishes `strike_target.active = false`
- Does **not** command the vehicle to move — only cleans up state

---

## TECS Reset on Abort

During Strike, TECS (Total Energy Control System) is bypassed — throttle is set directly to 80% by `control_strike()`. To prevent throttle "sticking" high after abort, when `FixedwingPositionControl` exits `FW_POSCTRL_MODE_STRIKE`:

```cpp
// FixedwingPositionControl.cpp — set_control_mode_current()
if (_control_mode_current == FW_POSCTRL_MODE_STRIKE) {
    _tecs.resetIntegrals();
    _tecs.handle_alt_step(_current_altitude, -_local_pos.vz);
}
```

This forces TECS to re-initialize from the current altitude and vertical speed, preventing aggressive throttle commands on mode switch.

---

## Auto-Start

The module is started automatically by the fixed-wing apps startup script:

**File**: `ROMFS/px4fmu_common/init.d/rc.fw_apps`
```bash
striker start
```

> **Note**: Do not add `striker start` to `init.d-posix/rcS` — this would start it twice in SITL and cause a *"task already running"* error.

---

## Building & Running

```bash
# Build for SITL
make px4_sitl

# Build and launch SITL with Gazebo
make px4_sitl gz_advanced_plane

# Module console commands
striker start
striker status
striker stop
```

---

## Files

| File                        | Description                                           |
|-----------------------------|-------------------------------------------------------|
| `strike_manager.cpp`        | Main module: command handler, watchdog, abort logic   |
| `strike_manager.h`          | Class definition, subscriptions, parameter handle     |
| `strike_manager_params.c`   | Parameter definitions (`STR_REC_ALT`)                 |
| `module.yaml`               | Module metadata                                       |
| `CMakeLists.txt`            | Build config                                          |

### Related Files (Modified Upstream)

| File                                                    | Change                                     |
|---------------------------------------------------------|--------------------------------------------|
| `msg/versioned/VehicleStatus.msg`                       | Added `NAVIGATION_STATE_STRIKE = 7`        |
| `src/modules/commander/ModeUtil/control_mode.cpp`       | Control mode flags for Strike              |
| `src/modules/commander/Commander.hpp/.cpp`              | `strike_target` subscription & nav switch  |
| `src/modules/fw_pos_control/FixedwingPositionControl.cpp` | `control_strike()`, TECS reset           |
| `src/modules/commander/px4_custom_mode.h`               | QGC mode display mapping                  |
| `src/modules/navigator/mission_block.cpp`               | Mission item support for `MAV_CMD_USER_1` |
| `src/modules/mavlink/mavlink_mission.cpp`               | Whitelisted `MAV_CMD_USER_1 (31010)`      |

---

## Troubleshooting

### Module not starting on hardware
Add `striker start` to `ROMFS/px4fmu_common/init.d/rc.fw_apps`.

### Vehicle flies over target without diving
Increase pitch limit in `FixedwingPositionControl::control_strike()`:
```cpp
pitch = constrain(pitch, -radians(45.0f), radians(45.0f));
```

### `STR_REC_ALT` not visible in QGC
Make sure `updateParams()` is called in `StrikeManager::init()`. Rebuild with `make px4_sitl` and refresh parameters in QGC.

### Throttle stuck high after abort
TECS reset (described above) should handle this. If it persists, verify the `set_control_mode_current()` change is present in `FixedwingPositionControl.cpp`.
