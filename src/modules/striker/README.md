# Striker Module

The `striker` module is a PX4 custom flight mode that enables **fixed-wing guided strike maneuvers**. It receives target designation commands via MAVLink (`MAV_CMD_USER_1`), manages the strike state machine, and issues guidance commands to the vehicle.

It also runs on **VTOL airframes** (currently validated on a quad tailsitter), where STRIKE is only enterable — and forcibly exited if interrupted — while the vehicle is in full fixed-wing flight, never mid-transition. See "VTOL / Transition Gating" below.

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
        │                                        │  FixedWingModeManager       │
        │                                        │  control_strike() drives    │
        │                                        │  StrikeGuidance (IP→AHP→APN)│
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
>
> This matters for **mission-planned** Strike items specifically: QGC's Plan
> editor always shows and uploads a coordinate for a Strike item regardless
> of which Action is selected (its generic item editor can't conditionally
> hide the coordinate picker based on another field's value) — so a
> mission-planned Abort item will carry whatever lat/lon was last set on it.
> That's harmless by design: it's exactly the zero-means-cancel /
> non-zero-means-guided-abort behavior above, so just confirm the Abort
> item's coordinate is genuinely `0,0` if you want a plain cancel rather
> than a guided reposition.

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

| Field                    | Type   | Description                                                    |
|--------------------------|--------|------------------------------------------------------------------|
| `x`, `y`, `z`            | float  | Target in local NED frame (meters)                               |
| `ip_x`, `ip_y`, `ip_z`   | float  | Initial Point NED (INGRESS destination)                          |
| `ahp_x`, `ahp_y`, `ahp_z`| float  | Attack Heading Point NED (TERMINAL transition trigger)            |
| `x_kinematic`            | float  | Horizontal dive reach [m] — TERMINAL transition threshold         |
| `cruise_speed`           | float  | Snapshotted `STR_CRUISE_SPD` [m/s]                                |
| `descent_angle`          | float  | Snapshotted `STR_DESCENT_ANG` [rad]                               |
| `loiter_radius`          | float  | Snapshotted `STR_LOITER_RAD` [m]                                  |
| `max_attempts`           | uint8  | Snapshotted `STR_MAX_ATTEMPT`                                     |
| `dive_vne`               | float  | Snapshotted `STR_DIVE_VNE` [m/s] — TERMINAL-only overspeed ceiling, separate from cruise `FW_AIRSPD_MAX` |
| `dive_pitch_lim`         | float  | Snapshotted `STR_DIVE_PITCH` [rad] — TERMINAL-only nose-down pitch magnitude, separate from cruise `FW_P_LIM_MIN` |
| `designation_id`         | uint32 | Increments only on a genuine new `MAV_CMD_USER_1`; distinguishes a fresh command from a heartbeat/reset-reprojection republish (prevents silently auto-resuming after a failsafe-forced departure — see Abort Behavior) |
| `active`                 | bool   | Whether a strike is ongoing                                       |
| `action_type`            | uint8  | `0` = Strike, `1` = Abort                                         |

---

## Parameters

| Parameter        | Default | Description                                                     |
|------------------|---------|-----------------------------------------------------------------|
| `STR_REC_ALT`    | 100 m   | Altitude (AMSL) to climb to during Guided Abort                 |
| `STR_IP_ALT`     | —       | Initial Point altitude AGL                                       |
| `STR_DIVE_ANG`   | —       | Terminal dive angle, sets `x_kinematic`                          |
| `STR_SETTLE_T`   | —       | Settle time used for the IP standoff buffer                      |
| `STR_CRUISE_SPD` | 15 m/s  | Commanded EAS during INGRESS/ALIGNMENT                           |
| `STR_DESCENT_ANG`| 10 deg  | Nose-down pitch for the fast INGRESS descent                     |
| `STR_LOITER_RAD` | 80 m    | IP holding-orbit radius                                          |
| `STR_MAX_ATTEMPT`| 3       | ALIGNMENT→INGRESS retries before giving up                       |
| `STR_DIVE_VNE`   | 35 m/s  | TERMINAL dive-only overspeed ceiling — **not** `FW_AIRSPD_MAX` (cruise); must be tuned to the airframe's actual dive VNE, since a full-throttle TERMINAL dive is expected to exceed cruise airspeed |
| `STR_DIVE_PITCH` | 60 deg | Steepest nose-down pitch the TERMINAL dive may command — **not** `FW_P_LIM_MIN` (cruise, e.g. -15 deg on the stock advanced_plane airframes); must be validated by real bench/flight test on this specific airframe, not just raised |

All `STR_*` values are snapshotted into `strike_target` at designation time, so
the guidance flies the same numbers the IP/AHP geometry was computed with.
Set `STR_REC_ALT` and `STR_IP_ALT` to values high enough to safely clear
terrain and obstacles near the strike zone.

`StrikeGuidance` does **not** have its own hardcoded pitch/roll limits — the
TERMINAL dive and RECOVERY pull-out are bounded by the airframe's own
`FW_R_LIM`/`FW_P_LIM_MIN`/`FW_P_LIM_MAX` parameters, passed in from
`FixedWingModeManager::control_strike()` each cycle. Tune those, not a
constant in the guidance source, if the dive/recovery angle needs adjusting.

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

The actual guidance is implemented in `fw_mode_manager/strike_guidance/StrikeGuidance`,
driven by `FixedWingModeManager::control_strike()` in `FW_POSCTRL_MODE_STRIKE`.
It is a staged **INGRESS → ALIGNMENT → TERMINAL** trajectory with **RECOVERY** as
the abort-out-of-dive path, not the single-stage Pro-Nav described in older
revisions of this file. `FixedwingPositionControl::control_strike()` no longer
exists.

The terminal phase is **pure proportional navigation** in the horizontal plane
(`a_lat = N·V·λ̇`, issued directly as a lateral acceleration) combined with an
elevation-angle pursuit term in pitch. It is therefore not full 3-D APN, despite
the phase name.

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
- Triggered automatically if `nav_state` changes away from `STRIKE` (e.g., user flips RC switch to Loiter, or a failsafe — RC loss, geofence, low battery — forces an exit)
- Updates internal `_strike_active` flag and publishes `strike_target.active = false`
- Does **not** command the vehicle to move — only cleans up state
- Does **not** silently resume: `strike_target.active` staying `true` (the
  10 Hz heartbeat) is not by itself enough to re-enter STRIKE after any
  departure from it. Commander tracks the `designation_id` of the last
  strike it successfully entered and requires a **new** `MAV_CMD_USER_1`
  (a fresh `designation_id`) to re-enter — otherwise a strike interrupted
  by a transient failsafe would resume the instant that failsafe cleared,
  with no new operator command.

---

## TECS/NPFG Bypass During Strike

This section previously described an explicit `_tecs.resetIntegrals()` /
`handle_alt_step()` workaround in `FixedwingPositionControl::set_control_mode_current()`.
**That code no longer exists** — TECS lives in `FwLateralLongitudinalControl`
now, not `FixedWingModeManager`, and there is no strike-specific reset hook
into it.

Instead, `StrikeGuidance::Output` uses the NaN convention documented in the
top-level CLAUDE.md: a **finite** `course`/`altitude` means NPFG/TECS drive
that axis; `NAN` means they are bypassed and `lateral_acceleration`/
`pitch_direct`/`throttle_direct` are used directly. TERMINAL sets `altitude`
and `airspeed` to `NAN` (TECS fully bypassed) and commands `pitch_direct`/
`throttle_direct` directly instead. When STRIKE ends and control returns to a
mode that drives `altitude`/`course` normally again, TECS/NPFG simply resume
computing their own outputs from the live state on the next cycle — there is
no separate "stuck throttle" state to reset, since TECS was never given a
setpoint to integrate against while bypassed.

---

## VTOL / Transition Gating

On a VTOL (tailsitter or otherwise), `StrikeGuidance` runs unmodified: once a
tailsitter completes its front-transition into `vtol_mode::FW_MODE`,
`vtol_att_control` has already normalized attitude to the same convention a
plain fixed-wing airframe uses, so the FW-only guidance/control chain
(`fw_mode_manager`, TECS, NPFG) sees nothing VTOL-specific.

What's gated instead is **entry**: `Commander.cpp`'s strike arbitration
requires both `!_vehicle_status.in_transition_mode` and
`_vehicle_status.vehicle_type == VEHICLE_TYPE_FIXED_WING` before switching
into `NAVIGATION_STATE_STRIKE`. The second check matters as much as the
first: a VTOL sitting in steady rotary-wing hover has `in_transition_mode ==
false`, so without it a strike command would be accepted, switch nav_state
to STRIKE, and then simply do nothing —
`FixedWingModeManager::set_control_mode_current()` no-ops to
`FW_POSCTRL_MODE_OTHER` for `ROTARY_WING` outside a transition, and nothing
in `striker`/`Commander` commands a front-transition on its own, so the
strike would never actually execute. Both checks together mean STRIKE is
only enterable once the vehicle is already established in FW flight.

A `MAV_CMD_USER_1` arriving while either condition fails is simply not acted
on that cycle (it retries every cycle via the existing designation-id logic,
same as any other transient rejection — see "Abort Behavior"). If a
transition starts *while already in STRIKE* (e.g. a failsafe forces a
back-transition), the same checks make the entry condition go false, and
control falls straight into the existing "strike ended" branch — which
restores whatever nav_state STRIKE interrupted, exactly as if the strike had
been passively aborted. No separate mid-transition abort path was needed;
the existing entry/exit branches already cover it once these checks are
added to the entry condition.

For a plain (non-VTOL) fixed-wing airframe, `vehicle_type` is always
`FIXED_WING` and `in_transition_mode` is always `false`, so neither check
changes existing fixed-wing behavior.

`striker start` must be reachable on the airframe's init path for any of
this to matter — see "Auto-Start" below.

---

## Auto-Start

The module is started automatically by the fixed-wing and VTOL apps startup scripts:

**Files**: `ROMFS/px4fmu_common/init.d/rc.fw_apps` and `ROMFS/px4fmu_common/init.d/rc.vtol_apps`
```bash
striker start
```

Not present in `rc.mc_apps` — STRIKE is fixed-wing-flight-only (see "VTOL /
Transition Gating"), so a pure multicopter airframe has no use for it.

> **Note**: Do not add `striker start` to `init.d-posix/rcS` — this would start it twice in SITL and cause a *"task already running"* error.

---

## Building & Running

```bash
# Build for SITL
make px4_sitl

# Build and launch SITL with Gazebo
make px4_sitl gz_advanced_plane

# Build and launch SITL with Gazebo, quad tailsitter VTOL
make px4_sitl gz_quadtailsitter

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
| `strike_manager_params.c`   | Parameter definitions (`STR_*`)                       |
| `module.yaml`               | Module metadata                                       |
| `CMakeLists.txt`            | Build config                                          |

### Related Files (Modified Upstream)

| File                                                    | Change                                     |
|---------------------------------------------------------|--------------------------------------------|
| `msg/versioned/VehicleStatus.msg`                       | Added `NAVIGATION_STATE_STRIKE = 7`        |
| `src/modules/commander/ModeUtil/control_mode.cpp`       | Control mode flags for Strike              |
| `src/modules/commander/ModeUtil/mode_requirements.cpp`  | Arming/health-check requirements for Strike (position, prevent-arming-on-ground) |
| `src/modules/commander/Commander.hpp/.cpp`              | `strike_target` subscription, nav switch, `designation_id` re-entry guard |
| `src/modules/fw_mode_manager/FixedWingModeManager.cpp`   | `control_strike()` publishes the setpoints  |
| `src/modules/fw_mode_manager/strike_guidance/StrikeGuidance.hpp/.cpp` | State machine, PN law, `FW_R_LIM`/`FW_P_LIM_*`-bounded pitch/roll, EKF-reset hold, terrain-aware RECOVERY floor |
| `src/modules/commander/px4_custom_mode.h`               | QGC mode display mapping                  |
| `src/modules/navigator/mission_block.cpp`               | Mission item support for `MAV_CMD_USER_1` |
| `src/modules/mavlink/mavlink_mission.cpp`               | Whitelisted `MAV_CMD_USER_1 (31010)`      |
| `ROMFS/px4fmu_common/init.d/rc.vtol_apps`               | `striker start`, so STRIKE is reachable on VTOL airframes |

---

## Troubleshooting

### Module not starting on hardware
Add `striker start` to `ROMFS/px4fmu_common/init.d/rc.fw_apps` (fixed-wing)
or `ROMFS/px4fmu_common/init.d/rc.vtol_apps` (VTOL), matching the airframe.

### `MAV_CMD_USER_1` rejected / ignored on a VTOL
Expected if the vehicle is mid-transition — see "VTOL / Transition Gating".
The command isn't dropped; it's simply not actionable that cycle. Re-issue
it (or just wait — a mission-triggered strike naturally retries) once the
front-transition to FW mode completes.

### Vehicle flies over target without diving
The terminal dive's pitch command is bounded by the airframe's own
`FW_P_LIM_MIN` (nose-down, this is the one that matters for a dive) and
`FW_P_LIM_MAX` (nose-up, matters for RECOVERY pull-out) — not a constant in
`StrikeGuidance`. If the dive angle looks capped too shallow, widen
`FW_P_LIM_MIN` (and `FW_R_LIM` if turns toward the target look under-banked).

### `STR_REC_ALT` not visible in QGC
Make sure `updateParams()` is called in `StrikeManager::init()`. Rebuild with `make px4_sitl` and refresh parameters in QGC.

### Throttle stuck high after abort
See "TECS/NPFG Bypass During Strike" above — TECS resumes computing its own
throttle from live state once STRIKE ends, there is nothing to manually
reset. If throttle is genuinely stuck, look at whatever mode was resumed
(`_pre_strike_nav_state` in `Commander.cpp`) rather than at strike/guidance
code.
