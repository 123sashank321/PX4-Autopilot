uint64 timestamp        # time since system start (microseconds)

float32 x               # Target NED-x / North [m]
float32 y               # Target NED-y / East  [m]
float32 z               # Target NED-z / Down  [m]  (positive = below home)

float32 ip_x            # Initial Point NED-x [m]
float32 ip_y            # Initial Point NED-y [m]
float32 ip_z            # Initial Point NED-z [m]  (negative = above home)

float32 ahp_x           # Attack Heading Point NED-x [m]
float32 ahp_y           # Attack Heading Point NED-y [m]
float32 ahp_z           # Attack Heading Point NED-z [m]

float32 x_kinematic     # Horizontal reach of terminal dive [m] — TERMINAL transition threshold

# Flight-profile parameters, snapshotted from STR_* at designation time so the
# guidance flies the same numbers the IP/AHP geometry was computed with.
float32 cruise_speed    # [m/s] commanded EAS during INGRESS/ALIGNMENT (STR_CRUISE_SPD)
float32 descent_angle   # [rad] nose-down pitch used for the fast INGRESS descent (STR_DESCENT_ANG)
float32 loiter_radius   # [m] IP holding-orbit radius (STR_LOITER_RAD)
uint8 max_attempts      # max ALIGNMENT->INGRESS retries before giving up (STR_MAX_ATTEMPT)
float32 dive_vne        # [m/s] TERMINAL dive-only overspeed threshold, separate from cruise FW_AIRSPD_MAX (STR_DIVE_VNE)
float32 dive_pitch_lim  # [rad] TERMINAL dive-only nose-down pitch magnitude, separate from cruise FW_P_LIM_MIN (STR_DIVE_PITCH)

# Increments only on a genuine new MAV_CMD_USER_1 STRIKE command. The 10 Hz
# heartbeat and EKF-reset re-projection republish the SAME value, so Commander
# can tell "still the strike already flown/departed from" apart from "a fresh
# designation" and avoid silently auto-resuming after a failsafe clears.
uint32 designation_id

bool active             # Guidance activation state
uint8 action_type       # 0=Strike, 1=Abort

uint8 ACTION_STRIKE = 0
uint8 ACTION_ABORT = 1
