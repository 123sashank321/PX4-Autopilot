/**
 * @file strike_manager_params.c
 * Strike Manager parameters
 *
 * @author PX4 Development Team
 */

/**
 * Strike Recovery Altitude
 *
 * Altitude (AMSL) to climb to during a Guided Abort before returning to home/loiter point.
 *
 * @unit m
 * @min 10
 * @max 500
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_REC_ALT, 100.0f);

/**
 * Strike Initial Point Altitude (AGL above home)
 *
 * Height above home the aircraft must reach before the ALIGNMENT phase.
 * Determines x_kinematic (horizontal dive reach): x_k = STR_IP_ALT / tan(STR_DIVE_ANG).
 *
 * @unit m
 * @min 20
 * @max 1000
 * @decimal 1
 * @increment 5
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_IP_ALT, 100.0f);

/**
 * Strike Terminal Dive Angle
 *
 * Angle below horizontal at which the APN terminal dive is initiated.
 * Smaller angle = shallower dive, longer approach distance.
 *
 * @unit deg
 * @min 5
 * @max 80
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_DIVE_ANG, 20.0f);

/**
 * Strike Approach Settle Time
 *
 * Time the aircraft flies from Initial Point toward the AHP on the attack
 * bearing. x_buffer = STR_CRUISE_SPD * STR_SETTLE_T gives standoff buffer.
 *
 * @unit s
 * @min 1
 * @max 30
 * @decimal 1
 * @increment 0.5
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_SETTLE_T, 3.0f);

/**
 * Strike Approach Cruise Speed (geometry only)
 *
 * Used only for computing x_buffer = STR_CRUISE_SPD * STR_SETTLE_T.
 * Does not command airspeed directly (TECS handles that).
 *
 * @unit m/s
 * @min 5
 * @max 50
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_CRUISE_SPD, 15.0f);

/**
 * Strike Ingress Descent Angle
 *
 * Expected TECS descent slope during Ingress. Used to compute how far back
 * the IP must be placed so the aircraft descends to STR_IP_ALT before it
 * arrives at the IP — avoiding a long loiter.
 *   x_descent = (current_alt_AGL - STR_IP_ALT) / tan(STR_DESCENT_ANG)
 *   IP placed at x_kinematic + max(x_buffer, x_descent) from target.
 *
 * @unit deg
 * @min 3
 * @max 30
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_DESCENT_ANG, 10.0f);

/**
 * Strike IP Loiter Radius
 *
 * Radius of the holding orbit flown over the Initial Point while the aircraft
 * descends to STR_IP_ALT. Was previously hard-coded to 150 m in
 * StrikeGuidance, which is far larger than a small airframe's minimum turn
 * radius and pushed the orbit well outside the planned approach corridor.
 *
 * @unit m
 * @min 20
 * @max 500
 * @decimal 0
 * @increment 10
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_LOITER_RAD, 80.0f);

/**
 * Strike Maximum Ingress Attempts
 *
 * Number of times the guidance may fall back from ALIGNMENT to INGRESS after
 * an alignment timeout before giving up. Without a limit the aircraft can
 * re-fly the IP indefinitely when the approach geometry is unachievable.
 *
 * @min 1
 * @max 10
 * @group Striker
 */
PARAM_DEFINE_INT32(STR_MAX_ATTEMPT, 3);

/**
 * Strike Terminal Dive Never-Exceed Speed
 *
 * Airspeed ceiling used ONLY during the TERMINAL dive's overspeed protection
 * (shallows the dive to a fixed angle above this speed). Deliberately
 * separate from FW_AIRSPD_MAX: TERMINAL commands full throttle for a steep
 * powered dive, which is expected to exceed the normal cruise airspeed
 * ceiling almost immediately — reusing FW_AIRSPD_MAX here made the shallow-
 * dive protection active for the entire dive by default, not as a genuine
 * emergency fallback, and prevented the aircraft from ever reaching the
 * commanded dive angle.
 *
 * Set this to the airframe's actual structural never-exceed speed for a
 * dive, NOT simply raised to avoid the shallowing behavior — this is a
 * genuine airspeed limit, and setting it above what the airframe can safely
 * withstand risks structural failure during the dive.
 *
 * @unit m/s
 * @min 10
 * @max 100
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_DIVE_VNE, 35.0f);

/**
 * Strike Terminal Dive Steepest Nose-Down Pitch
 *
 * Magnitude of the steepest nose-down pitch angle the TERMINAL phase's
 * elevation-angle pursuit may command. Deliberately separate from
 * FW_P_LIM_MIN: that parameter governs the airframe's EVERYDAY flight
 * envelope (typically a shallow, conservative descent limit — e.g. -15 deg
 * on the stock advanced_plane airframes) and was never tuned with "also used
 * for an intentional full-power terminal dive" in mind. Reusing it here
 * capped every strike dive to the same shallow angle as a normal descent,
 * which is far too flat to close on the target accurately.
 *
 * This is NOT license to ignore the airframe's real capability the way the
 * old hardcoded +/-45 deg limit did (that was the actual bug it replaced) —
 * it is a separate, deliberately-configured limit specifically for the
 * strike dive, which the operator must set based on real bench/flight
 * testing of this specific airframe, not simply maximized.
 *
 * RECOVERY's nose-UP pull-out bound still uses FW_P_LIM_MAX unchanged —
 * over-rotating nose-up risks a stall/over-G in any mode, strike included,
 * so that limit is intentionally not overridden here.
 *
 * @unit deg
 * @min 15
 * @max 85
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_DIVE_PITCH, 60.0f);
