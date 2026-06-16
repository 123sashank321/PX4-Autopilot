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
