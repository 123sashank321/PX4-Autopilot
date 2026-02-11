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
 * This ensures the vehicle clears obstacles or terrain when aborting a strike from low altitude.
 *
 * @unit m
 * @min 10
 * @max 500
 * @decimal 1
 * @increment 1
 * @group Striker
 */
PARAM_DEFINE_FLOAT(STR_REC_ALT, 100.0f);
