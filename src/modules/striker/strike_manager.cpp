/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "strike_manager.h"
#include <lib/geo/geo.h>
#include <uORB/topics/vehicle_global_position.h>

StrikeManager::StrikeManager()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool StrikeManager::init()
{
	// Schedule periodic run at 10Hz (100ms) for watchdog
	ScheduleOnInterval(100_ms);

	// Register callback for vehicle_command topic
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("vehicle_command callback registration failed");
		return false;
	}

	// Load parameters
	updateParams();

	PX4_INFO("Strike Manager initialized");
	return true;
}

void StrikeManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 1. Refresh STR_* parameters when the user changes them in the GCS.
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}


	// 2. Heartbeat. strike_target was previously published only on the
	//    designation event, so a target left over from an earlier run looked
	//    identical to a live one. Re-emitting it each cycle gives the guidance
	//    a timestamp it can actually test for staleness.
	if (_strike_active) {
		_active_target.timestamp = hrt_absolute_time();
		_strike_target_pub.publish(_active_target);
	}

	// 3. EKF reset handling.
	//    vehicle_local_position is re-referenced on a position reset, so a NED
	//    target computed before the reset now points somewhere else. Re-project
	//    from the retained lat/lon rather than flying to a stale point.
	if (_strike_active) {
		vehicle_local_position_s lp;

		if (_local_pos_sub.copy(&lp)) {
			if (lp.xy_reset_counter != _last_xy_reset || lp.z_reset_counter != _last_z_reset) {
				_last_xy_reset = lp.xy_reset_counter;
				_last_z_reset  = lp.z_reset_counter;

				matrix::Vector3f target_ned;

				if (global_to_local(_target_lat, _target_lon, _target_alt, target_ned)) {
					strike_target_s msg{};
					msg.timestamp = hrt_absolute_time();
					msg.action_type = strike_target_s::ACTION_STRIKE;
					msg.active = true;
					msg.x = target_ned(0);
					msg.y = target_ned(1);
					msg.z = target_ned(2);
					compute_geometry(target_ned, matrix::Vector3f(lp.x, lp.y, lp.z), msg);
					msg.designation_id = _active_target.designation_id;  // preserve — not a new command
					_active_target = msg;
					_strike_target_pub.publish(msg);
					PX4_WARN("EKF reset (xy=%u z=%u): strike target re-projected",
						 (unsigned)lp.xy_reset_counter, (unsigned)lp.z_reset_counter);

				} else {
					PX4_ERR("EKF reset and no global reference: aborting strike");
					_strike_active = false;
					strike_target_s abort_msg{};
					abort_msg.timestamp = hrt_absolute_time();
					abort_msg.active = false;
					abort_msg.action_type = strike_target_s::ACTION_ABORT;
					_strike_target_pub.publish(abort_msg);
				}
			}
		}
	}

	// 4. Watchdog for External Mode Changes (User pressed Hold/Pause)
	vehicle_status_s status;

	if (_vehicle_status_sub.updated()) {
		if (_vehicle_status_sub.copy(&status)) {

			// If we think we are striking, but the system is NOT in Strike mode,
			// it means the user or system has switched modes (e.g. to Loiter, RTL, or Stick Override).
			// We must update our internal state and notify listeners.
			const bool grace_elapsed =
				(hrt_elapsed_time(&_strike_requested_time) > STRIKE_MODE_GRACE_US);

			if (_strike_active && grace_elapsed
			    && status.nav_state != vehicle_status_s::NAVIGATION_STATE_STRIKE) {
				_strike_active = false;

				strike_target_s abort_msg{};
				abort_msg.timestamp = hrt_absolute_time();
				abort_msg.active = false;
				abort_msg.action_type = 1; // Abort
				abort_msg.x = NAN;
				abort_msg.y = NAN;
				abort_msg.z = NAN;

				_strike_target_pub.publish(abort_msg);
				PX4_INFO("Strike Aborted by External Mode Switch (To NavState: %d)", status.nav_state);
			}
		}
	}

	// 5. Process vehicle commands (The main trigger)
	vehicle_command_s vcmd{};

	if (_vehicle_command_sub.update(&vcmd)) {
		handle_vehicle_command(&vcmd);
	}
}

void StrikeManager::handle_vehicle_command(const vehicle_command_s *vehicle_command)
{
	if (vehicle_command == nullptr) {
		return;
	}

	// Filter for MAV_CMD_USER_1 (31010)
	if (vehicle_command->command == 31010) {  // MAV_CMD_USER_1

		PX4_INFO("Received MAV_CMD_USER_1 strike command");

		// Extract parameters
		uint8_t action_type = static_cast<uint8_t>(vehicle_command->param1);
		double lat = vehicle_command->param5;
		double lon = vehicle_command->param6;
		float alt = vehicle_command->param7;

		strike_target_s strike_target{};
		strike_target.timestamp = hrt_absolute_time();
		strike_target.action_type = action_type;

		if (action_type == 1) { // ABORT
			strike_target.active = false;
			strike_target.x = NAN;
			strike_target.y = NAN;
			strike_target.z = NAN;

			_strike_active = false;

			_strike_target_pub.publish(strike_target);
			send_command_ack(*vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

			// Check if Param 5/6 (Lat/Lon) are provided for a Guided Abort
			if (fabs(lat) > 0.000001 && fabs(lon) > 0.000001) {
				// Get current altitude for debugging
				vehicle_global_position_s global_pos;
				float current_alt = 0.0f;

				if (_global_pos_sub.copy(&global_pos)) {
					current_alt = global_pos.alt;
				}

				// Get recovery altitude from parameter
				float recovery_alt = _param_str_rec_alt.get();

				// Log what QGC sent us
				//PX4_INFO("ABORT: QGC sent alt=%.2f (param7), Using STR_REC_ALT=%.2f", (double)alt, (double)recovery_alt);
				PX4_INFO("ABORT: Current vehicle alt=%.2f", (double)current_alt);

				// Send Reposition Command
				vehicle_command_s reposition_cmd{};
				reposition_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
				reposition_cmd.param1 = -1.0f; // Ground Speed (Default)
				reposition_cmd.param2 = 1.0f; // Bit 1: Reposition
				reposition_cmd.param5 = lat;
				reposition_cmd.param6 = lon;
				reposition_cmd.param7 = recovery_alt; // Use parameter

				reposition_cmd.target_system = 1;
				reposition_cmd.target_component = 1;
				reposition_cmd.timestamp = hrt_absolute_time();

				uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
				vcmd_pub.publish(reposition_cmd);

				mavlink_log_info(&_mavlink_log_pub, "ABORTED: Repositioning to %.5f, %.5f @ %.0fm AMSL", lat, lon, (double)recovery_alt);
				PX4_INFO("ABORT: Commanded DO_REPOSITION with alt=%.2f AMSL", (double)recovery_alt);
				//PX4_INFO("ABORT: Expected climb from %.2f to %.2f", (double)current_alt, (double)recovery_alt);
			}

		} else { // STRIKE
			matrix::Vector3f target_ned;

			if (global_to_local(lat, lon, alt, target_ned)) {
				// Snapshot vehicle NED position at command reception time
				vehicle_local_position_s local_pos{};
				matrix::Vector3f vehicle_ned(0.f, 0.f, 0.f);

				if (_local_pos_sub.copy(&local_pos)) {
					vehicle_ned = matrix::Vector3f(local_pos.x, local_pos.y, local_pos.z);
				}

				strike_target.active = true;
				strike_target.x = target_ned(0);
				strike_target.y = target_ned(1);
				strike_target.z = target_ned(2);

				// Compute IP / AHP and fill geometry fields
				compute_geometry(target_ned, vehicle_ned, strike_target);

				// New designation — a fresh id lets Commander tell this apart
				// from a heartbeat/reset-reprojection republish of a strike it
				// has already flown and departed from (see Commander.cpp).
				strike_target.designation_id = _strike_target_count + 1;

				// Retain the geodetic target so it can be re-projected after an
				// EKF reset, and latch the reset counters we are consistent with.
				_target_lat = lat;
				_target_lon = lon;
				_target_alt = alt;
				_last_xy_reset = local_pos.xy_reset_counter;
				_last_z_reset  = local_pos.z_reset_counter;

				_strike_requested_time = hrt_absolute_time();
				_strike_active = true;
				_active_target = strike_target;
				_strike_target_pub.publish(strike_target);
				_strike_target_count++;

				send_command_ack(*vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
				mavlink_log_info(&_mavlink_log_pub, "STRIKE: %.4f, %.4f @ %.0fm", lat, lon,
						 static_cast<double>(alt));
				PX4_INFO("STRIKE target #%u: x=%.1f y=%.1f z=%.1f | IP=(%.1f,%.1f) AHP=(%.1f,%.1f) xk=%.1fm",
					 (unsigned)_strike_target_count,
					 (double)target_ned(0), (double)target_ned(1), (double)target_ned(2),
					 (double)strike_target.ip_x, (double)strike_target.ip_y,
					 (double)strike_target.ahp_x, (double)strike_target.ahp_y,
					 (double)strike_target.x_kinematic);

			} else {
				send_command_ack(*vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
				mavlink_log_critical(&_mavlink_log_pub, "Strike Failed: no global position reference");
			}
		}
	}
}


void StrikeManager::send_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = cmd.command;
	ack.result = result;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	_command_ack_pub.publish(ack);
}

int StrikeManager::print_status()
{
	PX4_INFO("Strike targets published: %u", (unsigned)_strike_target_count);
	return 0;
}

int StrikeManager::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}

bool StrikeManager::global_to_local(double lat, double lon, float alt, matrix::Vector3f &ned)
{
	vehicle_local_position_s local_pos;

	// The target must be expressed in the SAME frame the guidance consumes.
	// StrikeGuidance works entirely in vehicle_local_position, whose origin is
	// the EKF reference (ref_lat/ref_lon/ref_alt) - NOT home_position. Home is
	// set at arming and can be moved later by DO_SET_HOME, so projecting the
	// target against home places it in a different frame from the vehicle and
	// offsets it by the home-to-origin delta, laterally and vertically.
	// SITL usually hides this because the two happen to coincide.
	if (!_local_pos_sub.copy(&local_pos) || !local_pos.xy_global || !local_pos.z_global) {
		return false;
	}

	MapProjection map_ref(local_pos.ref_lat, local_pos.ref_lon);
	float x, y;
	map_ref.project(lat, lon, x, y);

	// alt = 0 is the common "ground strike, altitude omitted" case from a GCS.
	// Treat it as the local origin altitude so NED z = 0, rather than several
	// hundred metres underground (which would saturate the terminal pitch).
	const float target_amsl = (alt < 1.0f) ? local_pos.ref_alt : alt;
	const float z = -(target_amsl - local_pos.ref_alt);

	ned = matrix::Vector3f(x, y, z);
	return true;
}

void StrikeManager::compute_geometry(const matrix::Vector3f &target_ned,
				     const matrix::Vector3f &vehicle_ned,
				     strike_target_s &msg)
{
	// --- Parameters ---
	const float ip_alt_agl  = _param_str_ip_alt.get();                    // [m] AGL
	const float dive_ang    = math::radians(_param_str_dive_ang.get());   // [rad]
	const float settle_t    = _param_str_settle_t.get();                  // [s]
	const float cruise_spd  = _param_str_cruise_spd.get();               // [m/s]

	// --- Horizontal dive reach (fixed by target geometry) ---
	const float x_kinematic = ip_alt_agl / tanf(math::max(dive_ang, 0.01f));

	// --- Settle buffer (standoff for ALIGNMENT bearing lock) ---
	// Fast descent is handled in StrikeGuidance via height_rate setpoint,
	// so IP distance stays compact regardless of current altitude.
	const float x_buffer    = cruise_spd * settle_t;
	const float x_ip_total  = x_kinematic + x_buffer;

	PX4_INFO("Geometry: xk=%.0fm x_buf=%.0fm total_IP=%.0fm",
		 (double)x_kinematic, (double)x_buffer, (double)x_ip_total);


	// --- 2D approach unit vector: from target toward vehicle at command time ---
	const matrix::Vector2f target2d(target_ned(0), target_ned(1));
	const matrix::Vector2f vehicle2d(vehicle_ned(0), vehicle_ned(1));
	matrix::Vector2f approach = vehicle2d - target2d;

	const float approach_mag = approach.norm();

	if (approach_mag < 1.0f) {
		approach = matrix::Vector2f(1.0f, 0.0f);  // default North if directly overhead

	} else {
		approach = approach / approach_mag;
	}

	// --- NED-z: negative = above home (ip_alt_agl above ground → z = -ip_alt_agl) ---
	const float ip_z = target_ned(2) - ip_alt_agl;

	// IP: x_ip_total from target (accounts for descent + settle)
	msg.ip_x = target2d(0) + approach(0) * x_ip_total;
	msg.ip_y = target2d(1) + approach(1) * x_ip_total;
	msg.ip_z = ip_z;

	// AHP: exactly x_kinematic from target (terminal dive trigger point)
	msg.ahp_x = target2d(0) + approach(0) * x_kinematic;
	msg.ahp_y = target2d(1) + approach(1) * x_kinematic;
	msg.ahp_z = ip_z;

	msg.x_kinematic = x_kinematic;

	// Snapshot the flight profile so the guidance flies the same numbers this
	// geometry was computed with. x_buffer above assumes the vehicle actually
	// cruises at STR_CRUISE_SPD, which only holds if it is commanded.
	msg.cruise_speed  = cruise_spd;
	msg.descent_angle = math::radians(_param_str_descent_ang.get());
	msg.loiter_radius = _param_str_loiter_rad.get();
	msg.max_attempts  = static_cast<uint8_t>(_param_str_max_attempt.get());
	msg.dive_vne       = _param_str_dive_vne.get();
	msg.dive_pitch_lim = math::radians(_param_str_dive_pitch_lim.get());
}



int StrikeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Strike Manager Module - Handles MAV_CMD_USER_1 (31010) commands for designating strike targets.

Subscribes to vehicle_command topic, filters for MAV_CMD_USER_1, extracts GPS coordinates
and action type from command parameters, and publishes to strike_target topic.

Parameters:
- param1: Action Type (0 = STRIKE, 1 = ABORT)
- param5: Latitude in degrees (float64)
- param6: Longitude in degrees (float64)
- param7: Altitude in meters (float32, optional)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("striker", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int StrikeManager::task_spawn(int argc, char *argv[])
{
	StrikeManager *instance = new StrikeManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("Alloc failed");
	}

	// Cleanup on failure
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int striker_main(int argc, char *argv[])
{
	return StrikeManager::main(argc, argv);
}
