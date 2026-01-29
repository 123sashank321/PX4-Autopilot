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

	// 1. Check for dynamic target updates (from ROS 2 or other modules)


	// 2. Watchdog for External Mode Changes (User pressed Hold/Pause)
	vehicle_status_s status;
	if (_vehicle_status_sub.updated()) {
		if (_vehicle_status_sub.copy(&status)) {

			// Detect Mode SWITCH (Edge Detection)
			// We only want to abort if the user *switches* to a manual mode from something else,
			// or if we are active and the mode *changes*.
			// But wait, if we are in Strike, the underlying nav_state might actally BE 'POSCTL' or 'LOITER'
			// if FlightModeManager injects the task on top of it.
			// Actually, FlightModeManager usually maps NavState -> FlightTask.
			// e.g. NAV_STATE_POSCTL -> FlightTaskManualPosition.
			// Our logic in FlightModeManager *overrides* this mapping.
			// So if the system thinks it is in POSCTL, but we forced FlightTaskStrike,
			// the `status.nav_state` will report `POSCTL`.
			// So `status.nav_state == POSCTL` is TRUE while striking!
			// This causes the immediate abort.

			// FIX: We cannot rely on static nav_state checks if our Task hijacks the state.
			// We should only abort if we see a REQUEST for a mode change or a change in state.

			// Better yet: We should only check for "Safe" modes that definitely imply "Give me control"
			// AND ensure we aren't just reading the steady state.

			static uint8_t last_nav_state = 0;
			static hrt_abstime strike_activation_time = 0;

			// Initialize last state on first run
			if (last_nav_state == 0) last_nav_state = status.nav_state;

			if (_strike_active) {
				// Grace period: Don't abort for 2 seconds after activation
				// This gives FlightModeManager time to switch tasks
				if (strike_activation_time == 0) {
					strike_activation_time = hrt_absolute_time();
				}

				if (hrt_elapsed_time(&strike_activation_time) < 2_s) {
					// Within grace period, don't check watchdog
					last_nav_state = status.nav_state;
					return;
				}

				// Only abort if the state *changed* to a manual mode
				if (status.nav_state != last_nav_state) {
					if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
						status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL ||
						status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
						status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {

						_strike_active = false;

						strike_target_s abort_msg{};
						abort_msg.timestamp = hrt_absolute_time();
						abort_msg.active = false;
						abort_msg.action_type = 1; // Abort
						abort_msg.x = NAN;
						abort_msg.y = NAN;
						abort_msg.z = NAN;

						_strike_target_pub.publish(abort_msg);
						PX4_INFO("User Override: Strike Aborted by Mode Switch (To %d)", status.nav_state);
						mavlink_log_info(&_mavlink_log_pub, "Strike Aborted by Mode Switch");
					}
				}
			} else {
				// Reset activation time when inactive
				strike_activation_time = 0;
			}
			last_nav_state = status.nav_state;
		}
	}

	// 3. Process vehicle commands (The main trigger)
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

			// Check if Param 5/6 (Lat/Lon) are provided for a Guided Abort
			if (fabs(lat) > 0.000001 && fabs(lon) > 0.000001) {
				// Send Reposition Command
				vehicle_command_s reposition_cmd{};
				reposition_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
				reposition_cmd.param1 = -1.0f; // Ground Speed (Default)
				reposition_cmd.param2 = 1.0f; // Bit 1: Reposition
				reposition_cmd.param5 = lat;
				reposition_cmd.param6 = lon;
				reposition_cmd.param7 = alt;

				reposition_cmd.target_system = 1;
				reposition_cmd.target_component = 1;
				reposition_cmd.source_system = 1;
				reposition_cmd.source_component = 1;
				reposition_cmd.from_external = false;
				reposition_cmd.confirmation = 0;
				reposition_cmd.timestamp = hrt_absolute_time();

				uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
				vcmd_pub.publish(reposition_cmd);

				mavlink_log_info(&_mavlink_log_pub, "ABORTED: Repositioning to %.5f, %.5f", lat, lon);
				PX4_INFO("Published Reposition to Lat: %.5f Lon: %.5f", lat, lon);
			}

		} else { // STRIKE
			// Update Target State
			matrix::Vector3f ned;
			if (global_to_local(lat, lon, alt, ned)) {
				strike_target.active = true;
				strike_target.x = ned(0);
				strike_target.y = ned(1);
				strike_target.z = ned(2);

				_strike_active = true;

				_strike_target_pub.publish(strike_target);
				_strike_target_count++;

				mavlink_log_info(&_mavlink_log_pub, "STRIKE: %.4f, %.4f @ %.0fm", lat, lon, static_cast<double>(alt));
				PX4_INFO("Published STRIKE target #%u: x=%.2f, y=%.2f, z=%.2f",
					 _strike_target_count, (double)ned(0), (double)ned(1), (double)ned(2));
			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Strike Failed: Invalid Home Position");
			}
		}
	}
}


int StrikeManager::print_status()
{
	PX4_INFO("Strike targets published: %u", _strike_target_count);
	return 0;
}

int StrikeManager::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}

bool StrikeManager::global_to_local(double lat, double lon, float alt, matrix::Vector3f &ned)
{
	home_position_s home;
	if (_home_position_sub.copy(&home) && home.valid_lpos) {
		MapProjection map_ref(home.lat, home.lon);
		float x, y;
		map_ref.project(lat, lon, x, y);
		float z = -(alt - home.alt); // Altitude to NED Z (down is positive)
		ned = matrix::Vector3f(x, y, z);
		return true;
	}
	return false;
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
