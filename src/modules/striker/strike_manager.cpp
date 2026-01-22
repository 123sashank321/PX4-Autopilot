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

StrikeManager::StrikeManager()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool StrikeManager::init()
{
	// Schedule periodic run at 10Hz for housekeeping
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

	// Process any pending vehicle commands
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
		// param1: Action Type (0 = STRIKE, 1 = ABORT)
		// param5: Latitude (degrees)
		// param6: Longitude (degrees)
		// param7: Altitude (meters, optional)

		uint8_t action_type = static_cast<uint8_t>(vehicle_command->param1);
		double lat = vehicle_command->param5;
		double lon = vehicle_command->param6;
		float alt = vehicle_command->param7;

		// Validate coordinates
		if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
			PX4_WARN("Invalid coordinates: lat=%.6f, lon=%.6f", lat, lon);
			send_vehicle_command_ack(vehicle_command->command,
			                         vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED,
			                         vehicle_command->source_system,
			                         vehicle_command->source_component);
			return;
		}

		// Validate action type
		if (action_type > 1) {
			PX4_WARN("Invalid action_type: %u (must be 0=STRIKE or 1=ABORT)", action_type);
			send_vehicle_command_ack(vehicle_command->command,
			                         vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED,
			                         vehicle_command->source_system,
			                         vehicle_command->source_component);
			return;
		}

		// Publish strike target
		strike_target_s strike_target{};
		strike_target.timestamp = hrt_absolute_time();
		strike_target.action_type = action_type;
		strike_target.lat = lat;
		strike_target.lon = lon;
		strike_target.alt = alt;

		_strike_target_pub.publish(strike_target);
		_strike_target_count++;

		const char* action_str = (action_type == 0) ? "STRIKE" : "ABORT";
		PX4_INFO("Published %s target #%u: lat=%.6f, lon=%.6f, alt=%.2f",
		         action_str, _strike_target_count, lat, lon, static_cast<double>(alt));

		// Send acknowledgment
		send_vehicle_command_ack(vehicle_command->command,
		                         vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
		                         vehicle_command->source_system,
		                         vehicle_command->source_component);
	}
}

void StrikeManager::send_vehicle_command_ack(uint32_t command, uint8_t result,
                                               uint8_t target_system, uint8_t target_component)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = command;
	ack.result = result;
	ack.target_system = target_system;
	ack.target_component = target_component;

	_vehicle_command_ack_pub.publish(ack);
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
