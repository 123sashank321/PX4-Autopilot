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

/**
 * @file strike_manager.h
 * @author PX4 Development Team
 *
 * Strike Manager Module - Handles MAV_CMD_USER_1 commands for strike target designation
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/strike_target.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/home_position.h>

#include <lib/systemlib/mavlink_log.h>

using namespace time_literals;

extern "C" __EXPORT int striker_main(int argc, char *argv[]);

/**
 * @brief Strike Manager Module
 *
 * Processes MAV_CMD_USER_1 (31010) commands from QGroundControl or mission plans,
 * extracts strike target coordinates (lat/lon from param5/param6, ID from param1),
 * and publishes them to the strike_target uORB topic for downstream processing.
 *
 *
 * This module uses a subscription callback mechanism to the 'vehicle_command' topic
 * and only runs when relevant commands are received.
 */
class StrikeManager : public ModuleBase<StrikeManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	StrikeManager();

	/** @see ModuleBase **/
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() **/
	int print_status() override;

	// Initialize the module
	bool init();

private:
	/**
	 * @brief Main Run function triggered by vehicle_command subscription callback
	 */
	void Run() override;

	/**
	 * @brief Process vehicle commands and filter for MAV_CMD_USER_1
	 *
	 * @param vehicle_command Pointer to received vehicle command (nullptr if none)
	 */
	void handle_vehicle_command(const vehicle_command_s *vehicle_command);

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};



	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};

	// Publications
	uORB::Publication<strike_target_s> _strike_target_pub{ORB_ID(strike_target)};


	// Statistics
	uint32_t _strike_target_count{0};

	// MAVLink log publisher
	orb_advert_t _mavlink_log_pub{nullptr};

	// Strike state tracking (for watchdog)
	bool _strike_active{false};

	// Helper to convert LLA to NED
	bool global_to_local(double lat, double lon, float alt, matrix::Vector3f &ned);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::STR_REC_ALT>) _param_str_rec_alt
	)

};
