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
#include <uORB/topics/strike_target.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command_ack.h>

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
	vehicle_status_s _vehicle_status{};  ///< latest, refreshed every Run() before commands are handled
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	// STR_* parameters are only re-read when this fires. Without it,
	// updateParams() runs once in init() and every later QGC change is
	// silently ignored until reboot.
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Publications
	uORB::Publication<strike_target_s> _strike_target_pub{ORB_ID(strike_target)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};


	// Statistics
	uint32_t _strike_target_count{0};

	// MAVLink log publisher
	orb_advert_t _mavlink_log_pub{nullptr};

	// Strike state tracking (for watchdog)
	bool _strike_active{false};

	// Time the strike was commanded. The watchdog must not fire until Commander
	// has had a chance to complete the mode switch, otherwise a strike can
	// self-abort within one 100 ms tick of being requested.
	hrt_abstime _strike_requested_time{0};
	static constexpr hrt_abstime STRIKE_MODE_GRACE_US = 1_s;

	// Target retained in GEODETIC form. vehicle_local_position is re-referenced
	// on an EKF reset, which silently invalidates any previously computed NED
	// target; keeping lat/lon lets us re-project instead of flying to a stale
	// point. See the reset handling in Run().
	// Last published target, retained so the active strike can be re-emitted
	// as a heartbeat. strike_target was previously published only on the
	// designation event, so the guidance had no way to tell a live target from
	// one left over from a previous run.
	strike_target_s _active_target{};

	double _target_lat{0.0};
	double _target_lon{0.0};
	float  _target_alt{0.f};
	uint8_t _last_xy_reset{0};
	uint8_t _last_z_reset{0};

	// Send the real command result. mavlink_receiver acks 31010 optimistically
	// before this module has run, so a failure here would otherwise be reported
	// to the GCS as success.
	void send_command_ack(const vehicle_command_s &cmd, uint8_t result);

	// Helper: convert geodetic to local NED
	bool global_to_local(double lat, double lon, float alt, matrix::Vector3f &ned);

	// Compute IP / AHP geometry and fill strike_target_s fields
	void compute_geometry(const matrix::Vector3f &target_ned,
			      const matrix::Vector3f &vehicle_ned,
			      strike_target_s &msg);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::STR_REC_ALT>)     _param_str_rec_alt,
		(ParamFloat<px4::params::STR_IP_ALT>)      _param_str_ip_alt,
		(ParamFloat<px4::params::STR_DIVE_ANG>)    _param_str_dive_ang,
		(ParamFloat<px4::params::STR_SETTLE_T>)    _param_str_settle_t,
		(ParamFloat<px4::params::STR_CRUISE_SPD>)  _param_str_cruise_spd,
		(ParamFloat<px4::params::STR_DESCENT_ANG>) _param_str_descent_ang,
		(ParamFloat<px4::params::STR_LOITER_RAD>)  _param_str_loiter_rad,
		(ParamInt<px4::params::STR_MAX_ATTEMPT>)   _param_str_max_attempt,
		(ParamFloat<px4::params::STR_DIVE_VNE>)    _param_str_dive_vne,
		(ParamFloat<px4::params::STR_DIVE_PITCH>) _param_str_dive_pitch_lim
	)

};
