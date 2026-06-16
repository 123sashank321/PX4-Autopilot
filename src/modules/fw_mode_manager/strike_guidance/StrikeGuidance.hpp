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
 * @file StrikeGuidance.hpp
 *
 * Fixed-wing APN strike guidance.
 *
 * Computes lateral and longitudinal setpoints for a terminal strike run
 * on a stationary NED ground target using:
 *   - 2D horizontal Proportional Navigation for lateral (roll) guidance
 *   - Elevation angle for pitch guidance (points nose directly at target)
 *
 * Outputs are consumed by fw_lateral_longitudinal_control which bypasses
 * NPFG (course=NAN) and TECS (pitch_direct + throttle_direct finite).
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/strike_target.h>
#include <uORB/topics/vehicle_local_position.h>

class StrikeGuidance
{
public:

	/**
	 * Output setpoints computed by compute().
	 * Pass directly to fixed_wing_lateral_setpoint and
	 * fixed_wing_longitudinal_setpoint topics.
	 */
	struct Output {
		float lateral_acceleration{0.0f}; ///< [m/s²] FRD lateral — 0 = wings level (no target)
		float pitch_direct{NAN};          ///< [rad]  NAN = let TECS hold altitude (no target)
		float throttle_direct{NAN};       ///< [0-1]  NAN = let TECS control throttle (no target)
		bool  valid{false};               ///< true = active target acquired and guidance running
	};

	StrikeGuidance()  = default;
	~StrikeGuidance() = default;

	/**
	 * @brief Compute strike guidance setpoints for this cycle.
	 *
	 * @param local_pos  Current vehicle local position (NED, m) and velocity (NED, m/s)
	 * @param yaw        Current vehicle yaw [rad] (NED, from North)
	 * @param airspeed_valid   True if EAS measurement is valid
	 * @param airspeed_eas     Equivalent airspeed [m/s] (used only for logging)
	 * @return Output    Setpoints to publish. valid=false if no target active.
	 */
	Output compute(const vehicle_local_position_s &local_pos,
		       float yaw,
		       bool  airspeed_valid,
		       float airspeed_eas);

private:

	// PN tuning
	static constexpr float PN_GAIN          = 4.0f;   ///< Navigation constant N
	static constexpr float MAX_ROLL_DEG     = 60.0f;  ///< Max lateral accel clamp [deg equiv]
	static constexpr float MAX_PITCH_DEG    = 45.0f;  ///< Max pitch command [deg]
	static constexpr float STRIKE_THROTTLE  = 1.0f;   ///< Full throttle during strike run

	uORB::Subscription _strike_target_sub{ORB_ID(strike_target)};

	float    _last_log_rd{-999.0f}; ///< Last Rd [m] at which debug was printed
};
