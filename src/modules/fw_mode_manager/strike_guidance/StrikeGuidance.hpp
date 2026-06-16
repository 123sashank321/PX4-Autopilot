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
 * Fixed-wing three-phase strike guidance.
 *
 * Phase 1 — INGRESS:   NPFG course to Initial Point (IP) + TECS altitude hold
 * Phase 2 — ALIGNMENT: NPFG course on IP→AHP attack bearing + TECS altitude hold
 * Phase 3 — TERMINAL:  2D horizontal PN (lateral) + elevation angle (pitch), TECS bypassed
 *
 * IP and AHP coordinates are pre-computed by the striker module at command
 * reception and carried in the strike_target uORB message.
 */

#pragma once

#include <climits>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/strike_target.h>
#include <uORB/topics/vehicle_local_position.h>

class StrikeGuidance
{
public:

	// ── Strike state machine ─────────────────────────────────────────────────
	enum class State : uint8_t {
		INGRESS   = 0,  ///< Fly to Initial Point at cruise altitude (NPFG + TECS)
		ALIGNMENT = 1,  ///< Fly IP→AHP on attack bearing (NPFG + TECS)
		TERMINAL  = 2,  ///< APN terminal dive (lateral_accel + pitch_direct)
	};

	// ── Setpoint bundle returned by compute() ───────────────────────────────
	//
	// control_strike() in FixedWingModeManager publishes these directly:
	//
	//  INGRESS / ALIGNMENT:
	//    lateral_sp.course              = course          (finite → NPFG active)
	//    lateral_sp.lateral_acceleration = 0              (unused)
	//    long_sp.altitude               = altitude        (finite → TECS active)
	//    long_sp.pitch_direct           = NAN             (unused)
	//    long_sp.throttle_direct        = NAN             (TECS controls throttle)
	//
	//  TERMINAL:
	//    lateral_sp.course              = NAN             (NPFG bypassed)
	//    lateral_sp.lateral_acceleration = lateral_acceleration
	//    long_sp.altitude               = NAN             (TECS bypassed)
	//    long_sp.pitch_direct           = pitch_direct
	//    long_sp.throttle_direct        = STRIKE_THROTTLE (1.0)
	//
	struct Output {
		// Lateral
		float course{NAN};               ///< [rad] NED bearing — finite → NPFG active
		float lateral_acceleration{0.f}; ///< [m/s²] FRD — used only in TERMINAL
		// Longitudinal
		float altitude{NAN};             ///< [m] AMSL for TECS — finite in INGRESS/ALIGNMENT
		float pitch_direct{NAN};         ///< [rad] — finite only in TERMINAL
		float throttle_direct{NAN};      ///< [0-1] — 1.0 only in TERMINAL
		// Status
		bool  valid{false};              ///< false = no target, hold altitude/wings level
		State state{State::INGRESS};     ///< current phase (for logging/debugging)
	};

	StrikeGuidance()  = default;
	~StrikeGuidance() = default;

	/**
	 * @brief Compute strike guidance setpoints for this cycle.
	 *
	 * @param local_pos  Current vehicle local position + velocity (NED)
	 * @param yaw        Current vehicle yaw [rad] (NED from North)
	 * @param airspeed_valid  True if EAS sensor reading is valid
	 * @param airspeed_eas    Equivalent airspeed [m/s]
	 * @return Output    Ready-to-publish setpoint bundle. valid=false → no target.
	 */
	Output compute(const vehicle_local_position_s &local_pos,
		       float yaw,
		       bool  airspeed_valid,
		       float airspeed_eas);

	/// Reset state machine (e.g. on abort or re-designation)
	void reset() { _state = State::INGRESS; _last_log_rd = INT_MIN; }

	State currentState() const { return _state; }

private:

	// ── PN tuning constants ──────────────────────────────────────────────────
	static constexpr float PN_GAIN         = 4.0f;  ///< Navigation constant N
	static constexpr float MAX_ROLL_DEG    = 60.0f; ///< Max lateral accel equiv [deg]
	static constexpr float MAX_PITCH_DEG   = 45.0f; ///< Max pitch command [deg]
	static constexpr float STRIKE_THROTTLE = 1.0f;  ///< Full throttle during APN dive

	// ── Ingress / Alignment thresholds ──────────────────────────────────────
	static constexpr float WP_ACCEPT_RADIUS = 50.0f;  ///< [m] waypoint acceptance circle
	static constexpr float ALT_TOLERANCE    = 10.0f;  ///< [m] altitude must-be-met window
	static constexpr float LOITER_RADIUS    = 150.0f; ///< [m] holding orbit radius at IP

	// ── State ────────────────────────────────────────────────────────────────
	State _state{State::INGRESS};
	int   _last_log_rd{INT_MIN}; ///< Last Rd milestone index logged (TERMINAL phase)

	// ── uORB ─────────────────────────────────────────────────────────────────
	uORB::Subscription _strike_target_sub{ORB_ID(strike_target)};
};
