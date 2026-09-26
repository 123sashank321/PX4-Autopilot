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
		RECOVERY  = 3,  ///< abort from terminal dive
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
		bool  needs_loiter{false};       ///< true when IP orbit is needed
		float loiter_center_x{0.f};      ///< IP position for navigateLoiter()
		float loiter_center_y{0.f};      ///< IP position for navigateLoiter()
		float loiter_radius{80.f};       ///< [m] orbit radius (STR_LOITER_RAD)
		// Longitudinal
		float altitude{NAN};             ///< [m] AMSL for TECS — finite in INGRESS/ALIGNMENT level
		float airspeed{NAN};             ///< [m/s] EAS for TECS — finite in INGRESS/ALIGNMENT
		float pitch_direct{NAN};         ///< [rad] — finite in TERMINAL and INGRESS fast-descent
		float throttle_direct{NAN};      ///< [0-1] — 1.0 only in TERMINAL
		// Status
		bool  valid{false};              ///< false = no target, hold altitude/wings level
		bool  state_changed{false};      ///< true for exactly one cycle when state transitions
		State state{State::INGRESS};     ///< current phase (for logging/debugging)
	};

	StrikeGuidance()  = default;
	~StrikeGuidance() = default;

	/**
	 * @brief Compute strike guidance setpoints for this cycle.
	 *
	 * @param now        Current time — passed in rather than read internally via
	 *                   hrt_absolute_time() so timeout/timer logic (ALIGNMENT/IP-orbit
	 *                   timeouts, EKF-reset hold, target staleness) is deterministically
	 *                   testable, matching this codebase's existing convention
	 *                   (e.g. Hysteresis, ManualControl::processInput(now)).
	 * @param local_pos  Current vehicle local position + velocity (NED)
	 * @param airspeed_valid  True if EAS sensor reading is valid
	 * @param airspeed_eas    Equivalent airspeed [m/s]
	 * @param roll_lim_rad      FW_R_LIM, radians — bounds TERMINAL lateral_acceleration
	 * @param pitch_lim_min_rad FW_P_LIM_MIN, radians — nose-down bound (dive/pitch-bypass phases)
	 * @param pitch_lim_max_rad FW_P_LIM_MAX, radians — nose-up bound (recovery pull-out)
	 * @return Output    Ready-to-publish setpoint bundle. valid=false → no target.
	 */
	Output compute(hrt_abstime now,
		       const vehicle_local_position_s &local_pos,
		       float current_pitch,
		       bool  airspeed_valid,
		       float airspeed_eas,
		       float airspeed_max,
		       float roll_lim_rad,
		       float pitch_lim_min_rad,
		       float pitch_lim_max_rad);

	/// Reset to the freshly-constructed state. Called internally on abort, and by
	/// FixedWingModeManager on every STRIKE mode entry/exit. Also reports the
	/// closest-approach distance achieved this attempt, if TERMINAL was ever
	/// entered — see StrikeGuidance.cpp for why this isn't inline anymore.
	void reset();

	State currentState() const { return _state; }

private:

	// ── PN tuning constants ──────────────────────────────────────────────────
	// Roll/pitch bounds are NOT hardcoded here — they come from the caller as
	// roll_lim_rad/pitch_lim_min_rad/pitch_lim_max_rad (FW_R_LIM/FW_P_LIM_MIN/
	// FW_P_LIM_MAX). TERMINAL and RECOVERY both bypass TECS/NPFG and command
	// pitch/lateral_acceleration directly, so nothing downstream re-clamps
	// them to the airframe's configured limits — this library must respect
	// them itself.
	static constexpr float PN_GAIN         = 4.0f;  ///< Navigation constant N
	static constexpr float STRIKE_THROTTLE = 1.0f;  ///< Full throttle during APN dive

	// ── Ingress / Alignment thresholds ──────────────────────────────────────
	static constexpr float WP_ACCEPT_RADIUS      = 50.0f;   ///< [m] waypoint acceptance circle
	static constexpr float ALT_TOLERANCE          = 10.0f;   ///< [m] altitude must-be-met window
	static constexpr float DESCENT_PITCH_DEG      = 20.0f;   ///< [deg] pitch-down during fast INGRESS descent
	static constexpr float INGRESS_DESCENT_THRESH = 20.0f;   ///< [m] above this error: fast descent; below: altitude hold

	// ── Safety ────────────────────────────────────────────────────────────────
	static constexpr float ALIGNMENT_TIMEOUT_S   = 60.0f;   ///< [s] timeout for alignment phase
	static constexpr float IP_ORBIT_TIMEOUT_S    = 120.0f;  ///< [s] timeout for IP orbit
	static constexpr float VNE_MARGIN_MPS        = 5.0f;    ///< [m/s] below VNE to trigger recovery
	static constexpr float DESCENT_THROTTLE_FAST = 0.3f;    ///< reduced throttle during pitch bypass
	static constexpr float RECOVERY_RAMP_TIME_S  = 2.0f;    ///< [s] seconds to ramp to level
	static constexpr float RECOVERY_PITCH_TARGET = 0.05f;   ///< [rad] ~3 deg nose up
	static constexpr float RECOVERY_THROTTLE_IDLE = 0.3f;   ///< partial throttle during recovery

	// ── TERMINAL airspeed protection ─────────────────────────────────────────
	// The dive commands full throttle with TECS bypassed, so nothing else in
	// the loop limits airspeed. Throttle is ramped to zero over the last
	// VNE_MARGIN_MPS before FW_AIRSPD_MAX, and past that the dive is shallowed
	// as well, because throttle alone cannot arrest a 45 deg descent.
	static constexpr float DIVE_PITCH_LIMITED_DEG = 15.0f;  ///< [deg] max dive angle once over VNE

	// ── RECOVERY altitude floor ──────────────────────────────────────────────
	// A fixed 2 s ramp at idle from a 45 deg dive descends a long way. Below
	// RECOVERY_URGENT_ALT_M the pull-out is made faster, steeper and powered,
	// because at that height the nominal ramp would reach the ground first.
	static constexpr float RECOVERY_URGENT_ALT_M     = 60.0f;  ///< [m] above origin
	static constexpr float RECOVERY_RAMP_URGENT_S    = 0.7f;   ///< [s] faster ramp when low
	static constexpr float RECOVERY_PITCH_URGENT     = 0.26f;  ///< [rad] ~15 deg nose up
	static constexpr float RECOVERY_THROTTLE_URGENT  = 1.0f;   ///< full power to trade for height

	// ── State ────────────────────────────────────────────────────────────────
	State _state{State::INGRESS};
	int   _last_log_rd{INT_MIN};

	/// ALIGNMENT timeouts fall back to INGRESS; without a cap the aircraft can
	/// re-fly the IP forever when the approach geometry is unachievable.
	uint8_t _ingress_attempts{0};

	/// A designation older than this is treated as stale. strike_manager
	/// re-publishes the active target at 10 Hz, so a live strike refreshes well
	/// inside this window.
	static constexpr hrt_abstime TARGET_TIMEOUT_US = 2000000;  ///< [us] = 2 s

	/// How long to hold the last output after detecting an EKF reset before
	/// giving up and treating it like a lost target. strike_manager's own
	/// reset handling runs at 10 Hz, so a re-projected target should arrive
	/// well inside this window.
	static constexpr hrt_abstime RESET_HOLD_TIMEOUT_US = 500000;  ///< [us] = 500 ms

	// ── Timers & Recovery ────────────────────────────────────────────────────
	hrt_abstime _alignment_entry_time{0};
	hrt_abstime _ip_orbit_entry_time{0};
	hrt_abstime _recovery_start_time{0};
	float       _recovery_pitch_start{NAN}; ///< [rad] pitch at RECOVERY entry, seeded from the live attitude

	// ── EKF reset handling ──────────────────────────────────────────────────
	// vehicle_local_position is re-referenced on a reset; strike_target's x/y/z
	// (computed by strike_manager against the OLD frame) becomes briefly
	// inconsistent with local_pos (already in the NEW frame). Detecting this
	// here — rather than relying solely on strike_manager's slower watchdog —
	// lets us freeze on the very next control-loop cycle instead of flying a
	// stale-vs-fresh mismatch for up to strike_manager's 100 ms tick period.
	bool        _reset_tracking_init{false};
	uint8_t     _last_xy_reset{0};
	uint8_t     _last_z_reset{0};
	bool        _reset_pending{false};
	hrt_abstime _reset_detected_time{0};
	Output      _held_output{};  ///< last good output, replayed while a reset is pending

	// ── Closest-approach tracking ────────────────────────────────────────────
	// Tracked only during TERMINAL (a fresh, actively-guided-toward target) —
	// not RECOVERY, where the target may already be stale/lost, i.e. exactly
	// why RECOVERY was entered. Reported once in reset(). NAN = no TERMINAL
	// entry happened this attempt, nothing to report.
	hrt_abstime _terminal_entry_time{0};
	float       _cpa_3d{NAN};         ///< [m] minimum 3D distance to target seen in TERMINAL
	float       _cpa_horizontal{NAN}; ///< [m] horizontal distance at that same instant

	// ── uORB ─────────────────────────────────────────────────────────────────
	uORB::Subscription _strike_target_sub{ORB_ID(strike_target)};
};
