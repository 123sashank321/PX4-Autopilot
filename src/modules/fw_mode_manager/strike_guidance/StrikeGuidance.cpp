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

#include "StrikeGuidance.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>

using math::constrain;
using math::radians;
using math::degrees;
using matrix::Vector2f;
using matrix::Vector3f;

StrikeGuidance::Output
StrikeGuidance::compute(const vehicle_local_position_s &local_pos,
			float yaw,
			float current_pitch,
			bool  airspeed_valid,
			float airspeed_eas,
			float airspeed_max)
{
	// ── 1. Fetch target ──────────────────────────────────────────────────────
	strike_target_s target{};
	const bool target_valid = _strike_target_sub.copy(&target) && target.active;

	if (!target_valid) {
		if (_state == State::TERMINAL) {
			// Dangerous: mid-dive abort. Enter RECOVERY instead of hard reset.
			if (_state != State::RECOVERY) {
				_recovery_pitch_start  = NAN;  // caller must seed from current pitch
				_recovery_start_time   = hrt_absolute_time();
				_state = State::RECOVERY;
				PX4_WARN("Strike: target lost in TERMINAL → RECOVERY");
			}
			// Fall through to RECOVERY case below
		} else if (_state != State::RECOVERY) {
			reset();
			return Output{};
		}
	}

	// ── 2. Common geometry ───────────────────────────────────────────────────
	const Vector2f pos2d(local_pos.x, local_pos.y);
	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);

	const Vector2f target2d(target.x, target.y);
	const Vector2f ip2d(target.ip_x, target.ip_y);
	const Vector2f ahp2d(target.ahp_x, target.ahp_y);

	const float dist_to_ip     = (ip2d  - pos2d).norm();
	const float dist_to_target = (target2d - pos2d).norm();

	// Altitude above home in meters (positive = above)
	// local_pos.z is NED (negative above home), local_pos.ref_alt is home AMSL
	const float alt_amsl       = local_pos.ref_alt + (-local_pos.z);
	const float ip_alt_amsl    = local_pos.ref_alt + (-target.ip_z);  // ip_z negative → above home
	const float alt_error      = alt_amsl - ip_alt_amsl;              // positive = too high

	Output out{};
	out.valid = true;
	out.state = _state;

	// ── 3. State machine ─────────────────────────────────────────────────────
	switch (_state) {

	// ─────────────────────────────────────────────────────────────────────────
	case State::INGRESS: {
	// Fly to the Initial Point (IP).
	//
	// Descent strategy (two-tier):
	//   A. Well above IP alt (err > INGRESS_DESCENT_THRESH):
	//      pitch_direct = -DESCENT_PITCH_DEG  (bypasses TECS pitch, ~10-20 m/s sink)
	//      altitude = NAN, throttle_direct = NAN  (TECS still controls throttle)
	//   B. Near IP alt (err ≤ INGRESS_DESCENT_THRESH):
	//      altitude = ip_alt_amsl, pitch_direct = NAN  (TECS altitude hold)
	//
	// Lateral: NPFG course toward IP; orbit at IP when altitude not yet met.
	// ─────────────────────────────────────────────────────────────────────────

		if (alt_error > INGRESS_DESCENT_THRESH) {
			// Airspeed guard: if approaching VNE, suspend pitch bypass
			const bool overspeed_risk = airspeed_valid &&
						    (airspeed_eas > airspeed_max - VNE_MARGIN_MPS);

			if (overspeed_risk) {
				// Revert to TECS altitude hold at current altitude to bleed speed
				PX4_WARN("Strike INGRESS: overspeed risk (%.1f m/s) → suspending pitch bypass",
					      (double)airspeed_eas);
				out.altitude     = alt_amsl;      // hold current altitude
				out.pitch_direct = NAN;           // TECS takes over pitch
				// throttle_direct stays NAN → TECS controls throttle

			} else {
				// Safe to descend: pitch bypass, reduced throttle
				out.altitude          = NAN;
				out.pitch_direct      = -radians(DESCENT_PITCH_DEG);
				out.throttle_direct   = DESCENT_THROTTLE_FAST;
			}
		} else {
			// Near IP altitude: TECS altitude hold, release pitch bypass
			out.altitude     = ip_alt_amsl;
			out.pitch_direct = NAN;
		}

		if (dist_to_ip > WP_ACCEPT_RADIUS) {
			// ── A. En-route to IP: fly straight toward it
			out.course = atan2f(ip2d(1) - pos2d(1), ip2d(0) - pos2d(0));

		} else if (fabsf(alt_error) > ALT_TOLERANCE) {
			// ── B. At IP, altitude not yet met → CCW orbit
			if (_ip_orbit_entry_time == 0) {
				_ip_orbit_entry_time = hrt_absolute_time();
				PX4_INFO("Strike: entering IP orbit (alt_err=%.1fm)", (double)alt_error);
			}

			const float orbit_elapsed = (hrt_absolute_time() - _ip_orbit_entry_time) * 1e-6f;

			if (orbit_elapsed > IP_ORBIT_TIMEOUT_S) {
				PX4_WARN("Strike: IP orbit timeout (%.0fs, alt_err=%.1fm) → aborting",
					 (double)orbit_elapsed, (double)alt_error);
				reset();
				return Output{};  // valid=false → caller holds altitude
			}

			out.needs_loiter    = true;
			out.loiter_center_x = target.ip_x;
			out.loiter_center_y = target.ip_y;
			out.course          = NAN;

		} else {
			// ── C. IP reached at correct altitude → ALIGNMENT
			_state = State::ALIGNMENT;
			_ip_orbit_entry_time = 0;  // reset timer on success
			out.state_changed = true;
			PX4_INFO("Strike: IP reached (d=%.0fm alt_err=%.1fm) → ALIGNMENT",
				 (double)dist_to_ip, (double)alt_error);
			out.course       = atan2f(ahp2d(1) - ip2d(1), ahp2d(0) - ip2d(0));
			out.altitude     = ip_alt_amsl;
			out.pitch_direct = NAN;
		}

		break;
	}


	// ─────────────────────────────────────────────────────────────────────────
	case State::ALIGNMENT: {
	// Fly on the fixed attack bearing from IP to AHP.
	// ─────────────────────────────────────────────────────────────────────────

		// Record entry time
		if (_alignment_entry_time == 0) {
			_alignment_entry_time = hrt_absolute_time();
		}

		out.course   = atan2f(ahp2d(1) - ip2d(1), ahp2d(0) - ip2d(0));
		out.altitude = ip_alt_amsl;

		const float elapsed = (hrt_absolute_time() - _alignment_entry_time) * 1e-6f;

		if (dist_to_target <= target.x_kinematic) {
			_state = State::TERMINAL;
			_alignment_entry_time = 0;   // reset for next use
			out.state_changed = true;
			PX4_INFO("Strike: AHP crossed (d=%.0fm xk=%.0fm) → TERMINAL",
				 (double)dist_to_target, (double)target.x_kinematic);

		} else if (elapsed > ALIGNMENT_TIMEOUT_S) {
			// Geometry failure: abort strike, loiter over IP
			PX4_WARN("Strike: ALIGNMENT timeout (%.0fs) → INGRESS loiter", (double)elapsed);
			_alignment_entry_time = 0;
			reset();  // back to INGRESS; caller sees valid=true, state=INGRESS next cycle
		}

		break;
	}

	// ─────────────────────────────────────────────────────────────────────────
	case State::TERMINAL: {
	// 2D horizontal Proportional Navigation (lateral) +
	// Elevation angle to target (pitch) + full throttle.
	// NPFG and TECS are both fully bypassed.
	// ─────────────────────────────────────────────────────────────────────────

		const Vector3f target_ned(target.x, target.y, target.z);
		const Vector3f R     = target_ned - pos;
		const float    R_mag = math::max(R.norm(), 0.5f);

		// 2D PN in horizontal plane
		const Vector2f R2d(R(0), R(1));
		const float    R2d_mag = math::max(R2d.norm(), 0.5f);
		const Vector2f vel2d(vel(0), vel(1));
		const Vector2f Rdot2d = -vel2d;  // stationary target

		const float omega2d = (R2d(0) * Rdot2d(1) - R2d(1) * Rdot2d(0)) / (R2d_mag * R2d_mag);
		const float V2d     = math::max(vel2d.norm(), 1.0f);
		const float a_horiz = PN_GAIN * V2d * omega2d;

		// Perpendicular-to-LOS NED acceleration
		const float a_NED_N = a_horiz * (-R2d(1) / R2d_mag);
		const float a_NED_E = a_horiz * ( R2d(0) / R2d_mag);

		// Project onto body lateral (FRD-Y): Body-Y in NED = (-sin_yaw, cos_yaw)
		const float a_lateral = constrain(
					  -a_NED_N * sinf(yaw) + a_NED_E * cosf(yaw),
					  -tanf(radians(MAX_ROLL_DEG)) * CONSTANTS_ONE_G,
					   tanf(radians(MAX_ROLL_DEG)) * CONSTANTS_ONE_G);

		// Elevation angle: negative when target below → nose down ✅
		const float pitch = constrain(atan2f(-R(2), R2d_mag),
					      radians(-MAX_PITCH_DEG), radians(MAX_PITCH_DEG));

		// Course = NAN bypasses NPFG; pitch_direct + throttle_direct bypass TECS
		out.course              = NAN;
		out.lateral_acceleration = a_lateral;
		out.altitude            = NAN;         // TECS bypassed
		out.pitch_direct        = pitch;
		out.throttle_direct     = STRIKE_THROTTLE;

		// Debug: log once per 5m Rd milestone
		const int rd_idx = static_cast<int>(floorf(R(2) / 5.0f));

		if (rd_idx != _last_log_rd) {
			_last_log_rd = rd_idx;
			const float V_closing = vel.dot(R) / R_mag;
			PX4_INFO("Strike TERMINAL: Rh=%.0fm Rd=%.0fm Vc=%.1fm/s a_lat=%.2f pitch=%.1fdeg",
				 (double)R2d_mag, (double)R(2),
				 (double)V_closing,
				 (double)a_lateral,
				 (double)degrees(pitch));
		}

		break;
	}

	// ─────────────────────────────────────────────────────────────────────────
	case State::RECOVERY: {
	// Graceful abort from TERMINAL dive: pitch ramp and idle throttle
	// ─────────────────────────────────────────────────────────────────────────
		const hrt_abstime now_us = hrt_absolute_time();
		const float t = math::constrain(
			(now_us - _recovery_start_time) * 1e-6f / RECOVERY_RAMP_TIME_S,
			0.0f, 1.0f);

		// Seed start pitch on first cycle
		if (!PX4_ISFINITE(_recovery_pitch_start)) {
			_recovery_pitch_start = current_pitch;
		}

		// Linearly ramp pitch from dive angle toward level
		out.pitch_direct    = _recovery_pitch_start + t * (RECOVERY_PITCH_TARGET - _recovery_pitch_start);
		out.throttle_direct = RECOVERY_THROTTLE_IDLE;
		out.course          = NAN;   // wings level, no course demand
		out.lateral_acceleration = 0.f;
		out.altitude        = NAN;   // TECS still bypassed during ramp
		out.valid           = true;
		out.state           = State::RECOVERY;

		if (t >= 1.0f) {
			// Ramp complete, hand back to normal altitude hold
			reset();
			PX4_INFO("Strike: RECOVERY complete → aborting");
		}

		break;
	}
	} // end switch

	out.state = _state;
	return out;
}
