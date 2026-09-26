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
StrikeGuidance::compute(hrt_abstime now,
			const vehicle_local_position_s &local_pos,
			float current_pitch,
			bool  airspeed_valid,
			float airspeed_eas,
			float airspeed_max,
			float roll_lim_rad,
			float pitch_lim_min_rad,
			float pitch_lim_max_rad)
{
	// ── 0. Estimator sanity ──────────────────────────────────────────────────
	// Every phase below is pure dead-reckoning against vehicle_local_position.
	// If the estimate is not valid the geometry is meaningless, so hold rather
	// than command a dive from garbage state.
	if (!local_pos.xy_valid || !local_pos.z_valid || !local_pos.v_xy_valid) {
		if (_state == State::TERMINAL) {
			// Mid-dive: pull out rather than freeze.
			if (!PX4_ISFINITE(_recovery_pitch_start)) {
				// Seed from measured attitude, but clamp: current_pitch can be
				// outside the configured envelope (tracking lag, gust) even
				// though every commanded pitch is bounded to it.
				_recovery_pitch_start = constrain(current_pitch, pitch_lim_min_rad, pitch_lim_max_rad);
				_recovery_start_time  = now;
			}

			_state = State::RECOVERY;
			PX4_WARN("Strike: local position invalid in TERMINAL → RECOVERY");

		} else if (_state != State::RECOVERY) {
			reset();
			return Output{};
		}
	}

	// ── 1. Fetch target ──────────────────────────────────────────────────────
	strike_target_s target{};
	const bool got_target = _strike_target_sub.copy(&target);

	// strike_manager re-publishes an active designation at 10 Hz, so anything
	// older than TARGET_TIMEOUT_US is leftover state, not a live command.
	const bool target_fresh = got_target
				  && ((now - target.timestamp) < TARGET_TIMEOUT_US);
	const bool target_valid = target_fresh && target.active;

	// ── 1b. EKF reset handling ────────────────────────────────────────────────
	// Detect a position-reset before trusting target/local_pos together: right
	// after a reset, target.x/y/z (computed by strike_manager against the OLD
	// frame) and local_pos (already in the NEW frame) are momentarily
	// inconsistent. Freeze on the last good output until strike_manager's
	// re-projected target arrives, rather than steering on the mismatch.
	if (!_reset_tracking_init) {
		_last_xy_reset = local_pos.xy_reset_counter;
		_last_z_reset  = local_pos.z_reset_counter;
		_reset_tracking_init = true;
	}

	if (!_reset_pending &&
	    (local_pos.xy_reset_counter != _last_xy_reset || local_pos.z_reset_counter != _last_z_reset)) {
		_reset_pending = true;
		_reset_detected_time = now;
		PX4_WARN("Strike: EKF reset detected, holding until target re-projected");
	}

	if (_reset_pending) {
		if (target_valid && target.timestamp > _reset_detected_time) {
			// strike_manager has re-projected and republished — resume normal guidance.
			_reset_pending = false;
			_last_xy_reset = local_pos.xy_reset_counter;
			_last_z_reset  = local_pos.z_reset_counter;

		} else if ((now - _reset_detected_time) > RESET_HOLD_TIMEOUT_US) {
			// No re-projected target arrived in time (e.g. strike_manager had no
			// global reference to re-project from) — treat like a lost target.
			_reset_pending = false;
			_last_xy_reset = local_pos.xy_reset_counter;
			_last_z_reset  = local_pos.z_reset_counter;

			if (_state == State::TERMINAL) {
				_recovery_pitch_start = NAN;   // seeded from live attitude below
				_recovery_start_time  = now;
				_state = State::RECOVERY;
				PX4_WARN("Strike: reset hold timeout in TERMINAL → RECOVERY");

			} else if (_state != State::RECOVERY) {
				reset();
				return Output{};
			}

			// else: fall through into the (now RECOVERY) state-machine below.

		} else {
			return _held_output;   // freeze last good setpoint, don't fly on the mismatch
		}
	}

	if (!target_valid) {
		if (_state == State::TERMINAL) {
			// Dangerous: mid-dive abort. Enter RECOVERY instead of a hard reset.
			_recovery_pitch_start = NAN;   // seeded from live attitude below
			_recovery_start_time  = now;
			_state = State::RECOVERY;
			PX4_WARN("Strike: target lost in TERMINAL → RECOVERY");
			// Fall through to the RECOVERY case below

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

	// Command the cruise airspeed the IP/AHP geometry was computed with.
	// x_buffer = cruise_spd * settle_t in strike_manager assumes the vehicle
	// actually flies at STR_CRUISE_SPD; leaving this NAN let TECS pick its own
	// trim speed, so planned and flown geometry diverged. TERMINAL overrides
	// this back to NAN below, where TECS is bypassed entirely.
	if (PX4_ISFINITE(target.cruise_speed) && (target.cruise_speed > 1.f)) {
		out.airspeed = target.cruise_speed;
	}

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
					// Safe to descend: pitch bypass, reduced throttle.
					// Descent angle comes from STR_DESCENT_ANG via the target
					// message; fall back to the built-in default if unset.
					const float descent = PX4_ISFINITE(target.descent_angle) && (target.descent_angle > 0.01f)
							      ? target.descent_angle
							      : radians(DESCENT_PITCH_DEG);
					out.altitude          = NAN;
					out.pitch_direct      = -descent;
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
					_ip_orbit_entry_time = now;
					PX4_INFO("Strike: entering IP orbit (alt_err=%.1fm)", (double)alt_error);
				}

				const float orbit_elapsed = (now - _ip_orbit_entry_time) * 1e-6f;

				if (orbit_elapsed > IP_ORBIT_TIMEOUT_S) {
					PX4_WARN("Strike: IP orbit timeout (%.0fs, alt_err=%.1fm) → aborting",
						 (double)orbit_elapsed, (double)alt_error);
					reset();
					return Output{};  // valid=false → caller holds altitude
				}

				out.needs_loiter    = true;
				out.loiter_center_x = target.ip_x;
				out.loiter_center_y = target.ip_y;
				out.loiter_radius   = (PX4_ISFINITE(target.loiter_radius) && target.loiter_radius > 10.f)
						      ? target.loiter_radius : 80.f;
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
				_alignment_entry_time = now;
			}

			out.course   = atan2f(ahp2d(1) - ip2d(1), ahp2d(0) - ip2d(0));
			out.altitude = ip_alt_amsl;

			const float elapsed = (now - _alignment_entry_time) * 1e-6f;

			if (dist_to_target <= target.x_kinematic) {
				_state = State::TERMINAL;
				_alignment_entry_time = 0;   // reset for next use
				_terminal_entry_time  = now;
				out.state_changed = true;
				PX4_INFO("Strike: AHP crossed (d=%.0fm xk=%.0fm) → TERMINAL",
					 (double)dist_to_target, (double)target.x_kinematic);

			} else if (elapsed > ALIGNMENT_TIMEOUT_S) {
				// Geometry failure. Fall back to INGRESS, but bound the number of
				// retries: reset() would clear the counter and allow the aircraft
				// to re-fly the IP indefinitely on an unachievable approach.
				_alignment_entry_time = 0;
				_ingress_attempts++;

				const uint8_t max_attempts = (target.max_attempts > 0) ? target.max_attempts : 3;

				if (_ingress_attempts >= max_attempts) {
					PX4_ERR("Strike: ALIGNMENT failed %u times → giving up",
						(unsigned)_ingress_attempts);
					reset();
					return Output{};   // valid=false → caller holds altitude
				}

				PX4_WARN("Strike: ALIGNMENT timeout (%.0fs) → INGRESS (attempt %u/%u)",
					 (double)elapsed, (unsigned)_ingress_attempts, (unsigned)max_attempts);
				_state = State::INGRESS;
				_ip_orbit_entry_time = 0;
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

			// Closest-approach tracking for post-flight accuracy analysis.
			// Uses the raw (unfloored) norms — R_mag/R2d_mag above are floored
			// to 0.5m to avoid a div-by-zero in the PN math below, which would
			// under-report a genuine sub-0.5m hit as exactly 0.5m here.
			const float r3d_now = R.norm();
			const float r2d_now = R2d.norm();

			if (!PX4_ISFINITE(_cpa_3d) || r3d_now < _cpa_3d) {
				_cpa_3d         = r3d_now;
				_cpa_horizontal = r2d_now;
			}

			const float omega2d = (R2d(0) * Rdot2d(1) - R2d(1) * Rdot2d(0)) / (R2d_mag * R2d_mag);
			const float V2d     = math::max(vel2d.norm(), 1.0f);
			const float a_horiz = PN_GAIN * V2d * omega2d;

			// PURE PROPORTIONAL NAVIGATION.
			//
			// a_horiz = N * V * lambda_dot is already the pure-PN command, defined
			// as an acceleration PERPENDICULAR TO THE VELOCITY VECTOR, and that is
			// exactly what the consumer produces: FwLateralLongitudinalControl maps
			// this field straight to bank via roll = atan(a_lat / g), and a banked
			// aircraft accelerates perpendicular to its flight path. So the command
			// is issued directly - no frame projection is needed or correct.
			//
			// The previous implementation decomposed a_horiz into a NED vector
			// perpendicular to the LOS and then projected that onto body-Y using
			// YAW. Expanding that algebra gives  a_horiz * cos(lead angle w.r.t.
			// heading), which introduced two errors at once:
			//   1. a spurious cos(lead) attenuation of the PN gain, worst exactly
			//      when the lead angle is large and the command matters most;
			//   2. a dependency on yaw rather than course, so in any crosswind the
			//      crab angle rotated the command and biased the miss distance.
			const float a_lateral = constrain(a_horiz,
							  -tanf(roll_lim_rad) * CONSTANTS_ONE_G,
							  tanf(roll_lim_rad) * CONSTANTS_ONE_G);

			// ── Airspeed protection ──────────────────────────────────────────
			// This phase commands full throttle with TECS fully bypassed, so
			// nothing else in the loop limits speed. The INGRESS overspeed guard
			// above does NOT apply here. Ramp throttle to zero over the last
			// VNE_MARGIN_MPS, and once past VNE shallow the dive too - throttle
			// alone cannot arrest a steep descent.
			//
			// Deliberately NOT airspeed_max (FW_AIRSPD_MAX): that is the normal
			// cruise ceiling, and a full-throttle TERMINAL dive is expected to
			// exceed it almost immediately - using it here made this "emergency"
			// shallowing active for the entire dive by default, not as a genuine
			// overspeed fallback, and the aircraft never reached the commanded
			// dive angle. target.dive_vne (STR_DIVE_VNE) is a dedicated, higher
			// dive-only ceiling; fall back to airspeed_max only if it was never
			// set (e.g. an older strike_target publisher).
			const float dive_vne = (PX4_ISFINITE(target.dive_vne) && target.dive_vne > 1.f)
					       ? target.dive_vne : airspeed_max;

			float throttle = STRIKE_THROTTLE;

			// Nose-down bound for the dive. Deliberately NOT pitch_lim_min_rad
			// (FW_P_LIM_MIN) as the primary source: that governs the airframe's
			// everyday cruise-flight envelope (e.g. -15 deg on the stock
			// advanced_plane airframes) and was never tuned expecting a full-
			// power terminal dive to also live inside it - reusing it here
			// capped every strike to the same shallow angle as a normal
			// descent. target.dive_pitch_lim (STR_DIVE_PITCH) is a
			// dedicated, separately-configured dive limit; fall back to
			// FW_P_LIM_MIN only if it was never set (e.g. an older
			// strike_target publisher). math::max() of two negative numbers
			// picks whichever is closer to level, so the VNE shallowing below
			// can only tighten this further — never loosen past whichever
			// floor was selected here.
			const float dive_pitch_lower = (PX4_ISFINITE(target.dive_pitch_lim) && target.dive_pitch_lim > 0.01f)
						       ? -target.dive_pitch_lim : pitch_lim_min_rad;
			float pitch_lower = dive_pitch_lower;
			const float pitch_upper = pitch_lim_max_rad;

			if (airspeed_valid && PX4_ISFINITE(dive_vne) && (dive_vne > VNE_MARGIN_MPS)) {
				const float ramp_start = dive_vne - VNE_MARGIN_MPS;

				if (airspeed_eas > ramp_start) {
					const float scale = constrain((dive_vne - airspeed_eas) / VNE_MARGIN_MPS, 0.f, 1.f);
					throttle = STRIKE_THROTTLE * scale;

					if (airspeed_eas > dive_vne) {
						pitch_lower = math::max(pitch_lower, -radians(DIVE_PITCH_LIMITED_DEG));
					}
				}
			}

			// Elevation angle: negative when target below → nose down
			const float pitch = constrain(atan2f(-R(2), R2d_mag), pitch_lower, pitch_upper);

			// Course = NAN bypasses NPFG; pitch_direct + throttle_direct bypass TECS
			out.course              = NAN;
			out.lateral_acceleration = a_lateral;
			out.altitude            = NAN;         // TECS bypassed
			out.airspeed            = NAN;         // TECS bypassed
			out.pitch_direct        = pitch;
			out.throttle_direct     = throttle;

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
			const hrt_abstime now_us = now;

			// Altitude floor: a fixed 2 s ramp at idle from a steep dive descends
			// a long way. When low, pull out faster, steeper and under power -
			// otherwise the nominal ramp reaches the ground before it completes.
			// Height-above-ground, best available source, in priority order:
			//   1. A real distance sensor reading.
			//   2. Height above the target's elevation. RECOVERY is only ever
			//      reached from TERMINAL, which only triggers within dive-reach
			//      distance of the target, so target.z is always geographically
			//      close by here — a much closer proxy to true terrain than height
			//      above the launch point, especially when diving toward terrain
			//      higher than home. Gated on got_target (not target_valid/
			//      freshness): a stale-but-once-real target elevation is still far
			//      better than no ground reference at all, and this matters most in
			//      exactly the "target lost" RECOVERY-entry path.
			//   3. Last resort: height above the local origin — no ground
			//      reference at all (e.g. RECOVERY entered from local-position loss
			//      before any target was ever received).
			float height_agl;

			if (local_pos.dist_bottom_valid && PX4_ISFINITE(local_pos.dist_bottom)) {
				height_agl = local_pos.dist_bottom;

			} else if (got_target && PX4_ISFINITE(target.z)) {
				height_agl = target.z - local_pos.z;

			} else {
				height_agl = -local_pos.z;
			}

			const bool  urgent = (height_agl < RECOVERY_URGENT_ALT_M);

			const float ramp_time    = urgent ? RECOVERY_RAMP_URGENT_S   : RECOVERY_RAMP_TIME_S;
			const float pitch_target = urgent ? RECOVERY_PITCH_URGENT    : RECOVERY_PITCH_TARGET;
			const float rec_throttle = urgent ? RECOVERY_THROTTLE_URGENT : RECOVERY_THROTTLE_IDLE;

			const float t = math::constrain(
						(now_us - _recovery_start_time) * 1e-6f / ramp_time,
						0.0f, 1.0f);

			// Seed start pitch on first cycle. Clamp: current_pitch is measured
			// attitude, not a commanded value, and can be outside the configured
			// envelope (tracking lag, gust) even though every pitch this module
			// commands is bounded.
			if (!PX4_ISFINITE(_recovery_pitch_start)) {
				_recovery_pitch_start = constrain(current_pitch, pitch_lim_min_rad, pitch_lim_max_rad);
			}

			// Linearly ramp pitch from dive angle toward the recovery target
			out.pitch_direct    = constrain(_recovery_pitch_start + t * (pitch_target - _recovery_pitch_start),
							pitch_lim_min_rad, pitch_lim_max_rad);
			out.throttle_direct = rec_throttle;
			out.course          = NAN;   // wings level, no course demand
			out.lateral_acceleration = 0.f;
			out.altitude        = NAN;   // TECS still bypassed during ramp
			out.airspeed        = NAN;
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
	_held_output = out;
	return out;
}

void StrikeGuidance::reset()
{
	// Report the closest-approach distance achieved this attempt, if TERMINAL
	// was ever entered. hrt_absolute_time() here (not an injected `now`) is
	// deliberate: this is a log-only timestamp, not control logic, so it
	// doesn't need the same deterministic-for-testing treatment as compute().
	if (PX4_ISFINITE(_cpa_3d)) {
		const float time_in_terminal_s = (hrt_absolute_time() - _terminal_entry_time) * 1e-6f;
		PX4_INFO("Strike: closest approach 3D=%.1fm horizontal=%.1fm (%.1fs in TERMINAL)",
			 (double)_cpa_3d, (double)_cpa_horizontal, (double)time_in_terminal_s);
	}

	_state = State::INGRESS;
	_ingress_attempts = 0;
	_last_log_rd = INT_MIN;
	_alignment_entry_time = 0;
	_ip_orbit_entry_time  = 0;
	_recovery_start_time  = 0;
	_reset_pending = false;
	_terminal_entry_time = 0;
	_cpa_3d = NAN;
	_cpa_horizontal = NAN;
}
