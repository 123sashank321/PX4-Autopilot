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
			bool  airspeed_valid,
			float airspeed_eas)
{
	// --- 1. Fetch target ---------------------------------------------------
	strike_target_s target{};

	if (!_strike_target_sub.copy(&target) || !target.active) {
		return Output{};  // valid=false → caller holds altitude, wings level
	}

	// --- 2. NED range vector -----------------------------------------------
	//   R = target_pos − vehicle_pos  (positive component = target is in that NED direction)
	//   R[2] positive = target is BELOW aircraft (NED-down convention)
	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
	const Vector3f target_ned(target.x, target.y, target.z);
	const Vector3f R     = target_ned - pos;
	const float    R_mag = math::max(R.norm(), 0.5f);

	// --- 3. Lateral: 2D horizontal Proportional Navigation -----------------
	//
	// Project into the NED horizontal plane. 3D PN acceleration is perpendicular
	// to the 3D LOS and cannot be cleanly decomposed into body lateral + pitch
	// (a_pn[z] gives nose-UP when target is below — wrong). Decouple instead:
	//
	//   omega2d = (R2d × Ṙ2d)_z / |R2d|²   [rad/s, scalar LOS rate]
	//   a_horiz  = N · |V_horiz| · omega2d   [m/s², perp to LOS in horiz plane]
	//
	// |V_horiz| is always positive → no sign inversion when not yet closing.
	//
	const Vector2f R2d(R(0), R(1));
	const float    R2d_mag = math::max(R2d.norm(), 0.5f);
	const Vector2f vel2d(vel(0), vel(1));
	const Vector2f Rdot2d = -vel2d;  // dR/dt = -V_vehicle for stationary target

	const float omega2d  = (R2d(0) * Rdot2d(1) - R2d(1) * Rdot2d(0)) / (R2d_mag * R2d_mag);
	const float V2d      = math::max(vel2d.norm(), 1.0f);
	const float a_horiz  = PN_GAIN * V2d * omega2d;

	// Perpendicular-to-LOS NED direction (90° CCW from R̂2d = (-R2d[1], R2d[0]) / R2d_mag)
	const float a_NED_N  = a_horiz * (-R2d(1) / R2d_mag);
	const float a_NED_E  = a_horiz * ( R2d(0) / R2d_mag);

	// Project NED horiz accel onto body lateral axis (FRD-Y): Body-Y in NED = (-sin_yaw, cos_yaw)
	const float a_lateral = constrain(
					  -a_NED_N * sinf(yaw) + a_NED_E * cosf(yaw),
					  -tanf(radians(MAX_ROLL_DEG)) * CONSTANTS_ONE_G,
					   tanf(radians(MAX_ROLL_DEG)) * CONSTANTS_ONE_G);

	// --- 4. Pitch: elevation angle to target --------------------------------
	//
	//   atan2f(-R[2], R2d_mag):
	//     target below (R[2] > 0) → -R[2] < 0 → negative angle → nose DOWN ✅
	//     target above (R[2] < 0) → -R[2] > 0 → positive angle → nose UP   ✅
	//
	const float pitch = constrain(atan2f(-R(2), R2d_mag),
				      radians(-MAX_PITCH_DEG), radians(MAX_PITCH_DEG));

	// --- 5. Debug: log once per 5m of descent (Rd = 340, 335, ..., 5, 0) ---
	// floorf(Rd / 5) * 5 snaps to the nearest 5m floor.
	// When Rd first drops below 5m, milestone = 0 → logs at the impact point.
	const float rd_milestone = floorf(R(2) / 5.0f) * 5.0f;

	if (rd_milestone != _last_log_rd) {
		_last_log_rd = rd_milestone;
		const float V_closing = vel.dot(R) / R_mag;
		PX4_INFO("Strike: Rh=%.0fm Rd=%.0fm Vc=%.1fm/s a_lat=%.2f pitch=%.1fdeg",
			 (double)R2d_mag, (double)R(2),
			 (double)V_closing,
			 (double)a_lateral,
			 (double)degrees(pitch));
	}


	return Output{
		.lateral_acceleration = a_lateral,
		.pitch_direct         = pitch,
		.throttle_direct      = STRIKE_THROTTLE,
		.valid                = true
	};
}
