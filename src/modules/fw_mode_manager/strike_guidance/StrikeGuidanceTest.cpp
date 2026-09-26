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

/******************************************************************
 * Test code for StrikeGuidance.
 * Run this test only using "make tests TESTFILTER=Strike"
 ******************************************************************/

#include <gtest/gtest.h>
#include <uORB/Publication.hpp>
#include <lib/geo/geo.h>
#include "StrikeGuidance.hpp"

using math::radians;

static constexpr hrt_abstime START_TIME = 1'000'000'000ull; // arbitrary, non-zero

class StrikeGuidanceTest : public ::testing::Test
{
public:
	StrikeGuidance _guidance;
	uORB::Publication<strike_target_s> _target_pub{ORB_ID(strike_target)};

	// Common limits used by most tests: generous, so tests that aren't
	// specifically exercising the limits don't accidentally clip on them.
	float _roll_lim_rad      = radians(60.f);
	float _pitch_lim_min_rad = radians(-45.f);
	float _pitch_lim_max_rad = radians(45.f);

	vehicle_local_position_s makePos(float x, float y, float z, float vx = 0.f, float vy = 0.f, float vz = 0.f)
	{
		vehicle_local_position_s pos{};
		pos.xy_valid = true;
		pos.z_valid = true;
		pos.v_xy_valid = true;
		pos.x = x;
		pos.y = y;
		pos.z = z;
		pos.vx = vx;
		pos.vy = vy;
		pos.vz = vz;
		pos.ref_alt = 0.f;
		pos.xy_reset_counter = 0;
		pos.z_reset_counter = 0;
		pos.dist_bottom_valid = false;
		return pos;
	}

	// Publishes a target straight ahead (North) of the vehicle: IP at 500m,
	// AHP/target at the origin, x_kinematic = 100m — matches a shallow dive
	// commonly used across tests unless a test overrides specific fields.
	// dive_vne/dive_pitch_lim default to 0.f (unset), which falls back to
	// whatever airspeed_max/pitch_lim_min_rad the test uses — matches the
	// pre-STR_DIVE_VNE/STR_DIVE_PITCH behavior for tests that don't care.
	strike_target_s publishTarget(hrt_abstime timestamp, bool active = true, uint32_t designation_id = 1,
				      float dive_vne = 0.f, float dive_pitch_lim = 0.f)
	{
		strike_target_s target{};
		target.timestamp = timestamp;
		target.active = active;
		target.action_type = strike_target_s::ACTION_STRIKE;
		target.designation_id = designation_id;
		target.x = 0.f;
		target.y = 0.f;
		target.z = 0.f;
		target.ip_x = 500.f;
		target.ip_y = 0.f;
		target.ip_z = -100.f;
		target.ahp_x = 100.f;
		target.ahp_y = 0.f;
		target.ahp_z = -100.f;
		target.x_kinematic = 100.f;
		target.cruise_speed = 15.f;
		target.descent_angle = radians(10.f);
		target.loiter_radius = 80.f;
		target.max_attempts = 3;
		target.dive_vne = dive_vne;
		target.dive_pitch_lim = dive_pitch_lim;
		_target_pub.publish(target);
		return target;
	}

	StrikeGuidance::Output compute(hrt_abstime now, const vehicle_local_position_s &pos, float pitch = 0.f,
				       bool airspeed_valid = true, float airspeed_eas = 15.f, float airspeed_max = 25.f)
	{
		return _guidance.compute(now, pos, pitch, airspeed_valid, airspeed_eas, airspeed_max,
					 _roll_lim_rad, _pitch_lim_min_rad, _pitch_lim_max_rad);
	}
};

TEST_F(StrikeGuidanceTest, IngressStartsWithCourseTowardIp)
{
	publishTarget(START_TIME);

	// Vehicle far behind the IP, at cruise altitude already (no alt_error).
	const auto pos = makePos(-1000.f, 0.f, -100.f);
	const auto out = compute(START_TIME, pos);

	EXPECT_TRUE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
	// IP is due North of the vehicle -> course ~= 0 rad (NED bearing to North).
	EXPECT_NEAR(out.course, 0.f, 0.05f);
}

TEST_F(StrikeGuidanceTest, AlignmentTransitionsToTerminalWithinKinematicRange)
{
	publishTarget(START_TIME);

	// Drive the state machine into ALIGNMENT first via repeated INGRESS calls
	// is unnecessary here: reaching ALIGNMENT only requires being at the IP
	// with the right altitude, which the state machine checks internally.
	// Simplest deterministic path: call once at the IP/correct-altitude point
	// to transition INGRESS->ALIGNMENT, then again within x_kinematic of the
	// target to transition ALIGNMENT->TERMINAL.
	const auto pos_at_ip = makePos(500.f, 0.f, -100.f);
	auto out = compute(START_TIME, pos_at_ip);
	ASSERT_EQ(out.state, StrikeGuidance::State::ALIGNMENT);

	// Now within x_kinematic (100m) of the target (at origin).
	const auto pos_near_target = makePos(50.f, 0.f, -100.f);
	out = compute(START_TIME + 1000, pos_near_target);

	EXPECT_TRUE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::TERMINAL);
	EXPECT_TRUE(out.state_changed);
}

// ── Fix 1: TERMINAL pitch/roll must respect the passed-in airframe limits ──

TEST_F(StrikeGuidanceTest, TerminalPitchNeverExceedsConfiguredNoseDownLimit)
{
	publishTarget(START_TIME);
	_pitch_lim_min_rad = radians(-10.f); // tight nose-down limit, well inside the old hardcoded 45 deg

	// Force TERMINAL: at the IP, then within kinematic range.
	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Target is far below (z=0, i.e. at/above the vehicle's -100m alt is
	// actually above; use a vehicle high above the target to force a steep
	// negative (nose-down) elevation-angle command).
	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -500.f));

	EXPECT_GE(out.pitch_direct, _pitch_lim_min_rad - 1e-4f);
}

TEST_F(StrikeGuidanceTest, TerminalPitchNeverExceedsConfiguredNoseUpLimit)
{
	publishTarget(START_TIME);
	_pitch_lim_max_rad = radians(8.f); // tight nose-up limit

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Vehicle below the target -> elevation angle wants to be positive (nose up).
	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -1.f));

	EXPECT_LE(out.pitch_direct, _pitch_lim_max_rad + 1e-4f);
}

TEST_F(StrikeGuidanceTest, TerminalLateralAccelNeverExceedsConfiguredRollLimit)
{
	publishTarget(START_TIME);
	_roll_lim_rad = radians(20.f); // tight roll limit, well inside the old hardcoded 60 deg

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Large cross-track offset + lateral velocity -> large line-of-sight rate.
	out = compute(START_TIME + 2000, makePos(50.f, 80.f, -100.f, 15.f, 0.f, 0.f));

	const float max_accel = tanf(_roll_lim_rad) * CONSTANTS_ONE_G;
	EXPECT_LE(fabsf(out.lateral_acceleration), max_accel + 1e-3f);
}

// ── Regression: TERMINAL's VNE-shallowing must use STR_DIVE_VNE, not the
//    cruise FW_AIRSPD_MAX — flight-tested 2026-09-21, where reusing cruise
//    airspeed_max (20 m/s default) as the dive's overspeed threshold meant a
//    full-throttle dive exceeded it almost immediately and stayed shallowed
//    to -15 deg for the ENTIRE dive, causing the aircraft to fly over the
//    target instead of hitting it (see PX4 console log: every single
//    TERMINAL cycle logged pitch=-15.0deg, from the very first one) ────────

TEST_F(StrikeGuidanceTest, TerminalDiveNotShallowedByCruiseAirspeedMaxAlone)
{
	// dive_vne (35) is well above cruise airspeed_max (20, the FW_AIRSPD_MAX
	// default) -- eas sits between the two, exactly the flight-tested case.
	publishTarget(START_TIME, true, 1, /*dive_vne=*/35.f);
	_pitch_lim_min_rad = radians(-45.f); // wide open, isolate the VNE effect specifically

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f),
			   0.f, true, /*airspeed_eas=*/15.f, /*airspeed_max=*/20.f);
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// eas=24 > cruise airspeed_max=20, but well under dive_vne=35.
	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -500.f),
		      0.f, true, /*airspeed_eas=*/24.f, /*airspeed_max=*/20.f);

	// Must NOT be clamped to the -15 deg emergency shallow angle just because
	// it's faster than cruise -- the dive should still be able to command a
	// steep nose-down angle toward the target below.
	EXPECT_LT(out.pitch_direct, radians(-15.f));
}

TEST_F(StrikeGuidanceTest, TerminalDiveStillShallowsPastDiveVne)
{
	// The protection itself must still work once genuinely past the dive's
	// own VNE, not just be disabled outright.
	publishTarget(START_TIME, true, 1, /*dive_vne=*/35.f);
	_pitch_lim_min_rad = radians(-45.f);

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f),
			   0.f, true, /*airspeed_eas=*/15.f, /*airspeed_max=*/20.f);
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// eas=40 > dive_vne=35 -- genuinely past the dive's own ceiling now.
	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -500.f),
		      0.f, true, /*airspeed_eas=*/40.f, /*airspeed_max=*/20.f);

	EXPECT_NEAR(out.pitch_direct, radians(-15.f), 1e-3f);
}

// ── Regression: TERMINAL's steady-state nose-down bound must use
//    STR_DIVE_PITCH, not the cruise FW_P_LIM_MIN — flight-tested
//    2026-09-22. The stock advanced_plane airframes set FW_P_LIM_MIN=-15deg
//    (a conservative EVERYDAY descent limit), and even after the
//    STR_DIVE_VNE fix above, the dive was still capped to exactly -15.0deg
//    on every TERMINAL cycle from the very first one — including at
//    Vc=14.2 m/s, nowhere near either airspeed ceiling — because
//    pitch_lim_min_rad (FW_P_LIM_MIN) itself, not the VNE branch, was the
//    active constraint the whole time. ─────────────────────────────────────

TEST_F(StrikeGuidanceTest, TerminalDiveNotShallowedByCruisePitchLimitAlone)
{
	// FW_P_LIM_MIN = -15deg, matching the real advanced_plane airframes.
	// STR_DIVE_PITCH = 60deg, a deliberately deeper dive-only limit.
	publishTarget(START_TIME, true, 1, /*dive_vne=*/0.f, /*dive_pitch_lim=*/radians(60.f));
	_pitch_lim_min_rad = radians(-15.f);

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Vehicle high above and close to the target -> geometry demands a very
	// steep nose-down angle, well past -15 deg.
	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -500.f));

	EXPECT_LT(out.pitch_direct, radians(-15.f));
	EXPECT_GE(out.pitch_direct, radians(-60.f) - 1e-4f);
}

TEST_F(StrikeGuidanceTest, TerminalDiveFallsBackToCruisePitchLimitWhenUnset)
{
	// dive_pitch_lim left at its default (0.f = unset) -- must fall back to
	// FW_P_LIM_MIN exactly as before STR_DIVE_PITCH existed, so an older
	// strike_target publisher (or a designation from before this parameter
	// was set) doesn't silently dive steeper than the operator configured.
	publishTarget(START_TIME);
	_pitch_lim_min_rad = radians(-15.f);

	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	out = compute(START_TIME + 2000, makePos(20.f, 0.f, -500.f));

	EXPECT_GE(out.pitch_direct, radians(-15.f) - 1e-4f);
}

// ── Fix 2: RECOVERY altitude floor should prefer target elevation over the
//    launch point's when no rangefinder is available ─────────────────────

TEST_F(StrikeGuidanceTest, RecoveryUsesTargetElevationWhenNoRangefinder)
{
	// Target sits 220m ABOVE the local origin (e.g. a hilltop strike). Height
	// above the LAUNCH point would read "safe" (>60m) long before height
	// above the actual TARGET does -- this is exactly the terrain-blind
	// scenario the fix addresses.
	strike_target_s target{};
	target.timestamp = START_TIME;
	target.active = true;
	target.action_type = strike_target_s::ACTION_STRIKE;
	target.designation_id = 1;
	target.x = 0.f;
	target.y = 0.f;
	target.z = -220.f;   // 220m ABOVE local origin (NED: negative = above)
	target.ip_x = 500.f;
	target.ip_y = 0.f;
	target.ip_z = -320.f; // 100m above the target
	target.ahp_x = 100.f;
	target.ahp_y = 0.f;
	target.ahp_z = -320.f;
	target.x_kinematic = 100.f;
	target.cruise_speed = 15.f;
	target.descent_angle = radians(10.f);
	target.loiter_radius = 80.f;
	target.max_attempts = 3;
	_target_pub.publish(target);

	hrt_abstime t = START_TIME;

	// Drive into TERMINAL via the normal state-machine path (same pattern as
	// the other transition tests).
	auto out = compute(t, makePos(500.f, 0.f, -320.f)); // at the IP, correct altitude
	ASSERT_EQ(out.state, StrikeGuidance::State::ALIGNMENT);

	t += 1000;
	out = compute(t, makePos(50.f, 0.f, -270.f)); // within x_kinematic of the target
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Let the target go stale (stop republishing, advance past
	// TARGET_TIMEOUT_US = 2s) -> "target lost in TERMINAL" -> RECOVERY.
	// Vehicle stays at z=-270 (270m above ORIGIN -- "safe" by the old, wrong
	// metric) but only 270-220 = 50m above the TARGET's actual elevation,
	// below RECOVERY_URGENT_ALT_M (60m).
	t += 2'100'000;
	out = compute(t, makePos(50.f, 0.f, -270.f), /*pitch=*/radians(-40.f));

	ASSERT_EQ(out.state, StrikeGuidance::State::RECOVERY);
	// Throttle (unlike pitch) is not ramped -- it directly reflects urgent vs.
	// non-urgent on this very first RECOVERY cycle. RECOVERY_THROTTLE_URGENT
	// (1.0) vs RECOVERY_THROTTLE_IDLE (0.3f) in StrikeGuidance.cpp: this only
	// comes out urgent if target elevation, not launch-point height, was used.
	EXPECT_FLOAT_EQ(out.throttle_direct, 1.0f);
}

// ── Fix 4: EKF-reset handling should freeze output, then resume on a fresh
//    re-projected target, or fall back to a safe state on timeout ─────────

TEST_F(StrikeGuidanceTest, ResetHoldFreezesOutputUntilFreshTargetArrives)
{
	publishTarget(START_TIME);

	// Due North of the IP (500, 0) -> course ~= 0 rad.
	const auto pos_before = makePos(-1000.f, 0.f, -100.f);
	const auto baseline = compute(START_TIME, pos_before);
	ASSERT_TRUE(baseline.valid);
	ASSERT_NEAR(baseline.course, 0.f, 0.05f);

	// Simulate an EKF reset: reset counters change and the vehicle's position
	// in the new frame is well off the North-South line (so a fresh, correct
	// computation against it would clearly NOT match the frozen baseline) --
	// but the target hasn't been re-projected into this frame yet.
	auto pos_after_reset = makePos(-50.f, 500.f, -100.f);
	pos_after_reset.xy_reset_counter = 1;

	const auto held = compute(START_TIME + 1000, pos_after_reset);

	// Output should be exactly the frozen baseline, not a fresh computation
	// against the (still target-side-stale) mismatched frame.
	EXPECT_FLOAT_EQ(held.course, baseline.course);
	EXPECT_EQ(held.state, baseline.state);

	// strike_manager re-projects and republishes with a newer timestamp.
	publishTarget(START_TIME + 2000);
	const auto resumed = compute(START_TIME + 3000, pos_after_reset);

	// Guidance should now compute fresh course toward the IP from the new
	// (500m East-offset) position, clearly different from the frozen value.
	EXPECT_TRUE(resumed.valid);
	EXPECT_NE(resumed.course, held.course);
	EXPECT_LT(resumed.course, -0.5f); // bearing should now lean strongly East-negative
}

TEST_F(StrikeGuidanceTest, ResetHoldTimesOutIntoRecoveryFromTerminal)
{
	publishTarget(START_TIME);

	// Drive into TERMINAL.
	compute(START_TIME, makePos(500.f, 0.f, -100.f));
	auto out = compute(START_TIME + 1000, makePos(50.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::TERMINAL);

	// Detect a reset.
	auto pos_reset = makePos(50.f, 0.f, -100.f);
	pos_reset.xy_reset_counter = 1;
	out = compute(START_TIME + 2000, pos_reset);
	EXPECT_EQ(out.state, StrikeGuidance::State::TERMINAL); // still holding, not yet timed out

	// Advance past RESET_HOLD_TIMEOUT_US (500ms) with no fresh target.
	out = compute(START_TIME + 2000 + 600'000, pos_reset, radians(-30.f));

	EXPECT_EQ(out.state, StrikeGuidance::State::RECOVERY);
}

// ── Pre-existing safety nets, locked in by regression tests ────────────────

TEST_F(StrikeGuidanceTest, StaleTargetIsTreatedAsInvalid)
{
	publishTarget(START_TIME);

	// TARGET_TIMEOUT_US is 2s; ask far beyond that with no republish.
	const auto out = compute(START_TIME + 5'000'000, makePos(-1000.f, 0.f, -100.f));

	EXPECT_FALSE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
}

TEST_F(StrikeGuidanceTest, AlignmentTimeoutRetriesAreBoundedByMaxAttempts)
{
	strike_target_s target = publishTarget(START_TIME);
	target.max_attempts = 2;
	_target_pub.publish(target);

	hrt_abstime t = START_TIME;

	// Enter ALIGNMENT.
	auto out = compute(t, makePos(500.f, 0.f, -100.f));
	ASSERT_EQ(out.state, StrikeGuidance::State::ALIGNMENT);

	// Keep re-publishing so the target never goes stale across the 60s
	// ALIGNMENT_TIMEOUT_S, while never actually reaching the target (stay far
	// from it) so ALIGNMENT keeps timing out and retrying.
	for (int attempt = 0; attempt < 2; attempt++) {
		t += 1'000'000; // 1s
		_target_pub.publish(target);
		out = compute(t, makePos(500.f, 1000.f, -100.f)); // never near target
		t += 61'000'000; // exceed ALIGNMENT_TIMEOUT_S (60s)
		_target_pub.publish(target);
		out = compute(t, makePos(500.f, 1000.f, -100.f));
	}

	// After max_attempts (2) exhausted, guidance gives up: reset() + invalid.
	EXPECT_FALSE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
}

TEST_F(StrikeGuidanceTest, InvalidLocalPositionHoldsRatherThanCommandsGarbage)
{
	publishTarget(START_TIME);

	vehicle_local_position_s pos = makePos(-1000.f, 0.f, -100.f);
	pos.xy_valid = false;

	const auto out = compute(START_TIME, pos);

	EXPECT_FALSE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
}

// Drives a fresh guidance instance through INGRESS -> ALIGNMENT -> TERMINAL
// using the default publishTarget() geometry.
static void reachTerminal(StrikeGuidanceTest &f, hrt_abstime &t, uint32_t designation_id)
{
	f.publishTarget(t, true, designation_id);
	f.compute(t, f.makePos(500.f, 0.f, -100.f));   // at IP, on altitude -> ALIGNMENT
	t += 100'000;
	f.publishTarget(t, true, designation_id);
	f.compute(t, f.makePos(90.f, 0.f, -100.f));    // inside x_kinematic -> TERMINAL
}

TEST_F(StrikeGuidanceTest, ResetAfterInterruptedTerminalRestartsAtIngress)
{
	// A strike interrupted mid-dive by a mode change never lets compute() see
	// the target go away; FixedWingModeManager resets guidance on STRIKE
	// entry/exit instead. The next designation must start from INGRESS, not
	// resume the old TERMINAL dive toward the new target.
	hrt_abstime t = START_TIME;
	reachTerminal(*this, t, 1);
	ASSERT_EQ(_guidance.currentState(), StrikeGuidance::State::TERMINAL);

	_guidance.reset();   // what FixedWingModeManager does on mode exit/entry

	t += 60'000'000;
	publishTarget(t, true, 2);
	const auto out = compute(t, makePos(3000.f, 0.f, -100.f));

	EXPECT_TRUE(out.valid);
	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
	EXPECT_FALSE(PX4_ISFINITE(out.throttle_direct));
}

TEST_F(StrikeGuidanceTest, ResetDoesNotReplayPreviousStrikeOutputOnEkfCounterChange)
{
	// EKF reset counters advance freely while not in STRIKE. After reset(),
	// guidance must re-latch them rather than read the change as a mid-strike
	// reset and replay the previous strike's last (dive) setpoint.
	hrt_abstime t = START_TIME;
	reachTerminal(*this, t, 1);
	t += 100'000;
	publishTarget(t, true, 1);
	const auto dive = compute(t, makePos(80.f, 0.f, -90.f));
	ASSERT_EQ(dive.state, StrikeGuidance::State::TERMINAL);

	_guidance.reset();

	t += 60'000'000;
	publishTarget(t - 50'000, true, 2);   // published just before the first compute, as in flight
	auto pos = makePos(3000.f, 0.f, -100.f);
	pos.xy_reset_counter = 3;
	pos.z_reset_counter = 2;
	const auto out = compute(t, pos);

	EXPECT_EQ(out.state, StrikeGuidance::State::INGRESS);
	EXPECT_FALSE(PX4_ISFINITE(out.throttle_direct));
}

TEST_F(StrikeGuidanceTest, RecoveryFromPositionLossRampsEvenAfterEarlierRecovery)
{
	// A completed RECOVERY must not leave state behind that makes a later
	// position-loss RECOVERY skip its pull-out ramp.
	hrt_abstime t = START_TIME;
	reachTerminal(*this, t, 1);
	t += 100'000;
	publishTarget(t, false, 1);
	compute(t, makePos(80.f, 0.f, -90.f), radians(-30.f));   // target lost -> RECOVERY
	ASSERT_EQ(_guidance.currentState(), StrikeGuidance::State::RECOVERY);

	for (int i = 0; i < 40 && _guidance.currentState() == StrikeGuidance::State::RECOVERY; i++) {
		t += 100'000;
		compute(t, makePos(80.f, 0.f, -90.f), radians(-10.f));
	}

	ASSERT_EQ(_guidance.currentState(), StrikeGuidance::State::INGRESS);

	t += 1'000'000;
	reachTerminal(*this, t, 2);
	ASSERT_EQ(_guidance.currentState(), StrikeGuidance::State::TERMINAL);

	t += 100'000;
	publishTarget(t, true, 2);
	auto lost = makePos(80.f, 0.f, -90.f);
	lost.xy_valid = false;
	const auto out = compute(t, lost, radians(-40.f));

	EXPECT_EQ(_guidance.currentState(), StrikeGuidance::State::RECOVERY);
	EXPECT_NEAR(math::degrees(out.pitch_direct), -40.f, 1.f);   // ramp starts at live attitude
}
