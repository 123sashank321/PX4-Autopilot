/**
 * @file FlightTaskStrike.cpp
 */

#include "FlightTaskStrike.hpp"
#include <mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

bool FlightTaskStrike::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	// Force update target from latest message
	if (_strike_target_sub.copy(&_target_msg)) {
		if (_target_msg.active) {
			_target_pos_ned(0) = _target_msg.x;
			_target_pos_ned(1) = _target_msg.y;
			_target_pos_ned(2) = _target_msg.z;
			PX4_INFO("Strike activated with target: [%.1f, %.1f, %.1f]",
				(double)_target_msg.x, (double)_target_msg.y, (double)_target_msg.z);
		}
	} else {
		PX4_WARN("Strike activated but no target message available!");
	}

	return ret;
}

bool FlightTaskStrike::update()
{
	// Check for target updates
	if (_strike_target_sub.updated()) {
		_strike_target_sub.copy(&_target_msg);
		if (_target_msg.active && _target_msg.action_type == 0) {
			_target_pos_ned(0) = _target_msg.x;
			_target_pos_ned(1) = _target_msg.y;
			_target_pos_ned(2) = _target_msg.z;
		}
	}

	// Detect vehicle type and use appropriate guidance
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.copy(&vehicle_status)) {
		if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			// Fixed-wing: publish attitude setpoints
			_calculateGuidanceFW();
		} else {
			// Multicopter: publish acceleration setpoints (existing logic)
			_calculateGuidance();
		}
	} else {
		// Default to MC if no status available
		_calculateGuidance();
	}

	return true;
}

void FlightTaskStrike::_calculateGuidance()
{
	// Implementation based on APNGuidance.cpp
	// This is the proper Augmented Proportional Navigation algorithm

	// 1. Calculate Relative Position and Velocity (NED Frame)
	// R = Target - Interceptor
	matrix::Vector3f R = _target_pos_ned - _position;
	matrix::Vector3f V = -_velocity; // Relative velocity (assuming stationary target)

	// 2. Range and safety check
	float R_mag = R.norm();
	if (R_mag < 0.001f) {
		// Too close, hold position
		_acceleration_setpoint.setZero();
		_position_setpoint.setNaN();
		_velocity_setpoint.setNaN();
		return;
	}

	// 3. Closing velocity (range rate)
	// V_closing = -R·V / |R|  (negative of range rate)
	float V_closing = -R.dot(V) / R_mag;

	// 4. Line of Sight (LOS) Rotation Vector
	// Omega = (R × V) / R²
	matrix::Vector3f omega = R.cross(V) / (R_mag * R_mag);

	// 5. Standard Proportional Navigation
	// a_pn = N * V_closing * (Omega × R_unit)
	// Direction of acceleration is perpendicular to both R and Omega
	matrix::Vector3f R_unit = R.normalized();
	matrix::Vector3f acc_pn = omega.cross(R_unit) * (_N * V_closing);

	// 6. Augmented Term (Target Acceleration perpendicular to LOS)
	// For a stationary ground target, the "target acceleration" is effectively
	// gravity pulling it down. We want the component perpendicular to LOS.
	// a_t_est = [0, 0, 9.81] (gravity in NED)
	matrix::Vector3f a_t_est(0.f, 0.f, 9.81f); // Gravity acceleration (down in NED)

	// Component perpendicular to R:
	// a_t_perp = a_t - (a_t · R_unit) * R_unit
	float at_proj = a_t_est.dot(R_unit);
	matrix::Vector3f at_perp = a_t_est - (R_unit * at_proj);

	// 7. Total APN Command
	// a_cmd = a_pn + (N/2) * a_t_perp
	_acceleration_setpoint = acc_pn + at_perp * (_N / 2.0f);

	// Add gravity compensation to maintain altitude (hover thrust)
	// This counters the weight of the vehicle
	matrix::Vector3f gravity_comp(0.f, 0.f, -9.81f); // Upward thrust to hover
	_acceleration_setpoint = _acceleration_setpoint + gravity_comp;

	// Set other setpoints to NaN to ensure acceleration control
	_position_setpoint.setNaN();
	_velocity_setpoint.setNaN();
}

void FlightTaskStrike::_calculateGuidanceFW()
{
	// Calculate APN acceleration (same as MC)
	matrix::Vector3f R = _target_pos_ned - _position;
	matrix::Vector3f V = -_velocity;

	float R_mag = R.norm();
	if (R_mag < 0.001f) {
		return; // Too close
	}

	float V_closing = -R.dot(V) / R_mag;
	matrix::Vector3f omega = R.cross(V) / (R_mag * R_mag);
	matrix::Vector3f R_unit = R.normalized();
	matrix::Vector3f acc_pn = omega.cross(R_unit) * (_N * V_closing);

	// Augmented Pro-Nav term (Gravity Compensation)
	// matrix::Vector3f a_t_est(0.f, 0.f, 9.81f);
	// float at_proj = a_t_est.dot(R_unit);
	// matrix::Vector3f at_perp = a_t_est - (R_unit * at_proj);

	// matrix::Vector3f accel_cmd = acc_pn + at_perp * (_N / 2.0f);
	matrix::Vector3f accel_cmd = acc_pn; // Pure Pro-Nav for now to prevent dive

	// Apply G-limits
	float accel_mag = accel_cmd.norm();
	float max_accel = _G_limit * 9.81f;
	if (accel_mag > max_accel) {
		accel_cmd = accel_cmd.normalized() * max_accel;
	}

	// Convert to attitude setpoint (FW specific)
	float a_y = accel_cmd(1); // Lateral
	float a_z = accel_cmd(2); // Vertical

	// Roll for coordinated turn
	float roll = atan2f(a_y, 9.81f);
	roll = math::constrain(roll, -math::radians(45.0f), math::radians(45.0f));

	// Pitch for dive/climb
	float pitch = -atan2f(a_z, 15.0f); // Assume ~15m/s airspeed
	pitch = math::constrain(pitch, -math::radians(30.0f), math::radians(30.0f));

	// Yaw to point at target
	float yaw = atan2f(R(1), R(0));

	// Convert to quaternion
	matrix::Eulerf euler(roll, pitch, yaw);
	matrix::Quatf q(euler);

	// Publish attitude setpoint
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = hrt_absolute_time();
	att_sp.q_d[0] = q(0);
	att_sp.q_d[1] = q(1);
	att_sp.q_d[2] = q(2);
	att_sp.q_d[3] = q(3);
	att_sp.thrust_body[0] = 0.8f; // 80% throttle
	att_sp.thrust_body[1] = 0.0f;
	att_sp.thrust_body[2] = 0.0f;
	att_sp.reset_integral = false;
	att_sp.fw_control_yaw_wheel = false;
	att_sp.yaw_sp_move_rate = 0.0f;

	_attitude_setpoint_pub.publish(att_sp);

	// Debug Logging (2Hz)
	static uint64_t counter = 0;
	if (counter++ % 25 == 0) {
		PX4_INFO("StrikeFW: T=[%.1f,%.1f,%.1f] P=[%.1f,%.1f,%.1f] R=[%.1f,%.1f,%.1f] V=[%.1f,%.1f,%.1f]",
			(double)_target_pos_ned(0), (double)_target_pos_ned(1), (double)_target_pos_ned(2),
			(double)_position(0), (double)_position(1), (double)_position(2),
			(double)R(0), (double)R(1), (double)R(2),
			(double)V(0), (double)V(1), (double)V(2));
		PX4_INFO("StrikeFW: AccCmd=[%.1f,%.1f,%.1f] Roll=%.1f Pitch=%.1f",
			(double)accel_cmd(0), (double)accel_cmd(1), (double)accel_cmd(2),
			(double)math::degrees(roll), (double)math::degrees(pitch));
	}
}
