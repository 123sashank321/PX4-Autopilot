/**
 * @file FlightTaskStrike.hpp
 *
 * Flight task for autonomous target interception using APN guidance.
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/strike_target.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

class FlightTaskStrike : public FlightTask
{
public:
	FlightTaskStrike() = default;
	virtual ~FlightTaskStrike() = default;

	bool update() override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;

private:
	uORB::Subscription _strike_target_sub{ORB_ID(strike_target)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	strike_target_s _target_msg{};
	matrix::Vector3f _target_pos_ned{};

	// APN Parameters
	float _N{3.5f}; // Navigation Constant
	float _G_limit{2.5f}; // G-limit for FW

	// Helpers
	void _calculateGuidance(); // MC guidance (acceleration setpoints)
	void _calculateGuidanceFW(); // FW guidance (attitude setpoints)
};
