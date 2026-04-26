/****************************************************************************
 *
 *   Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskKnob.cpp
 */

#include "FlightTaskKnob.hpp"

using namespace matrix;

bool FlightTaskKnob::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_altitude_setpoint = 0.0f;
	_velocityXYZ_setpoint = 0.0f;
	_heading_setpoint = 0.0f;
	_velocityXY_setpoint = 0.0f;

	return ret;
}

bool FlightTaskKnob::update()
{
	const float altitude_acceptance_radius = _param_nav_mc_alt_rad.get();
	const float climb_rate_dn = _param_mpc_z_vel_max_dn.get();
	const float climb_rate_up = _param_mpc_z_vel_max_up.get();

	bool altitude_reached = false;

	// Current altitude (AMSL)
	const float current_alt = -_position(2) + _global_local_alt0;
	const float diff_z = _altitude_setpoint - current_alt;

	if (diff_z > altitude_acceptance_radius) {
		// aircraft is located lower than altitude setpoint -> go up (negative NED z)
		_velocity_setpoint(2) = -1.0f;

	} else if (diff_z < -altitude_acceptance_radius) {
		// aircraft is located higher than altitude setpoint -> go down (positive NED z)
		_velocity_setpoint(2) = 1.0f;

	} else {
		altitude_reached = true;
		_velocity_setpoint(2) = 0.0f;
	}

	if (!altitude_reached) {
		if (_velocity_setpoint(2) < 0.0f) {
			// aircraft goes higher
			_velocity_setpoint(2) *= climb_rate_up;
			const float vxyz_sq = _velocityXYZ_setpoint * _velocityXYZ_setpoint;
			const float vz_sq = climb_rate_up * climb_rate_up;
			_velocityXY_setpoint = (vxyz_sq > vz_sq) ? sqrtf(vxyz_sq - vz_sq) : 0.0f;

		} else {
			// aircraft goes lower
			_velocity_setpoint(2) *= climb_rate_dn;
			const float vxyz_sq = _velocityXYZ_setpoint * _velocityXYZ_setpoint;
			const float vz_sq = climb_rate_dn * climb_rate_dn;
			_velocityXY_setpoint = (vxyz_sq > vz_sq) ? sqrtf(vxyz_sq - vz_sq) : 0.0f;
		}

	} else {
		_velocityXY_setpoint = _velocityXYZ_setpoint;
	}

	_velocity_setpoint(0) = cosf(_heading_setpoint) * _velocityXY_setpoint;
	_velocity_setpoint(1) = sinf(_heading_setpoint) * _velocityXY_setpoint;

	_position_setpoint(0) = _position_setpoint(1) = NAN;
	_acceleration_setpoint(0) = _acceleration_setpoint(1) = NAN;

	_yaw_setpoint = _heading_setpoint;

	return true;
}

bool FlightTaskKnob::applyCommandParameters(const vehicle_command_s &command, bool &success)
{
	success = true;

	// check heading value is valid
	if (command.param7 > 0.0f) {
		if (command.param1 < -360.0f || command.param1 > 360.0f) {
			success = false;
		}

	} else {
		success = false;
	}

	// check altitude is valid
	if (command.param2 < 0.0f || command.param3 < 0.0f) {
		success = false;
	}

	if (success) {
		_heading_setpoint = matrix::wrap_pi(math::radians(command.param1));
		_altitude_setpoint = command.param2;
		_velocityXYZ_setpoint = command.param3;
	}

	return true;
}
