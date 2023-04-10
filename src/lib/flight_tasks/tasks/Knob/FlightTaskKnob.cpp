#include "FlightTaskKnob.hpp"

using matrix::wrap_pi;

bool FlightTaskKnob::activate(const vehicle_local_position_setpoint_s &last_setpoint) {
	bool ret = FlightTask::activate(last_setpoint);

	// _position_setpoint(0) = _position(0);
	// _position_setpoint(1) = _position(1);

	// _origin_z = _position(2);

	// _yawspeed_setpoint = 45.0f * 3.142f / 180.f;
	// _velocity_setpoint(2) = -1.0f;

	_altitude_setpoint = 0.0f;
	_velocityXY_setpoint = 0.0f;
	_velocityXYZ_setpoint = 0.0f;
	_heading_setpoint = 0.0f;



	return ret;
}

bool FlightTaskKnob::update() {
	altitude_acceptance_radius = _param_nav_mc_alt_rad.get();
	_climb_rate_dn = _param_mpc_z_vel_dn.get();
	_climb_rate_up = _param_mpc_z_vel_up.get();

	bool altitude_reached = false;
	float current_alt = -_position(2) + _sub_vehicle_local_position.get().ref_alt;
	float diff_z = _altitude_setpoint - (current_alt);
	if(diff_z > altitude_acceptance_radius) {
		//aircraft is located lower than altitude setpoint -> go up
		_velocity_setpoint(2) = -1.0f; //NED Frame
	}
	else if(diff_z < -altitude_acceptance_radius) {
		//aircraft is located higher than altitude setpoint -> go down
		_velocity_setpoint(2) = 1.0f; //NED Frame
	}
	else {
		altitude_reached = true;
		_velocity_setpoint(2) = 0.0f;
	}

	if(!altitude_reached) {
		if(_velocity_setpoint(2) < 0.0f) {
			// aircraft goes higher
			_velocity_setpoint(2) *= _climb_rate_up;
			_velocityXY_setpoint = sqrtf((_velocityXYZ_setpoint + _climb_rate_up) * (_velocityXYZ_setpoint - _climb_rate_up));
		}
		else {
			// aircraft goes lower
			_velocity_setpoint(2) *= _climb_rate_dn;
			_velocityXY_setpoint = sqrtf((_velocityXYZ_setpoint + _climb_rate_dn) * (_velocityXYZ_setpoint - _climb_rate_dn));
		}
	}
	else {
		_velocityXY_setpoint = _velocityXYZ_setpoint;
	}


	// float heading_rad = math::radians(_heading_setpoint);
	// _velocity_setpoint(0) = cosf(heading_rad) * _velocityXY_setpoint;
	// _velocity_setpoint(1) = sinf(heading_rad) * _velocityXY_setpoint;

	_velocity_setpoint(0) = cosf(_heading_setpoint) * _velocityXY_setpoint;
	_velocity_setpoint(1) = sinf(_heading_setpoint) * _velocityXY_setpoint;

	_position_setpoint(0) = _position_setpoint(1) = NAN;
	_acceleration_setpoint(0) = _acceleration_setpoint(1) = NAN;

	// _yaw_setpoint = _heading_setpoint;
	_yaw_setpoint = _heading_setpoint;

	// _position_setpoint(0) = NAN;
	// _position_setpoint(1) = NAN;
	// _position_setpoint(2) = _position(2);
	// _velocity_setpoint(0) = NAN;
	// _velocity_setpoint(1) = NAN;
	// _velocity_setpoint(2) = NAN;
	// _yaw_setpoint = math::radians(220.0f);

	// PX4_INFO("%f %f %f", (double)_position(0), (double)_position(1), (double)_position(2));

	return true;

}

bool FlightTaskKnob::applyCommandParameters(const vehicle_command_s &command) {
	bool ret = true;

	PX4_INFO("commandParam, p1:%d, p2:%d, p3:%d, p7%d\n", command.param1, command.param2, command.param3, command.param7);

	// check heading value is valid
	if(command.param7 > 0.0f) {
		if(command.param1 < -360.0f || command.param1 > 360.0f) {
			ret = false;
		}
	}
	else {
		/* In multicopter only heading is vaild*/
		ret = false;
	}

	// check altitude is valid
	if(command.param2 < 0.0f || command.param3 < 0.0f) {
		ret = false;
	}

	if(ret) {
		_heading_setpoint = wrap_pi(math::radians(command.param1));
		_altitude_setpoint = command.param2;
		_velocityXYZ_setpoint = command.param3;
	}

	return ret;
}
