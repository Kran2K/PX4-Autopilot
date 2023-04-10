#pragma once

#include "FlightTask.hpp"

#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_global_position.h>

class FlightTaskKnob : public FlightTask {
public:
	FlightTaskKnob() = default;
	virtual ~FlightTaskKnob() = default;

	bool update() override;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool applyCommandParameters(const vehicle_command_s &command) override;

protected:
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>) _param_nav_mc_alt_rad /**< acceptance radius for altitude_setpoint */,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
					(ParamFloat<px4::params::MPC_Z_VEL_UP>) _param_mpc_z_vel_up,
					(ParamFloat<px4::params::MPC_Z_VEL_DN>) _param_mpc_z_vel_dn
					)


private:
	float _origin_z = 0.0f;
	float _altitude_setpoint = 0.0f;
	float _velocityXYZ_setpoint = 0.0f;
	float _heading_setpoint = 0.0f;
	float _velocityXY_setpoint = 0.0f;
	float _climb_rate_up = 0.0f;
	float _climb_rate_dn = 0.0f;

	float altitude_acceptance_radius = 0.8f;
};
