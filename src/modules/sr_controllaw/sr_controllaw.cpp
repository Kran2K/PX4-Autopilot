/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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



#ifdef __cplusplus
extern "C" {
#endif

#include "claw_ofp.h"
#include "claw_ofp_private.h"

#ifdef __cplusplus
}
#endif
#include "sr_controllaw.hpp"

#ifndef M_PI_2
#define M_PI_2 ((float)asin(1))
#endif


using namespace time_literals;

float tc_roll_max, tc_roll_min, tc_pitch_min, tc_pitch_max, tc_yaw_rate_min, tc_yaw_rate_max;
float tc_vx_max, tc_vx_min, tc_vy_max, tc_vy_min, tc_vz_max, tc_vz_min;
float tc_roll_rate_max, tc_roll_rate_min, tc_pitch_rate_max, tc_pitch_rate_min;
float knob_spd, knob_alt, knob_hdg;
int latMode, lonMode, dirMode, vertMode;
float waypoint_speed;
int invertPitchaxis;

/* define extern structure*/
BUS_ClawOutState claw_out_state;
BUS_ClawOutModeManager claw_out_mode_manager;
BUS_ClawOutPosControl claw_out_pos_control;
BUS_ClawOutAttControl claw_out_att_control;
BUS_ClawOutCmd claw_out_cmd;
BUS_ClawInSensor claw_in_sensor;
BUS_ClawInCmd claw_in_cmd;
BUS_ClawInOFP claw_in_ofp;
BUS_ClawInTest claw_in_test;

SynchropterControlLaw::SynchropterControlLaw() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

SynchropterControlLaw::~SynchropterControlLaw()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SynchropterControlLaw::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("sensor_accel callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	ScheduleOnInterval(5000_us); // 5000 us interval, 200 Hz rate
	CLAW_load_Parameter();
	CLAW_create_flt_mode_table();
	init_param();
	init_waypoints();
	actuator_outputs_pub.advertise();
	vehicle_attitude_setpoint_pub.advertise();
	vehicle_rates_setpoint_pub.advertise();
	vehicle_local_position_setpoint_pub.advertise();

	knob_spd        = 10.0f;
	knob_alt        = 100.0f;
	knob_hdg        = 0.0f;
	latMode         = 0;
	lonMode         = 0;
	dirMode         = 0;
	vertMode        = 0;
	waypoint_speed  = 15.0f;
	invertPitchaxis = 1;

	return true;
}

void SynchropterControlLaw::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	memset(&claw_in_cmd, 0, sizeof(claw_in_cmd));
	memset(&claw_in_sensor, 0, sizeof(claw_in_sensor));
	CLAW_copy_topics();
	mapping_input_sensor();
	mapping_input_cmd();
	mapping_input_ofp();
	// CLAW_update_all_param();

	claw_ofp_step();

	CLAW_actuator_outputs_s.timestamp = CLAW_airspeed_s.timestamp;
	CLAW_actuator_outputs_s.noutputs = 10;

	for (int i = 0; i < CLAW_actuator_outputs_s.noutputs; i++) {
		CLAW_actuator_outputs_s.output[i] = claw_out_att_control.act_out[i + 1];
	}

	  // djlee: to log some setpoint datas.
	// CLAW_actuator_outputs_s.roll_ref       = claw_out_att_control.roll_ref;
	// CLAW_actuator_outputs_s.roll_rate_ref  = claw_out_att_control.roll_rate_ref;
	// CLAW_actuator_outputs_s.pitch_ref      = claw_out_att_control.pitch_ref;
	// CLAW_actuator_outputs_s.pitch_rate_ref = claw_out_att_control.pitch_rate_ref;
	// CLAW_actuator_outputs_s.hdg_ref        = claw_out_att_control.hdg_ref;
	// CLAW_actuator_outputs_s.yaw_rate_ref   = claw_out_att_control.yaw_rate_ref;
	actuator_outputs_pub.publish(CLAW_actuator_outputs_s);

	  // djlee: flight review uses quaternion to draw graphs. so i needed to convert euler to quaternion in radians.
	CLAW_vehicle_attitude_setpoint_s.timestamp = CLAW_airspeed_s.timestamp;
	matrix::Quatf quat{ matrix::Eulerf{math::radians(claw_out_att_control.roll_ref),
					   math::radians(claw_out_att_control.pitch_ref), math::radians(claw_out_att_control.hdg_ref)} };
	CLAW_vehicle_attitude_setpoint_s.q_d[0] = quat(0);
	CLAW_vehicle_attitude_setpoint_s.q_d[1] = quat(1);
	CLAW_vehicle_attitude_setpoint_s.q_d[2] = quat(2);
	CLAW_vehicle_attitude_setpoint_s.q_d[3] = quat(3);
	vehicle_attitude_setpoint_pub.publish(CLAW_vehicle_attitude_setpoint_s);

	CLAW_vehicle_rates_setpoint_s.timestamp = CLAW_airspeed_s.timestamp;
	CLAW_vehicle_rates_setpoint_s.roll      = math::radians(claw_out_att_control.roll_rate_ref);
	CLAW_vehicle_rates_setpoint_s.pitch     = math::radians(claw_out_att_control.pitch_rate_ref);
	CLAW_vehicle_rates_setpoint_s.yaw       = math::radians(claw_out_att_control.yaw_rate_ref);
	vehicle_rates_setpoint_pub.publish(CLAW_vehicle_rates_setpoint_s);


	CLAW_sensor_mag_s.timestamp   = CLAW_airspeed_s.timestamp;
	CLAW_sensor_mag_s.temperature = CLAW_airspeed_s.air_temperature_celsius;
	CLAW_sensor_mag_s.x           = 0.0f;
	CLAW_sensor_mag_s.y           = 0.0f;
	CLAW_sensor_mag_s.z           = 0.0f;
	sensor_mag_pub.publish(CLAW_sensor_mag_s);

	perf_end(_loop_perf);
}

int SynchropterControlLaw::task_spawn(int argc, char *argv[])
{
	SynchropterControlLaw *instance = new SynchropterControlLaw();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SynchropterControlLaw::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int SynchropterControlLaw::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "showwp")) {
		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_lat[0]", "wp_lon[0]", "wp_alt[0]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_in_cmd.wp_lat[0], claw_in_cmd.wp_lon[0], claw_in_cmd.wp_alt[0]);
		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_lat[1]", "wp_lon[1]", "wp_alt[1]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_in_cmd.wp_lat[1], claw_in_cmd.wp_lon[1], claw_in_cmd.wp_alt[1]);
		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_lat[2]", "wp_lon[2]", "wp_alt[2]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_in_cmd.wp_lat[2], claw_in_cmd.wp_lon[2], claw_in_cmd.wp_alt[2]);
		PX4_INFO("+==================+==================+==================+\n");
		return 0;

	} else if (!strcmp(argv[0], "showcmd")) {
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "flight_mode", "ail_cmd", "ele_cmd", "rud_cmd");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8d | %-16.8f | %-16.8f | %-16.8f |", claw_in_cmd.flight_mode, claw_in_cmd.ail_cmd, claw_in_cmd.ele_cmd, claw_in_cmd.rud_cmd);
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "EMPTY", "roll_rate_cmd", "pitch_rate_cmd", "yaw_rate_cmd");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8f | %-16.8f | %-16.8f | %-16.8f |", 0.0, claw_in_cmd.roll_rate_cmd, claw_in_cmd.pitch_rate_cmd, claw_in_cmd.yaw_rate_cmd);
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "thr_cmd", "roll_cmd", "pitch_cmd", "hdg_cmd");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8f | %-16.8f | %-16.8f | %-16.8f |", claw_in_cmd.thr_cmd, claw_in_cmd.roll_cmd, claw_in_cmd.pitch_cmd, claw_in_cmd.hdg_cmd);
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "alt_cmd", "vel_x_cmd", "vel_y_cmd", "vel_z_cmd");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8f | %-16.8f | %-16.8f | %-16.8f |", claw_in_cmd.alt_cmd, claw_in_cmd.vel_x_cmd, claw_in_cmd.vel_y_cmd, claw_in_cmd.vel_z_cmd);
		PX4_INFO("+==================+==================+==================+==================+\n");
		return 0;

	} else if (!strcmp(argv[0], "showwplocal")) {

		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_x_local[0]", "wp_y_local[0]", "wp_z_local[0]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_out_cmd.wp_x_local[0], claw_out_cmd.wp_y_local[0], claw_out_cmd.wp_z_local[0]);
		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_x_local[1]", "wp_y_local[1]", "wp_z_local[1]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_out_cmd.wp_x_local[1], claw_out_cmd.wp_y_local[1], claw_out_cmd.wp_z_local[1]);
		PX4_INFO("+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s |", "wp_x_local[2]", "wp_y_local[2]", "wp_z_local[2]");
		PX4_INFO("+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8lf | %-16.8lf | %-16.8lf |", claw_out_cmd.wp_x_local[2], claw_out_cmd.wp_y_local[2], claw_out_cmd.wp_z_local[2]);
		PX4_INFO("+==================+==================+==================+\n");
		return 0;

	} else if (!strcmp(argv[0], "showofp")) {
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "armed", "ref_lat", "ref_lon", "ref_alt");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8d | %-16.8f | %-16.8f | %-16.8f |", claw_in_ofp.armed, claw_in_ofp.ref_lat, claw_in_ofp.ref_lon, claw_in_ofp.ref_alt);
		PX4_INFO("+==================+==================+==================+==================+");
		PX4_INFO("| %-16s | %-16s | %-16s | %-16s |", "air_data_available", "link_loss", "gps_available", "agl_available");
		PX4_INFO("+------------------+------------------+------------------+------------------+");
		PX4_INFO("| %-16.8d | %-16.8d | %-16.8d | %-16.8d |", claw_in_ofp.air_data_available, claw_in_ofp.link_loss, claw_in_ofp.gps_available, claw_in_ofp.agl_available);
		PX4_INFO("+==================+==================+==================+==================+\n");
		return 0;

	} else if (!strcmp(argv[0], "knob")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "spd")) {
				if (argc > 2) {
					knob_spd = atof(argv[2]);
				}

				PX4_INFO("Knob speed set to %f", knob_spd);

			} else if (!strcmp(argv[1], "alt")) {
				if (argc > 2) {
					knob_alt = atof(argv[2]);
				}

				PX4_INFO("Knob altitude set to %f", knob_alt);

			} else if (!strcmp(argv[1], "hdg")) {
				if (argc > 2) {
					knob_hdg = atof(argv[2]);
				}

				PX4_INFO("Knob heading set to %f", knob_hdg);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return print_usage("unknown command");
			}

			return 0;

		} else {
			return 0;
		}

	} else if (!strcmp(argv[0], "offboard")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "latMode")) {
				if (argc > 2) {
					latMode = atoi(argv[2]);
				}

				PX4_INFO("Offboard latMode set to %i", latMode);

			} else if (!strcmp(argv[1], "lonMode")) {
				if (argc > 2) {
					lonMode = atoi(argv[2]);
				}

				PX4_INFO("Offboard lonMode set to %i", lonMode);

			} else if (!strcmp(argv[1], "dirMode")) {
				if (argc > 2) {
					dirMode = atoi(argv[2]);
				}

				PX4_INFO("Offboard dirMode set to %i", dirMode);

			} else if (!strcmp(argv[1], "vertMode")) {
				if (argc > 2) {
					vertMode = atoi(argv[2]);
				}

				PX4_INFO("Offboard vertMode set to %i", vertMode);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return print_usage("unknown command");
			}

			return 0;

		} else {
			return 0;
		}

	} else if (!strcmp(argv[0], "waypoint_speed")) {
		if (argc > 1) {
			waypoint_speed = atof(argv[1]);
		}

		PX4_INFO("waypoint_speed set to %f", waypoint_speed);
		return 0;

	} else if (!strcmp(argv[0], "invert_pitch_axis")) {
		if (argc > 1)
			if (atof(argv[1]) == 1) {
				invertPitchaxis = -1;

			} else if (atof(argv[1]) == 0) {
				invertPitchaxis = 1;
			}

		PX4_INFO("Pitch Axis set to %d", invertPitchaxis);
		return 0;

	} else if (!strcmp(argv[0], "help")) {
		print_usage();

	} else {
		return 0;
	}

}

int SynchropterControlLaw::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
    		This is a custom module developed by CJU and SIT to replace the default controller in PX4-autopilot.

		)DESCR_STR"
	);

	PRINT_MODULE_USAGE_NAME("sr_controllaw", "controller");
	PRINT_MODULE_USAGE_COMMAND("This is a custom module developed by CJU and SIT to replace the default controller in PX4-autopilot.");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	// PRINT_MODULE_USAGE_COMMAND_DESCR("showwp", "print waypoint triplet");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("showcmd", "print command data");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("showofp", "print ofp data");

	PRINT_MODULE_USAGE_ARG("showwp", "Print waypoints", false);
	PRINT_MODULE_USAGE_ARG("showwplocal", "Print internal waypoints", false);
	PRINT_MODULE_USAGE_ARG("showcmd", "Print commands", false);
	PRINT_MODULE_USAGE_ARG("showofp", "Print module flags", false);
	PRINT_MODULE_USAGE_ARG("knob <axis> <value>", "Set the value of the knob manually\n\t<axis> : hdg | spd | alt", false);
	PRINT_MODULE_USAGE_ARG("offboard <axisMode> <value>", "Set the axis mode in offboard flight\n\t<axisMode> : latMode | lonMode | dirMode | vertMode\t*for debug only.", false);

	return 0;
}

void SynchropterControlLaw::CLAW_load_Parameter()
{
	tc_roll_max  = 30;
	tc_roll_min  = -30;
	tc_pitch_max = 30;
	tc_pitch_min = -30;

	tc_roll_rate_max  = 130;
	tc_roll_rate_min  = -130;
	tc_pitch_rate_max = 130;
	tc_pitch_rate_min = -130;
	tc_yaw_rate_max   = 60;
	tc_yaw_rate_min   = -60;

	tc_vx_max = 20;
	tc_vx_min = -20;
	tc_vy_max = 20;
	tc_vy_min = -20;
	tc_vz_max = 3;
	tc_vz_min = -3;
}

float SynchropterControlLaw::get_physicalvalue(float stick_value, float min, float max)
{
	float center = (max + min) / 2.0f;
	float range = max - min;

	return stick_value * range / 2.0f + center;
}


float SynchropterControlLaw::get_deadzonevalue(float stick_value, float min, float max)
{
	float ret = stick_value;

	if (stick_value > min && stick_value < max) {
		ret = 0.f;
	}

	return ret;
}

float SynchropterControlLaw::remap(float val, float in1, float in2, float out1, float out2)
{
	return out1 + (val - in1) * (out2 - out1) / (in2 - in1);
}

void SynchropterControlLaw::CLAW_create_flt_mode_table()
{
	flt_mode_table[vehicle_status_s::ARMING_STATE_INIT]             = CLAW_MODE_STANDBY;
	flt_mode_table[vehicle_status_s::ARMING_STATE_STANDBY]          = CLAW_MODE_STANDBY;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_STAB]         = CLAW_MODE_STICK_ATT;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_ALTCTL]       = CLAW_MODE_STICK_ALT;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_POSCTL]       = CLAW_MODE_STICK_POS;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_RTL]     = CLAW_MODE_POINT_NAV;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION] = CLAW_MODE_WAYPOINT;
	  // flt_mode_table[vehicle_status_s::NAVIGATION_STATE_ACRO]		= CLAW_MODE_STICK_RATE;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_RTL]     = CLAW_MODE_WAYPOINT;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER]  = CLAW_MODE_POINT_NAV;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_ORBIT]        = CLAW_MODE_ORBIT;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_LAND]    = CLAW_MODE_LAND;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF] = CLAW_MODE_TAKEOFF;
	  // 230213 djlee: temporarily mapping arco to knob
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_ACRO]         = CLAW_MODE_KNOB;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_OFFBOARD]     = CLAW_MODE_OFFBOARD;

	  // // Actually, contigency mode will be handled else where.
	  // flt_mode_table[vehicle_status_s::NAVIGATION_STATE_CONTI_A] 	= CLAW_MODE_POINT_NAV;
	  // flt_mode_table[vehicle_status_s::NAVIGATION_STATE_CONTI_B] 	= CLAW_MODE_POINT_NAV;

	  //For test,
	  // flt_mode_table[vehicle_status_s::NAVIGATION_STATE_RATE] 	= CLAW_MODE_STICK_RATE;
	flt_mode_table[vehicle_status_s::NAVIGATION_STATE_MANUAL]       = CLAW_MODE_STICK_MANUAL;
}

void SynchropterControlLaw::mapping_input_sensor()
{
	matrix::Eulerf euler{ matrix::Quatf{CLAW_vehicle_attitude_s.q} };
	claw_in_sensor.roll             = math::degrees(euler(0));
	claw_in_sensor.pitch            = math::degrees(euler(1));
	claw_in_sensor.heading          = math::degrees(euler(2));
	claw_in_sensor.roll_rate        = math::degrees(CLAW_sensor_gyro_s.x);
	claw_in_sensor.pitch_rate       = math::degrees(CLAW_sensor_gyro_s.y);
	claw_in_sensor.yaw_rate         = math::degrees(CLAW_sensor_gyro_s.z);
	claw_in_sensor.accel_x          = CLAW_sensor_accel_s.x;
	claw_in_sensor.accel_y          = CLAW_sensor_accel_s.y;
	claw_in_sensor.accel_z          = CLAW_sensor_accel_s.z;
	claw_in_sensor.vel_north        = CLAW_vehicle_local_position_s.vx;
	claw_in_sensor.vel_east         = CLAW_vehicle_local_position_s.vy;
	claw_in_sensor.vel_down         = CLAW_vehicle_local_position_s.vz;
	claw_in_sensor.ac_lat           = CLAW_vehicle_global_position_s.lat;
	claw_in_sensor.ac_lon           = CLAW_vehicle_global_position_s.lon;
	claw_in_sensor.ac_alt_gps       = CLAW_vehicle_global_position_s.alt;
	claw_in_sensor.cas              = CLAW_airspeed_s.indicated_airspeed_m_s;
	claw_in_sensor.tas              = CLAW_airspeed_s.true_airspeed_m_s;
	claw_in_sensor.alt_baro         = CLAW_vehicle_air_data_s.baro_alt_meter;
	claw_in_sensor.alt_rate         = -CLAW_vehicle_local_position_s.vz;
	claw_in_sensor.oat              = (CLAW_airspeed_s.air_temperature_celsius == 0 ? 15 : CLAW_airspeed_s.air_temperature_celsius);
	claw_in_sensor.tat              = (CLAW_airspeed_s.air_temperature_celsius == 0 ? 15 : CLAW_airspeed_s.air_temperature_celsius);
	claw_in_sensor.gimbal_azimuth   = 0.0f;// For Now, Not use
	claw_in_sensor.gimbal_elevation = 0.0f;//For Now, Not use
	claw_in_sensor.alt_agl          = CLAW_vehicle_local_position_s.dist_bottom;
}

void SynchropterControlLaw::mapping_input_cmd()
{
	float stick_x = CLAW_manual_control_setpoint_s.x * invertPitchaxis;
	float stick_y = CLAW_manual_control_setpoint_s.y;
	float stick_r = CLAW_manual_control_setpoint_s.r;
	float stick_z = CLAW_manual_control_setpoint_s.z;

	  // claw_in_cmd.flight_mode = flt_mode_table[CLAW_vehicle_status_s.nav_state];

	claw_in_cmd.ail_cmd = get_physicalvalue(stick_y, -1.0f, 1.0f);
	claw_in_cmd.ele_cmd = get_physicalvalue(stick_x, -1.0f, 1.0f);
	claw_in_cmd.rud_cmd = get_physicalvalue(stick_r, -1.0f, 1.0f);

	claw_in_cmd.roll_rate_cmd  = get_physicalvalue(stick_y, tc_roll_rate_min, tc_roll_rate_max);
	claw_in_cmd.pitch_rate_cmd = get_physicalvalue(stick_x, tc_pitch_rate_min, tc_pitch_rate_max);
	claw_in_cmd.yaw_rate_cmd   = get_physicalvalue(stick_r, tc_yaw_rate_min, tc_yaw_rate_max);

	claw_in_cmd.thr_cmd   = stick_z;
	claw_in_cmd.roll_cmd  = get_physicalvalue(stick_y, tc_roll_min, tc_roll_max);
	claw_in_cmd.pitch_cmd = get_physicalvalue(stick_x, tc_pitch_min, tc_pitch_max);
	claw_in_cmd.hdg_cmd   = 0.0f;                                                    //heading

	claw_in_cmd.alt_cmd   = 0.0f;
	claw_in_cmd.vel_x_cmd = -get_physicalvalue(stick_x, tc_vx_min, tc_vx_max);
	claw_in_cmd.vel_y_cmd = get_physicalvalue(stick_y, tc_vy_min, tc_vy_max);
	claw_in_cmd.vel_z_cmd = -remap(stick_z, 0.0f, 1.0f, tc_vz_min, tc_vz_max);

	CLAW_vehicle_local_position_setpoint_s.timestamp = CLAW_airspeed_s.timestamp;
	CLAW_vehicle_local_position_setpoint_s.vx        = claw_in_cmd.vel_x_cmd;
	CLAW_vehicle_local_position_setpoint_s.vy        = claw_in_cmd.vel_y_cmd;
	CLAW_vehicle_local_position_setpoint_s.vz        = claw_in_cmd.vel_z_cmd;
	//vehicle_local_position_setpoint_pub.publish(CLAW_vehicle_local_position_setpoint_s);

	CLAW_change_flt_mode();
}

void SynchropterControlLaw::mapping_input_ofp()
{
	claw_in_ofp.armed   = CLAW_vehicle_status_s.arming_state - 1;
	claw_in_ofp.ref_lat = CLAW_vehicle_local_position_s.ref_lat;
	claw_in_ofp.ref_lon = CLAW_vehicle_local_position_s.ref_lon;
	claw_in_ofp.ref_alt = CLAW_vehicle_local_position_s.ref_alt;
	//claw_in_ofp.gps_available = CLAW_gps_s.gps_available;
	claw_in_ofp.air_data_available = true; // For now, hard coding

	// claw_in_ofp.link_loss = CLAW_vehicle_status_s.rc_signal_lost || CLAW_vehicle_status_s.data_link_lost;
	claw_in_ofp.link_loss = false;

	claw_in_ofp.gps_available = true;
	//claw_in_ofp.gps_available = CLAW_gps_s.num_sat > 8 ? true : false;
	//claw_in_ofp.gps_available = CLAW_gps_s.gps_available && (CLAW_failsafe_s.fs_type & FS_TYPE_GPS_FAIL);
	claw_in_ofp.agl_available = false;
}

void SynchropterControlLaw::CLAW_copy_topics()
{
	vehicle_attitude_sub.copy(&CLAW_vehicle_attitude_s);
	sensor_gyro_sub.copy(&CLAW_sensor_gyro_s);
	sensor_accel_sub.copy(&CLAW_sensor_accel_s);
	vehicle_local_position_sub.copy(&CLAW_vehicle_local_position_s);
	vehicle_global_position_sub.copy(&CLAW_vehicle_global_position_s);
	airspeed_sub.copy(&CLAW_airspeed_s);
	vehicle_air_data_sub.copy(&CLAW_vehicle_air_data_s);
	manual_control_setpoint_sub.copy(&CLAW_manual_control_setpoint_s);
	vehicle_status_sub.copy(&CLAW_vehicle_status_s);
	position_setpoint_triplet_sub.copy(&CLAW_position_setpoint_triplet_s);
	trajectory_setpoint_sub.copy(&CLAW_trajectory_setpoint_s);
	home_position_sub.copy(&CLAW_home_position_s);
}

void SynchropterControlLaw::init_param()
{
	claw_in_test.tune_on = 1;
	set_param_control_default(&claw_in_test.roll_loop, 1);
	set_param_control_default(&claw_in_test.roll_rate_loop, 1);
	set_param_control_default(&claw_in_test.pitch_loop, 1);
	set_param_control_default(&claw_in_test.pitch_rate_loop, 1);
	set_param_control_default(&claw_in_test.hdg_loop, 1);
	set_param_control_default(&claw_in_test.yaw_rate_loop, 1);
	set_param_control_default(&claw_in_test.Pos_loop, 1);
	set_param_control_default(&claw_in_test.Vxy_loop, 1);
	set_param_control_default(&claw_in_test.Alt_loop, 1);
	set_param_control_default(&claw_in_test.Vz_loop, 1);
}

void CLAW_set_param_control_K_gain(BUS_Param_control *param, double value) { param->K_gain = value; }
void CLAW_set_param_control_P_gain(BUS_Param_control *param, double value) { param->P_gain = value; }
void CLAW_set_param_control_I_gain(BUS_Param_control *param, double value) { param->I_gain = value; }
void CLAW_set_param_control_D_gain(BUS_Param_control *param, double value) { param->D_gain = value; }
void CLAW_set_param_control_FF_gain(BUS_Param_control *param, double value) { param->FF_gain = value; }
void CLAW_set_param_control_output_max(BUS_Param_control *param, double value) { param->output_max = value; }
void CLAW_set_param_control_output_min(BUS_Param_control *param, double value) { param->output_min = value; }
void CLAW_set_param_control_integ_max(BUS_Param_control *param, double value) { param->integ_max = value; }
void CLAW_set_param_control_integ_min(BUS_Param_control *param, double value) { param->integ_min = value; }
void CLAW_set_param_control_LPF_freq(BUS_Param_control *param, double value) { param->LPF_freq = value; }
void CLAW_set_param_control_Dfilter_freq(BUS_Param_control *param, double value) { param->DFilter_freq = value; }


void SynchropterControlLaw::set_param_control_default(BUS_Param_control *param, double value)
{
	CLAW_set_param_control_K_gain(param, value);
	CLAW_set_param_control_P_gain(param, value);
	CLAW_set_param_control_I_gain(param, value);
	CLAW_set_param_control_D_gain(param, value);
	CLAW_set_param_control_FF_gain(param, value);

	CLAW_set_param_control_output_max(param, value);
	CLAW_set_param_control_output_min(param, value);
	CLAW_set_param_control_integ_max(param, value);
	CLAW_set_param_control_integ_min(param, value);

	CLAW_set_param_control_LPF_freq(param, value);
	CLAW_set_param_control_Dfilter_freq(param, value);
}

void SynchropterControlLaw::CLAW_change_flt_mode()
{
	switch (flt_mode_table[CLAW_vehicle_status_s.nav_state]) {
	case CLAW_MODE_TAKEOFF:
	case CLAW_MODE_LAND:
	case CLAW_MODE_POINT_NAV:
	case CLAW_MODE_WAYPOINT:
		CLAW_set_wp();
		break;

	case CLAW_MODE_OFFBOARD:
		CLAW_set_offboard_axis_mode();
		CLAW_set_offboard_commands();
		break;

	case CLAW_MODE_KNOB:
		claw_in_cmd.vel_x_cmd = knob_spd;
		claw_in_cmd.alt_cmd   = knob_alt;
		claw_in_cmd.hdg_cmd   = knob_hdg;
		break;

	default:
		break;
	}

	claw_in_cmd.flight_mode = flt_mode_table[CLAW_vehicle_status_s.nav_state];
}

void SynchropterControlLaw::CLAW_set_wp()
{
	if (isnan(CLAW_position_setpoint_triplet_s.previous.lat)) {
		claw_in_cmd.wp_lat[0] = CLAW_vehicle_global_position_s.lat;
		claw_in_cmd.wp_lon[0] = CLAW_vehicle_global_position_s.lon;
		claw_in_cmd.wp_alt[0] = CLAW_vehicle_local_position_s.dist_bottom;

	} else {
		claw_in_cmd.wp_lat[0] = CLAW_position_setpoint_triplet_s.previous.lat;
		claw_in_cmd.wp_lon[0] = CLAW_position_setpoint_triplet_s.previous.lon;
		claw_in_cmd.wp_alt[0] = CLAW_position_setpoint_triplet_s.previous.alt - CLAW_home_position_s.alt;
	}

	// -1.0f cruising speed = using parameter speed;
	claw_in_cmd.wp_spd[0] = CLAW_position_setpoint_triplet_s.previous.cruising_speed == -1.0f ? waypoint_speed :
				CLAW_position_setpoint_triplet_s.previous.cruising_speed;

	if (isnan(CLAW_position_setpoint_triplet_s.current.lat)) {
		claw_in_cmd.wp_lat[1] = CLAW_vehicle_global_position_s.lat;
		claw_in_cmd.wp_lon[1] = CLAW_vehicle_global_position_s.lon;
		claw_in_cmd.wp_alt[1] = CLAW_vehicle_local_position_s.dist_bottom;

	} else {
		claw_in_cmd.wp_lat[1] = CLAW_position_setpoint_triplet_s.current.lat;
		claw_in_cmd.wp_lon[1] = CLAW_position_setpoint_triplet_s.current.lon;
		claw_in_cmd.wp_alt[1] = CLAW_position_setpoint_triplet_s.current.alt - CLAW_home_position_s.alt;
	}

	claw_in_cmd.wp_spd[1] = CLAW_position_setpoint_triplet_s.current.cruising_speed == -1.0f ? waypoint_speed :
				CLAW_position_setpoint_triplet_s.current.cruising_speed;

	if (isnan(CLAW_position_setpoint_triplet_s.current.yaw)) {
		claw_in_cmd.wp_hdg_cmd_override = CLAW_HDG_FOLLOW_COURSE;

	} else {
		claw_in_cmd.hdg_cmd = math::degrees(CLAW_position_setpoint_triplet_s.current.yaw);
		claw_in_cmd.wp_hdg_cmd_override = CLAW_HDG_SET_HEADING;
	}


	if (isnan(CLAW_position_setpoint_triplet_s.next.lat)) {
		claw_in_cmd.wp_lat[2] = 0.0f;
		claw_in_cmd.wp_lon[2] = 0.0f;
		claw_in_cmd.wp_alt[2] = CLAW_vehicle_local_position_s.dist_bottom;

	} else {
		claw_in_cmd.wp_lat[2] = CLAW_position_setpoint_triplet_s.next.lat;
		claw_in_cmd.wp_lon[2] = CLAW_position_setpoint_triplet_s.next.lon;
		claw_in_cmd.wp_alt[2] = CLAW_position_setpoint_triplet_s.next.alt - CLAW_home_position_s.alt;
	}

	claw_in_cmd.wp_spd[2] = CLAW_position_setpoint_triplet_s.next.cruising_speed == -1.0f ? waypoint_speed :
				CLAW_position_setpoint_triplet_s.next.cruising_speed;

	claw_in_cmd.wp_loiter_type      = LOITER_TYPE_HOVER;
	claw_in_cmd.wp_loiter_radius    = 0.0f;
	claw_in_cmd.wp_loiter_length    = 0.0f;
	claw_in_cmd.wp_loiter_bearing   = 0.0f;
	claw_in_cmd.wp_loiter_dir       = 0;
	claw_in_cmd.wp_alt_behaviour    = 0;
	claw_in_cmd.wp_flying_behaviour = 0;
}

void SynchropterControlLaw::init_waypoints()
{
	claw_in_cmd.wp_lat[0] = CLAW_vehicle_local_position_s.ref_lat;
	claw_in_cmd.wp_lon[0] = CLAW_vehicle_local_position_s.ref_lon;
	claw_in_cmd.wp_alt[0] = CLAW_vehicle_local_position_s.ref_alt;
	claw_in_cmd.wp_lat[1] = CLAW_vehicle_local_position_s.ref_lat;
	claw_in_cmd.wp_lon[1] = CLAW_vehicle_local_position_s.ref_lon;
	claw_in_cmd.wp_alt[1] = CLAW_vehicle_local_position_s.ref_alt;
	claw_in_cmd.wp_lat[2] = CLAW_vehicle_local_position_s.ref_lat;
	claw_in_cmd.wp_lon[2] = CLAW_vehicle_local_position_s.ref_lon;
	claw_in_cmd.wp_alt[2] = CLAW_vehicle_local_position_s.ref_alt;
}

void SynchropterControlLaw::CLAW_set_offboard_axis_mode(){
	// offboard_control_mode_sub.copy(&CLAW_offboard_control_mode_s);

	// //lat
	// if(!CLAW_offboard_control_mode_s.x_position){
	// 	latMode = LAT_AXIS_POS;
	// } else if (!CLAW_offboard_control_mode_s.x_velocity){
	// 	latMode = LAT_AXIS_VEL;
	// } else if (!CLAW_offboard_control_mode_s.x_acceleration){
	// 	latMode = LAT_AXIS_ACCEL;
	// } else {
	// 	latMode = LAT_AXIS_MAN;
	// }

	// //lon
	// if(!CLAW_offboard_control_mode_s.y_position){
	// 	lonMode = LON_AXIS_POS;
	// } else if (!CLAW_offboard_control_mode_s.y_velocity){
	// 	lonMode = LON_AXIS_VEL;
	// } else if (!CLAW_offboard_control_mode_s.y_acceleration){
	// 	lonMode = LON_AXIS_ACCEL;
	// } else {
	// 	lonMode = LON_AXIS_MAN;
	// }

	// //dir
	// if(!CLAW_offboard_control_mode_s.yaw){
	// 	dirMode = DIR_AXIS_HDG;
	// } else if (!CLAW_offboard_control_mode_s.yaw_rate){
	// 	dirMode = DIR_AXIS_HDG_RATE;
	// } else {
	// 	dirMode = DIR_AXIS_MAN;
	// }

	// //vert
	// if(!CLAW_offboard_control_mode_s.z_position){
	// 	vertMode = VERT_AXIS_ALT;
	// } else if (!CLAW_offboard_control_mode_s.z_velocity){
	// 	vertMode = VERT_AXIS_VEL;
	// } else {
	// 	vertMode = VERT_AXIS_MAN;
	// }


	// claw_in_cmd.lat_axis_mode  = latMode;
	// claw_in_cmd.lon_axis_mode  = lonMode;
	// claw_in_cmd.dir_axis_mode  = dirMode;
	// claw_in_cmd.vert_axis_mode = vertMode;
}

void SynchropterControlLaw::CLAW_set_offboard_commands()
{
	claw_in_cmd.wp_lat[1] = isnan(CLAW_trajectory_setpoint_s.x) ? CLAW_vehicle_global_position_s.lat : CLAW_trajectory_setpoint_s.x;
	claw_in_cmd.wp_lon[1] = isnan(CLAW_trajectory_setpoint_s.y) ? CLAW_vehicle_global_position_s.lon : CLAW_trajectory_setpoint_s.y;
	claw_in_cmd.wp_alt[1] = isnan(CLAW_trajectory_setpoint_s.z) ? CLAW_vehicle_global_position_s.alt : CLAW_trajectory_setpoint_s.z;
	claw_in_cmd.alt_cmd   = isnan(CLAW_trajectory_setpoint_s.z) ? CLAW_vehicle_global_position_s.alt : CLAW_trajectory_setpoint_s.z;
	claw_in_cmd.wp_spd[1] = waypoint_speed;

	claw_in_cmd.vel_x_cmd = isnan(CLAW_trajectory_setpoint_s.vx) ? 0.0f : CLAW_trajectory_setpoint_s.vx;
	claw_in_cmd.vel_y_cmd = isnan(CLAW_trajectory_setpoint_s.vy) ? 0.0f : CLAW_trajectory_setpoint_s.vy;
	claw_in_cmd.vel_z_cmd = isnan(CLAW_trajectory_setpoint_s.vz) ? 0.0f : CLAW_trajectory_setpoint_s.vz;

	claw_in_cmd.hdg_cmd = isnan(CLAW_trajectory_setpoint_s.yaw) ? 0.0f : CLAW_trajectory_setpoint_s.yaw;
	claw_in_cmd.yaw_rate_cmd = isnan(CLAW_trajectory_setpoint_s.yawspeed) ? 0.0f : CLAW_trajectory_setpoint_s.yawspeed;

	claw_in_cmd.wp_loiter_type      = LOITER_TYPE_HOVER;
	claw_in_cmd.wp_loiter_radius    = 0.0f;
	claw_in_cmd.wp_loiter_length    = 0.0f;
	claw_in_cmd.wp_loiter_bearing   = 0.0f;
	claw_in_cmd.wp_loiter_dir       = 0;
	claw_in_cmd.wp_alt_behaviour    = 0;
	claw_in_cmd.wp_flying_behaviour = 0;
	claw_in_cmd.wp_hdg_cmd_override = CLAW_HDG_FOLLOW_COURSE;
}

//Main Function Entry Point
extern "C" __EXPORT int sr_controllaw_main(int argc, char *argv[])
{
	return SynchropterControlLaw::main(argc, argv);
}
