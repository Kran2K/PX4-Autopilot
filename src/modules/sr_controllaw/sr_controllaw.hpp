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


#define MODULE_NAME "sr_controllaw"


#ifdef __cplusplus
extern "C" {
#endif

#include "claw_ofp.h"

#ifdef __cplusplus
}
#endif


#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/offboard_control_mode.h>


#define ALT_TYPE_ALT 1
#define ALT_TYPE_V_SPD 2

#define LOITER_TYPE_CIRCULAR 1
#define LOITER_TYPE_RACE 2
#define LOITER_TYPE_FIRGURE8 3
#define LOITER_TYPE_HOVER 4
#define LOITER_DIR_CW 1 // ClockWise
#define LOITER_DIR_CCW 2 // Counter-ClockWise


using namespace time_literals;

class SynchropterControlLaw : public ModuleBase<SynchropterControlLaw>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	SynchropterControlLaw();
	~SynchropterControlLaw() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/*
	* 제어법칙의 값들을 프린트 해주는 함수
	* showwp : 웨이포인트 출력
	* showcmd : 제어법칙으로 들어가는 명령값 출력
	* showofp : OFP에서 관리하는 상태값 출력
	*/
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:

	void Run() override;

	// Publications
	uORB::Publication<actuator_outputs_s>	actuator_outputs_pub{ ORB_ID(actuator_outputs) };
	uORB::Publication<sensor_mag_s>		sensor_mag_pub{ ORB_ID(sensor_mag) };
	uORB::Publication<vehicle_attitude_setpoint_s>	vehicle_attitude_setpoint_pub{ ORB_ID(vehicle_attitude_setpoint) };
	uORB::Publication<vehicle_rates_setpoint_s>	vehicle_rates_setpoint_pub{ ORB_ID(vehicle_rates_setpoint) };
	uORB::Publication<vehicle_local_position_setpoint_s> vehicle_local_position_setpoint_pub{ ORB_ID(vehicle_local_position_setpoint) };

	// Subscriptions
	// uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval      _parameter_update_sub{ ORB_ID(parameter_update), 1_s }; // subscription limited to 1 Hz updates
	uORB::Subscription              vehicle_attitude_sub{ ORB_ID(vehicle_attitude) }; 	 // regular subscription for additional data
	uORB::Subscription		sensor_gyro_sub{ ORB_ID(sensor_gyro) };
	uORB::Subscription		sensor_accel_sub{ ORB_ID(sensor_accel) };
	uORB::Subscription              vehicle_local_position_sub{ ORB_ID(vehicle_local_position) };
	uORB::Subscription		vehicle_global_position_sub{ ORB_ID(vehicle_global_position) };
	uORB::Subscription              airspeed_sub{ ORB_ID(airspeed) };
	uORB::Subscription		vehicle_air_data_sub{ ORB_ID(vehicle_air_data) };
	uORB::Subscription		manual_control_setpoint_sub{ ORB_ID(manual_control_setpoint) };
	uORB::Subscription		vehicle_status_sub{ ORB_ID(vehicle_status) };
	uORB::Subscription		position_setpoint_triplet_sub{ ORB_ID(position_setpoint_triplet) };
	uORB::Subscription		vehicle_local_position_setpoint_sub{ ORB_ID(vehicle_local_position_setpoint) };
	uORB::Subscription		trajectory_setpoint_sub{ ORB_ID(trajectory_setpoint) };
	uORB::Subscription		home_position_sub{ ORB_ID(home_position) };
	uORB::Subscription		offboard_control_mode_sub{ ORB_ID(offboard_control_mode) };

	//uORB Structures
	actuator_outputs_s 			CLAW_actuator_outputs_s;
	vehicle_attitude_s 			CLAW_vehicle_attitude_s;
	sensor_gyro_s 				CLAW_sensor_gyro_s;
	sensor_accel_s 				CLAW_sensor_accel_s;
	vehicle_local_position_s 		CLAW_vehicle_local_position_s;
	vehicle_global_position_s 		CLAW_vehicle_global_position_s;
	airspeed_s 				CLAW_airspeed_s;
	vehicle_air_data_s 			CLAW_vehicle_air_data_s;
	manual_control_setpoint_s 		CLAW_manual_control_setpoint_s;
	vehicle_status_s 			CLAW_vehicle_status_s;
	sensor_mag_s 				CLAW_sensor_mag_s;
	position_setpoint_triplet_s 		CLAW_position_setpoint_triplet_s;
	vehicle_attitude_setpoint_s 		CLAW_vehicle_attitude_setpoint_s;
	vehicle_rates_setpoint_s 		CLAW_vehicle_rates_setpoint_s;
	vehicle_local_position_setpoint_s 	CLAW_vehicle_local_position_setpoint_s;
	vehicle_local_position_setpoint_s 	CLAW_trajectory_setpoint_s;
	home_position_s 			CLAW_home_position_s;
	offboard_control_mode_s			CLAW_offboard_control_mode_s;


	/*
	* flt_mode_table
	*  - ICD상의 비행모드 값와 제어법칙 내부에서의 비행모드 값이 다르므로 table을 통해 매핑
	*/
	uint16_t flt_mode_table[vehicle_status_s::NAVIGATION_STATE_MAX];

	/* 열거형 변수 선언 */
	enum CLAW_MODE {
		CLAW_MODE_STANDBY = 0,  // Standby, 대기 모드
		CLAW_MODE_STICK_MANUAL, // Stick Manual, 조종간 수동 모드
		CLAW_MODE_STICK_RATE,   // Stick Rate, 조종간 각속도 모드(roll rate, pitch rate, yaw rate, throttle)
		CLAW_MODE_STICK_ATT,    // Stick Attitude, 조종간 자세 모드(roll angle, pitch angle, yaw rate, throttle)
		CLAW_MODE_STICK_ALT,    // Stick Altitude, 조종간 고도 모드(roll angle, pitch angle, yaw rate, vertical speed)
		CLAW_MODE_STICK_POS,    // Stick Position mode, 조종간 위치(GPS) 모드(vx, vy, yaw rate, vz)
		CLAW_MODE_POINT_NAV,    // Point Navigation, 점항법(lat, lon, alt, spd, hdg)
		CLAW_MODE_HOLD,         // Hold mode, 호버 모드, 현재 위치와 고도에서 호버링
		CLAW_MODE_WAYPOINT,     // Waypoint flight, 항로점 비행모드(lat, lon, alt, spd, hdg, pass_type)
		CLAW_MODE_ORBIT,        // Orbit, 항로점 선회 비행, 항로점 주위로 원선회비행하면서 헤딩은 중심점을 향함.(lat, lon, alt, spd, radius)
		CLAW_MODE_RETURN,       // Return mode, 리턴홈 위치로 복귀(lat, lon, alt, spd)
		CLAW_MODE_TAKEOFF,      // Takeoff, 자동이륙(lat, lon, climb rate)
		CLAW_MODE_LAND,         // Landing, 자동착륙(lat, lon)
		CLAW_MODE_TAXING,       // Taxing, 활주(lat, lon, spd)
		CLAW_MODE_CAMERA_GUIDE, // Camera Guided Flight, 카메라 유도모드(lat, lon, spd)
		CLAW_MODE_KNOB,         // Knob mode, 노브모드(spd, alt, hdg)
		CLAW_MODE_OFFBOARD,	// Offboard mode, 외부입력 모드(전체 기능 사용)
	};

	// maybe need to change
	enum CLAW_motor_output {
		CLAW_FOR_RIGHT = 0, // 전방 오른쪽(CCW)
		CLAW_REAR_LEFT,     // 후방 왼쪽(CCW)
		CLAW_FOR_LEFT,      // 전방 왼쪽(CW)
		CLAW_REAR_RIGHT     // 후방 오른쪽(CW)
	};

	enum CLAW_HDG_CMD_OVERRIDE {
		CLAW_HDG_FOLLOW_COURSE = 0,
		CLAW_HDG_SET_HEADING = 1
	};

	enum LAT_AXIS_MODE {
		LAT_AXIS_MAN = 0,	// Manual
		LAT_AXIS_RATE,		// Rate
		LAT_AXIS_ATT,		// Attitude (Pitch)
		LAT_AXIS_ACCEL,		// Acceleration
		LAT_AXIS_VEL,		// Velocity
		LAT_AXIS_POS,		// Position (Guidance)
		LAT_AXIS_POSVEL		// Position with velocity limit
	};

	enum LON_AXIS_MODE {
		LON_AXIS_MAN = 0,	// Manual
		LON_AXIS_RATE,		// Rate
		LON_AXIS_ATT,		// Attitude (Pitch)
		LON_AXIS_ACCEL,		// Acceleration
		LON_AXIS_VEL,		// Velocity
		LON_AXIS_POS,		// Position (Guidance)
		LON_AXIS_POSVEL		// Position with velocity limit
	};

	enum DIR_AXIS_MODE {
		DIR_AXIS_MAN = 0,	// Manual
		DIR_AXIS_RATE,		// Rate
		DIR_AXIS_HDG,		// Heading
		DIR_AXIS_HDG_RATE,	// Heading with rate limit
		DIR_AXIS_GD		// Guidance (Course)
	};

	enum VERT_AXIS_MODE {
		VERT_AXIS_MAN = 0, 	// Throttle Manual
		VERT_AXIS_VEL, 		// Throttle Velocity
		VERT_AXIS_ALT, 		// Altitude
		VERT_AXIS_ALTVEL, 	// Altitude with velocity limit
		VERT_AXIS_GD		// Throttle Guidance (Vertical speed/path control)
	};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{ perf_alloc(PC_ELAPSED, MODULE_NAME": cycle") };
	perf_counter_t	_loop_interval_perf{ perf_alloc(PC_INTERVAL, MODULE_NAME": interval") };


	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	/*
	* P3D에서 받아온 센서 값들을 제어법칙의 입력 구조체에 넣어주는 함수
	*/
	void mapping_input_sensor();

	/*
	* 명령 입력값을 제어법칙의 구조체에 넣어주는 함수
	*/
	void mapping_input_cmd();

	/*
	* OFP에서 관리하는 상태값들을 제어법칙의 구조체에 넣어주는 함수
	*/
	void mapping_input_ofp();

	/*
	* 제어이득 튜닝 및 시험용 데이터를 입력할 수 있도록 제어법칙의 구조체에 넣어주는 함수
	*/
	void set_input_test();

	/*
	* 조종기 명령값들을 각각의 물리적인 값(각도, 속도 등)으로 바꾸어 주는 함수
	* @param stick_value 조종기 명령값 (-1 ~ 1사이의 값을 가짐)
	* @param min         물리적인 값의 최소값. 파라매터로 선언될 예정
	* @param max         물리적인 값의 최대값. 파라매터로 선언될 예정
	*
	* @return -1 ~ 1 사이의 값을 최소 ~ 최대 값의 범위에 맞게 계산되어 나온 값
	*/
	float get_physicalvalue(float stick_value, float min, float max);

	/*
	* 조종기 명령값의 데드존 설정 함수
	* @param stick_value 조종기 명령값 (-1 ~ 1사이의 값을 가짐)
	* @param min         데드존의 최소값. 파라매터로 선언될 예정
	* @param max         데드존의 최대값. 파라매터로 선언될 예정
	*
	* @return 스틱 입력값이 데드존 범위 내부이면 0을 리턴 함.
	*/
	float get_deadzonevalue(float stick_value, float min, float max);

	/*
	* 리맵핑 해주는 함수
	* @param val	리맵핑 할 변수
	* @param in1	리맵핑 할 변수가 가질 수 있는 최소값
	* @param in2	리맵핑 할 변수가 가질 수 있는 최대값
	* @param out1	리맵핑 되고 난 후 범위의 최소값
	* @param out2	리맵핑 되고 난 후 범위의 최대값
	* @return 리맵핑 되어 나온 값
	*/
	float remap(float val, float in1, float in2, float out1, float out2);

	/*
	* 파라매터를 로드해주는 함수
	* 지금은 일단 하드코딩함
	*/
	void CLAW_load_Parameter();

	void set_param_control_default(BUS_Param_control *param, double value);

	/*
	*
	*/
	void CLAW_copy_topics();

	/*
	* 픽스호크의 비행모드를 제어법칙의 비행모드와 대응 시켜주는 테이블을 생성하는 함수
	*/
	void CLAW_create_flt_mode_table();

	void CLAW_set_flt_mode();

	void CLAW_change_flt_mode();

	void init_param();

	void init_waypoints();

	void CLAW_set_wp();

	void CLAW_set_offboard_commands();

	void CLAW_set_offboard_axis_mode();

	bool _armed{ false };
};
