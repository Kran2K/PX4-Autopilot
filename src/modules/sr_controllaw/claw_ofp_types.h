/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: claw_ofp_types.h
 *
 * Code generated for Simulink model 'claw_ofp'.
 *
 * Model version                  : 4.153
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Tue Feb 21 14:13:44 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives:
 *    1. Safety precaution
 *    2. Execution efficiency
 *    3. Traceability
 * Validation result: Not run
 */

#ifndef RTW_HEADER_claw_ofp_types_h_
#define RTW_HEADER_claw_ofp_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawInSensor_
#define DEFINED_TYPEDEF_FOR_BUS_ClawInSensor_

/* Sensor data from OFP */
typedef struct {
  /* roll angle (deg) */
  real32_T roll;

  /* pitch angle (deg) */
  real32_T pitch;

  /* true heading angle (deg) */
  real32_T heading;

  /* roll angle rate (deg/s) */
  real32_T roll_rate;

  /* pitch angle rate (deg/s) */
  real32_T pitch_rate;

  /* yaw angle rate (deg/s) */
  real32_T yaw_rate;

  /* acceleration of x-axis in body frame(forward is positive) (m/s/s) */
  real32_T accel_x;

  /* acceleration of y-axis in body frame(right-wing is positive) (m/s/s) */
  real32_T accel_y;

  /* acceleration of z-axis in body frame(down is positive) (m/s/s) */
  real32_T accel_z;

  /* Vn, North velocity (m/s) */
  real32_T vel_north;

  /* Ve, East velocity (m/s) */
  real32_T vel_east;

  /* Vd, Down velocity (m/s) */
  real32_T vel_down;

  /* aircraft lattitude (deg) */
  real_T ac_lat;

  /* aircraft longitude (deg) */
  real_T ac_lon;

  /* gps altitude (m) */
  real32_T ac_alt_gps;

  /* calibrated air speed (m/s) */
  real32_T cas;

  /* true air speed (m/s) */
  real32_T tas;

  /* baro altitude from pressure sensor (m) */
  real32_T alt_baro;

  /* altitude rate from the baro altimeter (m/s) */
  real32_T alt_rate;

  /* outer air temperature (°C) */
  real32_T oat;

  /* total air temperature (°C) */
  real32_T tat;

  /* gimbal azimuth (deg) */
  real32_T gimbal_azimuth;

  /* gimbal elevation */
  real32_T gimbal_elevation;

  /* AGL(Above Ground Level) Sensor distance such as LiDAR, Ultrasonic, etc.. */
  real32_T alt_agl;
} BUS_ClawInSensor;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawInOFP_
#define DEFINED_TYPEDEF_FOR_BUS_ClawInOFP_

/* processed data from OFP */
typedef struct {
  /* Arm/Disarm flag
     0=Disarmed(default)
     1=Armed

     for landing detector transition. */
  boolean_T armed;

  /* reference point latitude (deg)
     Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
     상대 위치 계산을 위한 기준점
     부팅후 GPS최초 정상 수신하는 경우, 그 시점의 위경도를 기준점으로 업데이트해준다. */
  real_T ref_lat;

  /* reference point longitude (deg)
     Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
     상대 위치 계산을 위한 기준점
     부팅후 GPS최초 정상 수신하는 경우, 그 시점의 위경도를 기준점으로 업데이트해준다. */
  real_T ref_lon;

  /* reference point altitude (m)
     Position of reference point (local NED frame origin) in global (GPS / WGS84) frame */
  real32_T ref_alt;

  /* gps available
     0 : Not available
     1 : Available */
  boolean_T gps_available;

  /* air data available
     0 : Not available
     1 : Available */
  boolean_T air_data_available;

  /* AGL Sensor availablility
     0-No AGL sensor
     1-AGL sensor available */
  boolean_T agl_available;

  /* link(uplink) loss
     0: Normal
     1: uplink(command) loss */
  boolean_T link_loss;
} BUS_ClawInOFP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawInCmd_
#define DEFINED_TYPEDEF_FOR_BUS_ClawInCmd_

/* Command data from OFP (Tele-Command) */
typedef struct {
  /* flight mode
     비행모드로 실제 동작은 claw_mode 출력을 확인해야 함.
     CLAW_MODE_STANDBY       = 0; % Standby, 대기 모드
     CLAW_MODE_STICK_MANUAL  = 1; % Stick Manual, 조종간 수동 모드
     CLAW_MODE_STICK_RATE    = 2; % Stick Rate, 조종간 각속도 모드(roll rate, pitch rate, yaw rate, throttle)
     CLAW_MODE_STICK_ATT     = 3; % Stick Attitude, 조종간 자세 모드(roll angle, pitch angle, yaw rate, throttle)
     CLAW_MODE_STICK_ALT     = 4; % Stick Altitude, 조종간 고도 모드(roll angle, pitch angle, yaw rate, vertical speed)
     CLAW_MODE_STICK_POS     = 5; % Stick Position mode, 조종간 위치(GPS) 모드(vx, vy, yaw rate, vz)
     CLAW_MODE_POINT_NAV     = 6; % Point Navigation, 점항법(lat, lon, alt, spd, hdg)
     CLAW_MODE_HOLD          = 7; % Hold mode, 호버 모드, 현재 위치와 고도에서 호버링
     CLAW_MODE_WAYPOINT      = 8; % Waypoint flight, 항로점 비행모드(lat, lon, alt, spd, hdg, pass_type)
     CLAW_MODE_ORBIT         = 9; % Orbit, 항로점 선회 비행, 항로점 주위로 원선회비행하면서 헤딩은 중심점을 향함.(lat, lon, alt, spd, radius)
     CLAW_MODE_RETURN        = 10; % Return mode, 리턴홈 위치로 복귀(lat, lon, alt, spd)
     CLAW_MODE_TAKEOFF       = 11; % Takeoff, 자동이륙(lat, lon, climb rate)
     CLAW_MODE_LAND          = 12; % Landing, 자동착륙(lat, lon)
     CLAW_MODE_TAXING        = 13; % Taxing, 활주(lat, lon, spd)
     CLAW_MODE_CAMERA_GUIDE  = 14; % Camera Guided Flight, 카메라 유도모드(lat, lon, spd)
     CLAW_MODE_KNOB          = 15; % Knob mode, 노브모드(spd, alt, hdg)
     CLAW_MODE_OFFBOARD     = 16; % Offboard mode, 외부입력 모드(전체 기능 사용) */
  uint16_T flight_mode;

  /* aileron command to test manual mode */
  real32_T ail_cmd;
  real32_T ele_cmd;
  real32_T rud_cmd;

  /* roll rate command (deg/s) */
  real32_T roll_rate_cmd;

  /* pitch rate command (deg/s) */
  real32_T pitch_rate_cmd;

  /* yaw rate command (deg/s) */
  real32_T yaw_rate_cmd;

  /* thrust command [0-1] */
  real32_T thr_cmd;

  /* roll angle command (deg) */
  real32_T roll_cmd;

  /* pitch angle command (deg) */
  real32_T pitch_cmd;

  /* heading command (deg) */
  real32_T hdg_cmd;

  /* altitude command (m) */
  real32_T alt_cmd;

  /* speed x-axis command (positive is forward) (m/s) */
  real32_T vel_x_cmd;

  /* speed y-axis command (right is positive) (m/s) */
  real32_T vel_y_cmd;

  /* speed z-axis command (down is positive) (m/s) */
  real32_T vel_z_cmd;

  /* waypoint lattitude (deg) */
  real_T wp_lat[3];

  /* waypoint longitude (deg) */
  real_T wp_lon[3];

  /* waypoint altitude (m) */
  real32_T wp_alt[3];

  /* waypoint speed (m/s) */
  real32_T wp_spd[3];

  /* waypoint loiter type
     STANAG Unique ID=0041.04

     0 = No loitering (by Moon)
     1 = Circular    (available)
     2 = Racetrack (Not available)
     3 = Figure 8    (Not available)
     4 = Hover       (available)
     5 = ATC Hold */
  uint16_T wp_loiter_type;

  /* waypoint loiter radius (m)
     STANAG Unique ID=0041.05 */
  real32_T wp_loiter_radius;

  /* waypoint loiter length
     STANAG Unique ID=0041.06

     Used for Racetrack and Figure 8 to define length of pattern, centred around the
     Loiter Point (defined in Message #2002) in the direction of the Loiter Bearing. */
  real32_T wp_loiter_length;

  /* waypoint loiter bearing
     STANAG Unique ID=0041.07

     The bearing of the loiter pattern, referenced to the Loiter Point (defined in
     Message #2002), from True North. */
  real32_T wp_loiter_bearing;

  /* waypoint loiter direction
     STANAG Unique ID=0041.08
     0 = Vehicle Dependent
     1 = Clockwise
     2 = Counter-Clockwise
     3 = Into the wind */
  uint16_T wp_loiter_dir;

  /* Altitude Change Behaviour
     STANAG Unique ID=0041.15

     0 = Manual
     1 = Critical Altitude
     2 = Gradual
     3 = Max Rate (ROC/ROD)
     4 = UAV Dependent
     (Optimum Performance)
     5 = Not Defined */
  uint16_T wp_alt_behaviour;

  /* waypoint flying behaviour
     STANAG Unique ID=2004.15

     0 = Flyby
     1 = Flyover
     2 = Vehicle Discretion
     (added for backwards compatibility to Edition 2) */
  uint16_T wp_flying_behaviour;

  /* Heading command override in Waypoint Mode (only for rotorcraft)
     0 : Heading follows waypoint track heading
     1 : Heading follows manual hdg_cmd */
  boolean_T wp_hdg_cmd_override;

  /* For offboard mode
     LAT_AXIS_MAN            = uint8(0); % Manual
     LAT_AXIS_RATE           = uint8(1); % Rate
     LAT_AXIS_ATT            = uint8(2); % Attitude (Pitch)
     LAT_AXIS_ACCEL          = uint8(3); % Acceleration
     LAT_AXIS_VEL            = uint8(4); % Velocity
     LAT_AXIS_POS            = uint8(5); % Position (Guidance)
     LAT_AXIS_POSVEL     = uint8(6); % Position with velocity limit */
  uint16_T lat_axis_mode;

  /* For offboard mode
     LON_AXIS_MAN           = uint8(0); % Manual
     LON_AXIS_RATE          = uint8(1); % Rate
     LON_AXIS_ATT           = uint8(2); % Attitude (Pitch)
     LON_AXIS_ACCEL         = uint8(3); % Acceleration
     LON_AXIS_VEL           = uint8(4); % Velocity
     LON_AXIS_POS           = uint8(5); % Position (Guidance)
     LON_AXIS_POSVEL     = uint8(6); % Position with velocity limit */
  uint16_T lon_axis_mode;

  /* For offboard mode
     DIR_AXIS_MAN    = uint8(0); % Manual
     DIR_AXIS_RATE   = uint8(1); % Rate
     DIR_AXIS_HDG    = uint8(2); % Heading
     DIR_AXIS_HDG_RATE    = uint8(3); % Heading with rate limit
     DIR_AXIS_GD     = uint8(4);  % Guidance (Course) */
  uint16_T dir_axis_mode;

  /* For offboard mode
     VERT_AXIS_MAN   = uint8(0); % Throttle Manual
     VERT_AXIS_VEL   = uint8(1); % Throttle Velocity
     VERT_AXIS_ALT   = uint8(2); % Altitude
     VERT_AXIS_ALTVEL = uint8(3); % Altitude with velocity limit
     VERT_AXIS_GD    = uint8(4); % Throttle Guidance (Vertical speed/path control) */
  uint16_T vert_axis_mode;
} BUS_ClawInCmd;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_Param_control_
#define DEFINED_TYPEDEF_FOR_BUS_Param_control_

typedef struct {
  /* P,I,D gain 전체를 조정 */
  real_T K_gain;
  real_T P_gain;
  real_T I_gain;
  real_T D_gain;

  /* Feed-Forward */
  real_T FF_gain;

  /* output max */
  real_T output_max;

  /* output min */
  real_T output_min;

  /* Integrator max */
  real_T integ_max;

  /* integrator min */
  real_T integ_min;

  /* Low-pass filter frequency */
  real_T LPF_freq;

  /* Differential filter frequency */
  real_T DFilter_freq;
} BUS_Param_control;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawInTest_
#define DEFINED_TYPEDEF_FOR_BUS_ClawInTest_

/* 제어이득 튜닝 및 시험용 데이터 입력
   tune_on
   1: 튜닝값 적용
   0: 튜닝값 미적용
   tune_on 이외 값은 모두 1로 초기화해야함.
   (사유: xxx_loop 내부 파라미터가 내부 제어기 이득에 곱해지기 때문에 0으로 입력될 경우, 제어 불가) */
typedef struct {
  boolean_T tune_on;
  BUS_Param_control roll_loop;
  BUS_Param_control roll_rate_loop;
  BUS_Param_control pitch_loop;
  BUS_Param_control pitch_rate_loop;
  BUS_Param_control hdg_loop;
  BUS_Param_control yaw_rate_loop;
  BUS_Param_control Pos_loop;
  BUS_Param_control Vxy_loop;
  BUS_Param_control Alt_loop;
  BUS_Param_control Vz_loop;
} BUS_ClawInTest;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawOutAttControl_
#define DEFINED_TYPEDEF_FOR_BUS_ClawOutAttControl_

/* Attitude Control output to OFP
   자세 제어 출력 */
typedef struct {
  /* motor output

     Quadcopter
     0 : forward right(CCW)
     1 : rear left(CCW)
     2 : forward left(CW)
     3 : rear right(CW) */
  real32_T motor_out[16];

  /* actuator out

     0:Throttle

     1~4 Single helicopter
     1:MR_theta0
     2:MR_A1
     3:MR_B1
     4:TR_theta0

     5~10 Intermeshing rotor
     5: Right Collective
     6: Right A1
     7: Right B1
     8: Left Collective
     9: Left A1
     10: Left B1

     11~16 Spare */
  real32_T act_out[16];

  /* roll angle reference (actual command) */
  real32_T roll_ref;
  real32_T roll_error;

  /* roll angle integrator */
  real32_T roll_int;

  /* roll rate reference(actual command) */
  real32_T roll_rate_ref;

  /* roll rate integrator */
  real32_T roll_rate_int;

  /* roll acceleration command (to control mixer) */
  real32_T roll_accel_out;
  real32_T pitch_ref;
  real32_T pitch_error;

  /* pitch angle integrator */
  real32_T pitch_int;
  real32_T pitch_rate_ref;

  /* pitch rate integrator */
  real32_T pitch_rate_int;

  /* pitch acceleration command (to control mixer) */
  real32_T pitch_accel_out;

  /* heading reference (actual command) */
  real32_T hdg_ref;
  real32_T hdg_error;
  real32_T hdg_int;

  /* yaw rate reference (actual command) */
  real32_T yaw_rate_ref;

  /* yaw rate integrator */
  real32_T yaw_rate_int;

  /* yaw acceleration command (to control mixer) */
  real32_T yaw_accel_out;
  real32_T thr_cmd;
} BUS_ClawOutAttControl;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawOutCmd_
#define DEFINED_TYPEDEF_FOR_BUS_ClawOutCmd_

typedef struct {
  BUS_ClawInCmd raw;

  /* flight mode
     비행모드로 실제 동작은 claw_mode 출력을 확인해야 함.
     CLAW_MODE_STANDBY       = 0; % Standby, 대기 모드
     CLAW_MODE_STICK_MANUAL  = 1; % Stick Manual, 조종간 수동 모드
     CLAW_MODE_STICK_RATE    = 2; % Stick Rate, 조종간 각속도 모드(roll rate, pitch rate, yaw rate, throttle)
     CLAW_MODE_STICK_ATT     = 3; % Stick Attitude, 조종간 자세 모드(roll angle, pitch angle, yaw rate, throttle)
     CLAW_MODE_STICK_ALT     = 4; % Stick Altitude, 조종간 고도 모드(roll angle, pitch angle, yaw rate, vertical speed)
     CLAW_MODE_STICK_POS     = 5; % Stick Position mode, 조종간 위치(GPS) 모드(vx, vy, yaw rate, vz)
     CLAW_MODE_POINT_NAV     = 6; % Point Navigation, 점항법(lat, lon, alt, spd, hdg)
     CLAW_MODE_HOLD          = 7; % Hold mode, 호버 모드, 현재 위치와 고도에서 호버링
     CLAW_MODE_WAYPOINT      = 8; % Waypoint flight, 항로점 비행모드(lat, lon, alt, spd, hdg, pass_type)
     CLAW_MODE_ORBIT         = 9; % Orbit, 항로점 선회 비행, 항로점 주위로 원선회비행하면서 헤딩은 중심점을 향함.(lat, lon, alt, spd, radius)
     CLAW_MODE_RETURN        = 10; % Return mode, 리턴홈 위치로 복귀(lat, lon, alt, spd)
     CLAW_MODE_TAKEOFF       = 11; % Takeoff, 자동이륙(lat, lon, climb rate)
     CLAW_MODE_LAND          = 12; % Landing, 자동착륙(lat, lon)
     CLAW_MODE_TAXING        = 13; % Taxing, 활주(lat, lon, spd)
     CLAW_MODE_CAMERA_GUIDE  = 14; % Camera Guided Flight, 카메라 유도모드(lat, lon, spd)
     CLAW_MODE_KNOB          = 15; % Knob mode, 노브모드(spd, alt, hdg) */
  uint16_T flight_mode;

  /* aileron command to test manual mode */
  real32_T ail_cmd;
  real32_T ele_cmd;
  real32_T rud_cmd;

  /* roll rate command (deg/s) */
  real32_T roll_rate_cmd;

  /* pitch rate command (deg/s) */
  real32_T pitch_rate_cmd;

  /* yaw rate command (deg/s) */
  real32_T yaw_rate_cmd;

  /* thrust command [0-1] */
  real32_T thr_cmd;

  /* roll angle command (deg) */
  real32_T roll_cmd;

  /* pitch angle command (deg) */
  real32_T pitch_cmd;

  /* heading command (deg) */
  real32_T hdg_cmd;

  /* altitude command (m) */
  real32_T alt_cmd;

  /* speed x-axis command (positive is forward) (m/s) */
  real32_T vel_x_cmd;

  /* speed y-axis command (right is positive) (m/s) */
  real32_T vel_y_cmd;

  /* speed z-axis command (down is positive) (m/s) */
  real32_T vel_z_cmd;

  /* waypoint lattitude (deg) */
  real_T wp_lat[3];

  /* waypoint longitude (deg) */
  real_T wp_lon[3];

  /* waypoint altitude (m) */
  real32_T wp_alt[3];

  /* waypoint speed (m/s) */
  real32_T wp_spd[3];

  /* waypoint loiter type
     STANAG Unique ID=0041.04
     0 = No loitering (by Moon)
     1 = Circular    (available)
     2 = Racetrack (Not available)
     3 = Figure 8    (Not available)
     4 = Hover       (available)
     5 = ATC Hold */
  uint16_T wp_loiter_type;

  /* waypoint loiter radius (m)
     STANAG Unique ID=0041.05 */
  real32_T wp_loiter_radius;

  /* waypoint loiter length
     STANAG Unique ID=0041.06

     Used for Racetrack and Figure 8 to define length of pattern, centred around the
     Loiter Point (defined in Message #2002) in the direction of the Loiter Bearing. */
  real32_T wp_loiter_length;

  /* waypoint loiter bearing
     STANAG Unique ID=0041.07

     The bearing of the loiter pattern, referenced to the Loiter Point (defined in
     Message #2002), from True North. */
  real32_T wp_loiter_bearing;

  /* waypoint loiter direction
     STANAG Unique ID=0041.08
     0 = Vehicle Dependent
     1 = Clockwise
     2 = Counter-Clockwise
     3 = Into the wind */
  uint16_T wp_loiter_dir;

  /* Altitude Change Behaviour
     STANAG Unique ID=0041.15

     0 = Manual
     1 = Critical Altitude
     2 = Gradual
     3 = Max Rate (ROC/ROD)
     4 = UAV Dependent
     (Optimum Performance)
     5 = Not Defined */
  uint16_T wp_alt_behaviour;

  /* waypoint flying behaviour
     STANAG Unique ID=2004.15

     0 = Flyby
     1 = Flyover
     2 = Vehicle Discretion
     (added for backwards compatibility to Edition 2) */
  uint16_T wp_flying_behaviour;

  /* Heading command override in Waypoint Mode (only for rotorcraft)
     0 : Heading follows waypoint track heading
     1 : Heading follows manual hdg_cmd */
  boolean_T wp_hdg_cmd_override;

  /* waypoint north (m) from ref_lat. */
  real_T wp_x_local[3];

  /* waypoint east (m) from ref_lon */
  real_T wp_y_local[3];

  /* waypoint z from ref_alt(ClawInOFP) (m)
     down is positive
     For example, altitude above the takeoff altitude. */
  real_T wp_z_local[3];
} BUS_ClawOutCmd;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawOutState_
#define DEFINED_TYPEDEF_FOR_BUS_ClawOutState_

/* Vehicle's states data (sensor + decision + estimator)
   제어법칙 내부에서 사용하는 센서 데이터(필터링 및 좌표변환 등을 포함) */
typedef struct {
  BUS_ClawInSensor sensor_raw;

  /* roll angle (deg) */
  real32_T roll;

  /* pitch angle (deg) */
  real32_T pitch;

  /* true heading angle (deg) */
  real32_T heading;

  /* roll angle rate (deg/s) */
  real32_T roll_rate;

  /* pitch angle rate (deg/s) */
  real32_T pitch_rate;

  /* yaw angle rate (deg/s) */
  real32_T yaw_rate;

  /* acceleration of x-axis in body frame(forward is positive) (m/s/s) */
  real32_T accel_x;

  /* acceleration of y-axis in body frame(right-wing is positive) (m/s/s) */
  real32_T accel_y;

  /* acceleration of z-axis in body frame(down is positive) (m/s/s) */
  real32_T accel_z;

  /* aircraft x position in local NED frame */
  real_T ac_x_local;

  /* aircraft y position in local NED frame */
  real_T ac_y_local;

  /* aircraft z position in local NED frame from ref_alt (positive is down direction) */
  real_T ac_z_local;

  /* heading from ref_hdg in local frame (deg) */
  real_T ac_psi_local;

  /* forward speed along with x-axis (horizontal plane) */
  real32_T fwd_spd;

  /* lateral speed along with y-axis (horizontal plane) */
  real32_T lat_spd;

  /* course heading from vn, ve */
  real32_T crs_hdg;

  /* path angle from vn, ve, vd */
  real32_T path_angle;

  /* true if vehicle is currently landed on the ground
     from landing detection. */
  boolean_T landed;

  /* from landing detection. */
  boolean_T cond_gnd_contact;

  /* from landing detection. */
  boolean_T cond_maybe_land;

  /* from landing detection. */
  boolean_T cond_not_rotating;

  /* from landing detection. */
  boolean_T cond_not_moving;

  /* armed flag
     0=disarmed
     1=armed */
  boolean_T armed;
} BUS_ClawOutState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawOutModeManager_
#define DEFINED_TYPEDEF_FOR_BUS_ClawOutModeManager_

/* Mode Management Output
 */
typedef struct {
  uint32_T state_id;

  /* control law mode
     제어법칙모드, 실제 동작중인 제어법칙 모드를 나타냄
     CLAW_MODE_STANDBY       = 0; % Standby, 대기 모드
     CLAW_MODE_STICK_MANUAL  = 1; % Stick Manual, 조종간 수동 모드
     CLAW_MODE_STICK_RATE    = 2; % Stick Rate, 조종간 각속도 모드(roll rate, pitch rate, yaw rate, throttle)
     CLAW_MODE_STICK_ATT     = 3; % Stick Attitude, 조종간 자세 모드(roll angle, pitch angle, yaw rate, throttle)
     CLAW_MODE_STICK_ALT     = 4; % Stick Altitude, 조종간 고도 모드(roll angle, pitch angle, yaw rate, vertical speed)
     CLAW_MODE_STICK_POS     = 5; % Stick Position mode, 조종간 위치(GPS) 모드(vx, vy, yaw rate, vz)
     CLAW_MODE_POINT_NAV     = 6; % Point Navigation, 점항법(lat, lon, alt, spd, hdg)
     CLAW_MODE_HOLD          = 7; % Hold mode, 호버 모드, 현재 위치와 고도에서 호버링
     CLAW_MODE_WAYPOINT      = 8; % Waypoint flight, 항로점 비행모드(lat, lon, alt, spd, hdg, pass_type)
     CLAW_MODE_ORBIT         = 9; % Orbit, 항로점 선회 비행, 항로점 주위로 원선회비행하면서 헤딩은 중심점을 향함.(lat, lon, alt, spd, radius)
     CLAW_MODE_RETURN        = 10; % Return mode, 리턴홈 위치로 복귀(lat, lon, alt, spd)
     CLAW_MODE_TAKEOFF       = 11; % Takeoff, 자동이륙(lat, lon, climb rate)
     CLAW_MODE_LAND          = 12; % Landing, 자동착륙(lat, lon)
     CLAW_MODE_TAXING        = 13; % Taxing, 활주(lat, lon, spd)
     CLAW_MODE_CAMERA_GUIDE  = 14; % Camera Guided Flight, 카메라 유도모드(lat, lon, spd) */
  uint16_T claw_mode;
  uint16_T roll_axis_mode;
  uint16_T pitch_axis_mode;
  uint16_T yaw_axis_mode;
  uint16_T vert_axis_mode;

  /* GD_TYPE_NOGD     = uint8(0); % No Guidance
     GD_TYPE_POSHOLD  = uint8(1); % Position Hold (x,y)
     GD_TYPE_TOWP     = uint8(2); % Fly to Waypoint (x,y,v)
     GD_TYPE_TRACKLEG = uint8(3); % Track Waypoint Leg (Cross-track) (x1,y1,x2,y2,v)
     GD_TYPE_ORBIT   = uint8(4); % Orbit guidance (x,y,r,dir,v) */
  uint16_T guide_type;
  real_T wp_x_local[3];
  real_T wp_y_local[3];
  real_T wp_z_local[3];
  real32_T wp_spd[3];
  real32_T wp_radius[3];

  /* waypoint orbit direction
     0: CCW
     1: CW (STANAG) */
  boolean_T wp_orbit_dir[3];
  uint16_T wp_loiter_type;
  boolean_T orbit_flag;
  real32_T roll_cmd;
  real32_T pitch_cmd;
  real32_T hdg_cmd;
  real32_T alt_cmd;

  /* roll rate command */
  real32_T roll_rate_cmd;

  /* pitch rate command */
  real32_T pitch_rate_cmd;

  /* yaw rate command */
  real32_T yaw_rate_cmd;

  /* aileron command to test manual mode */
  real32_T ail_cmd;
  real32_T ele_cmd;
  real32_T rud_cmd;

  /* speed x-axis command (positive is forward) */
  real32_T vel_x_cmd;

  /* speed y-axis command (right is positive) */
  real32_T vel_y_cmd;

  /* speed z-axis command (down is positive) */
  real32_T vel_z_cmd;

  /* thrust command[0-1] */
  real32_T thr_cmd;
} BUS_ClawOutModeManager;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_ClawOutPosControl_
#define DEFINED_TYPEDEF_FOR_BUS_ClawOutPosControl_

/* Position Control(guidance) Output */
typedef struct {
  real_T pos_ref_x_cur;
  real_T pos_ref_y_cur;
  real_T pos_ref_z_cur;
  real_T pos_err_x;
  real_T pos_err_y;
  real_T pos_err_z;

  /* velocity x reference */
  real32_T vel_ref_x;

  /* velocity y reference */
  real32_T vel_ref_y;

  /* velocity z-axis command */
  real32_T vel_ref_z;

  /* velocity x integrator */
  real32_T vx_int;

  /* velocity x integrator */
  real32_T vy_int;
  real32_T vz_int;
  real32_T acc_ref_x;
  real32_T acc_ref_y;
  real32_T acc_ref_z;
  real32_T roll_cmd;
  real32_T pitch_cmd;
  real32_T yaw_cmd;
  real32_T yaw_rate;

  /* Z-axis acceleration in body frame
     Thrust compensation for cruise flight */
  real32_T az_body;
  boolean_T pass_flag;
  real32_T dist_to_go;
  real32_T dist_to_wp;
  real32_T dist_from;
  uint16_T time_from;
  real32_T wp_leg_hdg;
  real32_T xtrk_hdg_error;
  real32_T xtrk_dist;
  real32_T hdg_to_wp;
  real32_T xtrk_hdg_int;
} BUS_ClawOutPosControl;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_claw_ofp_T RT_MODEL_claw_ofp_T;

#endif                                 /* RTW_HEADER_claw_ofp_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
