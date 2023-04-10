/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: claw_ofp.c
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

#include "claw_ofp.h"
#include "rtwtypes.h"
#include "claw_ofp_types.h"
#include "claw_ofp_private.h"
#include <math.h>
#include <float.h>
#define claw_ofp_IN_Active             (1U)
#define claw_ofp_IN_AltHold            (1U)
#define claw_ofp_IN_Circle             (1U)
#define claw_ofp_IN_Descending         (2U)
#define claw_ofp_IN_Ground_Contact     (1U)
#define claw_ofp_IN_Heading_Cmd_Auto   (1U)
#define claw_ofp_IN_Heading_Cmd_Manual (2U)
#define claw_ofp_IN_Hover              (2U)
#define claw_ofp_IN_InActive           (2U)
#define claw_ofp_IN_In_Air             (2U)
#define claw_ofp_IN_Inactive           (2U)
#define claw_ofp_IN_Init               (1U)
#define claw_ofp_IN_Landed             (3U)
#define claw_ofp_IN_Maybe_Landed       (4U)
#define claw_ofp_IN_Motor_startup      (1U)
#define claw_ofp_IN_Motor_stop         (3U)
#define claw_ofp_IN_NO_ACTIVE_CHILD    (0U)
#define claw_ofp_IN_Pos_Hold           (1U)
#define claw_ofp_IN_TakeoffHold        (2U)
#define claw_ofp_IN_Velocity           (2U)
#define claw_ofp_IN_Waypoint           (3U)
#define claw_ofp_IN_Yaw_rate_cmd       (2U)
#define claw_ofp_IN_altitude_climb     (2U)
#define claw_ofp_IN_altitude_hold      (3U)
#define claw_ofp_IN_counter_up         (1U)
#define claw_ofp_IN_init               (2U)
#define claw_ofp_IN_waypoint_following (3U)

/* Block signals (default storage) */
B_claw_ofp_T claw_ofp_B;

/* Block states (default storage) */
DW_claw_ofp_T claw_ofp_DW;

/* Real-time model */
static RT_MODEL_claw_ofp_T claw_ofp_M_;
RT_MODEL_claw_ofp_T *const claw_ofp_M = &claw_ofp_M_;

/* Forward declaration for local functions */
static real_T claw_ofp_rt_modd(real_T u0, real_T u1);
static real_T claw_ofp_rt_hypotd(real_T u0, real_T u1);
static void enter_atomic_waypoint_following(const BUS_ClawOutCmd
  *BusConversion_InsertedFor_point);
real_T look1_pbinlagpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T prevIndex[], uint32_T maxIndex)
{
  real_T frac;
  real_T y;
  real_T yL_0d0;
  uint32_T bpIdx;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'on'
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'on'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'on'
   */
  if (u0 < bp0[maxIndex]) {
    /* Binary Search using Previous Index */
    bpIdx = prevIndex[0UL];
    iLeft = 0UL;
    iRght = maxIndex;
    found = 0UL;
    while (found == 0UL) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx - 1UL;
        bpIdx = ((bpIdx + iLeft) - 1UL) >> 1UL;
      } else if (u0 < bp0[bpIdx + 1UL]) {
        found = 1UL;
      } else {
        iLeft = bpIdx + 1UL;
        bpIdx = ((bpIdx + iRght) + 1UL) >> 1UL;
      }
    }

    frac = (u0 - bp0[bpIdx]) / (bp0[bpIdx + 1UL] - bp0[bpIdx]);
  } else {
    bpIdx = maxIndex;
    frac = 0.0;
  }

  prevIndex[0UL] = bpIdx;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  if (bpIdx == maxIndex) {
    y = table[bpIdx];
  } else {
    yL_0d0 = table[bpIdx];
    y = (table[bpIdx + 1UL] - yL_0d0) * frac + yL_0d0;
  }

  return y;
}

real_T rt_hypotd(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Output and update for function-call system:
 *    '<S169>/Active.Horizontal.dist2d'
 *    '<S178>/Active.Horizontal.dist2d'
 */
void claw_ofp_ActiveHorizontaldist2d(real_T rtu_wp_x, real_T rtu_wp_y, real_T
  rtu_ac_x, real_T rtu_ac_y, real_T *rty_dist)
{
  /* Math: '<S170>/Hypot' incorporates:
   *  Sum: '<S170>/Subtract'
   *  Sum: '<S170>/Subtract1'
   */
  *rty_dist = rt_hypotd(rtu_wp_x - rtu_ac_x, rtu_wp_y - rtu_ac_y);
}

static real_T claw_ofp_rt_modd(real_T u0, real_T u1)
{
  real_T y;
  boolean_T yEq;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else {
    y = fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > floor(u1))) {
      claw_ofp_B.q = fabs(u0 / u1);
      yEq = (fabs(claw_ofp_B.q - floor(claw_ofp_B.q + 0.5)) <= DBL_EPSILON *
             claw_ofp_B.q);
    }

    if (yEq) {
      y = 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

static real_T claw_ofp_rt_hypotd(real_T u0, real_T u1)
{
  real_T y;
  claw_ofp_B.a = fabs(u0);
  claw_ofp_B.b = fabs(u1);
  if (claw_ofp_B.a < claw_ofp_B.b) {
    claw_ofp_B.a /= claw_ofp_B.b;
    y = sqrt(claw_ofp_B.a * claw_ofp_B.a + 1.0) * claw_ofp_B.b;
  } else if (claw_ofp_B.a > claw_ofp_B.b) {
    claw_ofp_B.b /= claw_ofp_B.a;
    y = sqrt(claw_ofp_B.b * claw_ofp_B.b + 1.0) * claw_ofp_B.a;
  } else {
    y = claw_ofp_B.a * 1.4142135623730951;
  }

  return y;
}

/* Function for Chart: '<S152>/point_nav' */
static void enter_atomic_waypoint_following(const BUS_ClawOutCmd
  *BusConversion_InsertedFor_point)
{
  claw_ofp_B.FMMOUT_do.roll_axis_mode = 5U;
  claw_ofp_B.FMMOUT_do.pitch_axis_mode = 5U;
  claw_ofp_B.FMMOUT_do.guide_type = 2U;
  claw_ofp_B.FMMOUT_do.orbit_flag = false;

  /* BusCreator generated from: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_loiter_type =
    BusConversion_InsertedFor_point->raw.wp_loiter_type;

  /* Outport: '<Root>/OutState' */
  /*  Speed decceleration effect */
  claw_ofp_B.FMMOUT_do.wp_x_local[0] = claw_out_state.ac_x_local;

  /* BusCreator generated from: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_x_local[1] =
    BusConversion_InsertedFor_point->wp_x_local[1];
  claw_ofp_B.FMMOUT_do.wp_x_local[2] =
    BusConversion_InsertedFor_point->wp_x_local[2];

  /* Outport: '<Root>/OutState' */
  claw_ofp_B.FMMOUT_do.wp_y_local[0] = claw_out_state.ac_y_local;

  /* BusCreator generated from: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_y_local[1] =
    BusConversion_InsertedFor_point->wp_y_local[1];
  claw_ofp_B.FMMOUT_do.wp_y_local[2] =
    BusConversion_InsertedFor_point->wp_y_local[2];
}

/* Model step function */
void claw_ofp_step(void)
{
  real32_T rtb_Abs1_e;
  real32_T rtb_Abs_d;
  real32_T rtb_Limit_n;
  real32_T rtb_MatrixConcatenate_idx_0;
  real32_T rtb_MatrixConcatenate_idx_1;
  real32_T rtb_MatrixConcatenate_idx_2;
  real32_T rtb_Sum4_i2_tmp;
  real32_T rtb_Sw;
  real32_T rtb_Sw_m;
  real32_T rtb_pitch_angular_accel;
  int16_T rtb_Compare_eu_0;
  boolean_T rtb_AND_c;
  boolean_T rtb_Compare_jr;
  boolean_T rtb_Compare_o1;
  boolean_T rtb_cond_gnd_contact;
  boolean_T rtb_cond_maybe_land;
  static const real_T sincos_cordicLUT_n11_d[11] = { 0.78539816339744828,
    0.46364760900080609, 0.24497866312686414, 0.12435499454676144,
    0.06241880999595735, 0.031239833430268277, 0.015623728620476831,
    0.0078123410601011111, 0.0039062301319669718, 0.0019531225164788188,
    0.00097656218955931946 };

  static const real_T sincos_cordicLUT_n15_d[15] = { 0.78539816339744828,
    0.46364760900080609, 0.24497866312686414, 0.12435499454676144,
    0.06241880999595735, 0.031239833430268277, 0.015623728620476831,
    0.0078123410601011111, 0.0039062301319669718, 0.0019531225164788188,
    0.00097656218955931946, 0.00048828121119489829, 0.00024414062014936177,
    0.00012207031189367021, 6.1035156174208773E-5 };

  /* Logic: '<S291>/AND' incorporates:
   *  Abs: '<S291>/Abs'
   *  Abs: '<S291>/Abs1'
   *  Abs: '<S291>/Abs2'
   *  Constant: '<S293>/Constant'
   *  Constant: '<S294>/Constant'
   *  Constant: '<S298>/Constant'
   *  DataTypeConversion: '<S291>/Cast To Double'
   *  DataTypeConversion: '<S291>/Cast To Double1'
   *  DataTypeConversion: '<S291>/Cast To Double2'
   *  Inport: '<Root>/in_sensor'
   *  RelationalOperator: '<S293>/Compare'
   *  RelationalOperator: '<S294>/Compare'
   *  RelationalOperator: '<S298>/Compare'
   */
  rtb_AND_c = ((fabs(claw_in_sensor.vel_north) < 0.2) && (fabs
    (claw_in_sensor.vel_east) < 0.2) && (fabs(claw_in_sensor.vel_down) < 0.2));

  /* Logic: '<S291>/AND1' incorporates:
   *  Memory generated from: '<S2>/Memory'
   *  RelationalOperator: '<S291>/Relational Operator'
   */
  rtb_cond_gnd_contact = (rtb_AND_c && (claw_out_att_control.thr_cmd <
    claw_ofp_ConstB.Sum_e));

  /* Logic: '<S291>/AND3' incorporates:
   *  Abs: '<S291>/Abs3'
   *  Abs: '<S291>/Abs4'
   *  Abs: '<S291>/Abs5'
   *  Constant: '<S295>/Constant'
   *  Constant: '<S296>/Constant'
   *  Constant: '<S297>/Constant'
   *  Memory generated from: '<S2>/Memory'
   *  RelationalOperator: '<S295>/Compare'
   *  RelationalOperator: '<S296>/Compare'
   *  RelationalOperator: '<S297>/Compare'
   */
  rtb_Compare_o1 = (((real32_T)fabs
                     (claw_ofp_DW.Memory_1_PreviousInput.sensor_raw.roll_rate) <
                     20.0F) && ((real32_T)fabs
    (claw_ofp_DW.Memory_1_PreviousInput.sensor_raw.pitch_rate) < 20.0F) &&
                    ((real32_T)fabs
                     (claw_ofp_DW.Memory_1_PreviousInput.sensor_raw.yaw_rate) <
                     20.0F));

  /* Logic: '<S291>/AND2' incorporates:
   *  Memory generated from: '<S2>/Memory'
   *  RelationalOperator: '<S291>/Relational Operator3'
   */
  rtb_cond_maybe_land = (rtb_Compare_o1 && (claw_out_att_control.thr_cmd <
    claw_ofp_ConstB.Sum1_bz));

  /* Chart: '<S291>/Chart' incorporates:
   *  Inport: '<Root>/in_OFP'
   */
  if (claw_ofp_DW.is_active_c24_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c24_claw_ofp = 1U;
    claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_Landed;
    claw_ofp_B.landed_flag = 1.0;
  } else {
    switch (claw_ofp_DW.is_c24_claw_ofp) {
     case claw_ofp_IN_Ground_Contact:
      if (claw_ofp_DW.time2 > 0.25) {
        claw_ofp_DW.is_Ground_Contact = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_Maybe_Landed;
        claw_ofp_DW.is_Maybe_Landed = claw_ofp_IN_init;
        claw_ofp_DW.time3 = 0.0;
      } else if (claw_in_ofp.armed && ((!rtb_cond_gnd_contact) ||
                  (!rtb_cond_maybe_land))) {
        claw_ofp_DW.is_Ground_Contact = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_In_Air;
        claw_ofp_B.landed_flag = 0.0;
        claw_ofp_DW.is_In_Air = claw_ofp_IN_init;
        claw_ofp_DW.time1 = 0.0;
      } else if (claw_ofp_DW.is_Ground_Contact == 1U) {
        if ((!rtb_cond_gnd_contact) || (!rtb_cond_maybe_land)) {
          claw_ofp_DW.is_Ground_Contact = claw_ofp_IN_init;
          claw_ofp_DW.time2 = 0.0;
        } else {
          claw_ofp_DW.time2 += 0.005;
        }

        /* case IN_init: */
      } else if (rtb_cond_gnd_contact && rtb_cond_maybe_land) {
        claw_ofp_DW.is_Ground_Contact = claw_ofp_IN_counter_up;
      }
      break;

     case claw_ofp_IN_In_Air:
      claw_ofp_B.landed_flag = 0.0;
      if (!claw_in_ofp.armed) {
        claw_ofp_DW.is_In_Air = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_Landed;
        claw_ofp_B.landed_flag = 1.0;
      } else if (claw_ofp_DW.time1 > 0.35) {
        claw_ofp_DW.is_In_Air = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_Ground_Contact;
        claw_ofp_DW.is_Ground_Contact = claw_ofp_IN_init;
        claw_ofp_DW.time2 = 0.0;
      } else if (claw_ofp_DW.is_In_Air == 1U) {
        if (!rtb_cond_gnd_contact) {
          claw_ofp_DW.is_In_Air = claw_ofp_IN_init;
          claw_ofp_DW.time1 = 0.0;
        } else {
          claw_ofp_DW.time1 += 0.005;
        }

        /* case IN_init: */
      } else if (rtb_cond_gnd_contact) {
        claw_ofp_DW.is_In_Air = claw_ofp_IN_counter_up;
      }
      break;

     case claw_ofp_IN_Landed:
      claw_ofp_B.landed_flag = 1.0;
      if (claw_in_ofp.armed && ((!rtb_cond_gnd_contact) || (!rtb_cond_maybe_land)))
      {
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_In_Air;
        claw_ofp_B.landed_flag = 0.0;
        claw_ofp_DW.is_In_Air = claw_ofp_IN_init;
        claw_ofp_DW.time1 = 0.0;
      }
      break;

     default:
      /* case IN_Maybe_Landed: */
      if (claw_ofp_DW.time3 > 0.3) {
        claw_ofp_DW.is_Maybe_Landed = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_Landed;
        claw_ofp_B.landed_flag = 1.0;
      } else if (claw_in_ofp.armed && ((!rtb_cond_gnd_contact) ||
                  (!rtb_cond_maybe_land))) {
        claw_ofp_DW.is_Maybe_Landed = claw_ofp_IN_NO_ACTIVE_CHILD;
        claw_ofp_DW.is_c24_claw_ofp = claw_ofp_IN_In_Air;
        claw_ofp_B.landed_flag = 0.0;
        claw_ofp_DW.is_In_Air = claw_ofp_IN_init;
        claw_ofp_DW.time1 = 0.0;
      } else if (claw_ofp_DW.is_Maybe_Landed == 1U) {
        if ((!rtb_cond_gnd_contact) || (!rtb_cond_maybe_land)) {
          claw_ofp_DW.is_Maybe_Landed = claw_ofp_IN_init;
          claw_ofp_DW.time3 = 0.0;
        } else {
          claw_ofp_DW.time3 += 0.005;
        }

        /* case IN_init: */
      } else if (rtb_cond_gnd_contact && rtb_cond_maybe_land) {
        claw_ofp_DW.is_Maybe_Landed = claw_ofp_IN_counter_up;
      }
      break;
    }
  }

  /* End of Chart: '<S291>/Chart' */

  /* RelationalOperator: '<S135>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_Compare_jr = (claw_in_cmd.flight_mode < 1U);

  /* Chart: '<S153>/Standby' incorporates:
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c22_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c22_claw_ofp = 1U;
    claw_ofp_DW.is_c22_claw_ofp = claw_ofp_IN_Inactive;
  } else if (claw_ofp_DW.is_c22_claw_ofp == 1U) {
    if (!rtb_Compare_jr) {
      claw_ofp_DW.is_c22_claw_ofp = claw_ofp_IN_Inactive;
    } else {
      claw_ofp_B.FMMOUT_e.pitch_rate_cmd = 0.0F;
      claw_ofp_B.FMMOUT_e.roll_rate_cmd = 0.0F;
      claw_ofp_B.FMMOUT_e.yaw_rate_cmd = 0.0F;
      claw_ofp_B.FMMOUT_e.thr_cmd = 0.0F;
    }

    /* case IN_Inactive: */
  } else if (rtb_Compare_jr) {
    claw_ofp_DW.is_c22_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_e = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_e.state_id = 1110000UL;
    claw_ofp_B.FMMOUT_e.claw_mode = 0U;
    claw_ofp_B.FMMOUT_e.pitch_axis_mode = 0U;
    claw_ofp_B.FMMOUT_e.roll_axis_mode = 0U;
    claw_ofp_B.FMMOUT_e.vert_axis_mode = 0U;
  }

  /* End of Chart: '<S153>/Standby' */

  /* RelationalOperator: '<S140>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_Compare_jr = (claw_in_cmd.flight_mode == 1U);

  /* Abs: '<S82>/Abs' incorporates:
   *  Abs: '<S104>/Abs'
   *  Abs: '<S126>/Abs'
   *  Abs: '<S315>/Abs'
   *  Inport: '<Root>/in_OFP'
   */
  claw_ofp_B.Abs1_g = fabs(claw_in_ofp.ref_lat);

  /* Switch: '<S82>/Switch' incorporates:
   *  Abs: '<S82>/Abs'
   *  Bias: '<S82>/Bias'
   *  Bias: '<S82>/Bias1'
   *  Constant: '<S82>/Constant2'
   *  Constant: '<S83>/Constant'
   *  Inport: '<Root>/in_OFP'
   *  Math: '<S82>/Math Function1'
   *  RelationalOperator: '<S83>/Compare'
   */
  if (claw_ofp_B.Abs1_g > 180.0) {
    claw_ofp_B.Switch_o = claw_ofp_rt_modd(claw_in_ofp.ref_lat + 180.0, 360.0) +
      -180.0;
  } else {
    claw_ofp_B.Switch_o = claw_in_ofp.ref_lat;
  }

  /* End of Switch: '<S82>/Switch' */

  /* Abs: '<S79>/Abs1' */
  claw_ofp_B.Abs1 = fabs(claw_ofp_B.Switch_o);

  /* Switch: '<S79>/Switch' incorporates:
   *  Bias: '<S79>/Bias'
   *  Bias: '<S79>/Bias1'
   *  Constant: '<S70>/Constant'
   *  Constant: '<S70>/Constant1'
   *  Constant: '<S81>/Constant'
   *  Gain: '<S79>/Gain'
   *  Product: '<S79>/Divide1'
   *  RelationalOperator: '<S81>/Compare'
   *  Switch: '<S70>/Switch1'
   */
  if (claw_ofp_B.Abs1 > 90.0) {
    /* Signum: '<S79>/Sign1' */
    if (claw_ofp_B.Switch_o < 0.0) {
      claw_ofp_B.Sum = -1.0;
    } else {
      claw_ofp_B.Sum = (claw_ofp_B.Switch_o > 0.0);
    }

    /* End of Signum: '<S79>/Sign1' */
    claw_ofp_B.Switch_o = (-(claw_ofp_B.Abs1 + -90.0) + 90.0) * claw_ofp_B.Sum;
    rtb_Compare_eu_0 = 180;
  } else {
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S79>/Switch' */

  /* Sum: '<S70>/Sum' incorporates:
   *  Inport: '<Root>/in_OFP'
   */
  claw_ofp_B.Sum = (real_T)rtb_Compare_eu_0 + claw_in_ofp.ref_lon;

  /* Sum: '<S66>/Sum1' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = claw_in_cmd.wp_lat[0] - claw_ofp_B.Switch_o;

  /* Switch: '<S76>/Switch' incorporates:
   *  Abs: '<S76>/Abs'
   *  Bias: '<S76>/Bias'
   *  Bias: '<S76>/Bias1'
   *  Constant: '<S76>/Constant2'
   *  Constant: '<S77>/Constant'
   *  Math: '<S76>/Math Function1'
   *  RelationalOperator: '<S77>/Compare'
   */
  if (fabs(claw_ofp_B.rtb_Sum1_idx_0) > 180.0) {
    claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_rt_modd(claw_ofp_B.rtb_Sum1_idx_0 +
      180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S76>/Switch' */

  /* Abs: '<S73>/Abs1' */
  claw_ofp_B.Abs1 = fabs(claw_ofp_B.rtb_Sum1_idx_0);

  /* Switch: '<S73>/Switch' incorporates:
   *  Bias: '<S73>/Bias'
   *  Bias: '<S73>/Bias1'
   *  Constant: '<S69>/Constant'
   *  Constant: '<S69>/Constant1'
   *  Constant: '<S75>/Constant'
   *  Gain: '<S73>/Gain'
   *  Product: '<S73>/Divide1'
   *  RelationalOperator: '<S75>/Compare'
   *  Switch: '<S69>/Switch1'
   */
  if (claw_ofp_B.Abs1 > 90.0) {
    /* Signum: '<S73>/Sign1' */
    if (claw_ofp_B.rtb_Sum1_idx_0 < 0.0) {
      claw_ofp_B.Sum_j = -1.0;
    } else {
      claw_ofp_B.Sum_j = (claw_ofp_B.rtb_Sum1_idx_0 > 0.0);
    }

    /* End of Signum: '<S73>/Sign1' */
    claw_ofp_B.Abs1 = (-(claw_ofp_B.Abs1 + -90.0) + 90.0) * claw_ofp_B.Sum_j;
    rtb_Compare_eu_0 = 180;
  } else {
    claw_ofp_B.Abs1 = claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S73>/Switch' */

  /* Switch: '<S80>/Switch' incorporates:
   *  Abs: '<S80>/Abs'
   *  Bias: '<S80>/Bias'
   *  Bias: '<S80>/Bias1'
   *  Constant: '<S80>/Constant2'
   *  Constant: '<S84>/Constant'
   *  Math: '<S80>/Math Function1'
   *  RelationalOperator: '<S84>/Compare'
   */
  if (fabs(claw_ofp_B.Sum) > 180.0) {
    claw_ofp_B.Sum = claw_ofp_rt_modd(claw_ofp_B.Sum + 180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S80>/Switch' */

  /* Sum: '<S69>/Sum' incorporates:
   *  Inport: '<Root>/in_cmd'
   *  Sum: '<S66>/Sum1'
   */
  claw_ofp_B.Sum = (claw_in_cmd.wp_lon[0] - claw_ofp_B.Sum) + (real_T)
    rtb_Compare_eu_0;

  /* UnitConversion: '<S72>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.rtb_Sum1_idx_0 = 0.017453292519943295 * claw_ofp_B.Abs1;

  /* UnitConversion: '<S87>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.Switch_o *= 0.017453292519943295;

  /* Trigonometry: '<S88>/Trigonometric Function1' */
  claw_ofp_B.DataType5 = sin(claw_ofp_B.Switch_o);

  /* Sum: '<S88>/Sum1' incorporates:
   *  Constant: '<S88>/Constant'
   *  Product: '<S88>/Product1'
   */
  claw_ofp_B.DataType5 = 1.0 - claw_ofp_ConstB.sqrt_n * claw_ofp_ConstB.sqrt_n *
    claw_ofp_B.DataType5 * claw_ofp_B.DataType5;

  /* Product: '<S86>/Product1' incorporates:
   *  Constant: '<S86>/Constant1'
   *  Sqrt: '<S86>/sqrt'
   */
  claw_ofp_B.Abs1 = 6.378137E+6 / sqrt(claw_ofp_B.DataType5);

  /* Product: '<S71>/dNorth' incorporates:
   *  Product: '<S86>/Product3'
   *  Trigonometry: '<S86>/Trigonometric Function1'
   */
  claw_ofp_B.DataType5 = claw_ofp_B.rtb_Sum1_idx_0 / atan2(1.0, claw_ofp_B.Abs1 *
    claw_ofp_ConstB.Sum1_b / claw_ofp_B.DataType5);

  /* Switch: '<S74>/Switch' incorporates:
   *  Abs: '<S74>/Abs'
   *  Bias: '<S74>/Bias'
   *  Bias: '<S74>/Bias1'
   *  Constant: '<S74>/Constant2'
   *  Constant: '<S78>/Constant'
   *  Math: '<S74>/Math Function1'
   *  RelationalOperator: '<S78>/Compare'
   */
  if (fabs(claw_ofp_B.Sum) > 180.0) {
    claw_ofp_B.Sum = claw_ofp_rt_modd(claw_ofp_B.Sum + 180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S74>/Switch' */

  /* Product: '<S71>/dEast' incorporates:
   *  Product: '<S86>/Product4'
   *  Trigonometry: '<S86>/Trigonometric Function'
   *  Trigonometry: '<S86>/Trigonometric Function2'
   *  UnitConversion: '<S72>/Unit Conversion'
   */
  claw_ofp_B.Abs1 = 1.0 / atan2(1.0, claw_ofp_B.Abs1 * cos(claw_ofp_B.Switch_o))
    * (0.017453292519943295 * claw_ofp_B.Sum);

  /* Switch: '<S104>/Switch' incorporates:
   *  Bias: '<S104>/Bias'
   *  Bias: '<S104>/Bias1'
   *  Constant: '<S104>/Constant2'
   *  Constant: '<S105>/Constant'
   *  Inport: '<Root>/in_OFP'
   *  Math: '<S104>/Math Function1'
   *  RelationalOperator: '<S105>/Compare'
   */
  if (claw_ofp_B.Abs1_g > 180.0) {
    claw_ofp_B.Switch_o = claw_ofp_rt_modd(claw_in_ofp.ref_lat + 180.0, 360.0) +
      -180.0;
  } else {
    claw_ofp_B.Switch_o = claw_in_ofp.ref_lat;
  }

  /* End of Switch: '<S104>/Switch' */

  /* Abs: '<S101>/Abs1' */
  claw_ofp_B.Sum = fabs(claw_ofp_B.Switch_o);

  /* Switch: '<S101>/Switch' incorporates:
   *  Bias: '<S101>/Bias'
   *  Bias: '<S101>/Bias1'
   *  Constant: '<S103>/Constant'
   *  Constant: '<S92>/Constant'
   *  Constant: '<S92>/Constant1'
   *  Gain: '<S101>/Gain'
   *  Product: '<S101>/Divide1'
   *  RelationalOperator: '<S103>/Compare'
   *  Switch: '<S92>/Switch1'
   */
  if (claw_ofp_B.Sum > 90.0) {
    /* Signum: '<S101>/Sign1' */
    if (claw_ofp_B.Switch_o < 0.0) {
      claw_ofp_B.Sum_j = -1.0;
    } else {
      claw_ofp_B.Sum_j = (claw_ofp_B.Switch_o > 0.0);
    }

    /* End of Signum: '<S101>/Sign1' */
    claw_ofp_B.Switch_o = (-(claw_ofp_B.Sum + -90.0) + 90.0) * claw_ofp_B.Sum_j;
    rtb_Compare_eu_0 = 180;
  } else {
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S101>/Switch' */

  /* Sum: '<S92>/Sum' incorporates:
   *  Inport: '<Root>/in_OFP'
   */
  claw_ofp_B.Sum_j = (real_T)rtb_Compare_eu_0 + claw_in_ofp.ref_lon;

  /* Sum: '<S67>/Sum1' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = claw_in_cmd.wp_lat[1] - claw_ofp_B.Switch_o;

  /* Switch: '<S98>/Switch' incorporates:
   *  Abs: '<S98>/Abs'
   *  Bias: '<S98>/Bias'
   *  Bias: '<S98>/Bias1'
   *  Constant: '<S98>/Constant2'
   *  Constant: '<S99>/Constant'
   *  Math: '<S98>/Math Function1'
   *  RelationalOperator: '<S99>/Compare'
   */
  if (fabs(claw_ofp_B.rtb_Sum1_idx_0) > 180.0) {
    claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_rt_modd(claw_ofp_B.rtb_Sum1_idx_0 +
      180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S98>/Switch' */

  /* Abs: '<S95>/Abs1' */
  claw_ofp_B.Sum = fabs(claw_ofp_B.rtb_Sum1_idx_0);

  /* Switch: '<S95>/Switch' incorporates:
   *  Bias: '<S95>/Bias'
   *  Bias: '<S95>/Bias1'
   *  Constant: '<S91>/Constant'
   *  Constant: '<S91>/Constant1'
   *  Constant: '<S97>/Constant'
   *  Gain: '<S95>/Gain'
   *  Product: '<S95>/Divide1'
   *  RelationalOperator: '<S97>/Compare'
   *  Switch: '<S91>/Switch1'
   */
  if (claw_ofp_B.Sum > 90.0) {
    /* Signum: '<S95>/Sign1' */
    if (claw_ofp_B.rtb_Sum1_idx_0 < 0.0) {
      claw_ofp_B.DataType4 = -1.0;
    } else {
      claw_ofp_B.DataType4 = (claw_ofp_B.rtb_Sum1_idx_0 > 0.0);
    }

    /* End of Signum: '<S95>/Sign1' */
    claw_ofp_B.Sum = (-(claw_ofp_B.Sum + -90.0) + 90.0) * claw_ofp_B.DataType4;
    rtb_Compare_eu_0 = 180;
  } else {
    claw_ofp_B.Sum = claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S95>/Switch' */

  /* Switch: '<S102>/Switch' incorporates:
   *  Abs: '<S102>/Abs'
   *  Bias: '<S102>/Bias'
   *  Bias: '<S102>/Bias1'
   *  Constant: '<S102>/Constant2'
   *  Constant: '<S106>/Constant'
   *  Math: '<S102>/Math Function1'
   *  RelationalOperator: '<S106>/Compare'
   */
  if (fabs(claw_ofp_B.Sum_j) > 180.0) {
    claw_ofp_B.Sum_j = claw_ofp_rt_modd(claw_ofp_B.Sum_j + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S102>/Switch' */

  /* Sum: '<S91>/Sum' incorporates:
   *  Inport: '<Root>/in_cmd'
   *  Sum: '<S67>/Sum1'
   */
  claw_ofp_B.Sum_j = (claw_in_cmd.wp_lon[1] - claw_ofp_B.Sum_j) + (real_T)
    rtb_Compare_eu_0;

  /* UnitConversion: '<S94>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.rtb_Sum1_idx_0 = 0.017453292519943295 * claw_ofp_B.Sum;

  /* UnitConversion: '<S109>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.Switch_o *= 0.017453292519943295;

  /* Trigonometry: '<S110>/Trigonometric Function1' */
  claw_ofp_B.DataType4 = sin(claw_ofp_B.Switch_o);

  /* Sum: '<S110>/Sum1' incorporates:
   *  Constant: '<S110>/Constant'
   *  Product: '<S110>/Product1'
   */
  claw_ofp_B.DataType4 = 1.0 - claw_ofp_ConstB.sqrt_h * claw_ofp_ConstB.sqrt_h *
    claw_ofp_B.DataType4 * claw_ofp_B.DataType4;

  /* Product: '<S108>/Product1' incorporates:
   *  Constant: '<S108>/Constant1'
   *  Sqrt: '<S108>/sqrt'
   */
  claw_ofp_B.Sum = 6.378137E+6 / sqrt(claw_ofp_B.DataType4);

  /* Product: '<S93>/dNorth' incorporates:
   *  Product: '<S108>/Product3'
   *  Trigonometry: '<S108>/Trigonometric Function1'
   */
  claw_ofp_B.DataType4 = claw_ofp_B.rtb_Sum1_idx_0 / atan2(1.0, claw_ofp_B.Sum *
    claw_ofp_ConstB.Sum1_o / claw_ofp_B.DataType4);

  /* Switch: '<S96>/Switch' incorporates:
   *  Abs: '<S96>/Abs'
   *  Bias: '<S96>/Bias'
   *  Bias: '<S96>/Bias1'
   *  Constant: '<S100>/Constant'
   *  Constant: '<S96>/Constant2'
   *  Math: '<S96>/Math Function1'
   *  RelationalOperator: '<S100>/Compare'
   */
  if (fabs(claw_ofp_B.Sum_j) > 180.0) {
    claw_ofp_B.Sum_j = claw_ofp_rt_modd(claw_ofp_B.Sum_j + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S96>/Switch' */

  /* Product: '<S93>/dEast' incorporates:
   *  Product: '<S108>/Product4'
   *  Trigonometry: '<S108>/Trigonometric Function'
   *  Trigonometry: '<S108>/Trigonometric Function2'
   *  UnitConversion: '<S94>/Unit Conversion'
   */
  claw_ofp_B.Sum = 1.0 / atan2(1.0, claw_ofp_B.Sum * cos(claw_ofp_B.Switch_o)) *
    (0.017453292519943295 * claw_ofp_B.Sum_j);

  /* Switch: '<S126>/Switch' incorporates:
   *  Bias: '<S126>/Bias'
   *  Bias: '<S126>/Bias1'
   *  Constant: '<S126>/Constant2'
   *  Constant: '<S127>/Constant'
   *  Inport: '<Root>/in_OFP'
   *  Math: '<S126>/Math Function1'
   *  RelationalOperator: '<S127>/Compare'
   */
  if (claw_ofp_B.Abs1_g > 180.0) {
    claw_ofp_B.Switch_o = claw_ofp_rt_modd(claw_in_ofp.ref_lat + 180.0, 360.0) +
      -180.0;
  } else {
    claw_ofp_B.Switch_o = claw_in_ofp.ref_lat;
  }

  /* End of Switch: '<S126>/Switch' */

  /* Abs: '<S123>/Abs1' */
  claw_ofp_B.Sum_j = fabs(claw_ofp_B.Switch_o);

  /* Switch: '<S123>/Switch' incorporates:
   *  Bias: '<S123>/Bias'
   *  Bias: '<S123>/Bias1'
   *  Constant: '<S114>/Constant'
   *  Constant: '<S114>/Constant1'
   *  Constant: '<S125>/Constant'
   *  Gain: '<S123>/Gain'
   *  Product: '<S123>/Divide1'
   *  RelationalOperator: '<S125>/Compare'
   *  Switch: '<S114>/Switch1'
   */
  if (claw_ofp_B.Sum_j > 90.0) {
    /* Signum: '<S123>/Sign1' */
    if (claw_ofp_B.Switch_o < 0.0) {
      claw_ofp_B.rtb_Sum1_idx_0 = -1.0;
    } else {
      claw_ofp_B.rtb_Sum1_idx_0 = (claw_ofp_B.Switch_o > 0.0);
    }

    /* End of Signum: '<S123>/Sign1' */
    claw_ofp_B.Switch_o = (-(claw_ofp_B.Sum_j + -90.0) + 90.0) *
      claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 180;
  } else {
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S123>/Switch' */

  /* Sum: '<S114>/Sum' incorporates:
   *  Inport: '<Root>/in_OFP'
   */
  claw_ofp_B.Sum_k = (real_T)rtb_Compare_eu_0 + claw_in_ofp.ref_lon;

  /* Sum: '<S68>/Sum1' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = claw_in_cmd.wp_lat[2] - claw_ofp_B.Switch_o;

  /* Switch: '<S120>/Switch' incorporates:
   *  Abs: '<S120>/Abs'
   *  Bias: '<S120>/Bias'
   *  Bias: '<S120>/Bias1'
   *  Constant: '<S120>/Constant2'
   *  Constant: '<S121>/Constant'
   *  Math: '<S120>/Math Function1'
   *  RelationalOperator: '<S121>/Compare'
   */
  if (fabs(claw_ofp_B.rtb_Sum1_idx_0) > 180.0) {
    claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_rt_modd(claw_ofp_B.rtb_Sum1_idx_0 +
      180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S120>/Switch' */

  /* Abs: '<S117>/Abs1' */
  claw_ofp_B.Sum_j = fabs(claw_ofp_B.rtb_Sum1_idx_0);

  /* Switch: '<S117>/Switch' incorporates:
   *  Bias: '<S117>/Bias'
   *  Bias: '<S117>/Bias1'
   *  Constant: '<S113>/Constant'
   *  Constant: '<S113>/Constant1'
   *  Constant: '<S119>/Constant'
   *  Gain: '<S117>/Gain'
   *  Product: '<S117>/Divide1'
   *  RelationalOperator: '<S119>/Compare'
   *  Switch: '<S113>/Switch1'
   */
  if (claw_ofp_B.Sum_j > 90.0) {
    /* Signum: '<S117>/Sign1' */
    if (claw_ofp_B.rtb_Sum1_idx_0 < 0.0) {
      claw_ofp_B.rtb_Sum1_idx_0 = -1.0;
    } else {
      claw_ofp_B.rtb_Sum1_idx_0 = (claw_ofp_B.rtb_Sum1_idx_0 > 0.0);
    }

    /* End of Signum: '<S117>/Sign1' */
    claw_ofp_B.Sum_j = (-(claw_ofp_B.Sum_j + -90.0) + 90.0) *
      claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 180;
  } else {
    claw_ofp_B.Sum_j = claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S117>/Switch' */

  /* Switch: '<S124>/Switch' incorporates:
   *  Abs: '<S124>/Abs'
   *  Bias: '<S124>/Bias'
   *  Bias: '<S124>/Bias1'
   *  Constant: '<S124>/Constant2'
   *  Constant: '<S128>/Constant'
   *  Math: '<S124>/Math Function1'
   *  RelationalOperator: '<S128>/Compare'
   */
  if (fabs(claw_ofp_B.Sum_k) > 180.0) {
    claw_ofp_B.Sum_k = claw_ofp_rt_modd(claw_ofp_B.Sum_k + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S124>/Switch' */

  /* Sum: '<S113>/Sum' incorporates:
   *  Inport: '<Root>/in_cmd'
   *  Sum: '<S68>/Sum1'
   */
  claw_ofp_B.Sum_k = (claw_in_cmd.wp_lon[2] - claw_ofp_B.Sum_k) + (real_T)
    rtb_Compare_eu_0;

  /* UnitConversion: '<S116>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.rtb_Sum1_idx_0 = 0.017453292519943295 * claw_ofp_B.Sum_j;

  /* UnitConversion: '<S131>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.Switch_o *= 0.017453292519943295;

  /* Trigonometry: '<S132>/Trigonometric Function1' */
  claw_ofp_B.DataType_n = sin(claw_ofp_B.Switch_o);

  /* Sum: '<S132>/Sum1' incorporates:
   *  Constant: '<S132>/Constant'
   *  Product: '<S132>/Product1'
   */
  claw_ofp_B.DataType_n = 1.0 - claw_ofp_ConstB.sqrt_k * claw_ofp_ConstB.sqrt_k *
    claw_ofp_B.DataType_n * claw_ofp_B.DataType_n;

  /* Product: '<S130>/Product1' incorporates:
   *  Constant: '<S130>/Constant1'
   *  Sqrt: '<S130>/sqrt'
   */
  claw_ofp_B.Sum_j = 6.378137E+6 / sqrt(claw_ofp_B.DataType_n);

  /* Product: '<S115>/dNorth' incorporates:
   *  Product: '<S130>/Product3'
   *  Trigonometry: '<S130>/Trigonometric Function1'
   */
  claw_ofp_B.DataType_n = claw_ofp_B.rtb_Sum1_idx_0 / atan2(1.0,
    claw_ofp_B.Sum_j * claw_ofp_ConstB.Sum1_n / claw_ofp_B.DataType_n);

  /* Switch: '<S118>/Switch' incorporates:
   *  Abs: '<S118>/Abs'
   *  Bias: '<S118>/Bias'
   *  Bias: '<S118>/Bias1'
   *  Constant: '<S118>/Constant2'
   *  Constant: '<S122>/Constant'
   *  Math: '<S118>/Math Function1'
   *  RelationalOperator: '<S122>/Compare'
   */
  if (fabs(claw_ofp_B.Sum_k) > 180.0) {
    claw_ofp_B.Sum_k = claw_ofp_rt_modd(claw_ofp_B.Sum_k + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S118>/Switch' */

  /* Product: '<S115>/dEast' incorporates:
   *  Product: '<S130>/Product4'
   *  Trigonometry: '<S130>/Trigonometric Function'
   *  Trigonometry: '<S130>/Trigonometric Function2'
   *  UnitConversion: '<S116>/Unit Conversion'
   */
  claw_ofp_B.Sum_j = 1.0 / atan2(1.0, claw_ofp_B.Sum_j * cos(claw_ofp_B.Switch_o))
    * (0.017453292519943295 * claw_ofp_B.Sum_k);

  /* SignalConversion generated from: '<S148>/knob' incorporates:
   *  Product: '<S115>/x*cos'
   *  Product: '<S115>/y*sin'
   *  Product: '<S71>/x*cos'
   *  Product: '<S71>/y*sin'
   *  Product: '<S93>/x*cos'
   *  Product: '<S93>/y*sin'
   *  Sum: '<S115>/Sum2'
   *  Sum: '<S71>/Sum2'
   *  Sum: '<S93>/Sum2'
   */
  claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0] =
    claw_ofp_B.DataType5 * claw_ofp_ConstB.SinCos_o2 + claw_ofp_B.Abs1 *
    claw_ofp_ConstB.SinCos_o1;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1] =
    claw_ofp_B.DataType4 * claw_ofp_ConstB.SinCos_o2_a + claw_ofp_B.Sum *
    claw_ofp_ConstB.SinCos_o1_d;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2] =
    claw_ofp_B.DataType_n * claw_ofp_ConstB.SinCos_o2_g + claw_ofp_B.Sum_j *
    claw_ofp_ConstB.SinCos_o1_i;

  /* SignalConversion generated from: '<S148>/knob' incorporates:
   *  Product: '<S115>/x*sin'
   *  Product: '<S115>/y*cos'
   *  Product: '<S71>/x*sin'
   *  Product: '<S71>/y*cos'
   *  Product: '<S93>/x*sin'
   *  Product: '<S93>/y*cos'
   *  Sum: '<S115>/Sum3'
   *  Sum: '<S71>/Sum3'
   *  Sum: '<S93>/Sum3'
   */
  claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0] = claw_ofp_B.Abs1 *
    claw_ofp_ConstB.SinCos_o2 - claw_ofp_B.DataType5 * claw_ofp_ConstB.SinCos_o1;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1] = claw_ofp_B.Sum *
    claw_ofp_ConstB.SinCos_o2_a - claw_ofp_B.DataType4 *
    claw_ofp_ConstB.SinCos_o1_d;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2] = claw_ofp_B.Sum_j *
    claw_ofp_ConstB.SinCos_o2_g - claw_ofp_B.DataType_n *
    claw_ofp_ConstB.SinCos_o1_i;

  /* SignalConversion generated from: '<S148>/knob' incorporates:
   *  DataTypeConversion: '<S65>/Data Type Conversion'
   *  DataTypeConversion: '<S65>/Data Type Conversion1'
   *  Inport: '<Root>/in_OFP'
   *  Inport: '<Root>/in_cmd'
   *  Sum: '<S66>/Sum'
   *  Sum: '<S67>/Sum'
   *  Sum: '<S68>/Sum'
   *  UnaryMinus: '<S65>/Unary Minus'
   *  UnaryMinus: '<S66>/Ze2height'
   *  UnaryMinus: '<S67>/Ze2height'
   *  UnaryMinus: '<S68>/Ze2height'
   */
  claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[0] = -((real_T)
    claw_in_cmd.wp_alt[0] + -claw_in_ofp.ref_alt);
  claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1] = -((real_T)
    claw_in_cmd.wp_alt[1] + -claw_in_ofp.ref_alt);
  claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[2] = -((real_T)
    claw_in_cmd.wp_alt[2] + -claw_in_ofp.ref_alt);

  /* Chart: '<S156>/StickManual' incorporates:
   *  BusCreator generated from: '<S156>/StickManual'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c16_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c16_claw_ofp = 1U;
    claw_ofp_DW.is_c16_claw_ofp = claw_ofp_IN_Inactive;
  } else if (claw_ofp_DW.is_c16_claw_ofp == 1U) {
    if (!rtb_Compare_jr) {
      claw_ofp_DW.is_c16_claw_ofp = claw_ofp_IN_Inactive;
    } else {
      claw_ofp_B.FMMOUT_h.ail_cmd = claw_in_cmd.ail_cmd;
      claw_ofp_B.FMMOUT_h.ele_cmd = claw_in_cmd.ele_cmd;
      claw_ofp_B.FMMOUT_h.rud_cmd = claw_in_cmd.rud_cmd;
      claw_ofp_B.FMMOUT_h.thr_cmd = claw_in_cmd.thr_cmd;
    }

    /* case IN_Inactive: */
  } else if (rtb_Compare_jr) {
    claw_ofp_DW.is_c16_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_h = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_h.state_id = 1110000UL;
    claw_ofp_B.FMMOUT_h.claw_mode = 0U;
    claw_ofp_B.FMMOUT_h.pitch_axis_mode = 0U;
    claw_ofp_B.FMMOUT_h.roll_axis_mode = 0U;
    claw_ofp_B.FMMOUT_h.yaw_axis_mode = 0U;
    claw_ofp_B.FMMOUT_h.vert_axis_mode = 0U;
  }

  /* End of Chart: '<S156>/StickManual' */

  /* RelationalOperator: '<S142>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_Compare_jr = (claw_in_cmd.flight_mode == 2U);

  /* Chart: '<S158>/stick_rate' incorporates:
   *  BusCreator generated from: '<S158>/stick_rate'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c18_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c18_claw_ofp = 1U;
    claw_ofp_DW.is_c18_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c18_claw_ofp == 1U) {
    if (!rtb_Compare_jr) {
      claw_ofp_DW.is_c18_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_j.roll_rate_cmd = claw_in_cmd.roll_rate_cmd;
      claw_ofp_B.FMMOUT_j.pitch_rate_cmd = claw_in_cmd.pitch_rate_cmd;
      claw_ofp_B.FMMOUT_j.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;
      claw_ofp_B.FMMOUT_j.thr_cmd = claw_in_cmd.thr_cmd;
    }

    /* case IN_InActive: */
  } else if (rtb_Compare_jr) {
    claw_ofp_DW.is_c18_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_j = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_j.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_j.claw_mode = 2U;
    claw_ofp_B.FMMOUT_j.roll_axis_mode = 1U;
    claw_ofp_B.FMMOUT_j.pitch_axis_mode = 1U;
    claw_ofp_B.FMMOUT_j.yaw_axis_mode = 1U;
    claw_ofp_B.FMMOUT_j.vert_axis_mode = 0U;
  }

  /* End of Chart: '<S158>/stick_rate' */

  /* Sum: '<S283>/Sum' incorporates:
   *  DataTypeConversion: '<S274>/Data Type '
   *  Inport: '<Root>/in_sensor'
   *  UnitDelay: '<S283>/Unit Delay1'
   */
  claw_ofp_B.Sum_j = claw_in_sensor.roll_rate + claw_ofp_DW.UnitDelay1_DSTATE;

  /* Sum: '<S285>/Sum' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 4'
   *  Inport: '<Root>/in_sensor'
   *  UnitDelay: '<S285>/Unit Delay1'
   */
  claw_ofp_B.Sum = claw_in_sensor.pitch_rate + claw_ofp_DW.UnitDelay1_DSTATE_p;

  /* Sum: '<S287>/Sum' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 5'
   *  Inport: '<Root>/in_sensor'
   *  UnitDelay: '<S287>/Unit Delay1'
   */
  claw_ofp_B.Abs1 = claw_in_sensor.yaw_rate + claw_ofp_DW.UnitDelay1_DSTATE_pz;

  /* Switch: '<S315>/Switch' incorporates:
   *  Bias: '<S315>/Bias'
   *  Bias: '<S315>/Bias1'
   *  Constant: '<S315>/Constant2'
   *  Constant: '<S316>/Constant'
   *  Inport: '<Root>/in_OFP'
   *  Math: '<S315>/Math Function1'
   *  RelationalOperator: '<S316>/Compare'
   */
  if (claw_ofp_B.Abs1_g > 180.0) {
    claw_ofp_B.Switch_o = claw_ofp_rt_modd(claw_in_ofp.ref_lat + 180.0, 360.0) +
      -180.0;
  } else {
    claw_ofp_B.Switch_o = claw_in_ofp.ref_lat;
  }

  /* End of Switch: '<S315>/Switch' */

  /* Abs: '<S312>/Abs1' */
  claw_ofp_B.Abs1_g = fabs(claw_ofp_B.Switch_o);

  /* Switch: '<S312>/Switch' incorporates:
   *  Bias: '<S312>/Bias'
   *  Bias: '<S312>/Bias1'
   *  Constant: '<S303>/Constant'
   *  Constant: '<S303>/Constant1'
   *  Constant: '<S314>/Constant'
   *  Gain: '<S312>/Gain'
   *  Product: '<S312>/Divide1'
   *  RelationalOperator: '<S314>/Compare'
   *  Switch: '<S303>/Switch1'
   */
  if (claw_ofp_B.Abs1_g > 90.0) {
    /* Signum: '<S312>/Sign1' */
    if (claw_ofp_B.Switch_o < 0.0) {
      claw_ofp_B.Switch_o = -1.0;
    } else {
      claw_ofp_B.Switch_o = (claw_ofp_B.Switch_o > 0.0);
    }

    /* End of Signum: '<S312>/Sign1' */
    claw_ofp_B.Switch_o *= -(claw_ofp_B.Abs1_g + -90.0) + 90.0;
    rtb_Compare_eu_0 = 180;
  } else {
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S312>/Switch' */

  /* Sum: '<S303>/Sum' incorporates:
   *  Inport: '<Root>/in_OFP'
   */
  claw_ofp_B.Abs1_g = (real_T)rtb_Compare_eu_0 + claw_in_ofp.ref_lon;

  /* Sum: '<S299>/Sum1' incorporates:
   *  Inport: '<Root>/in_sensor'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = claw_in_sensor.ac_lat - claw_ofp_B.Switch_o;

  /* Switch: '<S309>/Switch' incorporates:
   *  Abs: '<S309>/Abs'
   *  Bias: '<S309>/Bias'
   *  Bias: '<S309>/Bias1'
   *  Constant: '<S309>/Constant2'
   *  Constant: '<S310>/Constant'
   *  Math: '<S309>/Math Function1'
   *  RelationalOperator: '<S310>/Compare'
   */
  if (fabs(claw_ofp_B.rtb_Sum1_idx_0) > 180.0) {
    claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_rt_modd(claw_ofp_B.rtb_Sum1_idx_0 +
      180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S309>/Switch' */

  /* Abs: '<S306>/Abs1' */
  claw_ofp_B.DataType5 = fabs(claw_ofp_B.rtb_Sum1_idx_0);

  /* Switch: '<S306>/Switch' incorporates:
   *  Bias: '<S306>/Bias'
   *  Bias: '<S306>/Bias1'
   *  Constant: '<S302>/Constant'
   *  Constant: '<S302>/Constant1'
   *  Constant: '<S308>/Constant'
   *  Gain: '<S306>/Gain'
   *  Product: '<S306>/Divide1'
   *  RelationalOperator: '<S308>/Compare'
   *  Switch: '<S302>/Switch1'
   */
  if (claw_ofp_B.DataType5 > 90.0) {
    /* Signum: '<S306>/Sign1' */
    if (claw_ofp_B.rtb_Sum1_idx_0 < 0.0) {
      claw_ofp_B.DataType4 = -1.0;
    } else {
      claw_ofp_B.DataType4 = (claw_ofp_B.rtb_Sum1_idx_0 > 0.0);
    }

    /* End of Signum: '<S306>/Sign1' */
    claw_ofp_B.TrigonometricFunction1 = (-(claw_ofp_B.DataType5 + -90.0) + 90.0)
      * claw_ofp_B.DataType4;
    rtb_Compare_eu_0 = 180;
  } else {
    claw_ofp_B.TrigonometricFunction1 = claw_ofp_B.rtb_Sum1_idx_0;
    rtb_Compare_eu_0 = 0;
  }

  /* End of Switch: '<S306>/Switch' */

  /* Switch: '<S313>/Switch' incorporates:
   *  Abs: '<S313>/Abs'
   *  Bias: '<S313>/Bias'
   *  Bias: '<S313>/Bias1'
   *  Constant: '<S313>/Constant2'
   *  Constant: '<S317>/Constant'
   *  Math: '<S313>/Math Function1'
   *  RelationalOperator: '<S317>/Compare'
   */
  if (fabs(claw_ofp_B.Abs1_g) > 180.0) {
    claw_ofp_B.Abs1_g = claw_ofp_rt_modd(claw_ofp_B.Abs1_g + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S313>/Switch' */

  /* Sum: '<S302>/Sum' incorporates:
   *  Inport: '<Root>/in_sensor'
   *  Sum: '<S299>/Sum1'
   */
  claw_ofp_B.Abs1_g = (claw_in_sensor.ac_lon - claw_ofp_B.Abs1_g) + (real_T)
    rtb_Compare_eu_0;

  /* UnitConversion: '<S305>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.rtb_Sum1_idx_0 = 0.017453292519943295 *
    claw_ofp_B.TrigonometricFunction1;

  /* UnitConversion: '<S320>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  claw_ofp_B.Switch_o *= 0.017453292519943295;

  /* Trigonometry: '<S321>/Trigonometric Function1' */
  claw_ofp_B.Limit4 = sin(claw_ofp_B.Switch_o);

  /* Sum: '<S321>/Sum1' incorporates:
   *  Constant: '<S321>/Constant'
   *  Product: '<S321>/Product1'
   */
  claw_ofp_B.Limit4 = 1.0 - claw_ofp_ConstB.sqrt_i * claw_ofp_ConstB.sqrt_i *
    claw_ofp_B.Limit4 * claw_ofp_B.Limit4;

  /* Product: '<S319>/Product1' incorporates:
   *  Constant: '<S319>/Constant1'
   *  Sqrt: '<S319>/sqrt'
   */
  claw_ofp_B.TrigonometricFunction1 = 6.378137E+6 / sqrt(claw_ofp_B.Limit4);

  /* Product: '<S304>/dNorth' incorporates:
   *  Product: '<S319>/Product3'
   *  Trigonometry: '<S319>/Trigonometric Function1'
   */
  claw_ofp_B.Limit4 = claw_ofp_B.rtb_Sum1_idx_0 / atan2(1.0,
    claw_ofp_B.TrigonometricFunction1 * claw_ofp_ConstB.Sum1_b3 /
    claw_ofp_B.Limit4);

  /* Switch: '<S307>/Switch' incorporates:
   *  Abs: '<S307>/Abs'
   *  Bias: '<S307>/Bias'
   *  Bias: '<S307>/Bias1'
   *  Constant: '<S307>/Constant2'
   *  Constant: '<S311>/Constant'
   *  Math: '<S307>/Math Function1'
   *  RelationalOperator: '<S311>/Compare'
   */
  if (fabs(claw_ofp_B.Abs1_g) > 180.0) {
    claw_ofp_B.Abs1_g = claw_ofp_rt_modd(claw_ofp_B.Abs1_g + 180.0, 360.0) +
      -180.0;
  }

  /* End of Switch: '<S307>/Switch' */

  /* Product: '<S304>/dEast' incorporates:
   *  Product: '<S319>/Product4'
   *  Trigonometry: '<S319>/Trigonometric Function'
   *  Trigonometry: '<S319>/Trigonometric Function2'
   *  UnitConversion: '<S305>/Unit Conversion'
   */
  claw_ofp_B.TrigonometricFunction1 = 1.0 / atan2(1.0,
    claw_ofp_B.TrigonometricFunction1 * cos(claw_ofp_B.Switch_o)) *
    (0.017453292519943295 * claw_ofp_B.Abs1_g);

  /* UnitDelay: '<S326>/Unit Delay2' */
  claw_ofp_B.Abs1_g = claw_ofp_DW.UnitDelay2_DSTATE;

  /* Product: '<S326>/Product6' incorporates:
   *  DataTypeConversion: '<S301>/Cast To Double'
   *  Inport: '<Root>/in_sensor'
   *  UnitDelay: '<S326>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE = claw_in_sensor.alt_agl *
    claw_ofp_ConstB.wts2wts;

  /* Sum: '<S326>/Sum1' incorporates:
   *  Product: '<S326>/Product5'
   *  UnitDelay: '<S326>/Unit Delay2'
   *  UnitDelay: '<S326>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE = (claw_ofp_B.Abs1_g +
    claw_ofp_DW.UnitDelay2_DSTATE) + claw_ofp_ConstB.uwts2wts *
    claw_ofp_DW.UnitDelay3_DSTATE;

  /* Outputs for Enabled SubSystem: '<S301>/Enabled Subsystem2' incorporates:
   *  EnablePort: '<S330>/Enable'
   */
  /* Logic: '<S301>/AND' incorporates:
   *  Inport: '<Root>/in_OFP'
   *  Logic: '<S301>/NOT'
   */
  if ((!claw_in_ofp.agl_available) && claw_in_ofp.gps_available) {
    /* SignalConversion generated from: '<S330>/in' incorporates:
     *  DataTypeConversion: '<S301>/Cast To Double2'
     *  Inport: '<Root>/in_sensor'
     *  Sum: '<S301>/Subtract4'
     *  UnitDelay: '<S326>/Unit Delay3'
     */
    claw_ofp_B.in = claw_ofp_DW.UnitDelay3_DSTATE - claw_in_sensor.ac_alt_gps;
  }

  /* End of Logic: '<S301>/AND' */
  /* End of Outputs for SubSystem: '<S301>/Enabled Subsystem2' */

  /* UnitDelay: '<S327>/Unit Delay2' */
  claw_ofp_B.Abs1_g = claw_ofp_DW.UnitDelay2_DSTATE_h;

  /* Product: '<S327>/Product6' incorporates:
   *  DataTypeConversion: '<S301>/Cast To Double1'
   *  Inport: '<Root>/in_sensor'
   *  UnitDelay: '<S327>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_h = claw_in_sensor.alt_baro *
    claw_ofp_ConstB.wts2wts_c;

  /* Sum: '<S327>/Sum1' incorporates:
   *  Product: '<S327>/Product5'
   *  UnitDelay: '<S327>/Unit Delay2'
   *  UnitDelay: '<S327>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE_j = (claw_ofp_B.Abs1_g +
    claw_ofp_DW.UnitDelay2_DSTATE_h) + claw_ofp_ConstB.uwts2wts_c *
    claw_ofp_DW.UnitDelay3_DSTATE_j;

  /* UnitDelay: '<S328>/Unit Delay2' */
  claw_ofp_B.Abs1_g = claw_ofp_DW.UnitDelay2_DSTATE_l;

  /* Product: '<S328>/Product6' incorporates:
   *  DataTypeConversion: '<S301>/Cast To Double2'
   *  Inport: '<Root>/in_sensor'
   *  Sum: '<S301>/Subtract3'
   *  UnitDelay: '<S327>/Unit Delay3'
   *  UnitDelay: '<S328>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_l = (claw_ofp_DW.UnitDelay3_DSTATE_j -
    claw_in_sensor.ac_alt_gps) * claw_ofp_ConstB.wts2wts_j;

  /* Sum: '<S328>/Sum1' incorporates:
   *  Product: '<S328>/Product5'
   *  UnitDelay: '<S328>/Unit Delay2'
   *  UnitDelay: '<S328>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE_d = (claw_ofp_B.Abs1_g +
    claw_ofp_DW.UnitDelay2_DSTATE_l) + claw_ofp_ConstB.uwts2wts_e *
    claw_ofp_DW.UnitDelay3_DSTATE_d;

  /* Outputs for Enabled SubSystem: '<S301>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S329>/Enable'
   */
  /* Inport: '<Root>/in_OFP' */
  if (claw_in_ofp.gps_available) {
    /* SignalConversion generated from: '<S329>/in' incorporates:
     *  UnitDelay: '<S328>/Unit Delay3'
     */
    claw_ofp_B.in_h = claw_ofp_DW.UnitDelay3_DSTATE_d;
  }

  /* End of Outputs for SubSystem: '<S301>/Enabled Subsystem1' */

  /* Gain: '<S289>/Gain1' incorporates:
   *  DataTypeConversion: '<S276>/Cast To Double'
   *  Inport: '<Root>/in_sensor'
   */
  claw_ofp_B.Limit3 = 0.017453292519943295 * claw_in_sensor.heading;

  /* Trigonometry: '<S290>/Trigonometric Function1' */
  claw_ofp_B.Sum_k = cos(claw_ofp_B.Limit3);

  /* Trigonometry: '<S290>/Trigonometric Function2' */
  claw_ofp_B.Limit3 = sin(claw_ofp_B.Limit3);

  /* UnitConversion: '<S333>/Unit Conversion' incorporates:
   *  DataTypeConversion: '<S279>/Data Type Conversion1'
   *  DataTypeConversion: '<S279>/Data Type Conversion2'
   *  Inport: '<Root>/in_sensor'
   *  Trigonometry: '<S331>/Trigonometric Function1'
   */
  /* Unit Conversion - from: rad to: deg
     Expression: output = (57.2958*input) + (0) */
  claw_ofp_B.DataType5 = 57.295779513082323 * atan2(claw_in_sensor.vel_east,
    claw_in_sensor.vel_north);

  /* BusCreator: '<S275>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 1'
   *  DataTypeConversion: '<S274>/Data Type 2'
   *  DataTypeConversion: '<S274>/Data Type 3'
   *  Inport: '<Root>/in_sensor'
   *  Outport: '<Root>/OutState'
   *  Product: '<S283>/Product1'
   *  Product: '<S285>/Product1'
   *  Product: '<S287>/Product1'
   *  Product: '<S304>/x*cos'
   *  Product: '<S304>/x*sin'
   *  Product: '<S304>/y*cos'
   *  Product: '<S304>/y*sin'
   *  Sum: '<S304>/Sum2'
   *  Sum: '<S304>/Sum3'
   */
  /* Unit Conversion - from: rad to: deg
     Expression: output = (57.2958*input) + (0) */
  claw_out_state.sensor_raw = claw_in_sensor;
  claw_out_state.roll = claw_in_sensor.roll;
  claw_out_state.pitch = claw_in_sensor.pitch;
  claw_out_state.heading = claw_in_sensor.heading;
  claw_out_state.roll_rate = (real32_T)(claw_ofp_B.Sum_j *
    claw_ofp_ConstB.Product3);
  claw_out_state.pitch_rate = (real32_T)(claw_ofp_B.Sum *
    claw_ofp_ConstB.Product3_n);
  claw_out_state.yaw_rate = (real32_T)(claw_ofp_B.Abs1 *
    claw_ofp_ConstB.Product3_m);
  claw_out_state.accel_x = claw_in_sensor.accel_x;
  claw_out_state.accel_y = claw_in_sensor.accel_y;
  claw_out_state.accel_z = claw_in_sensor.accel_z;
  claw_out_state.ac_x_local = claw_ofp_B.Limit4 * claw_ofp_ConstB.SinCos_o2_o +
    claw_ofp_B.TrigonometricFunction1 * claw_ofp_ConstB.SinCos_o1_a;
  claw_out_state.ac_y_local = claw_ofp_B.TrigonometricFunction1 *
    claw_ofp_ConstB.SinCos_o2_o - claw_ofp_B.Limit4 *
    claw_ofp_ConstB.SinCos_o1_a;

  /* Switch: '<S301>/Switch1' incorporates:
   *  Inport: '<Root>/in_OFP'
   *  Sum: '<S301>/Subtract1'
   *  Sum: '<S301>/Subtract2'
   *  Switch: '<S301>/Switch'
   *  UnitDelay: '<S326>/Unit Delay3'
   *  UnitDelay: '<S327>/Unit Delay3'
   */
  if (claw_in_ofp.agl_available) {
    claw_ofp_B.Abs1_g = claw_ofp_DW.UnitDelay3_DSTATE - claw_ofp_B.in;
  } else if (claw_in_ofp.gps_available) {
    /* Switch: '<S301>/Switch' incorporates:
     *  DataTypeConversion: '<S301>/Cast To Double2'
     *  Inport: '<Root>/in_sensor'
     */
    claw_ofp_B.Abs1_g = claw_in_sensor.ac_alt_gps;
  } else {
    claw_ofp_B.Abs1_g = claw_ofp_DW.UnitDelay3_DSTATE_j - claw_ofp_B.in_h;
  }

  /* End of Switch: '<S301>/Switch1' */

  /* BusCreator: '<S275>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S278>/Data Type Conversion1'
   *  Inport: '<Root>/in_OFP'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S299>/Sum'
   *  UnaryMinus: '<S278>/Unary Minus'
   *  UnaryMinus: '<S299>/Ze2height'
   */
  claw_out_state.ac_z_local = -(claw_ofp_B.Abs1_g + -claw_in_ofp.ref_alt);

  /* Outputs for Atomic SubSystem: '<S278>/ShortHdg' */
  /* Switch: '<S300>/Switch1' incorporates:
   *  Constant: '<S324>/Constant'
   *  Constant: '<S325>/Constant'
   *  Inport: '<Root>/in_sensor'
   *  RelationalOperator: '<S324>/Compare'
   *  RelationalOperator: '<S325>/Compare'
   *  Switch: '<S300>/Switch'
   */
  if (claw_in_sensor.heading >= 180.0F) {
    /* BusCreator: '<S275>/Bus Creator' incorporates:
     *  Constant: '<S300>/   1'
     *  Outport: '<Root>/OutState'
     *  Sum: '<S300>/Add1'
     */
    claw_out_state.ac_psi_local = claw_in_sensor.heading - 360.0F;
  } else if (claw_in_sensor.heading < -180.0F) {
    /* Switch: '<S300>/Switch' incorporates:
     *  BusCreator: '<S275>/Bus Creator'
     *  Constant: '<S300>/   '
     *  Outport: '<Root>/OutState'
     *  Sum: '<S300>/Add'
     */
    claw_out_state.ac_psi_local = claw_in_sensor.heading + 360.0F;
  } else {
    /* BusCreator: '<S275>/Bus Creator' incorporates:
     *  Outport: '<Root>/OutState'
     *  Switch: '<S300>/Switch'
     */
    claw_out_state.ac_psi_local = claw_in_sensor.heading;
  }

  /* End of Switch: '<S300>/Switch1' */
  /* End of Outputs for SubSystem: '<S278>/ShortHdg' */

  /* BusCreator: '<S275>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S276>/Data Type Conversion'
   *  DataTypeConversion: '<S276>/Data Type Conversion1'
   *  Inport: '<Root>/in_sensor'
   *  Outport: '<Root>/OutState'
   *  Product: '<S290>/Product2'
   *  Reshape: '<S290>/Reshape3'
   *  SignalConversion generated from: '<S290>/Matrix Concatenate'
   *  UnaryMinus: '<S290>/Unary Minus'
   */
  claw_out_state.fwd_spd = (real32_T)(claw_ofp_B.Sum_k *
    claw_in_sensor.vel_north + claw_ofp_B.Limit3 * claw_in_sensor.vel_east);
  claw_out_state.lat_spd = (real32_T)(-claw_ofp_B.Limit3 *
    claw_in_sensor.vel_north + claw_ofp_B.Sum_k * claw_in_sensor.vel_east);

  /* Switch: '<S334>/Switch1' incorporates:
   *  Constant: '<S335>/Constant'
   *  RelationalOperator: '<S335>/Compare'
   */
  if (claw_ofp_B.DataType5 >= 0.0) {
    /* BusCreator: '<S275>/Bus Creator' incorporates:
     *  DataTypeConversion: '<S279>/Data Type Conversion5'
     *  Outport: '<Root>/OutState'
     */
    claw_out_state.crs_hdg = (real32_T)claw_ofp_B.DataType5;
  } else {
    /* BusCreator: '<S275>/Bus Creator' incorporates:
     *  Constant: '<S334>/   '
     *  DataTypeConversion: '<S279>/Data Type Conversion5'
     *  Outport: '<Root>/OutState'
     *  Sum: '<S334>/Add'
     */
    claw_out_state.crs_hdg = (real32_T)(claw_ofp_B.DataType5 + 360.0);
  }

  /* End of Switch: '<S334>/Switch1' */

  /* Math: '<S331>/Distance' incorporates:
   *  DataTypeConversion: '<S279>/Data Type Conversion1'
   *  DataTypeConversion: '<S279>/Data Type Conversion2'
   *  Inport: '<Root>/in_sensor'
   */
  claw_ofp_B.PY2 = claw_ofp_rt_hypotd((real_T)claw_in_sensor.vel_north, (real_T)
    claw_in_sensor.vel_east);

  /* Saturate: '<S331>/VRange' */
  if (claw_ofp_B.PY2 > 200.0) {
    claw_ofp_B.PY2 = 200.0;
  } else if (claw_ofp_B.PY2 < 1.0) {
    claw_ofp_B.PY2 = 1.0;
  }

  /* End of Saturate: '<S331>/VRange' */

  /* BusCreator: '<S275>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S279>/Data Type Conversion3'
   *  DataTypeConversion: '<S279>/Data Type Conversion4'
   *  DataTypeConversion: '<S291>/Data Type Conversion2'
   *  Inport: '<Root>/in_OFP'
   *  Inport: '<Root>/in_sensor'
   *  Outport: '<Root>/OutState'
   *  Product: '<S331>/Divide'
   *  Trigonometry: '<S331>/Trigonometric Function'
   *  UnaryMinus: '<S331>/Unary Minus'
   *  UnitConversion: '<S332>/Unit Conversion'
   */
  claw_out_state.path_angle = (real32_T)(atan(-(real_T)claw_in_sensor.vel_down /
    claw_ofp_B.PY2) * 57.295779513082323);
  claw_out_state.landed = (claw_ofp_B.landed_flag != 0.0);
  claw_out_state.cond_gnd_contact = rtb_cond_gnd_contact;
  claw_out_state.cond_maybe_land = rtb_cond_maybe_land;
  claw_out_state.cond_not_rotating = rtb_Compare_o1;
  claw_out_state.cond_not_moving = rtb_AND_c;
  claw_out_state.armed = claw_in_ofp.armed;

  /* RelationalOperator: '<S141>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 3U);

  /* Chart: '<S155>/stick_att' incorporates:
   *  BusCreator generated from: '<S155>/stick_att'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.is_active_c15_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c15_claw_ofp = 1U;
    claw_ofp_DW.is_c15_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c15_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Directional_axis_b = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c15_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_i.roll_cmd = claw_in_cmd.roll_cmd;
      claw_ofp_B.FMMOUT_i.pitch_cmd = claw_in_cmd.pitch_cmd;
      claw_ofp_B.FMMOUT_i.thr_cmd = claw_in_cmd.thr_cmd;
      if (claw_ofp_DW.is_Directional_axis_b == 1U) {
        if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) >
             claw_ofp_DW.MC_YAWRATE_DZ_m * 1.2) || (!claw_out_state.armed)) {
          /* Disarm될 경우, default로 전환 */
          claw_ofp_DW.is_Directional_axis_b = claw_ofp_IN_Yaw_rate_cmd;
          claw_ofp_B.FMMOUT_i.yaw_axis_mode = 1U;
        }

        /* case IN_Yaw_rate_cmd: */
      } else if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) <=
                  claw_ofp_DW.MC_YAWRATE_DZ_m) && claw_out_state.armed) {
        claw_ofp_DW.is_Directional_axis_b = 1U;
        claw_ofp_B.FMMOUT_i.yaw_axis_mode = 2U;
        claw_ofp_B.FMMOUT_i.hdg_cmd = claw_out_state.sensor_raw.heading;
      } else {
        claw_ofp_B.FMMOUT_i.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c15_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_i = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_i.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_i.claw_mode = 3U;
    claw_ofp_B.FMMOUT_i.roll_axis_mode = 2U;
    claw_ofp_B.FMMOUT_i.pitch_axis_mode = 2U;
    claw_ofp_B.FMMOUT_i.vert_axis_mode = 0U;
    claw_ofp_DW.MC_YAWRATE_DZ_m = 1.8;
    claw_ofp_DW.is_Directional_axis_b = claw_ofp_IN_Yaw_rate_cmd;
    claw_ofp_B.FMMOUT_i.yaw_axis_mode = 1U;
  }

  /* End of Chart: '<S155>/stick_att' */

  /* RelationalOperator: '<S143>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 4U);

  /* Chart: '<S154>/stick_alt' incorporates:
   *  BusCreator generated from: '<S154>/stick_alt'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.is_active_c14_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c14_claw_ofp = 1U;
    claw_ofp_DW.is_c14_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c14_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Vertical_axis_j = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_Directional_axis_a = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c14_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_d.roll_cmd = claw_in_cmd.roll_cmd;
      claw_ofp_B.FMMOUT_d.pitch_cmd = claw_in_cmd.pitch_cmd;
      if (claw_ofp_DW.is_Directional_axis_a == 1U) {
        if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) >
             claw_ofp_DW.MC_YAWRATE_DZ_b * 1.2) || (!claw_out_state.armed)) {
          /* Disarm될 경우, default로 전환 */
          claw_ofp_DW.is_Directional_axis_a = claw_ofp_IN_Yaw_rate_cmd;
          claw_ofp_B.FMMOUT_d.yaw_axis_mode = 1U;
        }

        /* case IN_Yaw_rate_cmd: */
      } else if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) <=
                  claw_ofp_DW.MC_YAWRATE_DZ_b) && claw_out_state.armed) {
        claw_ofp_DW.is_Directional_axis_a = 1U;
        claw_ofp_B.FMMOUT_d.yaw_axis_mode = 2U;
        claw_ofp_B.FMMOUT_d.hdg_cmd = claw_out_state.sensor_raw.heading;
      } else {
        claw_ofp_B.FMMOUT_d.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;
      }

      if (claw_ofp_DW.is_Vertical_axis_j == 1U) {
        if (((real32_T)fabs(claw_in_cmd.vel_z_cmd) > claw_ofp_DW.VEL_Z_THOLD_h *
             1.2) || (!claw_out_state.armed)) {
          /* Disarmed인 경우는 default로 전환 */
          claw_ofp_DW.is_Vertical_axis_j = claw_ofp_IN_Velocity;
          claw_ofp_B.FMMOUT_d.vert_axis_mode = 1U;
        }

        /* case IN_Velocity: */
      } else if (((real32_T)fabs(claw_in_cmd.vel_z_cmd) <=
                  claw_ofp_DW.VEL_Z_THOLD_h) && claw_out_state.armed &&
                 (!claw_out_state.landed)) {
        /* Arm되고 이륙 후 속도명령이 작을 때 */
        claw_ofp_DW.is_Vertical_axis_j = claw_ofp_IN_Pos_Hold;
        claw_ofp_B.FMMOUT_d.vert_axis_mode = 2U;
        claw_ofp_B.FMMOUT_d.alt_cmd = (real32_T)-claw_out_state.ac_z_local;
      } else {
        claw_ofp_B.FMMOUT_d.vel_z_cmd = claw_in_cmd.vel_z_cmd;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c14_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_d = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_d.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_d.claw_mode = 4U;
    claw_ofp_B.FMMOUT_d.roll_axis_mode = 2U;
    claw_ofp_B.FMMOUT_d.pitch_axis_mode = 2U;
    claw_ofp_DW.MC_YAWRATE_DZ_b = 1.8;
    claw_ofp_DW.is_Directional_axis_a = claw_ofp_IN_Yaw_rate_cmd;
    claw_ofp_B.FMMOUT_d.yaw_axis_mode = 1U;
    claw_ofp_DW.VEL_Z_THOLD_h = 0.06;
    claw_ofp_DW.is_Vertical_axis_j = claw_ofp_IN_Velocity;
    claw_ofp_B.FMMOUT_d.vert_axis_mode = 1U;
  }

  /* End of Chart: '<S154>/stick_alt' */

  /* RelationalOperator: '<S144>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 5U);

  /* Chart: '<S157>/stick_pos' incorporates:
   *  BusCreator generated from: '<S157>/stick_pos'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.is_active_c17_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c17_claw_ofp = 1U;
    claw_ofp_DW.is_c17_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c17_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Horizontal_k = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_Vertical_axis_p = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_Directional_axis_f = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c17_claw_ofp = claw_ofp_IN_InActive;
    } else {
      if (claw_ofp_DW.is_Directional_axis_f == 1U) {
        if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) >
             claw_ofp_DW.MC_YAWRATE_DZ * 1.2) || (!claw_out_state.armed)) {
          /* Disarm될 경우, default로 전환 */
          claw_ofp_DW.is_Directional_axis_f = claw_ofp_IN_Yaw_rate_cmd;
          claw_ofp_B.FMMOUT_a.yaw_axis_mode = 1U;
        }

        /* case IN_Yaw_rate_cmd: */
      } else if (((real32_T)fabs(claw_in_cmd.yaw_rate_cmd) <=
                  claw_ofp_DW.MC_YAWRATE_DZ) && claw_out_state.armed) {
        claw_ofp_DW.is_Directional_axis_f = 1U;
        claw_ofp_B.FMMOUT_a.yaw_axis_mode = 2U;
        claw_ofp_B.FMMOUT_a.hdg_cmd = claw_out_state.sensor_raw.heading;
      } else {
        claw_ofp_B.FMMOUT_a.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;
      }

      if (claw_ofp_DW.is_Vertical_axis_p == 1U) {
        if (((real32_T)fabs(claw_in_cmd.vel_z_cmd) > claw_ofp_DW.VEL_Z_THOLD *
             1.2) || (!claw_out_state.armed)) {
          /* Disarmed인 경우는 default로 전환 */
          claw_ofp_DW.is_Vertical_axis_p = claw_ofp_IN_Velocity;
          claw_ofp_B.FMMOUT_a.vert_axis_mode = 1U;
        }

        /* case IN_Velocity: */
      } else if (((real32_T)fabs(claw_in_cmd.vel_z_cmd) <=
                  claw_ofp_DW.VEL_Z_THOLD) && claw_out_state.armed &&
                 (!claw_out_state.landed)) {
        /* Arm되고 이륙 후 속도명령이 작을 때 */
        claw_ofp_DW.is_Vertical_axis_p = claw_ofp_IN_Pos_Hold;
        claw_ofp_B.FMMOUT_a.vert_axis_mode = 2U;
        claw_ofp_B.FMMOUT_a.alt_cmd = (real32_T)(-claw_out_state.ac_z_local -
          claw_out_state.sensor_raw.vel_down * 0.4);
      } else {
        claw_ofp_B.FMMOUT_a.vel_z_cmd = claw_in_cmd.vel_z_cmd;

        /*  To save flight log */
        claw_ofp_B.FMMOUT_a.alt_cmd = (real32_T)-claw_out_state.ac_z_local;
      }

      if (claw_ofp_DW.is_Horizontal_k == 1U) {
        if (((real32_T)fabs(claw_in_cmd.vel_x_cmd) > claw_ofp_DW.POS_XY_THOLD *
             1.2) || ((real32_T)fabs(claw_in_cmd.vel_y_cmd) >
                      claw_ofp_DW.POS_XY_THOLD * 1.2) || (!claw_out_state.armed))
        {
          /* Disarmed인 경우는 default로 전환 */
          claw_ofp_DW.is_Horizontal_k = claw_ofp_IN_Velocity;
          claw_ofp_B.FMMOUT_a.roll_axis_mode = 4U;
          claw_ofp_B.FMMOUT_a.pitch_axis_mode = 4U;
        } else {
          /*  To save flight data log */
          claw_ofp_B.FMMOUT_a.vel_x_cmd = claw_out_state.fwd_spd;
          claw_ofp_B.FMMOUT_a.vel_y_cmd = claw_out_state.lat_spd;
        }

        /* case IN_Velocity: */
      } else if (((real32_T)fabs(claw_in_cmd.vel_x_cmd) <=
                  claw_ofp_DW.POS_XY_THOLD) && ((real32_T)fabs
                  (claw_in_cmd.vel_y_cmd) <= claw_ofp_DW.POS_XY_THOLD) &&
                 claw_out_state.armed) {
        /*  (abs(Vsm.fwd_spd)<POS_XY_THOLD)&&(abs(Vsm.lat_spd)<POS_XY_THOLD)&&... */
        claw_ofp_DW.is_Horizontal_k = claw_ofp_IN_Pos_Hold;
        claw_ofp_B.FMMOUT_a.roll_axis_mode = 5U;
        claw_ofp_B.FMMOUT_a.pitch_axis_mode = 5U;
        claw_ofp_B.Abs1_g = claw_out_state.sensor_raw.vel_north / 0.5 * 0.4 +
          claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_a.wp_x_local[0] = claw_ofp_B.Abs1_g;
        claw_ofp_B.DataType5 = claw_out_state.sensor_raw.vel_east / 0.5 * 0.4 +
          claw_out_state.ac_y_local;
        claw_ofp_B.FMMOUT_a.wp_y_local[0] = claw_ofp_B.DataType5;
        claw_ofp_B.FMMOUT_a.wp_x_local[1] = claw_ofp_B.Abs1_g;
        claw_ofp_B.FMMOUT_a.wp_y_local[1] = claw_ofp_B.DataType5;
        claw_ofp_B.FMMOUT_a.wp_x_local[2] = claw_ofp_B.Abs1_g;
        claw_ofp_B.FMMOUT_a.wp_y_local[2] = claw_ofp_B.DataType5;
      } else {
        claw_ofp_B.FMMOUT_a.vel_x_cmd = claw_in_cmd.vel_x_cmd;
        claw_ofp_B.FMMOUT_a.vel_y_cmd = claw_in_cmd.vel_y_cmd;
        claw_ofp_B.FMMOUT_a.wp_x_local[0] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_a.wp_y_local[0] = claw_out_state.ac_y_local;
        claw_ofp_B.FMMOUT_a.wp_x_local[1] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_a.wp_y_local[1] = claw_out_state.ac_y_local;
        claw_ofp_B.FMMOUT_a.wp_x_local[2] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_a.wp_y_local[2] = claw_out_state.ac_y_local;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c17_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_a = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_a.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_a.claw_mode = 5U;
    claw_ofp_DW.MC_YAWRATE_DZ = 1.8;
    claw_ofp_DW.is_Directional_axis_f = claw_ofp_IN_Yaw_rate_cmd;
    claw_ofp_B.FMMOUT_a.yaw_axis_mode = 1U;
    claw_ofp_DW.VEL_Z_THOLD = 0.06;
    claw_ofp_DW.is_Vertical_axis_p = claw_ofp_IN_Velocity;
    claw_ofp_B.FMMOUT_a.vert_axis_mode = 1U;
    claw_ofp_DW.POS_XY_THOLD = 0.4;
    claw_ofp_B.FMMOUT_a.guide_type = 1U;
    claw_ofp_DW.is_Horizontal_k = claw_ofp_IN_Velocity;
    claw_ofp_B.FMMOUT_a.roll_axis_mode = 4U;
    claw_ofp_B.FMMOUT_a.pitch_axis_mode = 4U;
  }

  /* End of Chart: '<S157>/stick_pos' */

  /* RelationalOperator: '<S146>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 6U);

  /* BusCreator generated from: '<S152>/point_nav' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  claw_ofp_B.BusConversion_InsertedFor_point.raw = claw_in_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.flight_mode =
    claw_in_cmd.flight_mode;
  claw_ofp_B.BusConversion_InsertedFor_point.ail_cmd = claw_in_cmd.ail_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.ele_cmd = claw_in_cmd.ele_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.rud_cmd = claw_in_cmd.rud_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.roll_rate_cmd =
    claw_in_cmd.roll_rate_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.pitch_rate_cmd =
    claw_in_cmd.pitch_rate_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.yaw_rate_cmd =
    claw_in_cmd.yaw_rate_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.thr_cmd = claw_in_cmd.thr_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.roll_cmd = claw_in_cmd.roll_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.pitch_cmd = claw_in_cmd.pitch_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.hdg_cmd = claw_in_cmd.hdg_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.alt_cmd = claw_in_cmd.alt_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.vel_x_cmd = claw_in_cmd.vel_x_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.vel_y_cmd = claw_in_cmd.vel_y_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.vel_z_cmd = claw_in_cmd.vel_z_cmd;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lat[0] = claw_in_cmd.wp_lat[0];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lon[0] = claw_in_cmd.wp_lon[0];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_alt[0] = claw_in_cmd.wp_alt[0];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_spd[0] = claw_in_cmd.wp_spd[0];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lat[1] = claw_in_cmd.wp_lat[1];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lon[1] = claw_in_cmd.wp_lon[1];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_alt[1] = claw_in_cmd.wp_alt[1];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_spd[1] = claw_in_cmd.wp_spd[1];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lat[2] = claw_in_cmd.wp_lat[2];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_lon[2] = claw_in_cmd.wp_lon[2];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_alt[2] = claw_in_cmd.wp_alt[2];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_spd[2] = claw_in_cmd.wp_spd[2];
  claw_ofp_B.BusConversion_InsertedFor_point.wp_loiter_type =
    claw_in_cmd.wp_loiter_type;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_loiter_radius =
    claw_in_cmd.wp_loiter_radius;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_loiter_length =
    claw_in_cmd.wp_loiter_length;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_loiter_bearing =
    claw_in_cmd.wp_loiter_bearing;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_loiter_dir =
    claw_in_cmd.wp_loiter_dir;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_alt_behaviour =
    claw_in_cmd.wp_alt_behaviour;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_flying_behaviour =
    claw_in_cmd.wp_flying_behaviour;
  claw_ofp_B.BusConversion_InsertedFor_point.wp_hdg_cmd_override =
    claw_in_cmd.wp_hdg_cmd_override;

  /* Chart: '<S152>/point_nav' incorporates:
   *  BusCreator generated from: '<S152>/point_nav'
   *  Inport: '<Root>/in_cmd'
   *  Memory generated from: '<S2>/Memory'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.is_active_c8_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c8_claw_ofp = 1U;
    claw_ofp_DW.is_c8_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c8_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c8_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_do.hdg_cmd = claw_in_cmd.hdg_cmd;
      claw_ofp_B.FMMOUT_do.alt_cmd = (real32_T)
        -claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1];

      /* Outputs for Function Call SubSystem: '<S169>/Active.Horizontal.dist2d' */
      /* Calculate new waypoint distance from av */
      claw_ofp_ActiveHorizontaldist2d
        (claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1],
         claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1],
         claw_out_state.ac_x_local, claw_out_state.ac_y_local,
         &claw_ofp_B.Abs1_g);

      /* End of Outputs for SubSystem: '<S169>/Active.Horizontal.dist2d' */
      switch (claw_ofp_DW.is_Horizontal_kb) {
       case claw_ofp_IN_Circle:
        if (claw_ofp_B.Abs1_g > claw_in_cmd.wp_loiter_radius * 2.0F) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_waypoint_following;
          enter_atomic_waypoint_following
            (&claw_ofp_B.BusConversion_InsertedFor_point);
        } else if ((int16_T)claw_in_cmd.wp_loiter_type == 4) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_Hover;
          claw_ofp_B.FMMOUT_do.guide_type = 1U;

          /*  Speed command use default MPC_HOLD_MAX_XY in POSHOLD type. */
          claw_ofp_B.FMMOUT_do.wp_loiter_type = 4U;
          claw_ofp_B.FMMOUT_do.orbit_flag = false;
        } else {
          claw_ofp_B.FMMOUT_do.wp_radius[0] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT_do.wp_spd[0] = claw_in_cmd.wp_spd[0];
          claw_ofp_B.FMMOUT_do.wp_radius[1] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT_do.wp_spd[1] = claw_in_cmd.wp_spd[1];
          claw_ofp_B.FMMOUT_do.wp_radius[2] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT_do.wp_spd[2] = claw_in_cmd.wp_spd[2];
          claw_ofp_B.FMMOUT_do.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT_do.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
        }
        break;

       case claw_ofp_IN_Hover:
        if (claw_ofp_B.Abs1_g > 6.0) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_waypoint_following;
          enter_atomic_waypoint_following
            (&claw_ofp_B.BusConversion_InsertedFor_point);
        } else if ((int16_T)claw_in_cmd.wp_loiter_type == 1) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_Circle;
          claw_ofp_B.FMMOUT_do.guide_type = 4U;
          claw_ofp_B.FMMOUT_do.wp_loiter_type = 1U;
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[0] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[1] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[2] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.orbit_flag = true;
        } else {
          claw_ofp_B.FMMOUT_do.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];

          /*  WP update */
          claw_ofp_B.FMMOUT_do.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
        }
        break;

       default:
        /* case IN_waypoint_following: */
        if (((int16_T)claw_in_cmd.wp_loiter_type != 1) && ((claw_ofp_B.Abs1_g <
              5.0) || claw_out_pos_control.pass_flag)) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_Hover;
          claw_ofp_B.FMMOUT_do.guide_type = 1U;

          /*  Speed command use default MPC_HOLD_MAX_XY in POSHOLD type. */
          claw_ofp_B.FMMOUT_do.wp_loiter_type = 4U;
          claw_ofp_B.FMMOUT_do.orbit_flag = false;
        } else if (((int16_T)claw_in_cmd.wp_loiter_type == 1) &&
                   claw_out_pos_control.pass_flag) {
          claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_Circle;
          claw_ofp_B.FMMOUT_do.guide_type = 4U;
          claw_ofp_B.FMMOUT_do.wp_loiter_type = 1U;
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[0] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[1] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.wp_orbit_dir[2] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT_do.orbit_flag = true;
        } else {
          claw_ofp_B.FMMOUT_do.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT_do.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
          claw_ofp_B.FMMOUT_do.wp_spd[0] = claw_in_cmd.wp_spd[0];
          claw_ofp_B.FMMOUT_do.wp_spd[1] = claw_in_cmd.wp_spd[1];
          claw_ofp_B.FMMOUT_do.wp_spd[2] = claw_in_cmd.wp_spd[2];
          claw_ofp_B.FMMOUT_do.wp_loiter_type = claw_in_cmd.wp_loiter_type;

          /* WP이동중 항로점 로이터 속성 변경시 속도 영향 고려. */
        }
        break;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c8_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_do = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_do.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_do.claw_mode = 6U;
    claw_ofp_B.FMMOUT_do.yaw_axis_mode = 2U;
    claw_ofp_B.FMMOUT_do.vert_axis_mode = 2U;

    /* Outputs for Function Call SubSystem: '<S169>/Active.Horizontal.dist2d' */
    /* Calculate new waypoint distance from av */
    claw_ofp_ActiveHorizontaldist2d
      (claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1],
       claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1],
       claw_out_state.ac_x_local, claw_out_state.ac_y_local, &claw_ofp_B.Abs1_g);

    /* End of Outputs for SubSystem: '<S169>/Active.Horizontal.dist2d' */
    claw_ofp_DW.is_Horizontal_kb = claw_ofp_IN_waypoint_following;
    enter_atomic_waypoint_following(&claw_ofp_B.BusConversion_InsertedFor_point);
  }

  /* End of Chart: '<S152>/point_nav' */

  /* RelationalOperator: '<S145>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 8U);

  /* Chart: '<S160>/waypoint' incorporates:
   *  BusCreator generated from: '<S160>/waypoint'
   *  Inport: '<Root>/in_cmd'
   *  Memory generated from: '<S2>/Memory'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.is_active_c20_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c20_claw_ofp = 1U;
    claw_ofp_DW.is_c20_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c20_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Horizontal = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_Directional_axis = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c20_claw_ofp = claw_ofp_IN_InActive;
    } else {
      if (claw_ofp_DW.is_Directional_axis == 1U) {
        if (claw_in_cmd.wp_hdg_cmd_override) {
          claw_ofp_DW.is_Directional_axis = claw_ofp_IN_Heading_Cmd_Manual;
          claw_ofp_B.FMMOUT.yaw_axis_mode = 2U;
        }

        /* case IN_Heading_Cmd_Manual: */
      } else if (!claw_in_cmd.wp_hdg_cmd_override) {
        claw_ofp_DW.is_Directional_axis = claw_ofp_IN_Heading_Cmd_Auto;

        /*  Follows waypoint track heading */
        claw_ofp_B.FMMOUT.yaw_axis_mode = 4U;
      } else {
        claw_ofp_B.FMMOUT.hdg_cmd = claw_in_cmd.hdg_cmd;
      }

      claw_ofp_B.FMMOUT.alt_cmd = (real32_T)
        -claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1];

      /* Outputs for Function Call SubSystem: '<S178>/Active.Horizontal.dist2d' */
      /* Calculate new waypoint distance from av */
      claw_ofp_ActiveHorizontaldist2d
        (claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1],
         claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1],
         claw_out_state.ac_x_local, claw_out_state.ac_y_local,
         &claw_ofp_B.Abs1_g);

      /* End of Outputs for SubSystem: '<S178>/Active.Horizontal.dist2d' */
      switch (claw_ofp_DW.is_Horizontal) {
       case claw_ofp_IN_Circle:
        if (claw_ofp_B.Abs1_g > claw_in_cmd.wp_loiter_radius * 2.0F) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_waypoint_following;
          claw_ofp_B.FMMOUT.roll_axis_mode = 5U;
          claw_ofp_B.FMMOUT.pitch_axis_mode = 5U;
          claw_ofp_B.FMMOUT.guide_type = 3U;
          claw_ofp_B.FMMOUT.orbit_flag = false;
          claw_ofp_B.FMMOUT.wp_loiter_type = claw_in_cmd.wp_loiter_type;

          /*  Speed decceleration effect */
          claw_ofp_B.FMMOUT.wp_x_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
          claw_ofp_B.FMMOUT.wp_y_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
          claw_ofp_B.FMMOUT.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
          claw_ofp_B.FMMOUT.wp_x_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
          claw_ofp_B.FMMOUT.wp_y_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
        } else if ((int16_T)claw_in_cmd.wp_loiter_type == 4) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_Hover;
          claw_ofp_B.FMMOUT.guide_type = 3U;

          /*  Speed command use default MPC_HOLD_MAX_XY in POSHOLD type. */
          claw_ofp_B.FMMOUT.wp_loiter_type = 4U;
          claw_ofp_B.FMMOUT.orbit_flag = false;
        } else {
          claw_ofp_B.FMMOUT.wp_radius[0] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT.wp_spd[0] = claw_in_cmd.wp_spd[0];
          claw_ofp_B.FMMOUT.wp_radius[1] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT.wp_spd[1] = claw_in_cmd.wp_spd[1];
          claw_ofp_B.FMMOUT.wp_radius[2] = claw_in_cmd.wp_loiter_radius;
          claw_ofp_B.FMMOUT.wp_spd[2] = claw_in_cmd.wp_spd[2];
          claw_ofp_B.FMMOUT.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
        }
        break;

       case claw_ofp_IN_Hover:
        if (claw_ofp_B.Abs1_g > 6.0) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_waypoint_following;
          claw_ofp_B.FMMOUT.roll_axis_mode = 5U;
          claw_ofp_B.FMMOUT.pitch_axis_mode = 5U;
          claw_ofp_B.FMMOUT.guide_type = 3U;
          claw_ofp_B.FMMOUT.orbit_flag = false;
          claw_ofp_B.FMMOUT.wp_loiter_type = claw_in_cmd.wp_loiter_type;

          /*  Speed decceleration effect */
          claw_ofp_B.FMMOUT.wp_x_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
          claw_ofp_B.FMMOUT.wp_y_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
          claw_ofp_B.FMMOUT.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
          claw_ofp_B.FMMOUT.wp_x_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
          claw_ofp_B.FMMOUT.wp_y_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
        } else if ((int16_T)claw_in_cmd.wp_loiter_type == 1) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_Circle;
          claw_ofp_B.FMMOUT.guide_type = 4U;
          claw_ofp_B.FMMOUT.wp_loiter_type = 1U;
          claw_ofp_B.FMMOUT.wp_orbit_dir[0] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.wp_orbit_dir[1] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.wp_orbit_dir[2] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.orbit_flag = true;
        } else {
          claw_ofp_B.FMMOUT.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];

          /*  WP update */
          claw_ofp_B.FMMOUT.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
        }
        break;

       default:
        /* case IN_waypoint_following: */
        if (((int16_T)claw_in_cmd.wp_loiter_type == 4) && ((claw_ofp_B.Abs1_g <
              5.0) || claw_out_pos_control.pass_flag)) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_Hover;
          claw_ofp_B.FMMOUT.guide_type = 3U;

          /*  Speed command use default MPC_HOLD_MAX_XY in POSHOLD type. */
          claw_ofp_B.FMMOUT.wp_loiter_type = 4U;
          claw_ofp_B.FMMOUT.orbit_flag = false;
        } else if (((int16_T)claw_in_cmd.wp_loiter_type == 1) &&
                   claw_out_pos_control.pass_flag) {
          claw_ofp_DW.is_Horizontal = claw_ofp_IN_Circle;
          claw_ofp_B.FMMOUT.guide_type = 4U;
          claw_ofp_B.FMMOUT.wp_loiter_type = 1U;
          claw_ofp_B.FMMOUT.wp_orbit_dir[0] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.wp_orbit_dir[1] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.wp_orbit_dir[2] = ((int16_T)
            claw_in_cmd.wp_loiter_dir != 0);
          claw_ofp_B.FMMOUT.orbit_flag = true;
        } else {
          claw_ofp_B.FMMOUT.wp_x_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
          claw_ofp_B.FMMOUT.wp_y_local[0] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
          claw_ofp_B.FMMOUT.wp_spd[0] = claw_in_cmd.wp_spd[0];
          claw_ofp_B.FMMOUT.wp_x_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
          claw_ofp_B.FMMOUT.wp_y_local[1] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
          claw_ofp_B.FMMOUT.wp_spd[1] = claw_in_cmd.wp_spd[1];
          claw_ofp_B.FMMOUT.wp_x_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
          claw_ofp_B.FMMOUT.wp_y_local[2] =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
          claw_ofp_B.FMMOUT.wp_spd[2] = claw_in_cmd.wp_spd[2];
          claw_ofp_B.FMMOUT.wp_loiter_type = claw_in_cmd.wp_loiter_type;

          /* WP이동중 항로점 로이터 속성 변경시 속도 영향 고려. */
        }
        break;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c20_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT.state_id = 1121000UL;
    claw_ofp_B.FMMOUT.claw_mode = 8U;
    claw_ofp_DW.is_Directional_axis = claw_ofp_IN_Heading_Cmd_Auto;

    /*  Follows waypoint track heading */
    claw_ofp_B.FMMOUT.yaw_axis_mode = 4U;
    claw_ofp_B.FMMOUT.vert_axis_mode = 2U;

    /* Outputs for Function Call SubSystem: '<S178>/Active.Horizontal.dist2d' */
    /* Calculate new waypoint distance from av */
    claw_ofp_ActiveHorizontaldist2d
      (claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1],
       claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1],
       claw_out_state.ac_x_local, claw_out_state.ac_y_local, &claw_ofp_B.Abs1_g);

    /* End of Outputs for SubSystem: '<S178>/Active.Horizontal.dist2d' */
    claw_ofp_DW.is_Horizontal = claw_ofp_IN_waypoint_following;
    claw_ofp_B.FMMOUT.roll_axis_mode = 5U;
    claw_ofp_B.FMMOUT.pitch_axis_mode = 5U;
    claw_ofp_B.FMMOUT.guide_type = 3U;
    claw_ofp_B.FMMOUT.orbit_flag = false;
    claw_ofp_B.FMMOUT.wp_loiter_type = claw_in_cmd.wp_loiter_type;

    /*  Speed decceleration effect */
    claw_ofp_B.FMMOUT.wp_x_local[0] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
    claw_ofp_B.FMMOUT.wp_y_local[0] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
    claw_ofp_B.FMMOUT.wp_x_local[1] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
    claw_ofp_B.FMMOUT.wp_y_local[1] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
    claw_ofp_B.FMMOUT.wp_x_local[2] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
    claw_ofp_B.FMMOUT.wp_y_local[2] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
  }

  /* End of Chart: '<S160>/waypoint' */

  /* RelationalOperator: '<S138>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 9U);

  /* Chart: '<S150>/loiter' incorporates:
   *  BusCreator generated from: '<S150>/loiter'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c3_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c3_claw_ofp = 1U;
    claw_ofp_DW.is_c3_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c3_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_c3_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_hl.hdg_cmd = claw_in_cmd.hdg_cmd;
      claw_ofp_B.FMMOUT_hl.alt_cmd = (real32_T)
        -claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1];
      claw_ofp_B.FMMOUT_hl.wp_x_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
      claw_ofp_B.FMMOUT_hl.wp_y_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
      claw_ofp_B.FMMOUT_hl.wp_spd[0] = claw_in_cmd.wp_spd[0];
      claw_ofp_B.FMMOUT_hl.wp_radius[0] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_hl.wp_orbit_dir[0] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
      claw_ofp_B.FMMOUT_hl.wp_x_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
      claw_ofp_B.FMMOUT_hl.wp_y_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
      claw_ofp_B.FMMOUT_hl.wp_spd[1] = claw_in_cmd.wp_spd[1];
      claw_ofp_B.FMMOUT_hl.wp_radius[1] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_hl.wp_orbit_dir[1] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
      claw_ofp_B.FMMOUT_hl.wp_x_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
      claw_ofp_B.FMMOUT_hl.wp_y_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
      claw_ofp_B.FMMOUT_hl.wp_spd[2] = claw_in_cmd.wp_spd[2];
      claw_ofp_B.FMMOUT_hl.wp_radius[2] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_hl.wp_orbit_dir[2] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c3_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_hl = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_hl.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_hl.claw_mode = 6U;
    claw_ofp_B.FMMOUT_hl.yaw_axis_mode = 2U;
    claw_ofp_B.FMMOUT_hl.vert_axis_mode = 2U;
    claw_ofp_B.FMMOUT_hl.roll_axis_mode = 5U;
    claw_ofp_B.FMMOUT_hl.pitch_axis_mode = 5U;
    claw_ofp_B.FMMOUT_hl.guide_type = 1U;
  }

  /* End of Chart: '<S150>/loiter' */

  /* RelationalOperator: '<S147>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 11U);

  /* Chart: '<S159>/takeoff' incorporates:
   *  BusCreator generated from: '<S159>/takeoff'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   */
  if (claw_ofp_DW.temporalCounter_i1 < MAX_uint32_T) {
    claw_ofp_DW.temporalCounter_i1++;
  }

  if (claw_ofp_DW.is_active_c19_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c19_claw_ofp = 1U;
    claw_ofp_DW.is_c19_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c19_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Horizontal_n = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_Vertical_axis = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c19_claw_ofp = claw_ofp_IN_InActive;
    } else {
      switch (claw_ofp_DW.is_Vertical_axis) {
       case claw_ofp_IN_Motor_startup:
        if (claw_ofp_DW.temporalCounter_i1 >= 400UL) {
          claw_ofp_DW.is_Vertical_axis = claw_ofp_IN_altitude_climb;
          claw_ofp_B.FMMOUT_k.vert_axis_mode = 1U;
          claw_ofp_B.FMMOUT_k.vel_z_cmd = 0.0F;
          claw_ofp_DW.del_vel_step = 0.0011666666666666668;
        } else {
          claw_ofp_B.FMMOUT_k.thr_cmd = (real32_T)(claw_ofp_B.FMMOUT_k.thr_cmd +
            0.001);
        }
        break;

       case claw_ofp_IN_altitude_climb:
        if (-claw_out_state.ac_z_local > 5.0) {
          /* 최소이륙고도를 초과하면, 항로점 고도로 상승 */
          claw_ofp_DW.is_Vertical_axis = claw_ofp_IN_altitude_hold;
          claw_ofp_B.FMMOUT_k.vert_axis_mode = 2U;
          claw_ofp_B.FMMOUT_k.vel_z_cmd = 1.5F;
        } else {
          /* 스텝마다 증가 고도 */
          claw_ofp_B.PY2 = claw_ofp_B.FMMOUT_k.vel_z_cmd -
            claw_ofp_DW.del_vel_step;
          if (claw_ofp_B.PY2 >= -0.7) {
            claw_ofp_B.FMMOUT_k.vel_z_cmd = (real32_T)claw_ofp_B.PY2;
          } else {
            claw_ofp_B.FMMOUT_k.vel_z_cmd = -0.7F;
          }

          claw_ofp_B.FMMOUT_k.alt_cmd = (real32_T)-claw_out_state.ac_z_local;

          /*  To view log data */
        }
        break;

       default:
        /* case IN_altitude_hold: */
        /* 절대 크기 기준 */
        if (-claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1] <= 5.0) {
          claw_ofp_B.FMMOUT_k.alt_cmd = 5.0F;
        } else {
          claw_ofp_B.FMMOUT_k.alt_cmd = (real32_T)
            -claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1];
        }

        /*  고도명령은 최소이륙고도 이상이 되어야 함 */
        break;
      }

      switch (claw_ofp_DW.is_Horizontal_n) {
       case claw_ofp_IN_Init:
        claw_ofp_DW.is_Horizontal_n = claw_ofp_IN_TakeoffHold;
        claw_ofp_B.FMMOUT_k.roll_axis_mode = 5U;
        claw_ofp_B.FMMOUT_k.pitch_axis_mode = 5U;
        claw_ofp_B.FMMOUT_k.guide_type = 1U;
        claw_ofp_B.FMMOUT_k.wp_x_local[0] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_k.wp_y_local[0] = claw_out_state.ac_y_local;
        claw_ofp_B.FMMOUT_k.wp_x_local[1] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_k.wp_y_local[1] = claw_out_state.ac_y_local;
        claw_ofp_B.FMMOUT_k.wp_x_local[2] = claw_out_state.ac_x_local;
        claw_ofp_B.FMMOUT_k.wp_y_local[2] = claw_out_state.ac_y_local;
        break;

       case claw_ofp_IN_TakeoffHold:
        if ((int16_T)claw_ofp_DW.is_Vertical_axis == (int16_T)
            claw_ofp_IN_altitude_hold) {
          /*  고도 유지모드가 활성화되는 경우, 항로점으로 이동 */
          claw_ofp_DW.is_Horizontal_n = claw_ofp_IN_Waypoint;
        } else {
          claw_ofp_B.FMMOUT_k.wp_spd[0] = 2.0F;
          claw_ofp_B.FMMOUT_k.wp_spd[1] = 2.0F;
          claw_ofp_B.FMMOUT_k.wp_spd[2] = 2.0F;
        }
        break;

       default:
        /* case IN_Waypoint: */
        claw_ofp_B.FMMOUT_k.wp_x_local[0] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
        claw_ofp_B.FMMOUT_k.wp_y_local[0] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
        claw_ofp_B.FMMOUT_k.wp_x_local[1] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
        claw_ofp_B.FMMOUT_k.wp_y_local[1] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
        claw_ofp_B.FMMOUT_k.wp_x_local[2] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
        claw_ofp_B.FMMOUT_k.wp_y_local[2] =
          claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
        break;
      }
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c19_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_k = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_k.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_k.claw_mode = 11U;
    claw_ofp_B.FMMOUT_k.yaw_axis_mode = 2U;
    claw_ofp_B.FMMOUT_k.hdg_cmd = claw_out_state.sensor_raw.heading;
    claw_ofp_DW.is_Vertical_axis = claw_ofp_IN_Motor_startup;
    claw_ofp_DW.temporalCounter_i1 = 0UL;
    claw_ofp_B.FMMOUT_k.vert_axis_mode = 0U;
    claw_ofp_B.FMMOUT_k.thr_cmd = 0.0F;
    claw_ofp_DW.is_Horizontal_n = claw_ofp_IN_Init;
  }

  /* End of Chart: '<S159>/takeoff' */

  /* RelationalOperator: '<S136>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 12U);

  /* Chart: '<S149>/land' incorporates:
   *  BusCreator generated from: '<S149>/land'
   *  Inport: '<Root>/in_cmd'
   *  Math: '<S163>/Hypot'
   *  Memory: '<S5>/Memory1'
   *  Outport: '<Root>/OutState'
   *  RelationalOperator: '<S163>/Relational Operator'
   *  Sum: '<S163>/Subtract'
   *  Sum: '<S163>/Subtract1'
   */
  if (claw_ofp_DW.is_active_c1_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c1_claw_ofp = 1U;
    claw_ofp_DW.is_c1_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c1_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_Vertical_axis_p1 = claw_ofp_IN_NO_ACTIVE_CHILD;
      claw_ofp_DW.is_c1_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_c.hdg_cmd = claw_in_cmd.hdg_cmd;
      switch (claw_ofp_DW.is_Vertical_axis_p1) {
       case claw_ofp_IN_AltHold:
        /* Outputs for Function Call SubSystem: '<S162>/Active.Vertical_axis.Check_boundary' */
        /* Lookup_n-D: '<S163>/1-D Lookup Table' incorporates:
         *  Outport: '<Root>/OutState'
         *  Sum: '<S163>/Subtract2'
         */
        claw_ofp_B.Abs1_g = look1_pbinlagpw
          (claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1] -
           claw_out_state.ac_z_local, claw_ofp_ConstP.uDLookupTable_bp01Data,
           claw_ofp_ConstP.uDLookupTable_tableData, &claw_ofp_DW.m_bpIndex, 3UL);
        if (claw_ofp_rt_hypotd
            (claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1] -
             claw_out_state.ac_x_local,
             claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1] -
             claw_out_state.ac_y_local) <= claw_ofp_B.Abs1_g) {
          claw_ofp_DW.is_Vertical_axis_p1 = claw_ofp_IN_Descending;
          claw_ofp_B.FMMOUT_c.vert_axis_mode = 1U;
          claw_ofp_B.FMMOUT_c.vel_z_cmd = 1.5F;
        }

        /* End of Outputs for SubSystem: '<S162>/Active.Vertical_axis.Check_boundary' */
        break;

       case claw_ofp_IN_Descending:
        if (claw_out_state.landed) {
          claw_ofp_DW.is_Vertical_axis_p1 = claw_ofp_IN_Motor_stop;
          claw_ofp_B.FMMOUT_c.vert_axis_mode = 0U;
          claw_ofp_B.FMMOUT_c.thr_cmd = 0.0F;
        } else {
          claw_ofp_B.FMMOUT_c.alt_cmd = (real32_T)-claw_out_state.ac_z_local;

          /*  To view logged data */
          claw_ofp_B.Abs1_g =
            claw_ofp_B.BusConversion_InsertedFor_point.wp_z_local[1] -
            claw_out_state.ac_z_local;

          /* Outputs for Function Call SubSystem: '<S162>/Active.Vertical_axis.Vz_commander' */
          /* Switch: '<S164>/Switch' incorporates:
           *  Constant: '<S164>/Constant'
           *  Constant: '<S164>/Constant2'
           *  Constant: '<S165>/Constant'
           *  Constant: '<S166>/Constant'
           *  RelationalOperator: '<S165>/Compare'
           *  RelationalOperator: '<S166>/Compare'
           *  Switch: '<S164>/Switch1'
           */
          if (claw_ofp_B.Abs1_g <= 5.0) {
            claw_ofp_B.FMMOUT_c.vel_z_cmd = 0.5F;
          } else if (claw_ofp_B.Abs1_g <= 15.0) {
            /* Switch: '<S164>/Switch1' incorporates:
             *  Constant: '<S164>/Constant1'
             */
            claw_ofp_B.FMMOUT_c.vel_z_cmd = 1.0F;
          } else {
            claw_ofp_B.FMMOUT_c.vel_z_cmd = 1.5F;
          }

          /* End of Switch: '<S164>/Switch' */
          /* End of Outputs for SubSystem: '<S162>/Active.Vertical_axis.Vz_commander' */
        }
        break;

       default:
        /* case IN_Motor_stop: */
        break;
      }

      claw_ofp_B.FMMOUT_c.wp_x_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
      claw_ofp_B.FMMOUT_c.wp_y_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
      claw_ofp_B.FMMOUT_c.wp_x_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
      claw_ofp_B.FMMOUT_c.wp_y_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
      claw_ofp_B.FMMOUT_c.wp_x_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
      claw_ofp_B.FMMOUT_c.wp_y_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c1_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_c = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_c.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_c.claw_mode = 12U;
    claw_ofp_B.FMMOUT_c.yaw_axis_mode = 2U;
    claw_ofp_DW.is_Vertical_axis_p1 = claw_ofp_IN_AltHold;
    claw_ofp_B.FMMOUT_c.vert_axis_mode = 2U;
    claw_ofp_B.FMMOUT_c.alt_cmd = (real32_T)-claw_out_state.ac_z_local;
    claw_ofp_B.FMMOUT_c.roll_axis_mode = 5U;
    claw_ofp_B.FMMOUT_c.pitch_axis_mode = 5U;
    claw_ofp_B.FMMOUT_c.guide_type = 1U;
    claw_ofp_B.FMMOUT_c.wp_spd[0] = 2.0F;
    claw_ofp_B.FMMOUT_c.wp_spd[1] = 2.0F;
    claw_ofp_B.FMMOUT_c.wp_spd[2] = 2.0F;
  }

  /* End of Chart: '<S149>/land' */

  /* RelationalOperator: '<S137>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 15U);

  /* Chart: '<S148>/knob' incorporates:
   *  BusCreator generated from: '<S148>/knob'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c12_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c12_claw_ofp = 1U;
    claw_ofp_DW.is_c12_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c12_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_c12_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_b.hdg_cmd = claw_in_cmd.hdg_cmd;
      claw_ofp_B.FMMOUT_b.alt_cmd = claw_in_cmd.alt_cmd;
      claw_ofp_B.FMMOUT_b.vel_x_cmd = claw_in_cmd.vel_x_cmd;
      claw_ofp_B.FMMOUT_b.vel_y_cmd = 0.0F;
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c12_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_b = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_b.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_b.claw_mode = 5U;
    claw_ofp_B.FMMOUT_b.yaw_axis_mode = 2U;
    claw_ofp_B.FMMOUT_b.vert_axis_mode = 2U;
    claw_ofp_B.FMMOUT_b.roll_axis_mode = 4U;
    claw_ofp_B.FMMOUT_b.pitch_axis_mode = 4U;
  }

  /* End of Chart: '<S148>/knob' */

  /* RelationalOperator: '<S139>/Compare' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  rtb_AND_c = (claw_in_cmd.flight_mode == 16U);

  /* Chart: '<S151>/offboard' incorporates:
   *  BusCreator generated from: '<S151>/offboard'
   *  Inport: '<Root>/in_cmd'
   *  Memory: '<S5>/Memory1'
   */
  if (claw_ofp_DW.is_active_c2_claw_ofp == 0U) {
    claw_ofp_DW.is_active_c2_claw_ofp = 1U;
    claw_ofp_DW.is_c2_claw_ofp = claw_ofp_IN_InActive;
  } else if (claw_ofp_DW.is_c2_claw_ofp == 1U) {
    if (!rtb_AND_c) {
      claw_ofp_DW.is_c2_claw_ofp = claw_ofp_IN_InActive;
    } else {
      claw_ofp_B.FMMOUT_n.yaw_axis_mode = claw_in_cmd.dir_axis_mode;
      claw_ofp_B.FMMOUT_n.hdg_cmd = claw_in_cmd.hdg_cmd;

      /*  Heading mode */
      claw_ofp_B.FMMOUT_n.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;

      /*  Rate mode */
      claw_ofp_B.FMMOUT_n.rud_cmd = claw_in_cmd.rud_cmd;

      /*  Rudder */
      claw_ofp_B.FMMOUT_n.vert_axis_mode = claw_in_cmd.vert_axis_mode;
      claw_ofp_B.FMMOUT_n.alt_cmd = claw_in_cmd.alt_cmd;

      /*  Altitude */
      claw_ofp_B.FMMOUT_n.vel_z_cmd = claw_in_cmd.vel_z_cmd;

      /*  Velocity  */
      claw_ofp_B.FMMOUT_n.thr_cmd = claw_in_cmd.thr_cmd;

      /*  Throttle */
      claw_ofp_B.FMMOUT_n.roll_axis_mode = claw_in_cmd.lat_axis_mode;
      claw_ofp_B.FMMOUT_n.pitch_axis_mode = claw_in_cmd.lon_axis_mode;
      claw_ofp_B.FMMOUT_n.wp_loiter_type = claw_in_cmd.wp_loiter_type;
      claw_ofp_B.FMMOUT_n.wp_x_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
      claw_ofp_B.FMMOUT_n.wp_y_local[0] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
      claw_ofp_B.FMMOUT_n.wp_spd[0] = claw_in_cmd.wp_spd[0];
      claw_ofp_B.FMMOUT_n.wp_radius[0] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_n.wp_orbit_dir[0] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
      claw_ofp_B.FMMOUT_n.wp_x_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
      claw_ofp_B.FMMOUT_n.wp_y_local[1] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
      claw_ofp_B.FMMOUT_n.wp_spd[1] = claw_in_cmd.wp_spd[1];
      claw_ofp_B.FMMOUT_n.wp_radius[1] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_n.wp_orbit_dir[1] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
      claw_ofp_B.FMMOUT_n.wp_x_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
      claw_ofp_B.FMMOUT_n.wp_y_local[2] =
        claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
      claw_ofp_B.FMMOUT_n.wp_spd[2] = claw_in_cmd.wp_spd[2];
      claw_ofp_B.FMMOUT_n.wp_radius[2] = claw_in_cmd.wp_loiter_radius;
      claw_ofp_B.FMMOUT_n.wp_orbit_dir[2] = ((int16_T)claw_in_cmd.wp_loiter_dir
        != 0);
      claw_ofp_B.FMMOUT_n.guide_type = 2U;

      /* (x,y,v) */
      claw_ofp_B.FMMOUT_n.vel_x_cmd = claw_in_cmd.vel_x_cmd;
      claw_ofp_B.FMMOUT_n.vel_y_cmd = claw_in_cmd.vel_y_cmd;
      claw_ofp_B.FMMOUT_n.roll_cmd = claw_in_cmd.roll_cmd;
      claw_ofp_B.FMMOUT_n.pitch_cmd = claw_in_cmd.pitch_cmd;
      claw_ofp_B.FMMOUT_n.roll_rate_cmd = claw_in_cmd.roll_rate_cmd;
      claw_ofp_B.FMMOUT_n.pitch_rate_cmd = claw_in_cmd.pitch_rate_cmd;
      claw_ofp_B.FMMOUT_n.ail_cmd = claw_in_cmd.ail_cmd;
      claw_ofp_B.FMMOUT_n.ele_cmd = claw_in_cmd.ele_cmd;
    }

    /* case IN_InActive: */
  } else if (rtb_AND_c) {
    claw_ofp_DW.is_c2_claw_ofp = claw_ofp_IN_Active;
    claw_ofp_B.FMMOUT_n = claw_ofp_DW.Memory1_PreviousInput;

    /*  Get Previous FMM Data  */
    claw_ofp_B.FMMOUT_n.state_id = 1121000UL;
    claw_ofp_B.FMMOUT_n.claw_mode = 16U;
    claw_ofp_B.FMMOUT_n.yaw_axis_mode = claw_in_cmd.dir_axis_mode;
    claw_ofp_B.FMMOUT_n.hdg_cmd = claw_in_cmd.hdg_cmd;

    /*  Heading mode */
    claw_ofp_B.FMMOUT_n.yaw_rate_cmd = claw_in_cmd.yaw_rate_cmd;

    /*  Rate mode */
    claw_ofp_B.FMMOUT_n.rud_cmd = claw_in_cmd.rud_cmd;
    claw_ofp_B.FMMOUT_n.vert_axis_mode = claw_in_cmd.vert_axis_mode;
    claw_ofp_B.FMMOUT_n.alt_cmd = claw_in_cmd.alt_cmd;

    /*  Altitude */
    claw_ofp_B.FMMOUT_n.vel_z_cmd = claw_in_cmd.vel_z_cmd;

    /*  Velocity  */
    claw_ofp_B.FMMOUT_n.thr_cmd = claw_in_cmd.thr_cmd;
    claw_ofp_B.FMMOUT_n.roll_axis_mode = claw_in_cmd.lat_axis_mode;
    claw_ofp_B.FMMOUT_n.pitch_axis_mode = claw_in_cmd.lon_axis_mode;
    claw_ofp_B.FMMOUT_n.wp_loiter_type = claw_in_cmd.wp_loiter_type;
    claw_ofp_B.FMMOUT_n.wp_x_local[0] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[0];
    claw_ofp_B.FMMOUT_n.wp_y_local[0] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[0];
    claw_ofp_B.FMMOUT_n.wp_spd[0] = claw_in_cmd.wp_spd[0];
    claw_ofp_B.FMMOUT_n.wp_radius[0] = claw_in_cmd.wp_loiter_radius;
    claw_ofp_B.FMMOUT_n.wp_orbit_dir[0] = ((int16_T)claw_in_cmd.wp_loiter_dir !=
      0);
    claw_ofp_B.FMMOUT_n.wp_x_local[1] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[1];
    claw_ofp_B.FMMOUT_n.wp_y_local[1] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[1];
    claw_ofp_B.FMMOUT_n.wp_spd[1] = claw_in_cmd.wp_spd[1];
    claw_ofp_B.FMMOUT_n.wp_radius[1] = claw_in_cmd.wp_loiter_radius;
    claw_ofp_B.FMMOUT_n.wp_orbit_dir[1] = ((int16_T)claw_in_cmd.wp_loiter_dir !=
      0);
    claw_ofp_B.FMMOUT_n.wp_x_local[2] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_x_local[2];
    claw_ofp_B.FMMOUT_n.wp_y_local[2] =
      claw_ofp_B.BusConversion_InsertedFor_point.wp_y_local[2];
    claw_ofp_B.FMMOUT_n.wp_spd[2] = claw_in_cmd.wp_spd[2];
    claw_ofp_B.FMMOUT_n.wp_radius[2] = claw_in_cmd.wp_loiter_radius;
    claw_ofp_B.FMMOUT_n.wp_orbit_dir[2] = ((int16_T)claw_in_cmd.wp_loiter_dir !=
      0);
    claw_ofp_B.FMMOUT_n.guide_type = 2U;

    /* (x,y,v) */
    claw_ofp_B.FMMOUT_n.vel_x_cmd = claw_in_cmd.vel_x_cmd;
    claw_ofp_B.FMMOUT_n.vel_y_cmd = claw_in_cmd.vel_y_cmd;
    claw_ofp_B.FMMOUT_n.roll_cmd = claw_in_cmd.roll_cmd;
    claw_ofp_B.FMMOUT_n.pitch_cmd = claw_in_cmd.pitch_cmd;
    claw_ofp_B.FMMOUT_n.roll_rate_cmd = claw_in_cmd.roll_rate_cmd;
    claw_ofp_B.FMMOUT_n.pitch_rate_cmd = claw_in_cmd.pitch_rate_cmd;
    claw_ofp_B.FMMOUT_n.ail_cmd = claw_in_cmd.ail_cmd;
    claw_ofp_B.FMMOUT_n.ele_cmd = claw_in_cmd.ele_cmd;
  }

  /* End of Chart: '<S151>/offboard' */

  /* MultiPortSwitch: '<S5>/Multiport Switch' incorporates:
   *  Inport: '<Root>/in_cmd'
   */
  switch ((int16_T)claw_in_cmd.flight_mode) {
   case 0:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_e;
    break;

   case 1:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_h;
    break;

   case 2:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_j;
    break;

   case 3:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_i;
    break;

   case 4:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_d;
    break;

   case 5:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_a;
    break;

   case 6:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_do;
    break;

   case 7:
   case 10:
   case 13:
   case 14:
   case 17:
   case 18:
   case 19:
    break;

   case 8:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT;
    break;

   case 9:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_hl;
    break;

   case 11:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_k;
    break;

   case 12:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_c;
    break;

   case 15:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_b;
    break;

   case 16:
    claw_ofp_DW.Memory1_PreviousInput = claw_ofp_B.FMMOUT_n;
    break;
  }

  /* End of MultiPortSwitch: '<S5>/Multiport Switch' */

  /* MinMax: '<S337>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S337>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.roll_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.roll_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.roll_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.roll_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.roll_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.roll_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.roll_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.roll_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.roll_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.roll_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.roll_loop.DFilter_freq;
  claw_ofp_B.Abs1_g = claw_in_test.roll_loop.K_gain;

  /* MinMax: '<S337>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S337>/Min of Elements1'
   */
  claw_ofp_B.DataType5 = claw_in_test.roll_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S337>/Min of Elements1' incorporates:
     *  MinMax: '<S337>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.Abs1_g < claw_ofp_B.DataType4) {
      claw_ofp_B.Abs1_g = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S337>/Min of Elements' */
    if (claw_ofp_B.DataType5 > claw_ofp_B.DataType4) {
      claw_ofp_B.DataType5 = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S338>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S338>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.roll_rate_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.roll_rate_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.roll_rate_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.roll_rate_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.roll_rate_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.roll_rate_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.roll_rate_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.roll_rate_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.roll_rate_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.roll_rate_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.roll_rate_loop.DFilter_freq;
  claw_ofp_B.rtb_Sum1_idx_0 = claw_in_test.roll_rate_loop.K_gain;

  /* MinMax: '<S338>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S338>/Min of Elements1'
   */
  claw_ofp_B.Sum_k = claw_in_test.roll_rate_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S338>/Min of Elements1' incorporates:
     *  MinMax: '<S338>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.rtb_Sum1_idx_0 < claw_ofp_B.DataType4) {
      claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S338>/Min of Elements' */
    if (claw_ofp_B.Sum_k > claw_ofp_B.DataType4) {
      claw_ofp_B.Sum_k = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S339>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S339>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.pitch_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.pitch_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.pitch_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.pitch_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.pitch_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.pitch_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.pitch_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.pitch_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.pitch_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.pitch_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.pitch_loop.DFilter_freq;
  claw_ofp_B.DataType_n = claw_in_test.pitch_loop.K_gain;

  /* MinMax: '<S339>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S339>/Min of Elements1'
   */
  claw_ofp_B.Switch1_lx = claw_in_test.pitch_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S339>/Min of Elements1' incorporates:
     *  MinMax: '<S339>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.DataType_n < claw_ofp_B.DataType4) {
      claw_ofp_B.DataType_n = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S339>/Min of Elements' */
    if (claw_ofp_B.Switch1_lx > claw_ofp_B.DataType4) {
      claw_ofp_B.Switch1_lx = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S340>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S340>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.pitch_rate_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.pitch_rate_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.pitch_rate_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.pitch_rate_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.pitch_rate_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.pitch_rate_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.pitch_rate_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.pitch_rate_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.pitch_rate_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.pitch_rate_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.pitch_rate_loop.DFilter_freq;
  claw_ofp_B.integ_out_l = claw_in_test.pitch_rate_loop.K_gain;

  /* MinMax: '<S340>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S340>/Min of Elements1'
   */
  claw_ofp_B.Sum4_i2 = claw_in_test.pitch_rate_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S340>/Min of Elements1' incorporates:
     *  MinMax: '<S340>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.integ_out_l < claw_ofp_B.DataType4) {
      claw_ofp_B.integ_out_l = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S340>/Min of Elements' */
    if (claw_ofp_B.Sum4_i2 > claw_ofp_B.DataType4) {
      claw_ofp_B.Sum4_i2 = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S341>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S341>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.hdg_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.hdg_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.hdg_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.hdg_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.hdg_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.hdg_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.hdg_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.hdg_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.hdg_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.hdg_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.hdg_loop.DFilter_freq;
  claw_ofp_B.integ_out_m = claw_in_test.hdg_loop.K_gain;

  /* MinMax: '<S341>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S341>/Min of Elements1'
   */
  claw_ofp_B.integ_out_c = claw_in_test.hdg_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S341>/Min of Elements1' incorporates:
     *  MinMax: '<S341>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.integ_out_m < claw_ofp_B.DataType4) {
      claw_ofp_B.integ_out_m = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S341>/Min of Elements' */
    if (claw_ofp_B.integ_out_c > claw_ofp_B.DataType4) {
      claw_ofp_B.integ_out_c = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S342>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S342>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.yaw_rate_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.yaw_rate_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.yaw_rate_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.yaw_rate_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.yaw_rate_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.yaw_rate_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.yaw_rate_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.yaw_rate_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.yaw_rate_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.yaw_rate_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.yaw_rate_loop.DFilter_freq;
  claw_ofp_B.Limit4 = claw_in_test.yaw_rate_loop.K_gain;

  /* MinMax: '<S342>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S342>/Min of Elements1'
   */
  claw_ofp_B.Limit3 = claw_in_test.yaw_rate_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S342>/Min of Elements1' incorporates:
     *  MinMax: '<S342>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.Limit4 < claw_ofp_B.DataType4) {
      claw_ofp_B.Limit4 = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S342>/Min of Elements' */
    if (claw_ofp_B.Limit3 > claw_ofp_B.DataType4) {
      claw_ofp_B.Limit3 = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S343>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S343>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.Pos_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.Pos_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.Pos_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.Pos_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.Pos_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.Pos_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.Pos_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.Pos_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.Pos_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.Pos_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.Pos_loop.DFilter_freq;
  claw_ofp_B.RLL_RATE_K = claw_in_test.Pos_loop.K_gain;

  /* MinMax: '<S343>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S343>/Min of Elements1'
   */
  claw_ofp_B.minV = claw_in_test.Pos_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S343>/Min of Elements1' incorporates:
     *  MinMax: '<S343>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.RLL_RATE_K < claw_ofp_B.DataType4) {
      claw_ofp_B.RLL_RATE_K = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S343>/Min of Elements' */
    if (claw_ofp_B.minV > claw_ofp_B.DataType4) {
      claw_ofp_B.minV = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S344>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S344>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.Vxy_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.Vxy_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.Vxy_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.Vxy_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.Vxy_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.Vxy_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.Vxy_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.Vxy_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.Vxy_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.Vxy_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.Vxy_loop.DFilter_freq;
  claw_ofp_B.maxV = claw_in_test.Vxy_loop.K_gain;

  /* MinMax: '<S344>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S344>/Min of Elements1'
   */
  claw_ofp_B.Gain = claw_in_test.Vxy_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S344>/Min of Elements1' incorporates:
     *  MinMax: '<S344>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.maxV < claw_ofp_B.DataType4) {
      claw_ofp_B.maxV = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S344>/Min of Elements' */
    if (claw_ofp_B.Gain > claw_ofp_B.DataType4) {
      claw_ofp_B.Gain = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S345>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S345>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.Alt_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.Alt_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.Alt_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.Alt_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.Alt_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.Alt_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.Alt_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.Alt_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.Alt_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.Alt_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.Alt_loop.DFilter_freq;
  claw_ofp_B.rtb_Sum1_idx_1 = claw_in_test.Alt_loop.K_gain;

  /* MinMax: '<S345>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S345>/Min of Elements1'
   */
  claw_ofp_B.dist_to_wp = claw_in_test.Alt_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S345>/Min of Elements1' incorporates:
     *  MinMax: '<S345>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.rtb_Sum1_idx_1 < claw_ofp_B.DataType4) {
      claw_ofp_B.rtb_Sum1_idx_1 = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S345>/Min of Elements' */
    if (claw_ofp_B.dist_to_wp > claw_ofp_B.DataType4) {
      claw_ofp_B.dist_to_wp = claw_ofp_B.DataType4;
    }
  }

  /* MinMax: '<S346>/Min of Elements1' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S346>/Min of Elements'
   */
  claw_ofp_B.tmpForInput_tmp[0] = claw_in_test.Vz_loop.K_gain;
  claw_ofp_B.tmpForInput_tmp[1] = claw_in_test.Vz_loop.P_gain;
  claw_ofp_B.tmpForInput_tmp[2] = claw_in_test.Vz_loop.I_gain;
  claw_ofp_B.tmpForInput_tmp[3] = claw_in_test.Vz_loop.D_gain;
  claw_ofp_B.tmpForInput_tmp[4] = claw_in_test.Vz_loop.FF_gain;
  claw_ofp_B.tmpForInput_tmp[5] = claw_in_test.Vz_loop.output_max;
  claw_ofp_B.tmpForInput_tmp[6] = claw_in_test.Vz_loop.output_min;
  claw_ofp_B.tmpForInput_tmp[7] = claw_in_test.Vz_loop.integ_max;
  claw_ofp_B.tmpForInput_tmp[8] = claw_in_test.Vz_loop.integ_min;
  claw_ofp_B.tmpForInput_tmp[9] = claw_in_test.Vz_loop.LPF_freq;
  claw_ofp_B.tmpForInput_tmp[10] = claw_in_test.Vz_loop.DFilter_freq;
  claw_ofp_B.Switch_o = claw_in_test.Vz_loop.K_gain;

  /* MinMax: '<S346>/Min of Elements' incorporates:
   *  Inport: '<Root>/in_test'
   *  MinMax: '<S346>/Min of Elements1'
   */
  claw_ofp_B.Track_mode = claw_in_test.Vz_loop.K_gain;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 10; rtb_Compare_eu_0++) {
    /* MinMax: '<S346>/Min of Elements1' incorporates:
     *  MinMax: '<S346>/Min of Elements'
     */
    claw_ofp_B.DataType4 = claw_ofp_B.tmpForInput_tmp[(int32_T)(rtb_Compare_eu_0
      + 1)];
    if (claw_ofp_B.Switch_o < claw_ofp_B.DataType4) {
      claw_ofp_B.Switch_o = claw_ofp_B.DataType4;
    }

    /* MinMax: '<S346>/Min of Elements' */
    if (claw_ofp_B.Track_mode > claw_ofp_B.DataType4) {
      claw_ofp_B.Track_mode = claw_ofp_B.DataType4;
    }
  }

  /* Switch: '<S8>/Switch' incorporates:
   *  BusCreator generated from: '<S8>/Bus Assignment'
   *  Constant: '<S347>/Constant'
   *  Constant: '<S348>/Constant'
   *  Constant: '<S349>/Constant'
   *  Constant: '<S350>/Constant'
   *  Constant: '<S351>/Constant'
   *  Constant: '<S352>/Constant'
   *  Constant: '<S353>/Constant'
   *  Constant: '<S354>/Constant'
   *  Constant: '<S355>/Constant'
   *  Constant: '<S356>/Constant'
   *  Constant: '<S357>/Constant'
   *  Constant: '<S358>/Constant'
   *  Constant: '<S359>/Constant'
   *  Constant: '<S360>/Constant'
   *  Constant: '<S361>/Constant'
   *  Constant: '<S362>/Constant'
   *  Constant: '<S363>/Constant'
   *  Constant: '<S364>/Constant'
   *  Constant: '<S365>/Constant'
   *  Constant: '<S366>/Constant'
   *  Constant: '<S8>/Constant1'
   *  Constant: '<S8>/Constant4'
   *  Constant: '<S8>/Constant6'
   *  Inport: '<Root>/in_test'
   *  Logic: '<S336>/AND'
   *  Logic: '<S337>/AND'
   *  Logic: '<S338>/AND'
   *  Logic: '<S339>/AND'
   *  Logic: '<S340>/AND'
   *  Logic: '<S341>/AND'
   *  Logic: '<S342>/AND'
   *  Logic: '<S343>/AND'
   *  Logic: '<S344>/AND'
   *  Logic: '<S345>/AND'
   *  Logic: '<S346>/AND'
   *  Logic: '<S8>/AND'
   *  MinMax: '<S337>/Min of Elements'
   *  MinMax: '<S337>/Min of Elements1'
   *  MinMax: '<S338>/Min of Elements'
   *  MinMax: '<S338>/Min of Elements1'
   *  MinMax: '<S339>/Min of Elements'
   *  MinMax: '<S339>/Min of Elements1'
   *  MinMax: '<S340>/Min of Elements'
   *  MinMax: '<S340>/Min of Elements1'
   *  MinMax: '<S341>/Min of Elements'
   *  MinMax: '<S341>/Min of Elements1'
   *  MinMax: '<S342>/Min of Elements'
   *  MinMax: '<S342>/Min of Elements1'
   *  MinMax: '<S343>/Min of Elements'
   *  MinMax: '<S343>/Min of Elements1'
   *  MinMax: '<S344>/Min of Elements'
   *  MinMax: '<S344>/Min of Elements1'
   *  MinMax: '<S345>/Min of Elements'
   *  MinMax: '<S345>/Min of Elements1'
   *  MinMax: '<S346>/Min of Elements'
   *  MinMax: '<S346>/Min of Elements1'
   *  RelationalOperator: '<S347>/Compare'
   *  RelationalOperator: '<S348>/Compare'
   *  RelationalOperator: '<S349>/Compare'
   *  RelationalOperator: '<S350>/Compare'
   *  RelationalOperator: '<S351>/Compare'
   *  RelationalOperator: '<S352>/Compare'
   *  RelationalOperator: '<S353>/Compare'
   *  RelationalOperator: '<S354>/Compare'
   *  RelationalOperator: '<S355>/Compare'
   *  RelationalOperator: '<S356>/Compare'
   *  RelationalOperator: '<S357>/Compare'
   *  RelationalOperator: '<S358>/Compare'
   *  RelationalOperator: '<S359>/Compare'
   *  RelationalOperator: '<S360>/Compare'
   *  RelationalOperator: '<S361>/Compare'
   *  RelationalOperator: '<S362>/Compare'
   *  RelationalOperator: '<S363>/Compare'
   *  RelationalOperator: '<S364>/Compare'
   *  RelationalOperator: '<S365>/Compare'
   *  RelationalOperator: '<S366>/Compare'
   */
  if (claw_in_test.tune_on && ((claw_ofp_B.Abs1_g <= 10.0) &&
       (claw_ofp_B.DataType5 >= 0.1) && ((claw_ofp_B.rtb_Sum1_idx_0 <= 10.0) &&
        (claw_ofp_B.Sum_k >= 0.1)) && ((claw_ofp_B.DataType_n <= 10.0) &&
        (claw_ofp_B.Switch1_lx >= 0.1)) && ((claw_ofp_B.integ_out_l <= 10.0) &&
        (claw_ofp_B.Sum4_i2 >= 0.1)) && ((claw_ofp_B.integ_out_m <= 10.0) &&
        (claw_ofp_B.integ_out_c >= 0.1)) && ((claw_ofp_B.Limit4 <= 10.0) &&
        (claw_ofp_B.Limit3 >= 0.1)) && ((claw_ofp_B.RLL_RATE_K <= 10.0) &&
        (claw_ofp_B.minV >= 0.1)) && ((claw_ofp_B.maxV <= 10.0) &&
        (claw_ofp_B.Gain >= 0.1)) && ((claw_ofp_B.rtb_Sum1_idx_1 <= 10.0) &&
        (claw_ofp_B.dist_to_wp >= 0.1)) && ((claw_ofp_B.Switch_o <= 10.0) &&
        (claw_ofp_B.Track_mode >= 0.1)))) {
    claw_ofp_B.RLL_RATE_K = claw_in_test.roll_loop.P_gain;
    claw_ofp_B.integ_out_c = claw_in_test.roll_rate_loop.P_gain;
    claw_ofp_B.integ_out_m = claw_in_test.roll_rate_loop.I_gain;
    claw_ofp_B.Sum4_i2 = claw_in_test.pitch_loop.P_gain;
    claw_ofp_B.integ_out_l = claw_in_test.pitch_rate_loop.P_gain;
    claw_ofp_B.Switch1_lx = claw_in_test.pitch_rate_loop.I_gain;
    claw_ofp_B.DataType_n = claw_in_test.hdg_loop.P_gain;
    claw_ofp_B.DataType4 = claw_in_test.yaw_rate_loop.P_gain;
    claw_ofp_B.DataType5 = claw_in_test.yaw_rate_loop.I_gain;
    claw_ofp_B.Abs1_g = claw_in_test.yaw_rate_loop.FF_gain;
    claw_ofp_B.TrigonometricFunction2_mh = claw_in_test.Pos_loop.P_gain;
    claw_ofp_B.TrigonometricFunction1 = claw_in_test.Vxy_loop.P_gain;
    claw_ofp_B.K = claw_in_test.Vxy_loop.I_gain;
    claw_ofp_B.Product_g = claw_in_test.Alt_loop.P_gain;
    claw_ofp_B.Switch_o = claw_in_test.Vz_loop.P_gain;
  } else {
    claw_ofp_B.RLL_RATE_K = 1.0;
    claw_ofp_B.integ_out_c = 1.0;
    claw_ofp_B.integ_out_m = 1.0;
    claw_ofp_B.Sum4_i2 = 1.0;
    claw_ofp_B.integ_out_l = 1.0;
    claw_ofp_B.Switch1_lx = 1.0;
    claw_ofp_B.DataType_n = 1.0;
    claw_ofp_B.DataType4 = 1.0;
    claw_ofp_B.DataType5 = 1.0;
    claw_ofp_B.Abs1_g = 1.0;
    claw_ofp_B.TrigonometricFunction2_mh = 1.0;
    claw_ofp_B.TrigonometricFunction1 = 1.0;
    claw_ofp_B.K = 1.0;
    claw_ofp_B.Product_g = 1.0;
    claw_ofp_B.Switch_o = 1.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* Gain: '<S215>/Gain1' incorporates:
   *  DataTypeConversion: '<S188>/Cast To Double'
   *  Gain: '<S192>/Gain1'
   *  Gain: '<S201>/Gain1'
   *  Gain: '<S225>/Gain1'
   *  MultiPortSwitch: '<S180>/Multiport Switch'
   *  Outport: '<Root>/OutState'
   */
  claw_ofp_B.TrigonometricFunction1_d = 0.017453292519943295 *
    claw_out_state.sensor_raw.heading;

  /* Trigonometry: '<S216>/Trigonometric Function1' incorporates:
   *  Gain: '<S215>/Gain1'
   *  MultiPortSwitch: '<S180>/Multiport Switch'
   *  Trigonometry: '<S193>/Trigonometric Function1'
   */
  claw_ofp_B.rtb_Sum_k_tmp = cos(claw_ofp_B.TrigonometricFunction1_d);

  /* Trigonometry: '<S216>/Trigonometric Function2' incorporates:
   *  Gain: '<S215>/Gain1'
   *  MultiPortSwitch: '<S180>/Multiport Switch'
   *  Trigonometry: '<S193>/Trigonometric Function2'
   */
  claw_ofp_B.rtb_Limit4_tmp = sin(claw_ofp_B.TrigonometricFunction1_d);

  /* Sum: '<S188>/Add' incorporates:
   *  Outport: '<Root>/OutState'
   *  Sum: '<S186>/Add'
   *  Sum: '<S223>/Add'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_DW.Memory1_PreviousInput.wp_x_local[1] -
    claw_out_state.ac_x_local;
  claw_ofp_B.rtb_Sum1_idx_1 = claw_ofp_DW.Memory1_PreviousInput.wp_y_local[1] -
    claw_out_state.ac_y_local;

  /* Product: '<S216>/Product2' incorporates:
   *  Sum: '<S188>/Add'
   *  Trigonometry: '<S216>/Trigonometric Function1'
   *  Trigonometry: '<S216>/Trigonometric Function2'
   *  UnaryMinus: '<S216>/Unary Minus'
   */
  claw_ofp_B.minV = claw_ofp_B.rtb_Sum_k_tmp * claw_ofp_B.rtb_Sum1_idx_0 +
    claw_ofp_B.rtb_Limit4_tmp * claw_ofp_B.rtb_Sum1_idx_1;
  claw_ofp_B.maxV = -claw_ofp_B.rtb_Limit4_tmp * claw_ofp_B.rtb_Sum1_idx_0 +
    claw_ofp_B.rtb_Sum_k_tmp * claw_ofp_B.rtb_Sum1_idx_1;

  /* Sum: '<S189>/Add1' incorporates:
   *  Outport: '<Root>/OutState'
   *  UnaryMinus: '<S180>/Unary Minus'
   */
  claw_ofp_B.Sum_k = -claw_ofp_DW.Memory1_PreviousInput.alt_cmd -
    claw_out_state.ac_z_local;

  /* Trigonometry: '<S188>/Trigonometric Function1' */
  claw_ofp_B.Limit3 = atan2(claw_ofp_B.maxV, claw_ofp_B.minV);

  /* Product: '<S188>/Product' incorporates:
   *  Gain: '<S188>/Gain'
   *  Math: '<S188>/Hypot'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Dist = 0.5 * claw_ofp_rt_hypotd(claw_ofp_B.minV, claw_ofp_B.maxV) *
    claw_ofp_B.TrigonometricFunction2_mh;

  /* Saturate: '<S188>/Limit' */
  if (claw_ofp_B.Dist > 2.0) {
    claw_ofp_B.Dist = 2.0;
  } else if (claw_ofp_B.Dist < -2.0) {
    claw_ofp_B.Dist = -2.0;
  }

  /* End of Saturate: '<S188>/Limit' */

  /* Product: '<S188>/Product2' incorporates:
   *  Trigonometry: '<S188>/Trigonometric Function2'
   */
  claw_ofp_B.Vfwd = cos(claw_ofp_B.Limit3) * claw_ofp_B.Dist;

  /* Product: '<S188>/Product3' incorporates:
   *  Trigonometry: '<S188>/Trigonometric Function3'
   */
  claw_ofp_B.Vlat = claw_ofp_B.Dist * sin(claw_ofp_B.Limit3);

  /* Trigonometry: '<S223>/Trigonometric Function1' incorporates:
   *  Sum: '<S223>/Add1'
   */
  claw_ofp_B.Limit3 = atan2(claw_ofp_DW.Memory1_PreviousInput.wp_y_local[1] -
    claw_ofp_DW.Memory1_PreviousInput.wp_y_local[0],
    claw_ofp_DW.Memory1_PreviousInput.wp_x_local[1] -
    claw_ofp_DW.Memory1_PreviousInput.wp_x_local[0]);

  /* Gain: '<S230>/Gain' */
  claw_ofp_B.Gain = 57.295779513082323 * claw_ofp_B.Limit3;

  /* Trigonometry: '<S229>/Trigonometric Function' */
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 11; rtb_Compare_eu_0++) {
    claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0] =
      sincos_cordicLUT_n11_d[rtb_Compare_eu_0];
  }

  if (claw_ofp_B.Limit3 > 1.5707963267948966) {
    if (claw_ofp_B.Limit3 - 3.1415926535897931 <= 1.5707963267948966) {
      claw_ofp_B.Limit4 = claw_ofp_B.Limit3 - 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Limit4 = claw_ofp_B.Limit3 - 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else if (claw_ofp_B.Limit3 < -1.5707963267948966) {
    if (claw_ofp_B.Limit3 + 3.1415926535897931 >= -1.5707963267948966) {
      claw_ofp_B.Limit4 = claw_ofp_B.Limit3 + 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Limit4 = claw_ofp_B.Limit3 + 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else {
    claw_ofp_B.Limit4 = claw_ofp_B.Limit3;
    rtb_AND_c = false;
  }

  claw_ofp_B.Slope = 0.0;
  claw_ofp_B.rtb_DataType2_idx_0 = 0.60725303152913435;
  claw_ofp_B.xtmp = 0.60725303152913435;
  claw_ofp_B.ytmp = 0.0;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 11; rtb_Compare_eu_0++) {
    if (claw_ofp_B.Limit4 < 0.0) {
      claw_ofp_B.Limit4 += claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 += claw_ofp_B.ytmp;
      claw_ofp_B.Slope -= claw_ofp_B.xtmp;
    } else {
      claw_ofp_B.Limit4 -= claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 -= claw_ofp_B.ytmp;
      claw_ofp_B.Slope += claw_ofp_B.xtmp;
    }

    claw_ofp_B.xtmp = ldexp(claw_ofp_B.rtb_DataType2_idx_0, -(rtb_Compare_eu_0 +
      1));
    claw_ofp_B.ytmp = ldexp(claw_ofp_B.Slope, -(rtb_Compare_eu_0 + 1));
  }

  if (rtb_AND_c) {
    claw_ofp_B.Slope = -claw_ofp_B.Slope;
  }

  /* Trigonometry: '<S229>/Trigonometric Function1' */
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 11; rtb_Compare_eu_0++) {
    claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0] =
      sincos_cordicLUT_n11_d[rtb_Compare_eu_0];
  }

  if (claw_ofp_B.Limit3 > 1.5707963267948966) {
    if (claw_ofp_B.Limit3 - 3.1415926535897931 <= 1.5707963267948966) {
      claw_ofp_B.Limit3 -= 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Limit3 -= 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else if (claw_ofp_B.Limit3 < -1.5707963267948966) {
    if (claw_ofp_B.Limit3 + 3.1415926535897931 >= -1.5707963267948966) {
      claw_ofp_B.Limit3 += 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Limit3 += 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else {
    rtb_AND_c = false;
  }

  claw_ofp_B.Limit4 = 0.0;
  claw_ofp_B.rtb_DataType2_idx_0 = 0.60725303152913435;
  claw_ofp_B.xtmp = 0.60725303152913435;
  claw_ofp_B.ytmp = 0.0;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 11; rtb_Compare_eu_0++) {
    if (claw_ofp_B.Limit3 < 0.0) {
      claw_ofp_B.Limit3 += claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 += claw_ofp_B.ytmp;
      claw_ofp_B.Limit4 -= claw_ofp_B.xtmp;
    } else {
      claw_ofp_B.Limit3 -= claw_ofp_B.tmpForInput_tmp[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 -= claw_ofp_B.ytmp;
      claw_ofp_B.Limit4 += claw_ofp_B.xtmp;
    }

    claw_ofp_B.xtmp = ldexp(claw_ofp_B.rtb_DataType2_idx_0, -(rtb_Compare_eu_0 +
      1));
    claw_ofp_B.ytmp = ldexp(claw_ofp_B.Limit4, -(rtb_Compare_eu_0 + 1));
  }

  if (rtb_AND_c) {
    claw_ofp_B.rtb_DataType2_idx_0 = -claw_ofp_B.rtb_DataType2_idx_0;
  }

  /* Sum: '<S229>/Sum1' incorporates:
   *  Product: '<S229>/Product2'
   *  Product: '<S229>/Product3'
   *  Sum: '<S223>/Add'
   *  Trigonometry: '<S229>/Trigonometric Function'
   *  Trigonometry: '<S229>/Trigonometric Function1'
   */
  claw_ofp_B.Limit4 = claw_ofp_B.rtb_Sum1_idx_1 * claw_ofp_B.rtb_DataType2_idx_0
    - claw_ofp_B.rtb_Sum1_idx_0 * claw_ofp_B.Slope;

  /* Gain: '<S190>/XtrkGain' */
  claw_ofp_B.PY2 = 1.5 * claw_ofp_B.Limit4;

  /* Saturate: '<S190>/Limit1' */
  if (claw_ofp_B.PY2 > 90.0) {
    claw_ofp_B.PY2 = 90.0;
  } else if (claw_ofp_B.PY2 < -90.0) {
    claw_ofp_B.PY2 = -90.0;
  }

  /* End of Saturate: '<S190>/Limit1' */

  /* Sum: '<S190>/Sum1' */
  claw_ofp_B.Track_mode = claw_ofp_B.Gain + claw_ofp_B.PY2;

  /* Sum: '<S229>/Sum' incorporates:
   *  Product: '<S229>/Product'
   *  Product: '<S229>/Product1'
   *  Sum: '<S223>/Add'
   *  Trigonometry: '<S229>/Trigonometric Function'
   *  Trigonometry: '<S229>/Trigonometric Function1'
   */
  claw_ofp_B.Limit3 = claw_ofp_B.rtb_Sum1_idx_0 * claw_ofp_B.rtb_DataType2_idx_0
    + claw_ofp_B.rtb_Sum1_idx_1 * claw_ofp_B.Slope;

  /* Math: '<S223>/Hypot' */
  claw_ofp_B.dist_to_wp = claw_ofp_rt_hypotd(claw_ofp_B.Limit3,
    claw_ofp_B.Limit4);

  /* Logic: '<S190>/AND' incorporates:
   *  Constant: '<S220>/Constant'
   *  Constant: '<S221>/Constant'
   *  RelationalOperator: '<S220>/Compare'
   *  RelationalOperator: '<S221>/Compare'
   */
  rtb_cond_gnd_contact = ((claw_ofp_B.Limit3 > 0.0) && (claw_ofp_B.dist_to_wp >
    5.0));

  /* RelationalOperator: '<S197>/Compare' */
  rtb_AND_c = claw_ofp_DW.Memory1_PreviousInput.wp_orbit_dir[1];

  /* Switch: '<S208>/Switch' incorporates:
   *  Outport: '<Root>/OutState'
   */
  if (rtb_AND_c) {
    claw_ofp_B.rtb_DataType2_idx_0 = claw_out_state.ac_x_local;
    claw_ofp_B.Slope = claw_out_state.ac_y_local;
    claw_ofp_B.rtb_DataType2_idx_2 =
      claw_ofp_DW.Memory1_PreviousInput.wp_x_local[1];
    claw_ofp_B.PY2 = claw_ofp_DW.Memory1_PreviousInput.wp_y_local[1];
  } else {
    claw_ofp_B.rtb_DataType2_idx_0 = claw_out_state.ac_y_local;
    claw_ofp_B.Slope = claw_out_state.ac_x_local;
    claw_ofp_B.rtb_DataType2_idx_2 =
      claw_ofp_DW.Memory1_PreviousInput.wp_y_local[1];
    claw_ofp_B.PY2 = claw_ofp_DW.Memory1_PreviousInput.wp_x_local[1];
  }

  /* End of Switch: '<S208>/Switch' */

  /* Saturate: '<S187>/Saturation' */
  claw_ofp_B.Saturation_n = claw_ofp_DW.Memory1_PreviousInput.wp_radius[1];
  if (claw_ofp_B.Saturation_n > 1000.0F) {
    claw_ofp_B.Saturation_n = 1000.0F;
  } else if (claw_ofp_B.Saturation_n < 5.0F) {
    claw_ofp_B.Saturation_n = 5.0F;
  }

  /* End of Saturate: '<S187>/Saturation' */

  /* Chart: '<S198>/OrbitAlgorithm' incorporates:
   *  DataTypeConversion: '<S198>/Data Type Conversion'
   */
  claw_ofp_B.ytmp = claw_ofp_B.rtb_DataType2_idx_0 -
    claw_ofp_B.rtb_DataType2_idx_2;
  claw_ofp_B.xtmp = claw_ofp_B.Slope - claw_ofp_B.PY2;
  claw_ofp_B.Dist = 10.0 * claw_ofp_B.Saturation_n;

  /*  Virtual WP Distance
     Counter Clock Wise Orbit  */
  if ((claw_ofp_B.ytmp > -1.0E-5) && (claw_ofp_B.ytmp < 0.00001)) {
    if (claw_ofp_B.Slope > claw_ofp_B.PY2) {
      claw_ofp_B.Slope = 100000.0;

      /*  90deg  */
    } else {
      claw_ofp_B.Slope = -100000.0;

      /*  -90deg  */
    }
  } else {
    claw_ofp_B.Slope = claw_ofp_B.xtmp / claw_ofp_B.ytmp;
  }

  if ((claw_ofp_B.xtmp > -1.0E-5) && (claw_ofp_B.xtmp < 0.00001)) {
    if (claw_ofp_B.rtb_DataType2_idx_0 > claw_ofp_B.rtb_DataType2_idx_2) {
      claw_ofp_B.Slope = 0.000001;

      /*  0 deg  */
    } else {
      claw_ofp_B.Slope = -1.0E-6;

      /*  -0 deg  */
    }
  }

  claw_ofp_B.Slope = atan(claw_ofp_B.Slope);
  if (claw_ofp_B.rtb_DataType2_idx_0 > claw_ofp_B.rtb_DataType2_idx_2) {
    claw_ofp_B.rtb_DataType2_idx_0 = cos(claw_ofp_B.Slope);
    claw_ofp_B.rtb_DataType2_idx_2 += claw_ofp_B.Saturation_n *
      claw_ofp_B.rtb_DataType2_idx_0;
    claw_ofp_B.xtmp = sin(claw_ofp_B.Slope);
    claw_ofp_B.Slope = claw_ofp_B.Saturation_n * claw_ofp_B.xtmp +
      claw_ofp_B.PY2;
    claw_ofp_B.xtmp = claw_ofp_B.rtb_DataType2_idx_2 - claw_ofp_B.Dist *
      claw_ofp_B.xtmp;
    claw_ofp_B.PY2 = claw_ofp_B.Dist * claw_ofp_B.rtb_DataType2_idx_0 +
      claw_ofp_B.Slope;
  } else {
    claw_ofp_B.rtb_DataType2_idx_0 = cos(claw_ofp_B.Slope);
    claw_ofp_B.rtb_DataType2_idx_2 -= claw_ofp_B.Saturation_n *
      claw_ofp_B.rtb_DataType2_idx_0;
    claw_ofp_B.xtmp = sin(claw_ofp_B.Slope);
    claw_ofp_B.Slope = claw_ofp_B.PY2 - claw_ofp_B.Saturation_n *
      claw_ofp_B.xtmp;
    claw_ofp_B.xtmp = claw_ofp_B.Dist * claw_ofp_B.xtmp +
      claw_ofp_B.rtb_DataType2_idx_2;
    claw_ofp_B.PY2 = claw_ofp_B.Slope - claw_ofp_B.Dist *
      claw_ofp_B.rtb_DataType2_idx_0;
  }

  /* End of Chart: '<S198>/OrbitAlgorithm' */

  /* Switch: '<S209>/Switch' */
  if (rtb_AND_c) {
    claw_ofp_B.rtb_DataType2_idx_0 = claw_ofp_B.Slope;
    claw_ofp_B.Slope = claw_ofp_B.rtb_DataType2_idx_2;
    claw_ofp_B.rtb_DataType2_idx_2 = claw_ofp_B.PY2;
    claw_ofp_B.PY2 = claw_ofp_B.xtmp;
  } else {
    claw_ofp_B.rtb_DataType2_idx_0 = claw_ofp_B.rtb_DataType2_idx_2;
    claw_ofp_B.rtb_DataType2_idx_2 = claw_ofp_B.xtmp;
  }

  /* End of Switch: '<S209>/Switch' */

  /* Trigonometry: '<S211>/Trigonometric Function' incorporates:
   *  Sum: '<S211>/dX'
   *  Sum: '<S211>/dY'
   */
  claw_ofp_B.Dist = atan2(claw_ofp_B.rtb_DataType2_idx_2 -
    claw_ofp_B.rtb_DataType2_idx_0, claw_ofp_B.PY2 - claw_ofp_B.Slope);

  /* Sum: '<S206>/Subtract' incorporates:
   *  Outport: '<Root>/OutState'
   *  Sum: '<S205>/dX'
   *  Sum: '<S205>/dY'
   *  Trigonometry: '<S205>/Trigonometric Function'
   */
  claw_ofp_B.Subtract = claw_ofp_B.Dist - atan2(claw_ofp_B.rtb_DataType2_idx_2 -
    claw_out_state.ac_y_local, claw_ofp_B.PY2 - claw_out_state.ac_x_local);

  /* Trigonometry: '<S206>/Trigonometric Function' */
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 15; rtb_Compare_eu_0++) {
    claw_ofp_B.u1[rtb_Compare_eu_0] = sincos_cordicLUT_n15_d[rtb_Compare_eu_0];
  }

  if (claw_ofp_B.Subtract > 1.5707963267948966) {
    if (claw_ofp_B.Subtract - 3.1415926535897931 <= 1.5707963267948966) {
      claw_ofp_B.Subtract -= 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Subtract -= 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else if (claw_ofp_B.Subtract < -1.5707963267948966) {
    if (claw_ofp_B.Subtract + 3.1415926535897931 >= -1.5707963267948966) {
      claw_ofp_B.Subtract += 3.1415926535897931;
      rtb_AND_c = true;
    } else {
      claw_ofp_B.Subtract += 6.2831853071795862;
      rtb_AND_c = false;
    }
  } else {
    rtb_AND_c = false;
  }

  claw_ofp_B.Slope = 0.0;
  claw_ofp_B.rtb_DataType2_idx_0 = 0.60725293538591352;
  claw_ofp_B.xtmp = 0.60725293538591352;
  claw_ofp_B.ytmp = 0.0;
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 15; rtb_Compare_eu_0++) {
    if (claw_ofp_B.Subtract < 0.0) {
      claw_ofp_B.Subtract += claw_ofp_B.u1[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 += claw_ofp_B.ytmp;
      claw_ofp_B.Slope -= claw_ofp_B.xtmp;
    } else {
      claw_ofp_B.Subtract -= claw_ofp_B.u1[rtb_Compare_eu_0];
      claw_ofp_B.rtb_DataType2_idx_0 -= claw_ofp_B.ytmp;
      claw_ofp_B.Slope += claw_ofp_B.xtmp;
    }

    claw_ofp_B.xtmp = ldexp(claw_ofp_B.rtb_DataType2_idx_0, -(rtb_Compare_eu_0 +
      1));
    claw_ofp_B.ytmp = ldexp(claw_ofp_B.Slope, -(rtb_Compare_eu_0 + 1));
  }

  if (rtb_AND_c) {
    claw_ofp_B.Slope = -claw_ofp_B.Slope;
  }

  /* End of Trigonometry: '<S206>/Trigonometric Function' */

  /* Product: '<S198>/Product' incorporates:
   *  Constant: '<S187>/Constant'
   *  Math: '<S198>/DistanceToGo'
   *  Outport: '<Root>/OutState'
   *  Product: '<S206>/Product'
   *  Sum: '<S198>/Subtract'
   *  Sum: '<S198>/Subtract1'
   */
  claw_ofp_B.PY2 = claw_ofp_rt_hypotd(claw_out_state.ac_y_local -
    claw_ofp_B.rtb_DataType2_idx_2, claw_out_state.ac_x_local - claw_ofp_B.PY2) *
    claw_ofp_B.Slope * 30.0;

  /* Saturate: '<S198>/Orbit_Limit' */
  if (claw_ofp_B.PY2 > 30.0) {
    claw_ofp_B.PY2 = 30.0;
  } else if (claw_ofp_B.PY2 < -30.0) {
    claw_ofp_B.PY2 = -30.0;
  }

  /* End of Saturate: '<S198>/Orbit_Limit' */

  /* Sum: '<S198>/Add' incorporates:
   *  Gain: '<S210>/Gain'
   */
  claw_ofp_B.PY2 = 57.295779513082323 * claw_ofp_B.Dist - claw_ofp_B.PY2;

  /* MultiPortSwitch: '<S180>/Multiport Switch' incorporates:
   *  Product: '<S186>/Product2'
   *  Product: '<S186>/Product3'
   *  Product: '<S203>/Product2'
   *  Product: '<S227>/Product2'
   *  SignalConversion generated from: '<S203>/Matrix Concatenate'
   *  Trigonometry: '<S186>/Trigonometric Function2'
   *  Trigonometry: '<S186>/Trigonometric Function3'
   *  UnaryMinus: '<S203>/Unary Minus'
   *  UnaryMinus: '<S227>/Unary Minus'
   */
  switch ((int16_T)claw_ofp_DW.Memory1_PreviousInput.guide_type) {
   case 0:
   case 1:
    break;

   case 2:
    /* Product: '<S193>/Product2' incorporates:
     *  Sum: '<S186>/Add'
     *  UnaryMinus: '<S193>/Unary Minus'
     */
    claw_ofp_B.Dist = claw_ofp_B.rtb_Sum_k_tmp * claw_ofp_B.rtb_Sum1_idx_0 +
      claw_ofp_B.rtb_Limit4_tmp * claw_ofp_B.rtb_Sum1_idx_1;
    claw_ofp_B.Vfwd = -claw_ofp_B.rtb_Limit4_tmp * claw_ofp_B.rtb_Sum1_idx_0 +
      claw_ofp_B.rtb_Sum_k_tmp * claw_ofp_B.rtb_Sum1_idx_1;

    /* Switch: '<S186>/Switch' incorporates:
     *  RelationalOperator: '<S191>/Compare'
     *
     * Block description for '<S186>/Switch':
     *  Orbit_flag=True인 경우, 항로점 통과를 위해 속도 명령을 그대로 출력하고,
     *  Orbit_flag=False인경우는 거리에 따라 속도 감속후 정지
     */
    if (claw_ofp_DW.Memory1_PreviousInput.wp_loiter_type == 4U) {
      /* Product: '<S186>/Product' incorporates:
       *  Gain: '<S186>/Gain'
       *  Math: '<S186>/Hypot'
       *  Switch: '<S8>/Switch'
       */
      claw_ofp_B.TrigonometricFunction1_d = 0.5 * claw_ofp_rt_hypotd
        (claw_ofp_B.Dist, claw_ofp_B.Vfwd) *
        claw_ofp_B.TrigonometricFunction2_mh;

      /* Switch: '<S195>/Switch2' incorporates:
       *  RelationalOperator: '<S195>/LowerRelop1'
       *  RelationalOperator: '<S195>/UpperRelop'
       *  Switch: '<S195>/Switch'
       */
      if (claw_ofp_B.TrigonometricFunction1_d >
          claw_ofp_DW.Memory1_PreviousInput.wp_spd[1]) {
        claw_ofp_B.TrigonometricFunction1_d =
          claw_ofp_DW.Memory1_PreviousInput.wp_spd[1];
      } else if (claw_ofp_B.TrigonometricFunction1_d < 0.0) {
        /* Switch: '<S195>/Switch' */
        claw_ofp_B.TrigonometricFunction1_d = 0.0;
      }

      /* End of Switch: '<S195>/Switch2' */
    } else {
      claw_ofp_B.TrigonometricFunction1_d =
        claw_ofp_DW.Memory1_PreviousInput.wp_spd[1];
    }

    /* End of Switch: '<S186>/Switch' */

    /* Saturate: '<S186>/Limit' */
    if (claw_ofp_B.TrigonometricFunction1_d > 20.0) {
      claw_ofp_B.TrigonometricFunction1_d = 20.0;
    } else if (claw_ofp_B.TrigonometricFunction1_d < 0.0) {
      claw_ofp_B.TrigonometricFunction1_d = 0.0;
    }

    /* End of Saturate: '<S186>/Limit' */

    /* Trigonometry: '<S186>/Trigonometric Function1' */
    claw_ofp_B.Dist = atan2(claw_ofp_B.Vfwd, claw_ofp_B.Dist);
    claw_ofp_B.Vfwd = cos(claw_ofp_B.Dist) * claw_ofp_B.TrigonometricFunction1_d;
    claw_ofp_B.Vlat = claw_ofp_B.TrigonometricFunction1_d * sin(claw_ofp_B.Dist);
    break;

   case 3:
    /* Switch: '<S190>/Switch4' incorporates:
     *  DataTypeConversion: '<S190>/Data Type Conversion'
     *  Gain: '<S190>/Gain'
     *  RelationalOperator: '<S222>/Compare'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.wp_loiter_type == 4U) {
      claw_ofp_B.Vfwd = 0.5 * claw_ofp_B.dist_to_wp;
    } else {
      claw_ofp_B.Vfwd = claw_ofp_DW.Memory1_PreviousInput.wp_spd[1];
    }

    /* End of Switch: '<S190>/Switch4' */

    /* Switch: '<S190>/Switch' incorporates:
     *  Gain: '<S231>/Gain'
     *  Sum: '<S223>/Add'
     *  Switch: '<S190>/Switch2'
     *  Trigonometry: '<S223>/Trigonometric Function2'
     */
    if (rtb_cond_gnd_contact) {
      claw_ofp_B.Dist = claw_ofp_B.Track_mode;
    } else {
      claw_ofp_B.Dist = 57.295779513082323 * atan2(claw_ofp_B.rtb_Sum1_idx_1,
        claw_ofp_B.rtb_Sum1_idx_0);
    }

    /* End of Switch: '<S190>/Switch' */

    /* Gain: '<S226>/Gain1' */
    claw_ofp_B.rtb_DataType2_idx_2 = 0.017453292519943295 * claw_ofp_B.Dist;

    /* Switch: '<S224>/Switch2' incorporates:
     *  DataTypeConversion: '<S190>/Data Type Conversion'
     *  RelationalOperator: '<S224>/LowerRelop1'
     *  RelationalOperator: '<S224>/UpperRelop'
     *  Switch: '<S224>/Switch'
     */
    if (claw_ofp_B.Vfwd > claw_ofp_DW.Memory1_PreviousInput.wp_spd[1]) {
      claw_ofp_B.Vfwd = claw_ofp_DW.Memory1_PreviousInput.wp_spd[1];
    } else if (claw_ofp_B.Vfwd < 0.0) {
      /* Switch: '<S224>/Switch' */
      claw_ofp_B.Vfwd = 0.0;
    }

    /* End of Switch: '<S224>/Switch2' */

    /* Saturate: '<S190>/Limit2' */
    if (claw_ofp_B.Vfwd > 20.0) {
      claw_ofp_B.Vfwd = 20.0;
    } else if (claw_ofp_B.Vfwd < 0.0) {
      claw_ofp_B.Vfwd = 0.0;
    }

    /* End of Saturate: '<S190>/Limit2' */

    /* Trigonometry: '<S227>/Trigonometric Function2' */
    claw_ofp_B.TrigonometricFunction2_mh = sin
      (claw_ofp_B.TrigonometricFunction1_d);

    /* Trigonometry: '<S227>/Trigonometric Function1' */
    claw_ofp_B.Vlat = cos(claw_ofp_B.TrigonometricFunction1_d);

    /* Product: '<S227>/Product2' incorporates:
     *  Product: '<S228>/Product2'
     *  Product: '<S228>/Product3'
     *  Trigonometry: '<S228>/Trigonometric Function2'
     *  Trigonometry: '<S228>/Trigonometric Function3'
     */
    claw_ofp_B.Dist = cos(claw_ofp_B.rtb_DataType2_idx_2) * claw_ofp_B.Vfwd;
    claw_ofp_B.TrigonometricFunction1_d = claw_ofp_B.Vfwd * sin
      (claw_ofp_B.rtb_DataType2_idx_2);
    claw_ofp_B.Vfwd = claw_ofp_B.Vlat * claw_ofp_B.Dist +
      claw_ofp_B.TrigonometricFunction2_mh * claw_ofp_B.TrigonometricFunction1_d;
    claw_ofp_B.Vlat = -claw_ofp_B.TrigonometricFunction2_mh * claw_ofp_B.Dist +
      claw_ofp_B.Vlat * claw_ofp_B.TrigonometricFunction1_d;
    break;

   default:
    /* Gain: '<S202>/Gain1' */
    claw_ofp_B.Vfwd = 0.017453292519943295 * claw_ofp_B.PY2;

    /* Gain: '<S187>/Gain' */
    claw_ofp_B.Saturation_n *= 0.52359879F;

    /* Switch: '<S200>/Switch2' incorporates:
     *  RelationalOperator: '<S200>/LowerRelop1'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.wp_spd[1] <= claw_ofp_B.Saturation_n)
    {
      /* Switch: '<S200>/Switch' incorporates:
       *  Constant: '<S187>/Constant1'
       *  RelationalOperator: '<S200>/UpperRelop'
       */
      if (claw_ofp_DW.Memory1_PreviousInput.wp_spd[1] < 0.1) {
        claw_ofp_B.Saturation_n = 0.1F;
      } else {
        claw_ofp_B.Saturation_n = claw_ofp_DW.Memory1_PreviousInput.wp_spd[1];
      }

      /* End of Switch: '<S200>/Switch' */
    }

    /* End of Switch: '<S200>/Switch2' */

    /* Trigonometry: '<S203>/Trigonometric Function2' */
    claw_ofp_B.TrigonometricFunction2_mh = sin
      (claw_ofp_B.TrigonometricFunction1_d);

    /* Trigonometry: '<S203>/Trigonometric Function1' */
    claw_ofp_B.Vlat = cos(claw_ofp_B.TrigonometricFunction1_d);

    /* Product: '<S203>/Product2' incorporates:
     *  Product: '<S204>/Product2'
     *  Product: '<S204>/Product3'
     *  Trigonometry: '<S204>/Trigonometric Function2'
     *  Trigonometry: '<S204>/Trigonometric Function3'
     */
    claw_ofp_B.Dist = cos(claw_ofp_B.Vfwd) * claw_ofp_B.Saturation_n;
    claw_ofp_B.TrigonometricFunction1_d = claw_ofp_B.Saturation_n * sin
      (claw_ofp_B.Vfwd);
    claw_ofp_B.Vfwd = claw_ofp_B.Vlat * claw_ofp_B.Dist +
      claw_ofp_B.TrigonometricFunction2_mh * claw_ofp_B.TrigonometricFunction1_d;
    claw_ofp_B.Vlat = -claw_ofp_B.TrigonometricFunction2_mh * claw_ofp_B.Dist +
      claw_ofp_B.Vlat * claw_ofp_B.TrigonometricFunction1_d;
    break;
  }

  /* Switch: '<S185>/Sw2' incorporates:
   *  RelationalOperator: '<S271>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode >= 5U) {
    /* Switch: '<S185>/Sw3' incorporates:
     *  DataTypeConversion: '<S180>/Cast To Single'
     *  RelationalOperator: '<S268>/Compare'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode == 6U) {
      /* Abs: '<S185>/Abs' */
      claw_ofp_B.Saturation_n = (real32_T)fabs
        (claw_ofp_DW.Memory1_PreviousInput.vel_x_cmd);

      /* Switch: '<S265>/Switch2' incorporates:
       *  DataTypeConversion: '<S180>/Cast To Single'
       *  RelationalOperator: '<S265>/LowerRelop1'
       */
      if ((real32_T)claw_ofp_B.Vfwd <= claw_ofp_B.Saturation_n) {
        /* Switch: '<S265>/Switch' incorporates:
         *  RelationalOperator: '<S265>/UpperRelop'
         *  UnaryMinus: '<S185>/Unary Minus'
         */
        if ((real32_T)claw_ofp_B.Vfwd < -claw_ofp_B.Saturation_n) {
          claw_ofp_B.Saturation_n = -claw_ofp_B.Saturation_n;
        } else {
          claw_ofp_B.Saturation_n = (real32_T)claw_ofp_B.Vfwd;
        }

        /* End of Switch: '<S265>/Switch' */
      }

      /* End of Switch: '<S265>/Switch2' */
    } else {
      claw_ofp_B.Saturation_n = (real32_T)claw_ofp_B.Vfwd;
    }

    /* End of Switch: '<S185>/Sw3' */

    /* Saturate: '<S185>/Limit7' */
    if (claw_ofp_B.Saturation_n > 20.0F) {
      /* Saturate: '<S185>/Limit4' */
      claw_ofp_B.Saturation_n = 20.0F;
    } else if (claw_ofp_B.Saturation_n < -20.0F) {
      /* Saturate: '<S185>/Limit4' */
      claw_ofp_B.Saturation_n = -20.0F;
    }

    /* End of Saturate: '<S185>/Limit7' */
  } else {
    /* Saturate: '<S185>/Limit2' */
    claw_ofp_B.Saturation_n = claw_ofp_DW.Memory1_PreviousInput.vel_x_cmd;
    if (claw_ofp_B.Saturation_n > 20.0F) {
      /* Saturate: '<S185>/Limit4' */
      claw_ofp_B.Saturation_n = 20.0F;
    } else if (claw_ofp_B.Saturation_n < -20.0F) {
      /* Saturate: '<S185>/Limit4' */
      claw_ofp_B.Saturation_n = -20.0F;
    }

    /* End of Saturate: '<S185>/Limit2' */
  }

  /* End of Switch: '<S185>/Sw2' */

  /* Switch: '<S185>/Sw' incorporates:
   *  RelationalOperator: '<S269>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode >= 5U) {
    /* Switch: '<S185>/Sw4' incorporates:
     *  DataTypeConversion: '<S180>/Cast To Single'
     *  RelationalOperator: '<S272>/Compare'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode == 6U) {
      /* Abs: '<S185>/Abs1' */
      rtb_Abs1_e = (real32_T)fabs(claw_ofp_DW.Memory1_PreviousInput.vel_y_cmd);

      /* Switch: '<S266>/Switch2' incorporates:
       *  DataTypeConversion: '<S180>/Cast To Single'
       *  RelationalOperator: '<S266>/LowerRelop1'
       */
      if ((real32_T)claw_ofp_B.Vlat <= rtb_Abs1_e) {
        /* Switch: '<S266>/Switch' incorporates:
         *  RelationalOperator: '<S266>/UpperRelop'
         *  UnaryMinus: '<S185>/Unary Minus1'
         */
        if ((real32_T)claw_ofp_B.Vlat <
            -claw_ofp_DW.Memory1_PreviousInput.vel_y_cmd) {
          rtb_Abs1_e = -claw_ofp_DW.Memory1_PreviousInput.vel_y_cmd;
        } else {
          rtb_Abs1_e = (real32_T)claw_ofp_B.Vlat;
        }

        /* End of Switch: '<S266>/Switch' */
      }

      /* End of Switch: '<S266>/Switch2' */
    } else {
      rtb_Abs1_e = (real32_T)claw_ofp_B.Vlat;
    }

    /* End of Switch: '<S185>/Sw4' */

    /* Saturate: '<S185>/Limit6' */
    if (rtb_Abs1_e > 20.0F) {
      /* Saturate: '<S185>/Limit3' */
      rtb_Abs1_e = 20.0F;
    } else if (rtb_Abs1_e < -20.0F) {
      /* Saturate: '<S185>/Limit3' */
      rtb_Abs1_e = -20.0F;
    }

    /* End of Saturate: '<S185>/Limit6' */
  } else {
    /* Saturate: '<S185>/Limit1' */
    rtb_Abs1_e = claw_ofp_DW.Memory1_PreviousInput.vel_y_cmd;
    if (rtb_Abs1_e > 20.0F) {
      /* Saturate: '<S185>/Limit3' */
      rtb_Abs1_e = 20.0F;
    } else if (rtb_Abs1_e < -20.0F) {
      /* Saturate: '<S185>/Limit3' */
      rtb_Abs1_e = -20.0F;
    }

    /* End of Saturate: '<S185>/Limit1' */
  }

  /* End of Switch: '<S185>/Sw' */

  /* Abs: '<S189>/Abs' incorporates:
   *  Abs: '<S185>/Abs2'
   *  Switch: '<S185>/Sw1'
   *  Switch: '<S185>/Sw5'
   */
  claw_ofp_B.hdg_int = (real32_T)fabs
    (claw_ofp_DW.Memory1_PreviousInput.vel_z_cmd);

  /* MinMax: '<S189>/Min' incorporates:
   *  Abs: '<S189>/Abs'
   */
  if (claw_ofp_B.hdg_int <= 0.6) {
    claw_ofp_B.Dist = 0.6;
  } else {
    claw_ofp_B.Dist = claw_ofp_B.hdg_int;
  }

  /* End of MinMax: '<S189>/Min' */

  /* Product: '<S189>/Product' incorporates:
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Product_g *= claw_ofp_B.Sum_k;

  /* Switch: '<S218>/Switch2' incorporates:
   *  RelationalOperator: '<S218>/LowerRelop1'
   */
  if (claw_ofp_B.Product_g <= claw_ofp_B.Dist) {
    /* MinMax: '<S189>/Min1' incorporates:
     *  Abs: '<S189>/Abs'
     *  UnaryMinus: '<S189>/Unary Minus'
     */
    if (-claw_ofp_B.hdg_int <= -0.6) {
      claw_ofp_B.Dist = -claw_ofp_B.hdg_int;
    } else {
      claw_ofp_B.Dist = -0.6;
    }

    /* End of MinMax: '<S189>/Min1' */

    /* Switch: '<S218>/Switch' incorporates:
     *  RelationalOperator: '<S218>/UpperRelop'
     */
    if (claw_ofp_B.Product_g >= claw_ofp_B.Dist) {
      claw_ofp_B.Dist = claw_ofp_B.Product_g;
    }

    /* End of Switch: '<S218>/Switch' */
  }

  /* End of Switch: '<S218>/Switch2' */

  /* Switch: '<S185>/Sw1' incorporates:
   *  RelationalOperator: '<S270>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.vert_axis_mode >= 2U) {
    /* Switch: '<S185>/Sw5' incorporates:
     *  DataTypeConversion: '<S180>/Cast To Single1'
     *  RelationalOperator: '<S273>/Compare'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.vert_axis_mode == 3U) {
      /* Switch: '<S267>/Switch2' incorporates:
       *  DataTypeConversion: '<S180>/Cast To Single1'
       *  RelationalOperator: '<S267>/LowerRelop1'
       */
      if ((real32_T)claw_ofp_B.Dist <= claw_ofp_B.hdg_int) {
        /* Switch: '<S267>/Switch' incorporates:
         *  RelationalOperator: '<S267>/UpperRelop'
         *  UnaryMinus: '<S185>/Unary Minus2'
         */
        if ((real32_T)claw_ofp_B.Dist <
            -claw_ofp_DW.Memory1_PreviousInput.vel_z_cmd) {
          claw_ofp_B.hdg_int = -claw_ofp_DW.Memory1_PreviousInput.vel_z_cmd;
        } else {
          claw_ofp_B.hdg_int = (real32_T)claw_ofp_B.Dist;
        }

        /* End of Switch: '<S267>/Switch' */
      }

      /* End of Switch: '<S267>/Switch2' */
    } else {
      claw_ofp_B.hdg_int = (real32_T)claw_ofp_B.Dist;
    }
  } else {
    claw_ofp_B.hdg_int = claw_ofp_DW.Memory1_PreviousInput.vel_z_cmd;
  }

  /* Saturate: '<S185>/Limit5' */
  if (claw_ofp_B.hdg_int > 1.0F) {
    claw_ofp_B.hdg_int = 1.0F;
  } else if (claw_ofp_B.hdg_int < -3.0F) {
    claw_ofp_B.hdg_int = -3.0F;
  }

  /* End of Saturate: '<S185>/Limit5' */

  /* Outputs for Atomic SubSystem: '<S11>/yaw_rate_control' */
  /* Outputs for Atomic SubSystem: '<S11>/hdg_control' */
  /* Outputs for Atomic SubSystem: '<S13>/pitch_rate_control' */
  /* Outputs for Atomic SubSystem: '<S13>/pitch_control' */
  /* Outputs for Atomic SubSystem: '<S12>/roll_rate_control' */
  /* Outputs for Atomic SubSystem: '<S12>/roll_control' */
  /* Logic: '<S182>/NOT2' incorporates:
   *  Logic: '<S182>/NOT1'
   *  Logic: '<S19>/NOT'
   *  Logic: '<S20>/NOT'
   *  Logic: '<S38>/NOT'
   *  Logic: '<S39>/NOT'
   *  Logic: '<S52>/NOT'
   *  Logic: '<S53>/NOT'
   *  Outport: '<Root>/OutState'
   */
  rtb_AND_c = !claw_out_state.landed;

  /* End of Outputs for SubSystem: '<S12>/roll_control' */
  /* End of Outputs for SubSystem: '<S12>/roll_rate_control' */
  /* End of Outputs for SubSystem: '<S13>/pitch_control' */
  /* End of Outputs for SubSystem: '<S13>/pitch_rate_control' */
  /* End of Outputs for SubSystem: '<S11>/hdg_control' */
  /* End of Outputs for SubSystem: '<S11>/yaw_rate_control' */

  /* Logic: '<S182>/AND3' incorporates:
   *  Logic: '<S182>/NOT2'
   *  RelationalOperator: '<S239>/Compare'
   */
  rtb_Compare_o1 = (rtb_AND_c &&
                    (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode >= 4U));

  /* Outputs for Atomic SubSystem: '<S182>/PID_with_FF1' */
  /* Gain: '<S235>/K' incorporates:
   *  DataTypeConversion: '<S182>/Cast To Double4'
   *  DataTypeConversion: '<S182>/Cast To Double5'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S235>/Sum'
   */
  claw_ofp_B.Vfwd = (real_T)claw_ofp_B.Saturation_n - claw_out_state.fwd_spd;

  /* UnitDelay: '<S242>/Unit Delay2' */
  claw_ofp_B.Product_g = claw_ofp_DW.UnitDelay2_DSTATE_i;

  /* Product: '<S242>/Product6' incorporates:
   *  Constant: '<S242>/Constant2'
   *  Product: '<S242>/Product1'
   *  Sum: '<S242>/Sum'
   *  UnitDelay: '<S242>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_i = claw_ofp_B.Vfwd * 0.0728205070873382;

  /* Sum: '<S242>/Sum1' incorporates:
   *  Constant: '<S242>/Constant2'
   *  Product: '<S242>/Product4'
   *  Product: '<S242>/Product5'
   *  Sum: '<S242>/Sum2'
   *  Sum: '<S242>/Sum3'
   *  UnitDelay: '<S242>/Unit Delay2'
   *  UnitDelay: '<S242>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE_a = (claw_ofp_B.Product_g +
    claw_ofp_DW.UnitDelay2_DSTATE_i) + 0.85435898582532355 *
    claw_ofp_DW.UnitDelay3_DSTATE_a;

  /* DiscreteIntegrator: '<S235>/Discrete_Time_Integrator' */
  if (rtb_Compare_o1 && (claw_ofp_DW.Discrete_Time_Integrator_Prev_h <= 0)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = claw_ofp_ConstB.Zero1_a;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g >= 2.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = 2.0;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g <= -2.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = -2.0;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = -2.0;
  }

  claw_ofp_B.TrigonometricFunction1_d =
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g;

  /* Sum: '<S235>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S235>/Discrete_Time_Integrator'
   *  Gain: '<S235>/P'
   *  Product: '<S235>/Product'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Dist = claw_ofp_B.Vfwd * claw_ofp_B.TrigonometricFunction1 +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g;

  /* Update for UnitDelay: '<S243>/UD' incorporates:
   *  SampleTimeMath: '<S243>/TSamp'
   *  UnitDelay: '<S242>/Unit Delay3'
   *
   * About '<S243>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S243>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE_d = claw_ofp_DW.UnitDelay3_DSTATE_a * 200.0;

  /* Switch: '<S235>/Switch' */
  if (rtb_Compare_o1) {
    /* Switch: '<S235>/Switch1' incorporates:
     *  Constant: '<S240>/Constant'
     *  Constant: '<S241>/Constant'
     *  Gain: '<S235>/I'
     *  Logic: '<S235>/AND'
     *  Product: '<S235>/Product2'
     *  RelationalOperator: '<S240>/Compare'
     *  RelationalOperator: '<S241>/Compare'
     *  Switch: '<S8>/Switch'
     */
    if ((claw_ofp_B.Dist < 2.5) && (claw_ofp_B.Dist > -2.5)) {
      claw_ofp_B.Product_g = 0.5 * claw_ofp_B.Vfwd * claw_ofp_B.K;
    } else {
      claw_ofp_B.Product_g = 0.0;
    }

    /* End of Switch: '<S235>/Switch1' */
  } else {
    claw_ofp_B.Product_g = 0.0;
  }

  /* End of Switch: '<S235>/Switch' */

  /* Update for DiscreteIntegrator: '<S235>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_g += 0.005 * claw_ofp_B.Product_g;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = -2.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_h = (int16_T)rtb_Compare_o1;

  /* End of Update for DiscreteIntegrator: '<S235>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S182>/PID_with_FF1' */

  /* Logic: '<S182>/AND2' incorporates:
   *  RelationalOperator: '<S238>/Compare'
   */
  rtb_Compare_o1 = (rtb_AND_c &&
                    (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode >= 4U));

  /* Outputs for Atomic SubSystem: '<S182>/PID_with_FF2' */
  /* Gain: '<S236>/K' incorporates:
   *  DataTypeConversion: '<S182>/Cast To Double2'
   *  DataTypeConversion: '<S182>/Cast To Double3'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S236>/Sum'
   */
  claw_ofp_B.TrigonometricFunction2_mh = (real_T)rtb_Abs1_e -
    claw_out_state.lat_spd;

  /* UnitDelay: '<S246>/Unit Delay2' */
  claw_ofp_B.Product_g = claw_ofp_DW.UnitDelay2_DSTATE_b5;

  /* Product: '<S246>/Product6' incorporates:
   *  Constant: '<S246>/Constant2'
   *  Product: '<S246>/Product1'
   *  Sum: '<S246>/Sum'
   *  UnitDelay: '<S246>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_b5 = claw_ofp_B.TrigonometricFunction2_mh *
    0.0728205070873382;

  /* Sum: '<S246>/Sum1' incorporates:
   *  Constant: '<S246>/Constant2'
   *  Product: '<S246>/Product4'
   *  Product: '<S246>/Product5'
   *  Sum: '<S246>/Sum2'
   *  Sum: '<S246>/Sum3'
   *  UnitDelay: '<S246>/Unit Delay2'
   *  UnitDelay: '<S246>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE_p = (claw_ofp_B.Product_g +
    claw_ofp_DW.UnitDelay2_DSTATE_b5) + 0.85435898582532355 *
    claw_ofp_DW.UnitDelay3_DSTATE_p;

  /* DiscreteIntegrator: '<S236>/Discrete_Time_Integrator' */
  if (rtb_Compare_o1 && (claw_ofp_DW.Discrete_Time_Integrator_PrevRe <= 0)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = claw_ofp_ConstB.Zero1_o;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m >= 2.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = 2.0;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m <= -2.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = -2.0;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = -2.0;
  }

  claw_ofp_B.Vfwd = claw_ofp_DW.Discrete_Time_Integrator_DSTA_m;

  /* Sum: '<S236>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S236>/Discrete_Time_Integrator'
   *  Gain: '<S236>/P'
   *  Product: '<S236>/Product'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.TrigonometricFunction1 = claw_ofp_B.TrigonometricFunction2_mh *
    claw_ofp_B.TrigonometricFunction1 +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m;

  /* Update for UnitDelay: '<S247>/UD' incorporates:
   *  SampleTimeMath: '<S247>/TSamp'
   *  UnitDelay: '<S246>/Unit Delay3'
   *
   * About '<S247>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S247>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE_k = claw_ofp_DW.UnitDelay3_DSTATE_p * 200.0;

  /* Switch: '<S236>/Switch' */
  if (rtb_Compare_o1) {
    /* Switch: '<S236>/Switch1' incorporates:
     *  Constant: '<S244>/Constant'
     *  Constant: '<S245>/Constant'
     *  Gain: '<S236>/I'
     *  Logic: '<S236>/AND'
     *  Product: '<S236>/Product1'
     *  RelationalOperator: '<S244>/Compare'
     *  RelationalOperator: '<S245>/Compare'
     *  Switch: '<S8>/Switch'
     */
    if ((claw_ofp_B.TrigonometricFunction1 < 2.5) &&
        (claw_ofp_B.TrigonometricFunction1 > -2.5)) {
      claw_ofp_B.K *= 0.5 * claw_ofp_B.TrigonometricFunction2_mh;
    } else {
      claw_ofp_B.K = 0.0;
    }

    /* End of Switch: '<S236>/Switch1' */
  } else {
    claw_ofp_B.K = 0.0;
  }

  /* End of Switch: '<S236>/Switch' */

  /* Update for DiscreteIntegrator: '<S236>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_m += 0.005 * claw_ofp_B.K;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = -2.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_PrevRe = (int16_T)rtb_Compare_o1;

  /* End of Update for DiscreteIntegrator: '<S236>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S182>/PID_with_FF2' */

  /* Outputs for Atomic SubSystem: '<S182>/PID_with_FF3' */
  /* Gain: '<S237>/K' incorporates:
   *  DataTypeConversion: '<S182>/Cast To Double'
   *  DataTypeConversion: '<S182>/Cast To Double1'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S237>/Sum'
   */
  claw_ofp_B.K = (real_T)claw_ofp_B.hdg_int - claw_out_state.sensor_raw.vel_down;

  /* UnitDelay: '<S250>/Unit Delay2' */
  claw_ofp_B.Product_g = claw_ofp_DW.UnitDelay2_DSTATE_h3;

  /* Product: '<S250>/Product6' incorporates:
   *  Constant: '<S250>/Constant2'
   *  Product: '<S250>/Product1'
   *  Sum: '<S250>/Sum'
   *  UnitDelay: '<S250>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_h3 = claw_ofp_B.K * 0.0728205070873382;

  /* Sum: '<S250>/Sum1' incorporates:
   *  Constant: '<S250>/Constant2'
   *  Product: '<S250>/Product4'
   *  Product: '<S250>/Product5'
   *  Sum: '<S250>/Sum2'
   *  Sum: '<S250>/Sum3'
   *  UnitDelay: '<S250>/Unit Delay2'
   *  UnitDelay: '<S250>/Unit Delay3'
   */
  claw_ofp_DW.UnitDelay3_DSTATE_b = (claw_ofp_B.Product_g +
    claw_ofp_DW.UnitDelay2_DSTATE_h3) + 0.85435898582532355 *
    claw_ofp_DW.UnitDelay3_DSTATE_b;

  /* DiscreteIntegrator: '<S237>/Discrete_Time_Integrator' */
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTATE >= 1.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTATE = 1.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTATE <= -1.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTATE = -1.0;
  }

  /* Update for UnitDelay: '<S251>/UD' incorporates:
   *  SampleTimeMath: '<S251>/TSamp'
   *  UnitDelay: '<S250>/Unit Delay3'
   *
   * About '<S251>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S251>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE = claw_ofp_DW.UnitDelay3_DSTATE_b * 200.0;

  /* End of Outputs for SubSystem: '<S182>/PID_with_FF3' */

  /* Switch: '<S184>/Sw' incorporates:
   *  RelationalOperator: '<S263>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode < 4U) {
    claw_ofp_B.Dist = 0.0;
  }

  /* End of Switch: '<S184>/Sw' */

  /* Saturate: '<S184>/Limit3' */
  if (claw_ofp_B.Dist > 2.5) {
    claw_ofp_B.Dist = 2.5;
  } else if (claw_ofp_B.Dist < -2.5) {
    claw_ofp_B.Dist = -2.5;
  }

  /* End of Saturate: '<S184>/Limit3' */

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S181>/Data Type Conversion30'
   *  Outport: '<Root>/OutPosControl'
   */
  claw_out_pos_control.acc_ref_x = (real32_T)claw_ofp_B.Dist;

  /* Switch: '<S184>/Sw2' incorporates:
   *  RelationalOperator: '<S264>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode < 4U) {
    claw_ofp_B.TrigonometricFunction1 = 0.0;
  }

  /* End of Switch: '<S184>/Sw2' */

  /* Saturate: '<S184>/Limit2' */
  if (claw_ofp_B.TrigonometricFunction1 > 2.5) {
    claw_ofp_B.TrigonometricFunction1 = 2.5;
  } else if (claw_ofp_B.TrigonometricFunction1 < -2.5) {
    claw_ofp_B.TrigonometricFunction1 = -2.5;
  }

  /* End of Saturate: '<S184>/Limit2' */

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S181>/Data Type Conversion28'
   *  Outport: '<Root>/OutPosControl'
   */
  claw_out_pos_control.acc_ref_y = (real32_T)claw_ofp_B.TrigonometricFunction1;

  /* Switch: '<S184>/Sw1' incorporates:
   *  DiscreteIntegrator: '<S237>/Discrete_Time_Integrator'
   *  Gain: '<S237>/P'
   *  Product: '<S237>/Product'
   *  RelationalOperator: '<S262>/Compare'
   *  Sum: '<S237>/Sum4'
   *  Switch: '<S8>/Switch'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.vert_axis_mode >= 1U) {
    /* Outputs for Atomic SubSystem: '<S182>/PID_with_FF3' */
    claw_ofp_B.Switch_o = 3.0 * claw_ofp_B.K * claw_ofp_B.Switch_o +
      claw_ofp_DW.Discrete_Time_Integrator_DSTATE;

    /* End of Outputs for SubSystem: '<S182>/PID_with_FF3' */
  } else {
    claw_ofp_B.Switch_o = 0.0;
  }

  /* End of Switch: '<S184>/Sw1' */

  /* Saturate: '<S184>/Limit1' */
  if (claw_ofp_B.Switch_o > 1.0) {
    claw_ofp_B.Switch_o = 1.0;
  } else if (claw_ofp_B.Switch_o < -1.0) {
    claw_ofp_B.Switch_o = -1.0;
  }

  /* End of Saturate: '<S184>/Limit1' */

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S181>/Data Type Conversion29'
   *  Outport: '<Root>/OutPosControl'
   */
  claw_out_pos_control.acc_ref_z = (real32_T)claw_ofp_B.Switch_o;

  /* Sum: '<S252>/Sum' incorporates:
   *  Constant: '<S183>/Constant'
   *  UnaryMinus: '<S252>/Unary Minus1'
   */
  claw_ofp_B.Switch_o = -claw_ofp_B.Switch_o + 9.81;

  /* Trigonometry: '<S252>/Trigonometric Function1' */
  claw_ofp_B.TrigonometricFunction1 = atan2(claw_ofp_B.TrigonometricFunction1,
    claw_ofp_B.Switch_o);

  /* Switch: '<S258>/Switch2' incorporates:
   *  RelationalOperator: '<S258>/LowerRelop1'
   *  RelationalOperator: '<S258>/UpperRelop'
   *  Switch: '<S258>/Switch'
   */
  if (claw_ofp_B.TrigonometricFunction1 > claw_ofp_ConstB.Gain1[0]) {
    claw_ofp_B.TrigonometricFunction1 = claw_ofp_ConstB.Gain1[0];
  } else if (claw_ofp_B.TrigonometricFunction1 < claw_ofp_ConstB.UnaryMinus2) {
    /* Switch: '<S258>/Switch' */
    claw_ofp_B.TrigonometricFunction1 = claw_ofp_ConstB.UnaryMinus2;
  }

  /* End of Switch: '<S258>/Switch2' */

  /* Trigonometry: '<S252>/Trigonometric Function' incorporates:
   *  UnaryMinus: '<S252>/Unary Minus'
   */
  claw_ofp_B.K = atan2(-claw_ofp_B.Dist, claw_ofp_B.Switch_o);

  /* Switch: '<S257>/Switch2' incorporates:
   *  RelationalOperator: '<S257>/LowerRelop1'
   *  RelationalOperator: '<S257>/UpperRelop'
   *  Switch: '<S257>/Switch'
   */
  if (claw_ofp_B.K > claw_ofp_ConstB.Gain1[1]) {
    claw_ofp_B.K = claw_ofp_ConstB.Gain1[1];
  } else if (claw_ofp_B.K < claw_ofp_ConstB.UnaryMinus3) {
    /* Switch: '<S257>/Switch' */
    claw_ofp_B.K = claw_ofp_ConstB.UnaryMinus3;
  }

  /* End of Switch: '<S257>/Switch2' */

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S181>/Data Type Conversion21'
   *  Gain: '<S254>/Gain'
   *  Outport: '<Root>/OutPosControl'
   */
  claw_out_pos_control.pitch_cmd = (real32_T)(57.295779513082323 * claw_ofp_B.K);

  /* Gain: '<S217>/Gain' incorporates:
   *  Gain: '<S194>/Gain'
   *  MultiPortSwitch: '<S180>/Multiport Switch1'
   *  Sum: '<S188>/Add'
   *  Trigonometry: '<S188>/Trigonometric Function'
   */
  claw_ofp_B.rtb_Sum1_idx_0 = 57.295779513082323 * atan2
    (claw_ofp_B.rtb_Sum1_idx_1, claw_ofp_B.rtb_Sum1_idx_0);

  /* MultiPortSwitch: '<S180>/Multiport Switch1' incorporates:
   *  Gain: '<S217>/Gain'
   */
  switch ((int16_T)claw_ofp_DW.Memory1_PreviousInput.guide_type) {
   case 0:
    claw_ofp_B.Dist = claw_ofp_B.rtb_Sum1_idx_0;
    break;

   case 1:
    claw_ofp_B.Dist = claw_ofp_B.rtb_Sum1_idx_0;
    break;

   case 2:
    claw_ofp_B.Dist = claw_ofp_B.rtb_Sum1_idx_0;
    break;

   case 3:
    /* Switch: '<S190>/Switch3' incorporates:
     *  Switch: '<S190>/Switch2'
     */
    if (rtb_cond_gnd_contact) {
      claw_ofp_B.Dist = claw_ofp_B.Track_mode;
    } else {
      claw_ofp_B.Dist = claw_ofp_B.Gain;
    }

    /* End of Switch: '<S190>/Switch3' */
    break;

   default:
    claw_ofp_B.Dist = claw_ofp_B.PY2;
    break;
  }

  /* Product: '<S252>/Product1' incorporates:
   *  Trigonometry: '<S252>/Trigonometric Function2'
   *  Trigonometry: '<S252>/Trigonometric Function3'
   */
  claw_ofp_B.rtb_Sum1_idx_1 = cos(claw_ofp_B.K) * cos
    (claw_ofp_B.TrigonometricFunction1);

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  DataTypeConversion: '<S181>/Data Type Conversion1'
   *  DataTypeConversion: '<S181>/Data Type Conversion12'
   *  DataTypeConversion: '<S181>/Data Type Conversion20'
   *  DataTypeConversion: '<S181>/Data Type Conversion22'
   *  DataTypeConversion: '<S181>/Data Type Conversion3'
   *  DiscreteIntegrator: '<S237>/Discrete_Time_Integrator'
   *  Gain: '<S253>/Gain'
   *  Outport: '<Root>/OutPosControl'
   *  UnaryMinus: '<S180>/Unary Minus'
   */
  claw_out_pos_control.pos_ref_x_cur =
    claw_ofp_DW.Memory1_PreviousInput.wp_x_local[1];
  claw_out_pos_control.pos_ref_y_cur =
    claw_ofp_DW.Memory1_PreviousInput.wp_y_local[1];
  claw_out_pos_control.pos_ref_z_cur =
    -claw_ofp_DW.Memory1_PreviousInput.alt_cmd;
  claw_out_pos_control.pos_err_x = claw_ofp_B.minV;
  claw_out_pos_control.pos_err_y = claw_ofp_B.maxV;
  claw_out_pos_control.pos_err_z = claw_ofp_B.Sum_k;
  claw_out_pos_control.vel_ref_x = claw_ofp_B.Saturation_n;
  claw_out_pos_control.vel_ref_y = rtb_Abs1_e;
  claw_out_pos_control.vel_ref_z = claw_ofp_B.hdg_int;
  claw_out_pos_control.vx_int = (real32_T)claw_ofp_B.TrigonometricFunction1_d;
  claw_out_pos_control.vy_int = (real32_T)claw_ofp_B.Vfwd;

  /* Outputs for Atomic SubSystem: '<S182>/PID_with_FF3' */
  claw_out_pos_control.vz_int = (real32_T)
    claw_ofp_DW.Discrete_Time_Integrator_DSTATE;

  /* End of Outputs for SubSystem: '<S182>/PID_with_FF3' */
  claw_out_pos_control.roll_cmd = (real32_T)(57.295779513082323 *
    claw_ofp_B.TrigonometricFunction1);
  claw_out_pos_control.yaw_cmd = (real32_T)claw_ofp_B.Dist;
  claw_out_pos_control.yaw_rate = 0.0F;

  /* Switch: '<S256>/Switch' incorporates:
   *  Constant: '<S256>/Constant1'
   *  Constant: '<S259>/Constant'
   *  RelationalOperator: '<S259>/Compare'
   *  Switch: '<S256>/Switch2'
   */
  if (claw_ofp_B.rtb_Sum1_idx_1 > 0.0) {
    /* Switch: '<S256>/Switch1' incorporates:
     *  Constant: '<S256>/Constant'
     *  Constant: '<S260>/Constant'
     *  RelationalOperator: '<S260>/Compare'
     */
    if (claw_ofp_B.rtb_Sum1_idx_1 <= 0.01) {
      claw_ofp_B.rtb_Sum1_idx_1 = 0.01;
    }

    /* End of Switch: '<S256>/Switch1' */
  } else if (claw_ofp_B.rtb_Sum1_idx_1 >= -0.01) {
    claw_ofp_B.rtb_Sum1_idx_1 = -0.01;
  }

  /* End of Switch: '<S256>/Switch' */

  /* BusCreator: '<S181>/Bus Creator' incorporates:
   *  Constant: '<S233>/Constant'
   *  DataTypeConversion: '<S181>/Data Type Conversion10'
   *  DataTypeConversion: '<S181>/Data Type Conversion14'
   *  DataTypeConversion: '<S181>/Data Type Conversion17'
   *  DataTypeConversion: '<S181>/Data Type Conversion18'
   *  DataTypeConversion: '<S181>/Data Type Conversion5'
   *  DataTypeConversion: '<S183>/Cast To Double1'
   *  Gain: '<S217>/Gain'
   *  Outport: '<Root>/OutPosControl'
   *  Product: '<S252>/Divide'
   *  RelationalOperator: '<S233>/Compare'
   */
  claw_out_pos_control.az_body = (real32_T)(claw_ofp_B.Switch_o /
    claw_ofp_B.rtb_Sum1_idx_1);
  claw_out_pos_control.pass_flag = (claw_ofp_B.Limit3 <= 0.0);
  claw_out_pos_control.dist_to_go = (real32_T)claw_ofp_B.Limit3;
  claw_out_pos_control.dist_to_wp = (real32_T)claw_ofp_B.dist_to_wp;
  claw_out_pos_control.dist_from = 0.0F;
  claw_out_pos_control.time_from = 0U;
  claw_out_pos_control.wp_leg_hdg = (real32_T)claw_ofp_B.Gain;
  claw_out_pos_control.xtrk_hdg_error = 0.0F;
  claw_out_pos_control.xtrk_dist = (real32_T)claw_ofp_B.Limit4;
  claw_out_pos_control.hdg_to_wp = (real32_T)claw_ofp_B.rtb_Sum1_idx_0;
  claw_out_pos_control.xtrk_hdg_int = 0.0F;

  /* Switch: '<S36>/Sw' incorporates:
   *  Outport: '<Root>/OutPosControl'
   *  RelationalOperator: '<S41>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode >= 4U) {
    claw_ofp_B.Saturation_n = claw_out_pos_control.roll_cmd;
  } else {
    claw_ofp_B.Saturation_n = claw_ofp_DW.Memory1_PreviousInput.roll_cmd;
  }

  /* End of Switch: '<S36>/Sw' */

  /* Saturate: '<S36>/Limit' */
  if (claw_ofp_B.Saturation_n > 30.0F) {
    claw_ofp_B.Saturation_n = 30.0F;
  } else if (claw_ofp_B.Saturation_n < -30.0F) {
    claw_ofp_B.Saturation_n = -30.0F;
  }

  /* End of Saturate: '<S36>/Limit' */

  /* Outputs for Atomic SubSystem: '<S12>/roll_control' */
  /* Logic: '<S38>/Logical Operator' incorporates:
   *  RelationalOperator: '<S43>/Compare'
   */
  rtb_cond_gnd_contact = (rtb_AND_c &&
    (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode >= 2U));

  /* DiscreteIntegrator: '<S38>/Discrete_Time_Integrator' */
  if ((!rtb_cond_gnd_contact) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_f ==
       1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = claw_ofp_ConstB.Zero1_i;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o >= 0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = 0.0;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o <= -0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = -0.0;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = -0.0;
  }

  claw_ofp_B.rtb_Sum1_idx_0 = claw_ofp_DW.Discrete_Time_Integrator_DSTA_o;

  /* Sum: '<S38>/Sum' incorporates:
   *  Outport: '<Root>/OutState'
   */
  rtb_Abs1_e = claw_ofp_B.Saturation_n - claw_out_state.sensor_raw.roll;

  /* Sum: '<S38>/Sum4' incorporates:
   *  DataTypeConversion: '<S38>/Data Type Conversion1'
   *  DiscreteIntegrator: '<S38>/Discrete_Time_Integrator'
   *  Gain: '<S38>/PHI_P'
   *  Product: '<S38>/Product'
   *  Sum: '<S38>/Sum'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Limit4 = rtb_Abs1_e * 7.0 * claw_ofp_B.RLL_RATE_K +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o;

  /* Update for DiscreteIntegrator: '<S38>/Discrete_Time_Integrator' */
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = 0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_f = (int16_T)rtb_cond_gnd_contact;

  /* End of Update for DiscreteIntegrator: '<S38>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S12>/roll_control' */

  /* Switch: '<S37>/Sw' incorporates:
   *  RelationalOperator: '<S42>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode < 2U) {
    claw_ofp_B.Limit4 = claw_ofp_DW.Memory1_PreviousInput.roll_rate_cmd;
  }

  /* End of Switch: '<S37>/Sw' */

  /* Saturate: '<S37>/Limit' */
  if (claw_ofp_B.Limit4 > 130.0) {
    claw_ofp_B.Limit4 = 130.0;
  } else if (claw_ofp_B.Limit4 < -130.0) {
    claw_ofp_B.Limit4 = -130.0;
  }

  /* End of Saturate: '<S37>/Limit' */

  /* Outputs for Atomic SubSystem: '<S12>/roll_rate_control' */
  /* Gain: '<S39>/RLL_RATE_K' incorporates:
   *  DataTypeConversion: '<S39>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S39>/Sum'
   */
  claw_ofp_B.RLL_RATE_K = (claw_ofp_B.Limit4 - claw_out_state.roll_rate) *
    0.017453292519943295;

  /* Logic: '<S39>/Logical Operator' incorporates:
   *  RelationalOperator: '<S46>/Compare'
   */
  rtb_cond_gnd_contact = (rtb_AND_c &&
    (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode != 0U));

  /* DiscreteIntegrator: '<S39>/Discrete_Time_Integrator' */
  if ((!rtb_cond_gnd_contact) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_j ==
       1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = claw_ofp_ConstB.Zero1_n;
    if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd >= 0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DST_gd = 0.4;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd <= -0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DST_gd = -0.4;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = -0.4;
  }

  claw_ofp_B.Limit3 = claw_ofp_DW.Discrete_Time_Integrator_DST_gd;

  /* Sum: '<S47>/Sum' incorporates:
   *  DataTypeConversion: '<S39>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  UnitDelay: '<S47>/Unit Delay1'
   */
  claw_ofp_B.Product_g = claw_out_state.roll_rate +
    claw_ofp_DW.UnitDelay1_DSTATE_oj;

  /* Sum: '<S39>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S39>/Discrete_Time_Integrator'
   *  Gain: '<S39>/RLL_RATE_P'
   *  Product: '<S39>/Product'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Sum_k = 0.07 * claw_ofp_B.RLL_RATE_K * claw_ofp_B.integ_out_c +
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd;

  /* Update for UnitDelay: '<S47>/Unit Delay1' incorporates:
   *  DataTypeConversion: '<S39>/Data Type Conversion'
   *  Gain: '<S47>/Gain1'
   *  Outport: '<Root>/OutState'
   *  Product: '<S47>/Product5'
   *  Sum: '<S47>/Sum1'
   *  Sum: '<S47>/Sum2'
   *  UnitDelay: '<S47>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE_oj = (claw_ofp_B.Product_g *
    claw_ofp_ConstB.Product4_n + claw_out_state.roll_rate) * 2.0 +
    claw_ofp_DW.UnitDelay2_DSTATE_n;

  /* Sum: '<S47>/Sum3' incorporates:
   *  DataTypeConversion: '<S39>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  Product: '<S47>/Product2'
   *  UnitDelay: '<S47>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_n = claw_out_state.roll_rate -
    claw_ofp_B.Product_g * claw_ofp_ConstB.Product5_iz;

  /* Switch: '<S39>/Int_ONSW' incorporates:
   *  Constant: '<S39>/Zero'
   *  Gain: '<S39>/RLL_RATE_I'
   *  Product: '<S39>/Product1'
   *  Switch: '<S8>/Switch'
   */
  if (rtb_cond_gnd_contact) {
    claw_ofp_B.integ_out_m *= 0.03 * claw_ofp_B.RLL_RATE_K;
  } else {
    claw_ofp_B.integ_out_m = 0.0;
  }

  /* End of Switch: '<S39>/Int_ONSW' */

  /* Update for DiscreteIntegrator: '<S39>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DST_gd += 0.005 * claw_ofp_B.integ_out_m;
  if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_j = (int16_T)rtb_cond_gnd_contact;

  /* End of Update for DiscreteIntegrator: '<S39>/Discrete_Time_Integrator' */

  /* Update for UnitDelay: '<S45>/UD' incorporates:
   *  Product: '<S47>/Product1'
   *  SampleTimeMath: '<S45>/TSamp'
   *
   * About '<S45>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S45>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE_du = claw_ofp_B.Product_g * claw_ofp_ConstB.Product3_no *
    200.0;

  /* End of Outputs for SubSystem: '<S12>/roll_rate_control' */

  /* Switch: '<S35>/Sw' incorporates:
   *  RelationalOperator: '<S40>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S12>/roll_rate_control' */
    /* DataTypeConversion: '<S12>/Data Type Conversion' incorporates:
     *  Gain: '<S39>/RLL_RATE_K_FF'
     *  Sum: '<S39>/Sum1'
     */
    claw_ofp_B.hdg_int = (real32_T)(0.01 * claw_ofp_B.Limit4 + claw_ofp_B.Sum_k);

    /* End of Outputs for SubSystem: '<S12>/roll_rate_control' */
  } else {
    /* DataTypeConversion: '<S12>/Data Type Conversion' */
    claw_ofp_B.hdg_int = claw_ofp_DW.Memory1_PreviousInput.ail_cmd;
  }

  if (claw_ofp_DW.Memory1_PreviousInput.roll_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S12>/roll_rate_control' */
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' incorporates:
     *  DataTypeConversion: '<S12>/Data Type Conversion'
     *  Gain: '<S39>/RLL_RATE_K_FF'
     *  Sum: '<S39>/Sum1'
     *  Sum: '<S39>/Sum4'
     */
    rtb_MatrixConcatenate_idx_0 = (real32_T)(0.01 * claw_ofp_B.Limit4 +
      claw_ofp_B.Sum_k);

    /* End of Outputs for SubSystem: '<S12>/roll_rate_control' */
  } else {
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' */
    rtb_MatrixConcatenate_idx_0 = claw_ofp_DW.Memory1_PreviousInput.ail_cmd;
  }

  /* End of Switch: '<S35>/Sw' */

  /* Switch: '<S49>/Sw' incorporates:
   *  Outport: '<Root>/OutPosControl'
   *  RelationalOperator: '<S54>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode >= 4U) {
    rtb_Limit_n = claw_out_pos_control.pitch_cmd;
  } else {
    rtb_Limit_n = claw_ofp_DW.Memory1_PreviousInput.pitch_cmd;
  }

  /* End of Switch: '<S49>/Sw' */

  /* Saturate: '<S49>/Limit' */
  if (rtb_Limit_n > 30.0F) {
    rtb_Limit_n = 30.0F;
  } else if (rtb_Limit_n < -30.0F) {
    rtb_Limit_n = -30.0F;
  }

  /* End of Saturate: '<S49>/Limit' */

  /* Outputs for Atomic SubSystem: '<S13>/pitch_control' */
  /* Logic: '<S52>/Logical Operator' incorporates:
   *  RelationalOperator: '<S57>/Compare'
   */
  rtb_cond_gnd_contact = (rtb_AND_c &&
    (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode >= 2U));

  /* DiscreteIntegrator: '<S52>/Discrete_Time_Integrator' */
  if ((!rtb_cond_gnd_contact) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_k ==
       1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = claw_ofp_ConstB.Zero1_f;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h >= 0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = 0.0;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h <= -0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = -0.0;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = -0.0;
  }

  claw_ofp_B.integ_out_m = claw_ofp_DW.Discrete_Time_Integrator_DSTA_h;

  /* Sum: '<S52>/Sum' incorporates:
   *  Outport: '<Root>/OutState'
   */
  rtb_Sum4_i2_tmp = rtb_Limit_n - claw_out_state.sensor_raw.pitch;

  /* Sum: '<S52>/Sum4' incorporates:
   *  DataTypeConversion: '<S52>/Data Type Conversion1'
   *  DiscreteIntegrator: '<S52>/Discrete_Time_Integrator'
   *  Gain: '<S52>/Theta_P'
   *  Product: '<S52>/Product'
   *  Sum: '<S52>/Sum'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.Sum4_i2 = rtb_Sum4_i2_tmp * 6.0 * claw_ofp_B.Sum4_i2 +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h;

  /* Update for DiscreteIntegrator: '<S52>/Discrete_Time_Integrator' */
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = 0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_k = (int16_T)rtb_cond_gnd_contact;

  /* End of Update for DiscreteIntegrator: '<S52>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S13>/pitch_control' */

  /* Switch: '<S51>/Sw' incorporates:
   *  RelationalOperator: '<S56>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode < 2U) {
    claw_ofp_B.Sum4_i2 = claw_ofp_DW.Memory1_PreviousInput.pitch_rate_cmd;
  }

  /* End of Switch: '<S51>/Sw' */

  /* Saturate: '<S51>/Limit' */
  if (claw_ofp_B.Sum4_i2 > 130.0) {
    claw_ofp_B.Sum4_i2 = 130.0;
  } else if (claw_ofp_B.Sum4_i2 < -130.0) {
    claw_ofp_B.Sum4_i2 = -130.0;
  }

  /* End of Saturate: '<S51>/Limit' */

  /* Outputs for Atomic SubSystem: '<S13>/pitch_rate_control' */
  /* Gain: '<S53>/PCH_RATE_K' incorporates:
   *  DataTypeConversion: '<S53>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  Sum: '<S53>/Sum'
   */
  claw_ofp_B.RLL_RATE_K = (claw_ofp_B.Sum4_i2 - claw_out_state.pitch_rate) *
    0.017453292519943295;

  /* Logic: '<S53>/Logical Operator' incorporates:
   *  RelationalOperator: '<S60>/Compare'
   */
  rtb_cond_gnd_contact = (rtb_AND_c &&
    (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode != 0U));

  /* DiscreteIntegrator: '<S53>/Discrete_Time_Integrator' */
  if ((!rtb_cond_gnd_contact) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_e ==
       1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = claw_ofp_ConstB.Zero1_c;
    if (claw_ofp_DW.Discrete_Time_Integrator_DST_go >= 0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DST_go = 0.4;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_go <= -0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DST_go = -0.4;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DST_go >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_go <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = -0.4;
  }

  claw_ofp_B.integ_out_c = claw_ofp_DW.Discrete_Time_Integrator_DST_go;

  /* Sum: '<S61>/Sum' incorporates:
   *  DataTypeConversion: '<S53>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  UnitDelay: '<S61>/Unit Delay1'
   */
  claw_ofp_B.Sum_k = claw_out_state.pitch_rate + claw_ofp_DW.UnitDelay1_DSTATE_o;

  /* Sum: '<S53>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S53>/Discrete_Time_Integrator'
   *  Gain: '<S53>/PCH_RATE_P'
   *  Product: '<S53>/Product2'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.integ_out_l = 0.1 * claw_ofp_B.RLL_RATE_K * claw_ofp_B.integ_out_l
    + claw_ofp_DW.Discrete_Time_Integrator_DST_go;

  /* Sum: '<S61>/Sum1' incorporates:
   *  DataTypeConversion: '<S53>/Data Type Conversion'
   *  Gain: '<S61>/Gain1'
   *  Outport: '<Root>/OutState'
   *  Product: '<S61>/Product5'
   *  Sum: '<S61>/Sum2'
   *  UnitDelay: '<S61>/Unit Delay1'
   *  UnitDelay: '<S61>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE_o = (claw_ofp_B.Sum_k *
    claw_ofp_ConstB.Product4_e + claw_out_state.pitch_rate) * 2.0 +
    claw_ofp_DW.UnitDelay2_DSTATE_m;

  /* Sum: '<S61>/Sum3' incorporates:
   *  DataTypeConversion: '<S53>/Data Type Conversion'
   *  Outport: '<Root>/OutState'
   *  Product: '<S61>/Product2'
   *  UnitDelay: '<S61>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_m = claw_out_state.pitch_rate - claw_ofp_B.Sum_k
    * claw_ofp_ConstB.Product5_c;

  /* Switch: '<S53>/Int_ONSW' incorporates:
   *  Constant: '<S53>/Zero'
   *  Gain: '<S53>/PCH_RATE_I'
   *  Product: '<S53>/Product1'
   *  Switch: '<S8>/Switch'
   */
  if (rtb_cond_gnd_contact) {
    claw_ofp_B.Switch1_lx *= 0.04 * claw_ofp_B.RLL_RATE_K;
  } else {
    claw_ofp_B.Switch1_lx = 0.0;
  }

  /* End of Switch: '<S53>/Int_ONSW' */

  /* Update for DiscreteIntegrator: '<S53>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DST_go += 0.005 * claw_ofp_B.Switch1_lx;
  if (claw_ofp_DW.Discrete_Time_Integrator_DST_go >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_go <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_e = (int16_T)rtb_cond_gnd_contact;

  /* End of Update for DiscreteIntegrator: '<S53>/Discrete_Time_Integrator' */

  /* Update for UnitDelay: '<S59>/UD' incorporates:
   *  Product: '<S61>/Product1'
   *  SampleTimeMath: '<S59>/TSamp'
   *
   * About '<S59>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S59>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE_p = claw_ofp_B.Sum_k * claw_ofp_ConstB.Product3_o *
    200.0;

  /* End of Outputs for SubSystem: '<S13>/pitch_rate_control' */

  /* Switch: '<S50>/Sw' incorporates:
   *  RelationalOperator: '<S55>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S13>/pitch_rate_control' */
    /* DataTypeConversion: '<S13>/Data Type Conversion' incorporates:
     *  Gain: '<S53>/PCH_RATE_K_FF'
     *  Sum: '<S53>/Sum1'
     */
    rtb_pitch_angular_accel = (real32_T)(0.12 * claw_ofp_B.Sum4_i2 +
      claw_ofp_B.integ_out_l);

    /* End of Outputs for SubSystem: '<S13>/pitch_rate_control' */
  } else {
    /* DataTypeConversion: '<S13>/Data Type Conversion' */
    rtb_pitch_angular_accel = claw_ofp_DW.Memory1_PreviousInput.ele_cmd;
  }

  if (claw_ofp_DW.Memory1_PreviousInput.pitch_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S13>/pitch_rate_control' */
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' incorporates:
     *  DataTypeConversion: '<S13>/Data Type Conversion'
     *  Gain: '<S53>/PCH_RATE_K_FF'
     *  Sum: '<S53>/Sum1'
     *  Sum: '<S53>/Sum4'
     */
    rtb_MatrixConcatenate_idx_1 = (real32_T)(0.12 * claw_ofp_B.Sum4_i2 +
      claw_ofp_B.integ_out_l);

    /* End of Outputs for SubSystem: '<S13>/pitch_rate_control' */
  } else {
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' */
    rtb_MatrixConcatenate_idx_1 = claw_ofp_DW.Memory1_PreviousInput.ele_cmd;
  }

  /* End of Switch: '<S50>/Sw' */

  /* Switch: '<S16>/Sw' incorporates:
   *  Outport: '<Root>/OutPosControl'
   *  RelationalOperator: '<S21>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode >= 4U) {
    rtb_Sw = claw_out_pos_control.yaw_cmd;
  } else {
    rtb_Sw = claw_ofp_DW.Memory1_PreviousInput.hdg_cmd;
  }

  /* End of Switch: '<S16>/Sw' */

  /* Outputs for Atomic SubSystem: '<S11>/hdg_control' */
  /* DataTypeConversion: '<S19>/Data Type ' incorporates:
   *  Outport: '<Root>/OutState'
   *  Sum: '<S19>/Sum'
   */
  claw_ofp_B.Switch1_lx = rtb_Sw - claw_out_state.sensor_raw.heading;

  /* Outputs for Atomic SubSystem: '<S19>/ShortHdg' */
  /* Switch: '<S27>/Switch1' incorporates:
   *  Constant: '<S27>/   1'
   *  Constant: '<S28>/Constant'
   *  Constant: '<S29>/Constant'
   *  DataTypeConversion: '<S19>/Data Type '
   *  RelationalOperator: '<S28>/Compare'
   *  RelationalOperator: '<S29>/Compare'
   *  Sum: '<S27>/Add1'
   *  Switch: '<S27>/Switch'
   */
  if (claw_ofp_B.Switch1_lx >= 180.0) {
    claw_ofp_B.Switch1_lx -= 360.0;
  } else if (claw_ofp_B.Switch1_lx < -180.0) {
    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S27>/   '
     *  Sum: '<S27>/Add'
     */
    claw_ofp_B.Switch1_lx += 360.0;
  }

  /* End of Switch: '<S27>/Switch1' */
  /* End of Outputs for SubSystem: '<S19>/ShortHdg' */

  /* Logic: '<S19>/Logical Operator' incorporates:
   *  RelationalOperator: '<S26>/Compare'
   */
  rtb_cond_gnd_contact = (rtb_AND_c &&
    (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode >= 2U));

  /* DiscreteIntegrator: '<S19>/Discrete_Time_Integrator' */
  if ((!rtb_cond_gnd_contact) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_g ==
       1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = claw_ofp_ConstB.Zero1_nd;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f >= 0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = 0.0;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f <= -0.0) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = -0.0;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = -0.0;
  }

  claw_ofp_B.integ_out_l = claw_ofp_DW.Discrete_Time_Integrator_DSTA_f;

  /* Sum: '<S19>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete_Time_Integrator'
   *  Gain: '<S19>/PSI_P'
   *  Product: '<S19>/Product'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.K = 1.5 * claw_ofp_B.Switch1_lx * claw_ofp_B.DataType_n +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f;

  /* Update for DiscreteIntegrator: '<S19>/Discrete_Time_Integrator' */
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = 0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_g = (int16_T)rtb_cond_gnd_contact;

  /* End of Update for DiscreteIntegrator: '<S19>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S11>/hdg_control' */

  /* Switch: '<S18>/Sw' incorporates:
   *  RelationalOperator: '<S23>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode >= 2U) {
    /* Switch: '<S18>/Sw1' incorporates:
     *  RelationalOperator: '<S24>/Compare'
     */
    if (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode == 3U) {
      /* Abs: '<S18>/Abs' */
      rtb_Abs_d = (real32_T)fabs(claw_ofp_DW.Memory1_PreviousInput.yaw_rate_cmd);

      /* Switch: '<S25>/Switch2' incorporates:
       *  RelationalOperator: '<S25>/LowerRelop1'
       *  RelationalOperator: '<S25>/UpperRelop'
       *  Switch: '<S25>/Switch'
       *  UnaryMinus: '<S18>/Unary Minus'
       */
      if (claw_ofp_B.K > rtb_Abs_d) {
        claw_ofp_B.K = rtb_Abs_d;
      } else if (claw_ofp_B.K < -claw_ofp_DW.Memory1_PreviousInput.yaw_rate_cmd)
      {
        /* Switch: '<S25>/Switch' incorporates:
         *  UnaryMinus: '<S18>/Unary Minus'
         */
        claw_ofp_B.K = -claw_ofp_DW.Memory1_PreviousInput.yaw_rate_cmd;
      }

      /* End of Switch: '<S25>/Switch2' */
    }

    /* End of Switch: '<S18>/Sw1' */

    /* Saturate: '<S18>/Limit_Auto' */
    if (claw_ofp_B.K > 45.0) {
      claw_ofp_B.K = 45.0;
    } else if (claw_ofp_B.K < -45.0) {
      claw_ofp_B.K = -45.0;
    }

    /* End of Saturate: '<S18>/Limit_Auto' */
  } else {
    claw_ofp_B.K = claw_ofp_DW.Memory1_PreviousInput.yaw_rate_cmd;
  }

  /* End of Switch: '<S18>/Sw' */

  /* Saturate: '<S18>/Limit' */
  if (claw_ofp_B.K > 60.0) {
    claw_ofp_B.K = 60.0;
  } else if (claw_ofp_B.K < -60.0) {
    claw_ofp_B.K = -60.0;
  }

  /* End of Saturate: '<S18>/Limit' */

  /* Outputs for Atomic SubSystem: '<S11>/yaw_rate_control' */
  /* Sum: '<S33>/Sum' incorporates:
   *  Outport: '<Root>/OutState'
   *  UnitDelay: '<S33>/Unit Delay1'
   */
  claw_ofp_B.Sum_k = claw_out_state.yaw_rate + claw_ofp_DW.UnitDelay1_DSTATE_g;

  /* Logic: '<S20>/Logical Operator' incorporates:
   *  RelationalOperator: '<S32>/Compare'
   */
  rtb_AND_c = (rtb_AND_c && (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode >=
    1U));

  /* DiscreteIntegrator: '<S20>/Discrete_Time_Integrator' */
  if ((!rtb_AND_c) && (claw_ofp_DW.Discrete_Time_Integrator_Prev_i == 1)) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = claw_ofp_ConstB.Zero1_g;
    if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k >= 0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = 0.4;
    } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k <= -0.4) {
      claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = -0.4;
    }
  }

  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = -0.4;
  }

  claw_ofp_B.DataType_n = claw_ofp_DW.Discrete_Time_Integrator_DSTA_k;

  /* Gain: '<S20>/YAW_RATE_K' incorporates:
   *  Outport: '<Root>/OutState'
   *  Sum: '<S20>/Sum'
   */
  claw_ofp_B.RLL_RATE_K = (claw_ofp_B.K - claw_out_state.yaw_rate) *
    0.017453292519943295;

  /* Sum: '<S20>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S20>/Discrete_Time_Integrator'
   *  Gain: '<S20>/YAW_RATE_P'
   *  Product: '<S20>/Product'
   *  Switch: '<S8>/Switch'
   */
  claw_ofp_B.DataType4 = 0.05 * claw_ofp_B.RLL_RATE_K * claw_ofp_B.DataType4 +
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k;

  /* Update for UnitDelay: '<S33>/Unit Delay1' incorporates:
   *  Gain: '<S33>/Gain1'
   *  Outport: '<Root>/OutState'
   *  Product: '<S33>/Product5'
   *  Sum: '<S33>/Sum1'
   *  Sum: '<S33>/Sum2'
   *  UnitDelay: '<S33>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE_g = (claw_ofp_B.Sum_k *
    claw_ofp_ConstB.Product4_b + claw_out_state.yaw_rate) * 2.0 +
    claw_ofp_DW.UnitDelay2_DSTATE_bp;

  /* Update for UnitDelay: '<S33>/Unit Delay2' incorporates:
   *  Outport: '<Root>/OutState'
   *  Product: '<S33>/Product2'
   *  Sum: '<S33>/Sum3'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_bp = claw_out_state.yaw_rate - claw_ofp_B.Sum_k *
    claw_ofp_ConstB.Product5_a;

  /* Update for UnitDelay: '<S31>/UD' incorporates:
   *  Product: '<S33>/Product1'
   *  SampleTimeMath: '<S31>/TSamp'
   *
   * About '<S31>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S31>/UD':
   *
   *  Store in Global RAM
   */
  claw_ofp_DW.UD_DSTATE_a = claw_ofp_B.Sum_k * claw_ofp_ConstB.Product3_p *
    200.0;

  /* Switch: '<S20>/Int_ONSW' incorporates:
   *  Constant: '<S20>/Zero'
   *  Gain: '<S20>/YAW_RATE_I'
   *  Product: '<S20>/Product1'
   *  Switch: '<S8>/Switch'
   */
  if (rtb_AND_c) {
    claw_ofp_B.DataType5 *= 0.06 * claw_ofp_B.RLL_RATE_K;
  } else {
    claw_ofp_B.DataType5 = 0.0;
  }

  /* End of Switch: '<S20>/Int_ONSW' */

  /* Update for DiscreteIntegrator: '<S20>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_k += 0.005 * claw_ofp_B.DataType5;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_i = (int16_T)rtb_AND_c;

  /* End of Update for DiscreteIntegrator: '<S20>/Discrete_Time_Integrator' */
  /* End of Outputs for SubSystem: '<S11>/yaw_rate_control' */

  /* Switch: '<S17>/Sw' incorporates:
   *  RelationalOperator: '<S22>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S11>/yaw_rate_control' */
    /* DataTypeConversion: '<S11>/Data Type Conversion' incorporates:
     *  Gain: '<S20>/YAW_RATE_K_FF'
     *  Product: '<S20>/Product3'
     *  Sum: '<S20>/Sum1'
     *  Switch: '<S8>/Switch'
     */
    rtb_Abs_d = (real32_T)(0.05 * claw_ofp_B.K * claw_ofp_B.Abs1_g +
      claw_ofp_B.DataType4);

    /* End of Outputs for SubSystem: '<S11>/yaw_rate_control' */
  } else {
    /* DataTypeConversion: '<S11>/Data Type Conversion' */
    rtb_Abs_d = claw_ofp_DW.Memory1_PreviousInput.rud_cmd;
  }

  if (claw_ofp_DW.Memory1_PreviousInput.yaw_axis_mode != 0U) {
    /* Outputs for Atomic SubSystem: '<S11>/yaw_rate_control' */
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' incorporates:
     *  DataTypeConversion: '<S11>/Data Type Conversion'
     *  Gain: '<S20>/YAW_RATE_K_FF'
     *  Product: '<S20>/Product3'
     *  Sum: '<S20>/Sum1'
     *  Switch: '<S8>/Switch'
     */
    rtb_MatrixConcatenate_idx_2 = (real32_T)(0.05 * claw_ofp_B.K *
      claw_ofp_B.Abs1_g + claw_ofp_B.DataType4);

    /* End of Outputs for SubSystem: '<S11>/yaw_rate_control' */
  } else {
    /* SignalConversion generated from: '<S9>/Matrix Concatenate' */
    rtb_MatrixConcatenate_idx_2 = claw_ofp_DW.Memory1_PreviousInput.rud_cmd;
  }

  /* End of Switch: '<S17>/Sw' */

  /* Switch: '<S63>/Sw' incorporates:
   *  RelationalOperator: '<S64>/Compare'
   */
  if (claw_ofp_DW.Memory1_PreviousInput.vert_axis_mode != 0U) {
    /* Sum: '<S14>/Sum2' incorporates:
     *  Constant: '<S14>/weight-thrust_ratio'
     *  DataTypeConversion: '<S14>/Data Type Conversion'
     *  Gain: '<S14>/gravity'
     *  Outport: '<Root>/OutPosControl'
     *  Product: '<S14>/Product'
     */
    claw_ofp_B.PY2 = claw_out_pos_control.az_body * 0.5 * 0.1019367991845056 -
      claw_ofp_ConstB.thr_delta;

    /* Saturate: '<S14>/Limit' incorporates:
     *  DataTypeConversion: '<S14>/Data Type Conversion1'
     */
    if (claw_ofp_B.PY2 > 1.0) {
      rtb_Sw_m = 1.0F;
    } else if (claw_ofp_B.PY2 < 0.12) {
      rtb_Sw_m = 0.12F;
    } else {
      rtb_Sw_m = (real32_T)claw_ofp_B.PY2;
    }

    /* End of Saturate: '<S14>/Limit' */
  } else {
    /* DataTypeConversion: '<S14>/Data Type Conversion2' */
    claw_ofp_B.PY2 = claw_ofp_DW.Memory1_PreviousInput.thr_cmd;

    /* Saturate: '<S14>/Limit1' incorporates:
     *  DataTypeConversion: '<S14>/Data Type Conversion3'
     */
    if (claw_ofp_B.PY2 > 1.0) {
      rtb_Sw_m = 1.0F;
    } else if (claw_ofp_B.PY2 < 0.08) {
      rtb_Sw_m = 0.08F;
    } else {
      rtb_Sw_m = (real32_T)claw_ofp_B.PY2;
    }

    /* End of Saturate: '<S14>/Limit1' */
  }

  /* End of Switch: '<S63>/Sw' */
  for (rtb_Compare_eu_0 = 0; rtb_Compare_eu_0 < 16; rtb_Compare_eu_0++) {
    /* Product: '<S9>/Product' incorporates:
     *  Constant: '<S9>/mixer'
     *  DataTypeConversion: '<S9>/Data Type 2'
     *  SignalConversion generated from: '<S9>/Matrix Concatenate'
     */
    claw_ofp_B.PY2 = ((claw_ofp_ConstP.mixer_Value[rtb_Compare_eu_0 + 16] *
                       rtb_MatrixConcatenate_idx_1 +
                       claw_ofp_ConstP.mixer_Value[rtb_Compare_eu_0] *
                       rtb_MatrixConcatenate_idx_0) +
                      claw_ofp_ConstP.mixer_Value[rtb_Compare_eu_0 + 32] *
                      rtb_MatrixConcatenate_idx_2) +
      claw_ofp_ConstP.mixer_Value[rtb_Compare_eu_0 + 48] * rtb_Sw_m;

    /* Saturate: '<S9>/Limit' */
    if (claw_ofp_B.PY2 > 1.0) {
      /* DataTypeConversion: '<S9>/Data Type ' */
      claw_out_att_control.motor_out[rtb_Compare_eu_0] = 1.0F;
    } else if (claw_ofp_B.PY2 < 0.08) {
      /* DataTypeConversion: '<S9>/Data Type ' */
      claw_out_att_control.motor_out[rtb_Compare_eu_0] = 0.08F;
    } else {
      /* DataTypeConversion: '<S9>/Data Type ' */
      claw_out_att_control.motor_out[rtb_Compare_eu_0] = (real32_T)
        claw_ofp_B.PY2;
    }

    /* End of Saturate: '<S9>/Limit' */
  }

  /* Saturate: '<S9>/Saturation1' incorporates:
   *  DataTypeConversion: '<S9>/Data Type 4'
   */
  if (claw_ofp_B.hdg_int > 1.0F) {
    claw_ofp_B.Abs1_g = 1.0;
  } else if (claw_ofp_B.hdg_int < -1.0F) {
    claw_ofp_B.Abs1_g = -1.0;
  } else {
    claw_ofp_B.Abs1_g = claw_ofp_B.hdg_int;
  }

  /* End of Saturate: '<S9>/Saturation1' */

  /* Gain: '<S9>/Gain1' */
  claw_ofp_B.Sum_k = 6.0 * claw_ofp_B.Abs1_g;

  /* Saturate: '<S9>/Saturation2' incorporates:
   *  DataTypeConversion: '<S9>/Data Type 5'
   */
  if (rtb_pitch_angular_accel > 1.0F) {
    claw_ofp_B.Abs1_g = 1.0;
  } else if (rtb_pitch_angular_accel < -1.0F) {
    claw_ofp_B.Abs1_g = -1.0;
  } else {
    claw_ofp_B.Abs1_g = rtb_pitch_angular_accel;
  }

  /* End of Saturate: '<S9>/Saturation2' */

  /* Gain: '<S9>/Gain2' */
  claw_ofp_B.Dist = 6.0 * claw_ofp_B.Abs1_g;

  /* Saturate: '<S9>/Saturation3' incorporates:
   *  DataTypeConversion: '<S9>/Data Type 6'
   */
  if (rtb_Abs_d > 1.0F) {
    claw_ofp_B.Abs1_g = 1.0;
  } else if (rtb_Abs_d < -1.0F) {
    claw_ofp_B.Abs1_g = -1.0;
  } else {
    claw_ofp_B.Abs1_g = rtb_Abs_d;
  }

  /* End of Saturate: '<S9>/Saturation3' */

  /* Gain: '<S9>/Gain3' */
  claw_ofp_B.TrigonometricFunction1 = 6.0 * claw_ofp_B.Abs1_g;

  /* Bias: '<S9>/Bias' incorporates:
   *  DataTypeConversion: '<S9>/Data Type 3'
   *  Gain: '<S9>/Gain'
   */
  claw_ofp_B.PY2 = 12.0 * rtb_Sw_m + -1.0;

  /* DataTypeConversion: '<S9>/Data Type 1' incorporates:
   *  Saturate: '<S9>/Limit2'
   *  Saturate: '<S9>/Limit3'
   *  Saturate: '<S9>/Limit4'
   *  Saturate: '<S9>/Limit5'
   *  Saturate: '<S9>/Limit7'
   */
  claw_out_att_control.act_out[0] = (real32_T)claw_ofp_ConstB.Limit1;
  claw_out_att_control.act_out[1] = (real32_T)claw_ofp_B.PY2;
  claw_out_att_control.act_out[2] = (real32_T)claw_ofp_B.Sum_k;
  claw_out_att_control.act_out[3] = (real32_T)claw_ofp_B.Dist;
  claw_out_att_control.act_out[4] = (real32_T)claw_ofp_B.TrigonometricFunction1;
  claw_out_att_control.act_out[5] = (real32_T)claw_ofp_B.PY2;
  claw_out_att_control.act_out[6] = (real32_T)claw_ofp_B.Sum_k;

  /* Sum: '<S15>/Sum1' */
  claw_ofp_B.Abs1_g = claw_ofp_B.Dist + claw_ofp_B.TrigonometricFunction1;

  /* Saturate: '<S9>/Limit8' */
  if (claw_ofp_B.Abs1_g > 6.0) {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[7] = 6.0F;
  } else if (claw_ofp_B.Abs1_g < -6.0) {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[7] = -6.0F;
  } else {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[7] = (real32_T)claw_ofp_B.Abs1_g;
  }

  /* End of Saturate: '<S9>/Limit8' */

  /* DataTypeConversion: '<S9>/Data Type 1' incorporates:
   *  Saturate: '<S9>/Limit10'
   *  Saturate: '<S9>/Limit2'
   */
  claw_out_att_control.act_out[8] = (real32_T)claw_ofp_B.PY2;
  claw_out_att_control.act_out[9] = (real32_T)claw_ofp_B.Sum_k;

  /* Sum: '<S15>/Sum' */
  claw_ofp_B.PY2 = claw_ofp_B.Dist - claw_ofp_B.TrigonometricFunction1;

  /* Saturate: '<S9>/Limit11' */
  if (claw_ofp_B.PY2 > 6.0) {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[10] = 6.0F;
  } else if (claw_ofp_B.PY2 < -6.0) {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[10] = -6.0F;
  } else {
    /* DataTypeConversion: '<S9>/Data Type 1' */
    claw_out_att_control.act_out[10] = (real32_T)claw_ofp_B.PY2;
  }

  /* End of Saturate: '<S9>/Limit11' */

  /* DataTypeConversion: '<S9>/Data Type 1' incorporates:
   *  Constant: '<S9>/Constant1'
   *  Constant: '<S9>/Constant2'
   *  Constant: '<S9>/Constant3'
   *  Constant: '<S9>/Constant4'
   *  Constant: '<S9>/Constant5'
   */
  claw_out_att_control.act_out[11] = 0.0F;
  claw_out_att_control.act_out[12] = 0.0F;
  claw_out_att_control.act_out[13] = 0.0F;
  claw_out_att_control.act_out[14] = 0.0F;
  claw_out_att_control.act_out[15] = 0.0F;

  /* BusCreator: '<S10>/Bus Creator1' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion11'
   *  DataTypeConversion: '<S10>/Data Type Conversion20'
   *  DataTypeConversion: '<S10>/Data Type Conversion25'
   *  DataTypeConversion: '<S10>/Data Type Conversion3'
   *  DataTypeConversion: '<S10>/Data Type Conversion35'
   *  DataTypeConversion: '<S10>/Data Type Conversion4'
   *  DataTypeConversion: '<S10>/Data Type Conversion42'
   *  DataTypeConversion: '<S10>/Data Type Conversion44'
   *  DataTypeConversion: '<S10>/Data Type Conversion5'
   *  DataTypeConversion: '<S10>/Data Type Conversion9'
   *  Outport: '<Root>/OutAttControl'
   */
  claw_out_att_control.roll_ref = claw_ofp_B.Saturation_n;

  /* Outputs for Atomic SubSystem: '<S12>/roll_control' */
  claw_out_att_control.roll_error = rtb_Abs1_e;

  /* End of Outputs for SubSystem: '<S12>/roll_control' */
  claw_out_att_control.roll_int = (real32_T)claw_ofp_B.rtb_Sum1_idx_0;
  claw_out_att_control.roll_rate_ref = (real32_T)claw_ofp_B.Limit4;
  claw_out_att_control.roll_rate_int = (real32_T)claw_ofp_B.Limit3;
  claw_out_att_control.roll_accel_out = claw_ofp_B.hdg_int;
  claw_out_att_control.pitch_ref = rtb_Limit_n;

  /* Outputs for Atomic SubSystem: '<S13>/pitch_control' */
  claw_out_att_control.pitch_error = rtb_Sum4_i2_tmp;

  /* End of Outputs for SubSystem: '<S13>/pitch_control' */
  claw_out_att_control.pitch_int = (real32_T)claw_ofp_B.integ_out_m;
  claw_out_att_control.pitch_rate_ref = (real32_T)claw_ofp_B.Sum4_i2;
  claw_out_att_control.pitch_rate_int = (real32_T)claw_ofp_B.integ_out_c;
  claw_out_att_control.pitch_accel_out = rtb_pitch_angular_accel;
  claw_out_att_control.hdg_ref = rtb_Sw;
  claw_out_att_control.hdg_error = (real32_T)claw_ofp_B.Switch1_lx;
  claw_out_att_control.hdg_int = (real32_T)claw_ofp_B.integ_out_l;
  claw_out_att_control.yaw_rate_ref = (real32_T)claw_ofp_B.K;
  claw_out_att_control.yaw_rate_int = (real32_T)claw_ofp_B.DataType_n;
  claw_out_att_control.yaw_accel_out = rtb_Abs_d;
  claw_out_att_control.thr_cmd = rtb_Sw_m;

  /* Outport: '<Root>/OutModeManager' incorporates:
   *  MultiPortSwitch: '<S5>/Multiport Switch'
   */
  claw_out_mode_manager = claw_ofp_DW.Memory1_PreviousInput;

  /* Sum: '<S287>/Sum1' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 5'
   *  Gain: '<S287>/Gain1'
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S287>/Product5'
   *  Sum: '<S287>/Sum2'
   *  UnitDelay: '<S287>/Unit Delay1'
   *  UnitDelay: '<S287>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE_pz = (claw_ofp_B.Abs1 *
    claw_ofp_ConstB.Product4_fh + claw_in_sensor.yaw_rate) * 2.0 +
    claw_ofp_DW.UnitDelay2_DSTATE_g;

  /* Sum: '<S287>/Sum3' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 5'
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S287>/Product2'
   *  UnitDelay: '<S287>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_g = claw_in_sensor.yaw_rate - claw_ofp_B.Abs1 *
    claw_ofp_ConstB.Product5_i;

  /* Sum: '<S285>/Sum1' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 4'
   *  Gain: '<S285>/Gain1'
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S285>/Product5'
   *  Sum: '<S285>/Sum2'
   *  UnitDelay: '<S285>/Unit Delay1'
   *  UnitDelay: '<S285>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE_p = (claw_ofp_B.Sum * claw_ofp_ConstB.Product4_f
    + claw_in_sensor.pitch_rate) * 2.0 + claw_ofp_DW.UnitDelay2_DSTATE_b;

  /* Sum: '<S285>/Sum3' incorporates:
   *  DataTypeConversion: '<S274>/Data Type 4'
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S285>/Product2'
   *  UnitDelay: '<S285>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_b = claw_in_sensor.pitch_rate - claw_ofp_B.Sum *
    claw_ofp_ConstB.Product5_d;

  /* Sum: '<S283>/Sum1' incorporates:
   *  DataTypeConversion: '<S274>/Data Type '
   *  Gain: '<S283>/Gain1'
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S283>/Product5'
   *  Sum: '<S283>/Sum2'
   *  UnitDelay: '<S283>/Unit Delay1'
   *  UnitDelay: '<S283>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay1_DSTATE = (claw_ofp_B.Sum_j * claw_ofp_ConstB.Product4 +
    claw_in_sensor.roll_rate) * 2.0 + claw_ofp_DW.UnitDelay2_DSTATE_d;

  /* Sum: '<S283>/Sum3' incorporates:
   *  DataTypeConversion: '<S274>/Data Type '
   *  Inport: '<Root>/in_sensor'
   *  Product: '<S283>/Product2'
   *  UnitDelay: '<S283>/Unit Delay2'
   */
  claw_ofp_DW.UnitDelay2_DSTATE_d = claw_in_sensor.roll_rate - claw_ofp_B.Sum_j *
    claw_ofp_ConstB.Product5;

  /* Update for Memory generated from: '<S2>/Memory' incorporates:
   *  Outport: '<Root>/OutState'
   */
  claw_ofp_DW.Memory_1_PreviousInput = claw_out_state;
}

/* Model initialize function */
void claw_ofp_initialize(void)
{
  /* InitializeConditions for UnitDelay: '<S326>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE = 0.005;

  /* InitializeConditions for UnitDelay: '<S327>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE_j = 0.005;

  /* InitializeConditions for UnitDelay: '<S328>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE_d = 0.005;

  /* SystemInitialize for Chart: '<S153>/Standby' */
  claw_ofp_B.FMMOUT_e.state_id = 0UL;
  claw_ofp_B.FMMOUT_e.claw_mode = 0U;
  claw_ofp_B.FMMOUT_e.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_e.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_e.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_e.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_e.guide_type = 0U;
  claw_ofp_B.FMMOUT_e.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_e.orbit_flag = false;
  claw_ofp_B.FMMOUT_e.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_e.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S156>/StickManual' */
  claw_ofp_B.FMMOUT_h.state_id = 0UL;
  claw_ofp_B.FMMOUT_h.claw_mode = 0U;
  claw_ofp_B.FMMOUT_h.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_h.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_h.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_h.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_h.guide_type = 0U;
  claw_ofp_B.FMMOUT_h.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_h.orbit_flag = false;
  claw_ofp_B.FMMOUT_h.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_h.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S158>/stick_rate' */
  claw_ofp_B.FMMOUT_j.state_id = 0UL;
  claw_ofp_B.FMMOUT_j.claw_mode = 0U;
  claw_ofp_B.FMMOUT_j.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_j.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_j.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_j.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_j.guide_type = 0U;
  claw_ofp_B.FMMOUT_j.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_j.orbit_flag = false;
  claw_ofp_B.FMMOUT_j.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_j.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S155>/stick_att' */
  claw_ofp_B.FMMOUT_i.state_id = 0UL;
  claw_ofp_B.FMMOUT_i.claw_mode = 0U;
  claw_ofp_B.FMMOUT_i.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_i.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_i.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_i.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_i.guide_type = 0U;
  claw_ofp_B.FMMOUT_i.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_i.orbit_flag = false;
  claw_ofp_B.FMMOUT_i.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_i.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S154>/stick_alt' */
  claw_ofp_B.FMMOUT_d.state_id = 0UL;
  claw_ofp_B.FMMOUT_d.claw_mode = 0U;
  claw_ofp_B.FMMOUT_d.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_d.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_d.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_d.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_d.guide_type = 0U;
  claw_ofp_B.FMMOUT_d.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_d.orbit_flag = false;
  claw_ofp_B.FMMOUT_d.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_d.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S157>/stick_pos' */
  claw_ofp_B.FMMOUT_a.state_id = 0UL;
  claw_ofp_B.FMMOUT_a.claw_mode = 0U;
  claw_ofp_B.FMMOUT_a.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_a.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_a.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_a.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_a.guide_type = 0U;
  claw_ofp_B.FMMOUT_a.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_a.orbit_flag = false;
  claw_ofp_B.FMMOUT_a.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_a.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.state_id = 0UL;
  claw_ofp_B.FMMOUT_do.claw_mode = 0U;
  claw_ofp_B.FMMOUT_do.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_do.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_do.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_do.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_do.guide_type = 0U;
  claw_ofp_B.FMMOUT_do.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_do.orbit_flag = false;
  claw_ofp_B.FMMOUT_do.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_do.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S160>/waypoint' */
  claw_ofp_B.FMMOUT.state_id = 0UL;
  claw_ofp_B.FMMOUT.claw_mode = 0U;
  claw_ofp_B.FMMOUT.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT.guide_type = 0U;
  claw_ofp_B.FMMOUT.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT.orbit_flag = false;
  claw_ofp_B.FMMOUT.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S150>/loiter' */
  claw_ofp_B.FMMOUT_hl.state_id = 0UL;
  claw_ofp_B.FMMOUT_hl.claw_mode = 0U;
  claw_ofp_B.FMMOUT_hl.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_hl.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_hl.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_hl.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_hl.guide_type = 0U;
  claw_ofp_B.FMMOUT_hl.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_hl.orbit_flag = false;
  claw_ofp_B.FMMOUT_hl.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_hl.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S159>/takeoff' */
  claw_ofp_B.FMMOUT_k.state_id = 0UL;
  claw_ofp_B.FMMOUT_k.claw_mode = 0U;
  claw_ofp_B.FMMOUT_k.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_k.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_k.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_k.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_k.guide_type = 0U;
  claw_ofp_B.FMMOUT_k.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_k.orbit_flag = false;
  claw_ofp_B.FMMOUT_k.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_k.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S149>/land' */
  claw_ofp_B.FMMOUT_c.state_id = 0UL;
  claw_ofp_B.FMMOUT_c.claw_mode = 0U;
  claw_ofp_B.FMMOUT_c.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_c.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_c.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_c.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_c.guide_type = 0U;
  claw_ofp_B.FMMOUT_c.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_c.orbit_flag = false;
  claw_ofp_B.FMMOUT_c.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_c.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S148>/knob' */
  claw_ofp_B.FMMOUT_b.state_id = 0UL;
  claw_ofp_B.FMMOUT_b.claw_mode = 0U;
  claw_ofp_B.FMMOUT_b.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_b.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_b.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_b.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_b.guide_type = 0U;
  claw_ofp_B.FMMOUT_b.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_b.orbit_flag = false;
  claw_ofp_B.FMMOUT_b.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_b.thr_cmd = 0.0F;

  /* SystemInitialize for Chart: '<S151>/offboard' */
  claw_ofp_B.FMMOUT_n.state_id = 0UL;
  claw_ofp_B.FMMOUT_n.claw_mode = 0U;
  claw_ofp_B.FMMOUT_n.roll_axis_mode = 0U;
  claw_ofp_B.FMMOUT_n.pitch_axis_mode = 0U;
  claw_ofp_B.FMMOUT_n.yaw_axis_mode = 0U;
  claw_ofp_B.FMMOUT_n.vert_axis_mode = 0U;
  claw_ofp_B.FMMOUT_n.guide_type = 0U;

  /* SystemInitialize for Chart: '<S153>/Standby' */
  claw_ofp_B.FMMOUT_e.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S156>/StickManual' */
  claw_ofp_B.FMMOUT_h.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S158>/stick_rate' */
  claw_ofp_B.FMMOUT_j.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S155>/stick_att' */
  claw_ofp_B.FMMOUT_i.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S154>/stick_alt' */
  claw_ofp_B.FMMOUT_d.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S157>/stick_pos' */
  claw_ofp_B.FMMOUT_a.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S160>/waypoint' */
  claw_ofp_B.FMMOUT.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S150>/loiter' */
  claw_ofp_B.FMMOUT_hl.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S159>/takeoff' */
  claw_ofp_B.FMMOUT_k.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S149>/land' */
  claw_ofp_B.FMMOUT_c.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S148>/knob' */
  claw_ofp_B.FMMOUT_b.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S151>/offboard' */
  claw_ofp_B.FMMOUT_n.wp_x_local[0] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_y_local[0] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_z_local[0] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_spd[0] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_radius[0] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_orbit_dir[0] = false;

  /* SystemInitialize for Chart: '<S153>/Standby' */
  claw_ofp_B.FMMOUT_e.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S156>/StickManual' */
  claw_ofp_B.FMMOUT_h.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S158>/stick_rate' */
  claw_ofp_B.FMMOUT_j.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S155>/stick_att' */
  claw_ofp_B.FMMOUT_i.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S154>/stick_alt' */
  claw_ofp_B.FMMOUT_d.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S157>/stick_pos' */
  claw_ofp_B.FMMOUT_a.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S160>/waypoint' */
  claw_ofp_B.FMMOUT.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S150>/loiter' */
  claw_ofp_B.FMMOUT_hl.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S159>/takeoff' */
  claw_ofp_B.FMMOUT_k.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S149>/land' */
  claw_ofp_B.FMMOUT_c.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S148>/knob' */
  claw_ofp_B.FMMOUT_b.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S151>/offboard' */
  claw_ofp_B.FMMOUT_n.wp_x_local[1] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_y_local[1] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_z_local[1] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_spd[1] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_radius[1] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_orbit_dir[1] = false;

  /* SystemInitialize for Chart: '<S153>/Standby' */
  claw_ofp_B.FMMOUT_e.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_e.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_e.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S156>/StickManual' */
  claw_ofp_B.FMMOUT_h.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_h.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_h.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S158>/stick_rate' */
  claw_ofp_B.FMMOUT_j.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_j.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_j.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S155>/stick_att' */
  claw_ofp_B.FMMOUT_i.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_i.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_i.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S154>/stick_alt' */
  claw_ofp_B.FMMOUT_d.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_d.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_d.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S157>/stick_pos' */
  claw_ofp_B.FMMOUT_a.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_a.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_a.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S152>/point_nav' */
  claw_ofp_B.FMMOUT_do.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_do.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_do.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S160>/waypoint' */
  claw_ofp_B.FMMOUT.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S150>/loiter' */
  claw_ofp_B.FMMOUT_hl.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_hl.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_hl.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S159>/takeoff' */
  claw_ofp_B.FMMOUT_k.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_k.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_k.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S149>/land' */
  claw_ofp_B.FMMOUT_c.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_c.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_c.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S148>/knob' */
  claw_ofp_B.FMMOUT_b.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_b.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_b.wp_orbit_dir[2] = false;

  /* SystemInitialize for Chart: '<S151>/offboard' */
  claw_ofp_B.FMMOUT_n.wp_x_local[2] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_y_local[2] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_z_local[2] = 0.0;
  claw_ofp_B.FMMOUT_n.wp_spd[2] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_radius[2] = 0.0F;
  claw_ofp_B.FMMOUT_n.wp_orbit_dir[2] = false;
  claw_ofp_B.FMMOUT_n.wp_loiter_type = 0U;
  claw_ofp_B.FMMOUT_n.orbit_flag = false;
  claw_ofp_B.FMMOUT_n.roll_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.pitch_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.hdg_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.alt_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.roll_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.pitch_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.yaw_rate_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.ail_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.ele_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.rud_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.vel_x_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.vel_y_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.vel_z_cmd = 0.0F;
  claw_ofp_B.FMMOUT_n.thr_cmd = 0.0F;

  /* SystemInitialize for Atomic SubSystem: '<S182>/PID_with_FF1' */
  /* InitializeConditions for UnitDelay: '<S242>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE_a = 0.005;

  /* InitializeConditions for DiscreteIntegrator: '<S235>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = claw_ofp_ConstB.Zero1_a;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_g <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_g = -2.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_h = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S235>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S182>/PID_with_FF1' */

  /* SystemInitialize for Atomic SubSystem: '<S182>/PID_with_FF2' */
  /* InitializeConditions for UnitDelay: '<S246>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE_p = 0.005;

  /* InitializeConditions for DiscreteIntegrator: '<S236>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = claw_ofp_ConstB.Zero1_o;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m >= 2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = 2.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_m <= -2.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_m = -2.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_PrevRe = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S236>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S182>/PID_with_FF2' */

  /* SystemInitialize for Atomic SubSystem: '<S182>/PID_with_FF3' */
  /* InitializeConditions for UnitDelay: '<S250>/Unit Delay3' */
  claw_ofp_DW.UnitDelay3_DSTATE_b = 0.005;

  /* InitializeConditions for DiscreteIntegrator: '<S237>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTATE = claw_ofp_ConstB.Zero1;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTATE >= 1.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTATE = 1.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTATE <= -1.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTATE = -1.0;
  }

  /* End of InitializeConditions for DiscreteIntegrator: '<S237>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S182>/PID_with_FF3' */

  /* SystemInitialize for Atomic SubSystem: '<S12>/roll_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S38>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = claw_ofp_ConstB.Zero1_i;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_o <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_o = -0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_f = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S38>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S12>/roll_control' */

  /* SystemInitialize for Atomic SubSystem: '<S12>/roll_rate_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S39>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DST_gd = claw_ofp_ConstB.Zero1_n;
  if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_gd <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_gd = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_j = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S39>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S12>/roll_rate_control' */

  /* SystemInitialize for Atomic SubSystem: '<S13>/pitch_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S52>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = claw_ofp_ConstB.Zero1_f;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_h <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_h = -0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_k = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S52>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S13>/pitch_control' */

  /* SystemInitialize for Atomic SubSystem: '<S13>/pitch_rate_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S53>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DST_go = claw_ofp_ConstB.Zero1_c;
  if (claw_ofp_DW.Discrete_Time_Integrator_DST_go >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DST_go <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DST_go = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_e = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S53>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S13>/pitch_rate_control' */

  /* SystemInitialize for Atomic SubSystem: '<S11>/hdg_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S19>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = claw_ofp_ConstB.Zero1_nd;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f >= 0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = 0.0;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_f <= -0.0) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_f = -0.0;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_g = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S19>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S11>/hdg_control' */

  /* SystemInitialize for Atomic SubSystem: '<S11>/yaw_rate_control' */
  /* InitializeConditions for DiscreteIntegrator: '<S20>/Discrete_Time_Integrator' */
  claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = claw_ofp_ConstB.Zero1_g;
  if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k >= 0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = 0.4;
  } else if (claw_ofp_DW.Discrete_Time_Integrator_DSTA_k <= -0.4) {
    claw_ofp_DW.Discrete_Time_Integrator_DSTA_k = -0.4;
  }

  claw_ofp_DW.Discrete_Time_Integrator_Prev_i = 2;

  /* End of InitializeConditions for DiscreteIntegrator: '<S20>/Discrete_Time_Integrator' */
  /* End of SystemInitialize for SubSystem: '<S11>/yaw_rate_control' */
}

/* Model terminate function */
void claw_ofp_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
