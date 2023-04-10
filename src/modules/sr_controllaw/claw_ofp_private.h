/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: claw_ofp_private.h
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

#ifndef RTW_HEADER_claw_ofp_private_h_
#define RTW_HEADER_claw_ofp_private_h_
#include "rtwtypes.h"
#include "claw_ofp.h"
#include "claw_ofp_types.h"

/* Imported (extern) block signals */
extern BUS_ClawInSensor claw_in_sensor;/* '<Root>/in_sensor' */
extern BUS_ClawInOFP claw_in_ofp;      /* '<Root>/in_OFP' */
extern BUS_ClawInCmd claw_in_cmd;      /* '<Root>/in_cmd' */
extern BUS_ClawInTest claw_in_test;    /* '<Root>/in_test' */
extern BUS_ClawOutState claw_out_state;/* '<Root>/OutState' */
extern BUS_ClawOutModeManager claw_out_mode_manager;/* '<Root>/OutModeManager' */
extern BUS_ClawOutPosControl claw_out_pos_control;/* '<Root>/OutPosControl' */
extern BUS_ClawOutAttControl claw_out_att_control;/* '<Root>/OutAttControl' */
extern real_T rt_hypotd(real_T u0, real_T u1);
extern real_T look1_pbinlagpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T prevIndex[], uint32_T maxIndex);
extern void claw_ofp_ActiveHorizontaldist2d(real_T rtu_wp_x, real_T rtu_wp_y,
  real_T rtu_ac_x, real_T rtu_ac_y, real_T *rty_dist);

#endif                                 /* RTW_HEADER_claw_ofp_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
