/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: claw_ofp.h
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

#ifndef RTW_HEADER_claw_ofp_h_
#define RTW_HEADER_claw_ofp_h_
#ifndef claw_ofp_COMMON_INCLUDES_
#define claw_ofp_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* claw_ofp_COMMON_INCLUDES_ */

#include "claw_ofp_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  BUS_ClawOutCmd BusConversion_InsertedFor_point;
  BUS_ClawOutModeManager FMMOUT;       /* '<S160>/waypoint' */
  BUS_ClawOutModeManager FMMOUT_k;     /* '<S159>/takeoff' */
  BUS_ClawOutModeManager FMMOUT_j;     /* '<S158>/stick_rate' */
  BUS_ClawOutModeManager FMMOUT_a;     /* '<S157>/stick_pos' */
  BUS_ClawOutModeManager FMMOUT_h;     /* '<S156>/StickManual' */
  BUS_ClawOutModeManager FMMOUT_i;     /* '<S155>/stick_att' */
  BUS_ClawOutModeManager FMMOUT_d;     /* '<S154>/stick_alt' */
  BUS_ClawOutModeManager FMMOUT_e;     /* '<S153>/Standby' */
  BUS_ClawOutModeManager FMMOUT_do;    /* '<S152>/point_nav' */
  BUS_ClawOutModeManager FMMOUT_n;     /* '<S151>/offboard' */
  BUS_ClawOutModeManager FMMOUT_hl;    /* '<S150>/loiter' */
  BUS_ClawOutModeManager FMMOUT_c;     /* '<S149>/land' */
  BUS_ClawOutModeManager FMMOUT_b;     /* '<S148>/knob' */
  real_T u1[15];
  real_T tmpForInput_tmp[11];
  real_T in;                           /* '<S330>/in' */
  real_T in_h;                         /* '<S329>/in' */
  real_T landed_flag;                  /* '<S291>/Chart' */
  real_T minV;
  real_T maxV;
  real_T Slope;
  real_T Dist;
  real_T xtmp;
  real_T ytmp;
  real_T Abs1;                         /* '<S79>/Abs1' */
  real_T Sum;                          /* '<S70>/Sum' */
  real_T DataType5;                    /* '<S274>/Data Type 5' */
  real_T Sum_j;                        /* '<S92>/Sum' */
  real_T DataType4;                    /* '<S274>/Data Type 4' */
  real_T Sum_k;                        /* '<S114>/Sum' */
  real_T DataType_n;                   /* '<S274>/Data Type ' */
  real_T Switch_o;                     /* '<S315>/Switch' */
  real_T Abs1_g;                       /* '<S312>/Abs1' */
  real_T Limit4;                       /* '<S9>/Limit4' */
  real_T Limit3;                       /* '<S9>/Limit3' */
  real_T Vfwd;                         /* '<S188>/Product2' */
  real_T Vlat;                         /* '<S188>/Product3' */
  real_T Gain;                         /* '<S230>/Gain' */
  real_T Track_mode;                   /* '<S190>/Sum1' */
  real_T dist_to_wp;                   /* '<S223>/Hypot' */
  real_T PY2;                          /* '<S198>/OrbitAlgorithm' */
  real_T Subtract;                     /* '<S206>/Subtract' */
  real_T TrigonometricFunction1_d;     /* '<S193>/Trigonometric Function1' */
  real_T TrigonometricFunction2_mh;    /* '<S203>/Trigonometric Function2' */
  real_T Product_g;                    /* '<S189>/Product' */
  real_T K;                            /* '<S237>/K' */
  real_T TrigonometricFunction1;       /* '<S252>/Trigonometric Function1' */
  real_T RLL_RATE_K;                   /* '<S39>/RLL_RATE_K' */
  real_T integ_out_m;                  /* '<S52>/Discrete_Time_Integrator' */
  real_T Sum4_i2;                      /* '<S52>/Sum4' */
  real_T integ_out_c;                  /* '<S53>/Discrete_Time_Integrator' */
  real_T Switch1_lx;                   /* '<S27>/Switch1' */
  real_T integ_out_l;                  /* '<S19>/Discrete_Time_Integrator' */
  real_T rtb_DataType2_idx_0;
  real_T rtb_DataType2_idx_2;
  real_T rtb_Sum1_idx_0;
  real_T rtb_Sum1_idx_1;
  real_T rtb_Limit4_tmp;
  real_T rtb_Sum_k_tmp;
  real_T a;
  real_T b;
  real_T q;
  real32_T hdg_int;                    /* '<S10>/Data Type Conversion42' */
  real32_T Saturation_n;               /* '<S187>/Saturation' */
} B_claw_ofp_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  BUS_ClawOutState Memory_1_PreviousInput;/* '<S2>/Memory' */
  BUS_ClawOutModeManager Memory1_PreviousInput;/* '<S5>/Memory1' */
  real_T UnitDelay1_DSTATE;            /* '<S283>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_p;          /* '<S285>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_pz;         /* '<S287>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<S326>/Unit Delay2' */
  real_T UnitDelay3_DSTATE;            /* '<S326>/Unit Delay3' */
  real_T UnitDelay2_DSTATE_h;          /* '<S327>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_j;          /* '<S327>/Unit Delay3' */
  real_T UnitDelay2_DSTATE_l;          /* '<S328>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_d;          /* '<S328>/Unit Delay3' */
  real_T UnitDelay2_DSTATE_g;          /* '<S287>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S285>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_d;          /* '<S283>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_h3;         /* '<S250>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_b;          /* '<S250>/Unit Delay3' */
  real_T UD_DSTATE;                    /* '<S251>/UD' */
  real_T Discrete_Time_Integrator_DSTATE;/* '<S237>/Discrete_Time_Integrator' */
  real_T UnitDelay2_DSTATE_b5;         /* '<S246>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_p;          /* '<S246>/Unit Delay3' */
  real_T UD_DSTATE_k;                  /* '<S247>/UD' */
  real_T Discrete_Time_Integrator_DSTA_m;/* '<S236>/Discrete_Time_Integrator' */
  real_T UnitDelay2_DSTATE_i;          /* '<S242>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_a;          /* '<S242>/Unit Delay3' */
  real_T UD_DSTATE_d;                  /* '<S243>/UD' */
  real_T Discrete_Time_Integrator_DSTA_g;/* '<S235>/Discrete_Time_Integrator' */
  real_T Discrete_Time_Integrator_DST_go;/* '<S53>/Discrete_Time_Integrator' */
  real_T UnitDelay1_DSTATE_o;          /* '<S61>/Unit Delay1' */
  real_T UD_DSTATE_p;                  /* '<S59>/UD' */
  real_T UnitDelay2_DSTATE_m;          /* '<S61>/Unit Delay2' */
  real_T Discrete_Time_Integrator_DSTA_h;/* '<S52>/Discrete_Time_Integrator' */
  real_T Discrete_Time_Integrator_DST_gd;/* '<S39>/Discrete_Time_Integrator' */
  real_T UnitDelay1_DSTATE_oj;         /* '<S47>/Unit Delay1' */
  real_T UD_DSTATE_du;                 /* '<S45>/UD' */
  real_T UnitDelay2_DSTATE_n;          /* '<S47>/Unit Delay2' */
  real_T Discrete_Time_Integrator_DSTA_o;/* '<S38>/Discrete_Time_Integrator' */
  real_T UnitDelay1_DSTATE_g;          /* '<S33>/Unit Delay1' */
  real_T UnitDelay2_DSTATE_bp;         /* '<S33>/Unit Delay2' */
  real_T UD_DSTATE_a;                  /* '<S31>/UD' */
  real_T Discrete_Time_Integrator_DSTA_k;/* '<S20>/Discrete_Time_Integrator' */
  real_T Discrete_Time_Integrator_DSTA_f;/* '<S19>/Discrete_Time_Integrator' */
  real_T time1;                        /* '<S291>/Chart' */
  real_T time2;                        /* '<S291>/Chart' */
  real_T time3;                        /* '<S291>/Chart' */
  real_T del_vel_step;                 /* '<S159>/takeoff' */
  real_T MC_YAWRATE_DZ;                /* '<S157>/stick_pos' */
  real_T VEL_Z_THOLD;                  /* '<S157>/stick_pos' */
  real_T POS_XY_THOLD;                 /* '<S157>/stick_pos' */
  real_T MC_YAWRATE_DZ_m;              /* '<S155>/stick_att' */
  real_T MC_YAWRATE_DZ_b;              /* '<S154>/stick_alt' */
  real_T VEL_Z_THOLD_h;                /* '<S154>/stick_alt' */
  uint32_T temporalCounter_i1;         /* '<S159>/takeoff' */
  uint32_T m_bpIndex;                  /* '<S163>/1-D Lookup Table' */
  int16_T Discrete_Time_Integrator_PrevRe;/* '<S236>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_h;/* '<S235>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_e;/* '<S53>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_k;/* '<S52>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_j;/* '<S39>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_f;/* '<S38>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_i;/* '<S20>/Discrete_Time_Integrator' */
  int16_T Discrete_Time_Integrator_Prev_g;/* '<S19>/Discrete_Time_Integrator' */
  uint16_T is_active_c24_claw_ofp;     /* '<S291>/Chart' */
  uint16_T is_c24_claw_ofp;            /* '<S291>/Chart' */
  uint16_T is_In_Air;                  /* '<S291>/Chart' */
  uint16_T is_Maybe_Landed;            /* '<S291>/Chart' */
  uint16_T is_Ground_Contact;          /* '<S291>/Chart' */
  uint16_T is_active_c20_claw_ofp;     /* '<S160>/waypoint' */
  uint16_T is_c20_claw_ofp;            /* '<S160>/waypoint' */
  uint16_T is_Horizontal;              /* '<S160>/waypoint' */
  uint16_T is_Directional_axis;        /* '<S160>/waypoint' */
  uint16_T is_active_c19_claw_ofp;     /* '<S159>/takeoff' */
  uint16_T is_c19_claw_ofp;            /* '<S159>/takeoff' */
  uint16_T is_Vertical_axis;           /* '<S159>/takeoff' */
  uint16_T is_Horizontal_n;            /* '<S159>/takeoff' */
  uint16_T is_active_c18_claw_ofp;     /* '<S158>/stick_rate' */
  uint16_T is_c18_claw_ofp;            /* '<S158>/stick_rate' */
  uint16_T is_active_c17_claw_ofp;     /* '<S157>/stick_pos' */
  uint16_T is_c17_claw_ofp;            /* '<S157>/stick_pos' */
  uint16_T is_Horizontal_k;            /* '<S157>/stick_pos' */
  uint16_T is_Vertical_axis_p;         /* '<S157>/stick_pos' */
  uint16_T is_Directional_axis_f;      /* '<S157>/stick_pos' */
  uint16_T is_active_c16_claw_ofp;     /* '<S156>/StickManual' */
  uint16_T is_c16_claw_ofp;            /* '<S156>/StickManual' */
  uint16_T is_active_c15_claw_ofp;     /* '<S155>/stick_att' */
  uint16_T is_c15_claw_ofp;            /* '<S155>/stick_att' */
  uint16_T is_Directional_axis_b;      /* '<S155>/stick_att' */
  uint16_T is_active_c14_claw_ofp;     /* '<S154>/stick_alt' */
  uint16_T is_c14_claw_ofp;            /* '<S154>/stick_alt' */
  uint16_T is_Directional_axis_a;      /* '<S154>/stick_alt' */
  uint16_T is_Vertical_axis_j;         /* '<S154>/stick_alt' */
  uint16_T is_active_c22_claw_ofp;     /* '<S153>/Standby' */
  uint16_T is_c22_claw_ofp;            /* '<S153>/Standby' */
  uint16_T is_active_c8_claw_ofp;      /* '<S152>/point_nav' */
  uint16_T is_c8_claw_ofp;             /* '<S152>/point_nav' */
  uint16_T is_Horizontal_kb;           /* '<S152>/point_nav' */
  uint16_T is_active_c2_claw_ofp;      /* '<S151>/offboard' */
  uint16_T is_c2_claw_ofp;             /* '<S151>/offboard' */
  uint16_T is_active_c3_claw_ofp;      /* '<S150>/loiter' */
  uint16_T is_c3_claw_ofp;             /* '<S150>/loiter' */
  uint16_T is_active_c1_claw_ofp;      /* '<S149>/land' */
  uint16_T is_c1_claw_ofp;             /* '<S149>/land' */
  uint16_T is_Vertical_axis_p1;        /* '<S149>/land' */
  uint16_T is_active_c12_claw_ofp;     /* '<S148>/knob' */
  uint16_T is_c12_claw_ofp;            /* '<S148>/knob' */
} DW_claw_ofp_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Limit1;                 /* '<S9>/Limit1' */
  const real_T thr_delta;              /* '<S14>/Sum1' */
  const real_T UnitConversion;         /* '<S85>/Unit Conversion' */
  const real_T Sum;                    /* '<S89>/Sum' */
  const real_T Product1;               /* '<S90>/Product1' */
  const real_T Sum1;                   /* '<S90>/Sum1' */
  const real_T sqrt_n;                 /* '<S90>/sqrt' */
  const real_T Product2;               /* '<S86>/Product2' */
  const real_T Sum1_b;                 /* '<S86>/Sum1' */
  const real_T SinCos_o1;              /* '<S71>/SinCos' */
  const real_T SinCos_o2;              /* '<S71>/SinCos' */
  const real_T UnitConversion_j;       /* '<S107>/Unit Conversion' */
  const real_T Sum_b;                  /* '<S111>/Sum' */
  const real_T Product1_d;             /* '<S112>/Product1' */
  const real_T Sum1_d;                 /* '<S112>/Sum1' */
  const real_T sqrt_h;                 /* '<S112>/sqrt' */
  const real_T Product2_p;             /* '<S108>/Product2' */
  const real_T Sum1_o;                 /* '<S108>/Sum1' */
  const real_T SinCos_o1_d;            /* '<S93>/SinCos' */
  const real_T SinCos_o2_a;            /* '<S93>/SinCos' */
  const real_T UnitConversion_b;       /* '<S129>/Unit Conversion' */
  const real_T Sum_d;                  /* '<S133>/Sum' */
  const real_T Product1_c;             /* '<S134>/Product1' */
  const real_T Sum1_bw;                /* '<S134>/Sum1' */
  const real_T sqrt_k;                 /* '<S134>/sqrt' */
  const real_T Product2_o;             /* '<S130>/Product2' */
  const real_T Sum1_n;                 /* '<S130>/Sum1' */
  const real_T SinCos_o1_i;            /* '<S115>/SinCos' */
  const real_T SinCos_o2_g;            /* '<S115>/SinCos' */
  const real_T sw[2];                  /* '<S255>/sw' */
  const real_T sw1[2];                 /* '<S255>/sw1' */
  const real_T Gain1[2];               /* '<S261>/Gain1' */
  const real_T UnaryMinus2;            /* '<S252>/Unary Minus2' */
  const real_T UnaryMinus3;            /* '<S252>/Unary Minus3' */
  const real_T tau;                    /* '<S283>/tau' */
  const real_T q;                      /* '<S284>/Product2' */
  const real_T r;                      /* '<S284>/Add' */
  const real_T p;                      /* '<S284>/Gain' */
  const real_T d;                      /* '<S284>/Add1' */
  const real_T Add2;                   /* '<S284>/Add2' */
  const real_T Add3;                   /* '<S284>/Add3' */
  const real_T Product3;               /* '<S284>/Product3' */
  const real_T Product4;               /* '<S284>/Product4' */
  const real_T Product5;               /* '<S284>/Product5' */
  const real_T tau_m;                  /* '<S285>/tau' */
  const real_T q_o;                    /* '<S286>/Product2' */
  const real_T r_e;                    /* '<S286>/Add' */
  const real_T p_d;                    /* '<S286>/Gain' */
  const real_T d_e;                    /* '<S286>/Add1' */
  const real_T Add2_b;                 /* '<S286>/Add2' */
  const real_T Add3_g;                 /* '<S286>/Add3' */
  const real_T Product3_n;             /* '<S286>/Product3' */
  const real_T Product4_f;             /* '<S286>/Product4' */
  const real_T Product5_d;             /* '<S286>/Product5' */
  const real_T tau_c;                  /* '<S287>/tau' */
  const real_T q_l;                    /* '<S288>/Product2' */
  const real_T r_i;                    /* '<S288>/Add' */
  const real_T p_g;                    /* '<S288>/Gain' */
  const real_T d_l;                    /* '<S288>/Add1' */
  const real_T Add2_b1;                /* '<S288>/Add2' */
  const real_T Add3_c;                 /* '<S288>/Add3' */
  const real_T Product3_m;             /* '<S288>/Product3' */
  const real_T Product4_fh;            /* '<S288>/Product4' */
  const real_T Product5_i;             /* '<S288>/Product5' */
  const real_T Subtract;               /* '<S291>/Subtract' */
  const real_T Gain;                   /* '<S291>/Gain' */
  const real_T Subtract1;              /* '<S291>/Subtract1' */
  const real_T Gain2;                  /* '<S291>/Gain2' */
  const real_T Sum_e;                  /* '<S291>/Sum' */
  const real_T Sum1_bz;                /* '<S291>/Sum1' */
  const real_T UnitConversion_m;       /* '<S318>/Unit Conversion' */
  const real_T Sum_a;                  /* '<S322>/Sum' */
  const real_T Product1_h;             /* '<S323>/Product1' */
  const real_T Sum1_p;                 /* '<S323>/Sum1' */
  const real_T sqrt_i;                 /* '<S323>/sqrt' */
  const real_T Product2_c;             /* '<S319>/Product2' */
  const real_T Sum1_b3;                /* '<S319>/Sum1' */
  const real_T SinCos_o1_a;            /* '<S304>/SinCos' */
  const real_T SinCos_o2_o;            /* '<S304>/SinCos' */
  const real_T Product;                /* '<S326>/Product' */
  const real_T Sum_m;                  /* '<S326>/Sum' */
  const real_T wts2wts;                /* '<S326>/Product1' */
  const real_T Sum3;                   /* '<S326>/Sum3' */
  const real_T Sum2;                   /* '<S326>/Sum2' */
  const real_T uwts2wts;               /* '<S326>/Product4' */
  const real_T Product_m;              /* '<S327>/Product' */
  const real_T Sum_g;                  /* '<S327>/Sum' */
  const real_T wts2wts_c;              /* '<S327>/Product1' */
  const real_T Sum3_i;                 /* '<S327>/Sum3' */
  const real_T Sum2_l;                 /* '<S327>/Sum2' */
  const real_T uwts2wts_c;             /* '<S327>/Product4' */
  const real_T Product_e;              /* '<S328>/Product' */
  const real_T Sum_j;                  /* '<S328>/Sum' */
  const real_T wts2wts_j;              /* '<S328>/Product1' */
  const real_T Sum3_m;                 /* '<S328>/Sum3' */
  const real_T Sum2_f;                 /* '<S328>/Sum2' */
  const real_T uwts2wts_e;             /* '<S328>/Product4' */
  const real_T Zero1;                  /* '<S237>/Zero1' */
  const real_T Zero1_o;                /* '<S236>/Zero1' */
  const real_T Zero1_a;                /* '<S235>/Zero1' */
  const real_T tau_k;                  /* '<S61>/tau' */
  const real_T q_h;                    /* '<S62>/Product2' */
  const real_T r_m;                    /* '<S62>/Add' */
  const real_T p_m;                    /* '<S62>/Gain' */
  const real_T d_n;                    /* '<S62>/Add1' */
  const real_T Add2_a;                 /* '<S62>/Add2' */
  const real_T Add3_n;                 /* '<S62>/Add3' */
  const real_T Product3_o;             /* '<S62>/Product3' */
  const real_T Product4_e;             /* '<S62>/Product4' */
  const real_T Product5_c;             /* '<S62>/Product5' */
  const real_T Zero1_c;                /* '<S53>/Zero1' */
  const real_T Zero1_f;                /* '<S52>/Zero1' */
  const real_T tau_m0;                 /* '<S47>/tau' */
  const real_T q_f;                    /* '<S48>/Product2' */
  const real_T r_j;                    /* '<S48>/Add' */
  const real_T p_c;                    /* '<S48>/Gain' */
  const real_T d_m;                    /* '<S48>/Add1' */
  const real_T Add2_n;                 /* '<S48>/Add2' */
  const real_T Add3_o;                 /* '<S48>/Add3' */
  const real_T Product3_no;            /* '<S48>/Product3' */
  const real_T Product4_n;             /* '<S48>/Product4' */
  const real_T Product5_iz;            /* '<S48>/Product5' */
  const real_T Zero1_n;                /* '<S39>/Zero1' */
  const real_T Zero1_i;                /* '<S38>/Zero1' */
  const real_T tau_i;                  /* '<S33>/tau' */
  const real_T q_or;                   /* '<S34>/Product2' */
  const real_T r_k;                    /* '<S34>/Add' */
  const real_T p_p;                    /* '<S34>/Gain' */
  const real_T d_i;                    /* '<S34>/Add1' */
  const real_T Add2_ne;                /* '<S34>/Add2' */
  const real_T Add3_m;                 /* '<S34>/Add3' */
  const real_T Product3_p;             /* '<S34>/Product3' */
  const real_T Product4_b;             /* '<S34>/Product4' */
  const real_T Product5_a;             /* '<S34>/Product5' */
  const real_T Zero1_g;                /* '<S20>/Zero1' */
  const real_T Zero1_nd;               /* '<S19>/Zero1' */
} ConstB_claw_ofp_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [MPC_LAND_DIST2 MPC_LAND_DIST2 MPC_LAND_DIST1 MPC_LAND_DIST1]
   * Referenced by: '<S163>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[4];

  /* Expression: [0 MPC_LAND_ALT2 MPC_LAND_ALT1 MPC_LAND_ALT1+1]
   * Referenced by: '<S163>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[4];

  /* Expression: MC_MIXER
   * Referenced by: '<S9>/mixer'
   */
  real_T mixer_Value[64];
} ConstP_claw_ofp_T;

/* Real-time Model Data Structure */
struct tag_RTM_claw_ofp_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_claw_ofp_T claw_ofp_B;

/* Block states (default storage) */
extern DW_claw_ofp_T claw_ofp_DW;
extern const ConstB_claw_ofp_T claw_ofp_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_claw_ofp_T claw_ofp_ConstP;

/* Model entry point functions */
extern void claw_ofp_initialize(void);
extern void claw_ofp_step(void);
extern void claw_ofp_terminate(void);

/* Real-time Model object */
extern RT_MODEL_claw_ofp_T *const claw_ofp_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S25>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Data Type Propagation' : Unused code path elimination
 * Block '<S31>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S67>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate' : Unused code path elimination
 * Block '<S180>/Display' : Unused code path elimination
 * Block '<S195>/Data Type Duplicate' : Unused code path elimination
 * Block '<S195>/Data Type Propagation' : Unused code path elimination
 * Block '<S187>/Display' : Unused code path elimination
 * Block '<S212>/Gain1' : Unused code path elimination
 * Block '<S199>/Divide' : Unused code path elimination
 * Block '<S199>/Gain' : Unused code path elimination
 * Block '<S199>/Math Function' : Unused code path elimination
 * Block '<S214>/Compare' : Unused code path elimination
 * Block '<S214>/Constant' : Unused code path elimination
 * Block '<S213>/Switch' : Unused code path elimination
 * Block '<S213>/Zero' : Unused code path elimination
 * Block '<S199>/Sqrt' : Unused code path elimination
 * Block '<S199>/Sum' : Unused code path elimination
 * Block '<S199>/Sum1' : Unused code path elimination
 * Block '<S199>/Sum2' : Unused code path elimination
 * Block '<S199>/Tau' : Unused code path elimination
 * Block '<S199>/gain' : Unused code path elimination
 * Block '<S200>/Data Type Duplicate' : Unused code path elimination
 * Block '<S200>/Data Type Propagation' : Unused code path elimination
 * Block '<S218>/Data Type Duplicate' : Unused code path elimination
 * Block '<S218>/Data Type Propagation' : Unused code path elimination
 * Block '<S190>/Display' : Unused code path elimination
 * Block '<S224>/Data Type Duplicate' : Unused code path elimination
 * Block '<S224>/Data Type Propagation' : Unused code path elimination
 * Block '<S243>/Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Duplicate' : Unused code path elimination
 * Block '<S257>/Data Type Duplicate' : Unused code path elimination
 * Block '<S257>/Data Type Propagation' : Unused code path elimination
 * Block '<S258>/Data Type Duplicate' : Unused code path elimination
 * Block '<S258>/Data Type Propagation' : Unused code path elimination
 * Block '<S265>/Data Type Duplicate' : Unused code path elimination
 * Block '<S265>/Data Type Propagation' : Unused code path elimination
 * Block '<S266>/Data Type Duplicate' : Unused code path elimination
 * Block '<S266>/Data Type Propagation' : Unused code path elimination
 * Block '<S267>/Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Propagation' : Unused code path elimination
 * Block '<S299>/Data Type Duplicate' : Unused code path elimination
 * Block '<S301>/Display' : Unused code path elimination
 * Block '<S301>/Display1' : Unused code path elimination
 * Block '<S301>/Display2' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope10' : Unused code path elimination
 * Block '<Root>/Scope11' : Unused code path elimination
 * Block '<Root>/Scope12' : Unused code path elimination
 * Block '<Root>/Scope13' : Unused code path elimination
 * Block '<Root>/Scope14' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope4' : Unused code path elimination
 * Block '<Root>/Scope5' : Unused code path elimination
 * Block '<Root>/Scope6' : Unused code path elimination
 * Block '<Root>/Scope7' : Unused code path elimination
 * Block '<Root>/Scope8' : Unused code path elimination
 * Block '<Root>/Scope9' : Unused code path elimination
 * Block '<S9>/Bias1' : Eliminated nontunable bias of 0
 * Block '<S9>/Bias2' : Eliminated nontunable bias of 0
 * Block '<S9>/Bias3' : Eliminated nontunable bias of 0
 * Block '<S9>/Gain4' : Eliminated nontunable gain of 1
 * Block '<S9>/Gain5' : Eliminated nontunable gain of 1
 * Block '<S9>/Gain6' : Eliminated nontunable gain of 1
 * Block '<S9>/Gain7' : Eliminated nontunable gain of 1
 * Block '<S10>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion10' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion12' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion13' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion23' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion26' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S10>/Data Type Conversion7' : Eliminate redundant data type conversion
 * Block '<S39>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S53>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S65>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S65>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S65>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S65>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S193>/Reshape3' : Reshape block reduction
 * Block '<S216>/Reshape3' : Reshape block reduction
 * Block '<S189>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S189>/Gain2' : Eliminated nontunable gain of 1
 * Block '<S181>/Data Type Conversion11' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion13' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion15' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion16' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion19' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion23' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion24' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion25' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion26' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion7' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion8' : Eliminate redundant data type conversion
 * Block '<S181>/Data Type Conversion9' : Eliminate redundant data type conversion
 * Block '<S278>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S278>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S278>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S190>/Bias' : Unused code path elimination
 * Block '<S190>/Bias1' : Unused code path elimination
 * Block '<S190>/Constant' : Unused code path elimination
 * Block '<S232>/Gain' : Unused code path elimination
 * Block '<S223>/Trigonometric Function3' : Unused code path elimination
 * Block '<S190>/Switch1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'claw_ofp'
 * '<S1>'   : 'claw_ofp/Model Info'
 * '<S2>'   : 'claw_ofp/claw_core_main'
 * '<S3>'   : 'claw_ofp/claw_core_main/att_control'
 * '<S4>'   : 'claw_ofp/claw_core_main/cmd_generation'
 * '<S5>'   : 'claw_ofp/claw_core_main/mode_commander'
 * '<S6>'   : 'claw_ofp/claw_core_main/pos_control'
 * '<S7>'   : 'claw_ofp/claw_core_main/state_estimator'
 * '<S8>'   : 'claw_ofp/claw_core_main/test_generation'
 * '<S9>'   : 'claw_ofp/claw_core_main/att_control/ControlAllocation'
 * '<S10>'  : 'claw_ofp/claw_core_main/att_control/Flight_Control_OutputData'
 * '<S11>'  : 'claw_ofp/claw_core_main/att_control/dir_axis'
 * '<S12>'  : 'claw_ofp/claw_core_main/att_control/lat_axis'
 * '<S13>'  : 'claw_ofp/claw_core_main/att_control/lon_axis'
 * '<S14>'  : 'claw_ofp/claw_core_main/att_control/vert_axis'
 * '<S15>'  : 'claw_ofp/claw_core_main/att_control/ControlAllocation/Control_force_mixer'
 * '<S16>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_hdg'
 * '<S17>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_acc'
 * '<S18>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_rate'
 * '<S19>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/hdg_control'
 * '<S20>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control'
 * '<S21>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_hdg/If Not Manual Mode1'
 * '<S22>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_acc/If Not Manual Mode1'
 * '<S23>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_rate/If Heading control mode'
 * '<S24>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_rate/If Heading control with rate-limit'
 * '<S25>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/cmd_sw_yaw_rate/Saturation Dynamic'
 * '<S26>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/hdg_control/If Not Manual Mode'
 * '<S27>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/hdg_control/ShortHdg'
 * '<S28>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/hdg_control/ShortHdg/InCaseGuidance'
 * '<S29>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/hdg_control/ShortHdg/InCaseGuidance1'
 * '<S30>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control/Butterworth Lowpass'
 * '<S31>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control/Discrete Derivative'
 * '<S32>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control/If rate mode'
 * '<S33>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control/Butterworth Lowpass/SOS1'
 * '<S34>'  : 'claw_ofp/claw_core_main/att_control/dir_axis/yaw_rate_control/Butterworth Lowpass/SOS1/Arithmetic'
 * '<S35>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_acc'
 * '<S36>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_angle'
 * '<S37>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_rate'
 * '<S38>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_control'
 * '<S39>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control'
 * '<S40>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_acc/If Not Manual Mode1'
 * '<S41>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_angle/If Not Manual Mode1'
 * '<S42>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/cmd_sw_roll_rate/If Not Manual Mode1'
 * '<S43>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_control/If Not Manual Mode'
 * '<S44>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control/Butterworth Lowpass'
 * '<S45>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control/Discrete Derivative'
 * '<S46>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control/If Not Manual Mode'
 * '<S47>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control/Butterworth Lowpass/SOS1'
 * '<S48>'  : 'claw_ofp/claw_core_main/att_control/lat_axis/roll_rate_control/Butterworth Lowpass/SOS1/Arithmetic'
 * '<S49>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_pitch'
 * '<S50>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_pitch_acc'
 * '<S51>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_q'
 * '<S52>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_control'
 * '<S53>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control'
 * '<S54>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_pitch/If Not Manual Mode1'
 * '<S55>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_pitch_acc/If Not Manual Mode1'
 * '<S56>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/cmd_sw_q/If Not Manual Mode1'
 * '<S57>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_control/If Not Manual Mode'
 * '<S58>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control/Butterworth Lowpass'
 * '<S59>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control/Discrete Derivative'
 * '<S60>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control/If Not Manual Mode'
 * '<S61>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control/Butterworth Lowpass/SOS1'
 * '<S62>'  : 'claw_ofp/claw_core_main/att_control/lon_axis/pitch_rate_control/Butterworth Lowpass/SOS1/Arithmetic'
 * '<S63>'  : 'claw_ofp/claw_core_main/att_control/vert_axis/throttle_sw'
 * '<S64>'  : 'claw_ofp/claw_core_main/att_control/vert_axis/throttle_sw/If Not Manual Mode1'
 * '<S65>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position'
 * '<S66>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth'
 * '<S67>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1'
 * '<S68>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2'
 * '<S69>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap'
 * '<S70>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0'
 * '<S71>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem'
 * '<S72>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/pos_rad'
 * '<S73>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90'
 * '<S74>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Wrap Longitude'
 * '<S75>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S76>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S77>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S78>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S79>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90'
 * '<S80>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Wrap Longitude'
 * '<S81>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Compare To Constant'
 * '<S82>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180'
 * '<S83>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S84>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/LatLong wrap LL0/Wrap Longitude/Compare To Constant'
 * '<S85>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Angle Conversion2'
 * '<S86>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance'
 * '<S87>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/Angle Conversion2'
 * '<S88>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/denom'
 * '<S89>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/e'
 * '<S90>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/e^4'
 * '<S91>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap'
 * '<S92>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0'
 * '<S93>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem'
 * '<S94>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/pos_rad'
 * '<S95>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90'
 * '<S96>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Wrap Longitude'
 * '<S97>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S98>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S99>'  : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S100>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S101>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90'
 * '<S102>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Wrap Longitude'
 * '<S103>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Compare To Constant'
 * '<S104>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180'
 * '<S105>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S106>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/LatLong wrap LL0/Wrap Longitude/Compare To Constant'
 * '<S107>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Angle Conversion2'
 * '<S108>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Find Radian//Distance'
 * '<S109>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Find Radian//Distance/Angle Conversion2'
 * '<S110>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Find Radian//Distance/denom'
 * '<S111>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Find Radian//Distance/e'
 * '<S112>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth1/Subsystem/Find Radian//Distance/e^4'
 * '<S113>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap'
 * '<S114>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0'
 * '<S115>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem'
 * '<S116>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/pos_rad'
 * '<S117>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Latitude Wrap 90'
 * '<S118>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Wrap Longitude'
 * '<S119>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S120>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S121>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S122>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S123>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Latitude Wrap 90'
 * '<S124>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Wrap Longitude'
 * '<S125>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Latitude Wrap 90/Compare To Constant'
 * '<S126>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180'
 * '<S127>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S128>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/LatLong wrap LL0/Wrap Longitude/Compare To Constant'
 * '<S129>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Angle Conversion2'
 * '<S130>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Find Radian//Distance'
 * '<S131>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Find Radian//Distance/Angle Conversion2'
 * '<S132>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Find Radian//Distance/denom'
 * '<S133>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Find Radian//Distance/e'
 * '<S134>' : 'claw_ofp/claw_core_main/cmd_generation/cmd_local_position/LLA to Flat Earth2/Subsystem/Find Radian//Distance/e^4'
 * '<S135>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant1'
 * '<S136>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant10'
 * '<S137>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant11'
 * '<S138>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant12'
 * '<S139>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant13'
 * '<S140>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant2'
 * '<S141>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant3'
 * '<S142>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant4'
 * '<S143>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant5'
 * '<S144>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant6'
 * '<S145>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant7'
 * '<S146>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant8'
 * '<S147>' : 'claw_ofp/claw_core_main/mode_commander/Compare To Constant9'
 * '<S148>' : 'claw_ofp/claw_core_main/mode_commander/mode_knob'
 * '<S149>' : 'claw_ofp/claw_core_main/mode_commander/mode_land'
 * '<S150>' : 'claw_ofp/claw_core_main/mode_commander/mode_loiter'
 * '<S151>' : 'claw_ofp/claw_core_main/mode_commander/mode_offboard'
 * '<S152>' : 'claw_ofp/claw_core_main/mode_commander/mode_pointnav'
 * '<S153>' : 'claw_ofp/claw_core_main/mode_commander/mode_standby'
 * '<S154>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_alt'
 * '<S155>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_att'
 * '<S156>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_manual'
 * '<S157>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_pos'
 * '<S158>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_rate'
 * '<S159>' : 'claw_ofp/claw_core_main/mode_commander/mode_takeoff'
 * '<S160>' : 'claw_ofp/claw_core_main/mode_commander/mode_waypoint'
 * '<S161>' : 'claw_ofp/claw_core_main/mode_commander/mode_knob/knob'
 * '<S162>' : 'claw_ofp/claw_core_main/mode_commander/mode_land/land'
 * '<S163>' : 'claw_ofp/claw_core_main/mode_commander/mode_land/land/Active.Vertical_axis.Check_boundary'
 * '<S164>' : 'claw_ofp/claw_core_main/mode_commander/mode_land/land/Active.Vertical_axis.Vz_commander'
 * '<S165>' : 'claw_ofp/claw_core_main/mode_commander/mode_land/land/Active.Vertical_axis.Vz_commander/Compare To Constant'
 * '<S166>' : 'claw_ofp/claw_core_main/mode_commander/mode_land/land/Active.Vertical_axis.Vz_commander/Compare To Constant1'
 * '<S167>' : 'claw_ofp/claw_core_main/mode_commander/mode_loiter/loiter'
 * '<S168>' : 'claw_ofp/claw_core_main/mode_commander/mode_offboard/offboard'
 * '<S169>' : 'claw_ofp/claw_core_main/mode_commander/mode_pointnav/point_nav'
 * '<S170>' : 'claw_ofp/claw_core_main/mode_commander/mode_pointnav/point_nav/Active.Horizontal.dist2d'
 * '<S171>' : 'claw_ofp/claw_core_main/mode_commander/mode_standby/Standby'
 * '<S172>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_alt/stick_alt'
 * '<S173>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_att/stick_att'
 * '<S174>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_manual/StickManual'
 * '<S175>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_pos/stick_pos'
 * '<S176>' : 'claw_ofp/claw_core_main/mode_commander/mode_stick_rate/stick_rate'
 * '<S177>' : 'claw_ofp/claw_core_main/mode_commander/mode_takeoff/takeoff'
 * '<S178>' : 'claw_ofp/claw_core_main/mode_commander/mode_waypoint/waypoint'
 * '<S179>' : 'claw_ofp/claw_core_main/mode_commander/mode_waypoint/waypoint/Active.Horizontal.dist2d'
 * '<S180>' : 'claw_ofp/claw_core_main/pos_control/Guidance'
 * '<S181>' : 'claw_ofp/claw_core_main/pos_control/Subsystem'
 * '<S182>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control'
 * '<S183>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform'
 * '<S184>' : 'claw_ofp/claw_core_main/pos_control/cmd sw acc'
 * '<S185>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel'
 * '<S186>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint'
 * '<S187>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit'
 * '<S188>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold XY'
 * '<S189>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold Z'
 * '<S190>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg'
 * '<S191>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint/Compare To Constant'
 * '<S192>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint/D2R'
 * '<S193>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint/NE2XY'
 * '<S194>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint/R2D'
 * '<S195>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Fly To Waypoint/Saturation Dynamic'
 * '<S196>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Command_gen'
 * '<S197>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Compare To Constant'
 * '<S198>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance'
 * '<S199>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Radius_Compensator'
 * '<S200>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Saturation Dynamic'
 * '<S201>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Command_gen/D2R'
 * '<S202>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Command_gen/D2R1'
 * '<S203>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Command_gen/NE2XY'
 * '<S204>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Command_gen/Subsystem'
 * '<S205>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/AV_WP_Hdg'
 * '<S206>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/CrossTrackError'
 * '<S207>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/OrbitAlgorithm'
 * '<S208>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/OrbitDirectionSW'
 * '<S209>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/OrbitDirectionSW1'
 * '<S210>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/R2D'
 * '<S211>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Orbit_Guidance/WP_Leg_Hdg'
 * '<S212>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Radius_Compensator/D2R'
 * '<S213>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Radius_Compensator/Negative_Check'
 * '<S214>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Orbit/Radius_Compensator/Negative_Check/Compare To Zero'
 * '<S215>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold XY/D2R'
 * '<S216>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold XY/NE2XY'
 * '<S217>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold XY/R2D'
 * '<S218>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Position Hold Z/Saturation Dynamic'
 * '<S219>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Command_gen'
 * '<S220>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Compare To Constant'
 * '<S221>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Compare To Constant1'
 * '<S222>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Compare To Constant2'
 * '<S223>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Geometry'
 * '<S224>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Saturation Dynamic'
 * '<S225>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Command_gen/D2R'
 * '<S226>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Command_gen/D2R1'
 * '<S227>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Command_gen/NE2XY'
 * '<S228>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Command_gen/Subsystem'
 * '<S229>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Geometry/NE2XY'
 * '<S230>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Geometry/R2D'
 * '<S231>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Geometry/R2D1'
 * '<S232>' : 'claw_ofp/claw_core_main/pos_control/Guidance/Track_leg/Geometry/R2D2'
 * '<S233>' : 'claw_ofp/claw_core_main/pos_control/Subsystem/Compare To Zero'
 * '<S234>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/If Not Manual Mode1'
 * '<S235>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF1'
 * '<S236>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF2'
 * '<S237>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF3'
 * '<S238>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/condition4'
 * '<S239>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/condition5'
 * '<S240>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF1/Compare To Constant'
 * '<S241>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF1/Compare To Constant1'
 * '<S242>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF1/Discrete 1st LPF(Tustin)'
 * '<S243>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF1/Discrete Derivative'
 * '<S244>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF2/Compare To Constant'
 * '<S245>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF2/Compare To Constant1'
 * '<S246>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF2/Discrete 1st LPF(Tustin)'
 * '<S247>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF2/Discrete Derivative'
 * '<S248>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF3/Compare To Constant'
 * '<S249>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF3/Compare To Constant1'
 * '<S250>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF3/Discrete 1st LPF(Tustin)'
 * '<S251>' : 'claw_ofp/claw_core_main/pos_control/Velocity Control/PID_with_FF3/Discrete Derivative'
 * '<S252>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body'
 * '<S253>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/R2D'
 * '<S254>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/R2D1'
 * '<S255>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/max tilt'
 * '<S256>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body/AntiZero'
 * '<S257>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body/Saturation Dynamic'
 * '<S258>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body/Saturation Dynamic1'
 * '<S259>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body/AntiZero/Compare To Constant'
 * '<S260>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/Acc2Body/AntiZero/Compare To Constant1'
 * '<S261>' : 'claw_ofp/claw_core_main/pos_control/attitude_transform/max tilt/D2R'
 * '<S262>' : 'claw_ofp/claw_core_main/pos_control/cmd sw acc/If Not Manual Mode1'
 * '<S263>' : 'claw_ofp/claw_core_main/pos_control/cmd sw acc/condition1'
 * '<S264>' : 'claw_ofp/claw_core_main/pos_control/cmd sw acc/condition4'
 * '<S265>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/Saturation Dynamic'
 * '<S266>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/Saturation Dynamic1'
 * '<S267>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/Saturation Dynamic2'
 * '<S268>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode1'
 * '<S269>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode2'
 * '<S270>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode3'
 * '<S271>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode4'
 * '<S272>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode5'
 * '<S273>' : 'claw_ofp/claw_core_main/pos_control/cmd sw vel/if pos mode6'
 * '<S274>' : 'claw_ofp/claw_core_main/state_estimator/Filters'
 * '<S275>' : 'claw_ofp/claw_core_main/state_estimator/States_parsing'
 * '<S276>' : 'claw_ofp/claw_core_main/state_estimator/body_axis'
 * '<S277>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection'
 * '<S278>' : 'claw_ofp/claw_core_main/state_estimator/local_position'
 * '<S279>' : 'claw_ofp/claw_core_main/state_estimator/path_angles'
 * '<S280>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass1'
 * '<S281>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass2'
 * '<S282>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass3'
 * '<S283>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass1/SOS1'
 * '<S284>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass1/SOS1/Arithmetic'
 * '<S285>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass2/SOS1'
 * '<S286>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass2/SOS1/Arithmetic'
 * '<S287>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass3/SOS1'
 * '<S288>' : 'claw_ofp/claw_core_main/state_estimator/Filters/Butterworth Lowpass3/SOS1/Arithmetic'
 * '<S289>' : 'claw_ofp/claw_core_main/state_estimator/body_axis/D2R'
 * '<S290>' : 'claw_ofp/claw_core_main/state_estimator/body_axis/NE2XY'
 * '<S291>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector'
 * '<S292>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Chart'
 * '<S293>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant1'
 * '<S294>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant2'
 * '<S295>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant3'
 * '<S296>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant4'
 * '<S297>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant5'
 * '<S298>' : 'claw_ofp/claw_core_main/state_estimator/landing_detection/landing_detector/Compare To Constant6'
 * '<S299>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth'
 * '<S300>' : 'claw_ofp/claw_core_main/state_estimator/local_position/ShortHdg'
 * '<S301>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection'
 * '<S302>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap'
 * '<S303>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0'
 * '<S304>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem'
 * '<S305>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/pos_rad'
 * '<S306>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90'
 * '<S307>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Wrap Longitude'
 * '<S308>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S309>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S310>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S311>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S312>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90'
 * '<S313>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Wrap Longitude'
 * '<S314>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Compare To Constant'
 * '<S315>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180'
 * '<S316>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S317>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/LatLong wrap LL0/Wrap Longitude/Compare To Constant'
 * '<S318>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Angle Conversion2'
 * '<S319>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance'
 * '<S320>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/Angle Conversion2'
 * '<S321>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/denom'
 * '<S322>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/e'
 * '<S323>' : 'claw_ofp/claw_core_main/state_estimator/local_position/LLA to Flat Earth/Subsystem/Find Radian//Distance/e^4'
 * '<S324>' : 'claw_ofp/claw_core_main/state_estimator/local_position/ShortHdg/InCaseGuidance'
 * '<S325>' : 'claw_ofp/claw_core_main/state_estimator/local_position/ShortHdg/InCaseGuidance1'
 * '<S326>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection/AGL_sensor_filter'
 * '<S327>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection/Baro_sensor_Filter'
 * '<S328>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection/Delta_Filter'
 * '<S329>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection/Enabled Subsystem1'
 * '<S330>' : 'claw_ofp/claw_core_main/state_estimator/local_position/altitude_selection/Enabled Subsystem2'
 * '<S331>' : 'claw_ofp/claw_core_main/state_estimator/path_angles/DataConversion'
 * '<S332>' : 'claw_ofp/claw_core_main/state_estimator/path_angles/DataConversion/Angle Conversion'
 * '<S333>' : 'claw_ofp/claw_core_main/state_estimator/path_angles/DataConversion/Angle Conversion1'
 * '<S334>' : 'claw_ofp/claw_core_main/state_estimator/path_angles/DataConversion/HdgConversion'
 * '<S335>' : 'claw_ofp/claw_core_main/state_estimator/path_angles/DataConversion/HdgConversion/InCase'
 * '<S336>' : 'claw_ofp/claw_core_main/test_generation/check_range'
 * '<S337>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus'
 * '<S338>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus1'
 * '<S339>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus2'
 * '<S340>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus3'
 * '<S341>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus4'
 * '<S342>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus5'
 * '<S343>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus6'
 * '<S344>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus7'
 * '<S345>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus8'
 * '<S346>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus9'
 * '<S347>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus/Compare To Constant'
 * '<S348>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus/Compare To Constant1'
 * '<S349>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus1/Compare To Constant'
 * '<S350>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus1/Compare To Constant1'
 * '<S351>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus2/Compare To Constant'
 * '<S352>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus2/Compare To Constant1'
 * '<S353>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus3/Compare To Constant'
 * '<S354>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus3/Compare To Constant1'
 * '<S355>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus4/Compare To Constant'
 * '<S356>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus4/Compare To Constant1'
 * '<S357>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus5/Compare To Constant'
 * '<S358>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus5/Compare To Constant1'
 * '<S359>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus6/Compare To Constant'
 * '<S360>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus6/Compare To Constant1'
 * '<S361>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus7/Compare To Constant'
 * '<S362>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus7/Compare To Constant1'
 * '<S363>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus8/Compare To Constant'
 * '<S364>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus8/Compare To Constant1'
 * '<S365>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus9/Compare To Constant'
 * '<S366>' : 'claw_ofp/claw_core_main/test_generation/check_range/check_bus9/Compare To Constant1'
 */
#endif                                 /* RTW_HEADER_claw_ofp_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
