/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file firmwares/rotorcraft/guidance/guidance_v.c
 *  Vertical guidance for rotorcrafts.
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"

#include "math/pprz_algebra_int.h"


/* error if some gains are negative */
#if (GUIDANCE_V_HOVER_KP < 0) ||                   \
  (GUIDANCE_V_HOVER_KD < 0)   ||                   \
  (GUIDANCE_V_HOVER_KI < 0)
#error "ALL control gains must be positive!!!"
#endif


/* If only GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined,
 * disable the adaptive throttle estimation by default.
 * Otherwise enable adaptive estimation by default.
 */
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED FALSE
#  endif
#else
#  define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.4
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED TRUE
#  endif
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_NOMINAL_HOVER_THROTTLE)
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_THROTTLE_ENABLED)


#ifndef GUIDANCE_V_CLIMB_RC_DEADBAND
#define GUIDANCE_V_CLIMB_RC_DEADBAND MAX_PPRZ/10
#endif

#ifndef GUIDANCE_V_MAX_RC_CLIMB_SPEED
#define GUIDANCE_V_MAX_RC_CLIMB_SPEED GUIDANCE_V_REF_MIN_ZD
#endif

#ifndef GUIDANCE_V_MAX_RC_DESCENT_SPEED
#define GUIDANCE_V_MAX_RC_DESCENT_SPEED GUIDANCE_V_REF_MAX_ZD
#endif

#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR 2000000
#endif

uint8_t guidance_v_mode;
int32_t guidance_v_ff_cmd;
int32_t guidance_v_fb_cmd;
int32_t guidance_v_delta_t;

float guidance_v_nominal_throttle;
bool_t guidance_v_adapt_throttle_enabled;





/* HELI INDI STUFF */
#include "filters/heli_rate_filter.h"
#include "subsystems/sensors/rpm_sensor.h"

#define HELI_INDI_ACCEL_Z_FILTSIZE 16
struct SecondOrderNotchFilter accel_z_notchfilter;
struct SecondOrderNotchFilter thrust_actuator_inner_notchfilter;
int32_t inner_notch_output;
int32_t inner_iir_output;
int32_t accel_z_raw;
int32_t accel_z_notched;
int32_t accel_z_filtered;
int32_t previous_accel_z;
struct heli_rate_filter_t thrust_model;

int32_t global_new_thrust;
int32_t global_old_thrust;

/** Direct throttle from radio control.
 *  range 0:#MAX_PPRZ
 */
int32_t guidance_v_rc_delta_t;

/** Vertical speed setpoint from radio control.
 *  fixed point representation: Q12.19
 *  accuracy 0.0000019, range +/-4096
 */
int32_t guidance_v_rc_zd_sp;

int32_t guidance_v_z_sp;
int32_t guidance_v_zd_sp;
int32_t guidance_v_z_ref;
int32_t guidance_v_zd_ref;
int32_t guidance_v_zdd_ref;

int32_t guidance_v_kp;
int32_t guidance_v_kd;
int32_t guidance_v_ki;

int32_t guidance_v_z_sum_err;

int32_t guidance_v_thrust_coeff;


#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);        \
    guidance_v_z_ref = _pos;             \
    guidance_v_zd_ref = _speed;          \
    guidance_v_zdd_ref = _accel;             \
  }

static int32_t get_vertical_thrust_coeff(void);
static void run_hover_loop(bool_t in_flight);
static void run_indi_loop(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_vert_loop(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VERT_LOOP(trans, dev, AC_ID,
                          &guidance_v_z_sp, &guidance_v_zd_sp,
                          &(stateGetPositionNed_i()->z),
                          &(stateGetSpeedNed_i()->z),
                          &(stateGetAccelNed_i()->z),
                          &guidance_v_z_ref, &guidance_v_zd_ref,
                          &guidance_v_zdd_ref,
                          &gv_adapt_X,
                          &gv_adapt_P,
                          &gv_adapt_Xmeas,
                          &guidance_v_z_sum_err,
                          &guidance_v_ff_cmd,
                          &guidance_v_fb_cmd,
                          &guidance_v_delta_t);
}

static void send_tune_vert(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TUNE_VERT(trans, dev, AC_ID,
                          &global_old_thrust,
                          &accel_z_filtered,
                          &guidance_v_delta_t,
                          &guidance_v_thrust_coeff);
}
#endif

void guidance_v_init(void)
{

  guidance_v_mode = GUIDANCE_V_MODE_KILL;

  guidance_v_kp = GUIDANCE_V_HOVER_KP;
  guidance_v_kd = GUIDANCE_V_HOVER_KD;
  guidance_v_ki = GUIDANCE_V_HOVER_KI;

  guidance_v_z_sum_err = 0;

  guidance_v_nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
  guidance_v_adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

  gv_adapt_init();

  /* Init INDI STUFF */

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
  guidance_v_module_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VERT_LOOP, send_vert_loop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TUNE_VERT, send_tune_vert);
#endif
}


void guidance_v_read_rc(void)
{

  /* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
  guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE];

  /* used in RC_CLIMB */
  guidance_v_rc_zd_sp = (MAX_PPRZ / 2) - (int32_t)radio_control.values[RADIO_THROTTLE];
  DeadBand(guidance_v_rc_zd_sp, GUIDANCE_V_CLIMB_RC_DEADBAND);

  static const int32_t climb_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_CLIMB_SPEED) /
                                         (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
  static const int32_t descent_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_DESCENT_SPEED) /
                                       (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));

  if (guidance_v_rc_zd_sp > 0) {
    guidance_v_rc_zd_sp *= descent_scale;
  } else {
    guidance_v_rc_zd_sp *= climb_scale;
  }
}

void guidance_v_mode_changed(uint8_t new_mode)
{

  if (new_mode == guidance_v_mode) {
    return;
  }

  switch (new_mode) {
    case GUIDANCE_V_MODE_HOVER:
    case GUIDANCE_V_MODE_GUIDED:
      guidance_v_z_sp = stateGetPositionNed_i()->z; // set current altitude as setpoint
      guidance_v_z_sum_err = 0;
      GuidanceVSetRef(stateGetPositionNed_i()->z, 0, 0);
      break;

    case GUIDANCE_V_MODE_RC_CLIMB:
    case GUIDANCE_V_MODE_CLIMB:
      guidance_v_zd_sp = 0;
    case GUIDANCE_V_MODE_NAV:
      guidance_v_z_sum_err = 0;
      GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
      break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
    case GUIDANCE_V_MODE_MODULE:
      guidance_v_module_enter();
      break;
#endif

    case GUIDANCE_V_MODE_FLIP:
      break;

    default:
      break;

  }

  guidance_v_mode = new_mode;

}

void guidance_v_notify_in_flight(bool_t in_flight)
{
  if (in_flight) {
    gv_adapt_init();
  }
}


void guidance_v_run(bool_t in_flight)
{

  // FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
  // AKA SUPERVISION and co
  guidance_v_thrust_coeff = get_vertical_thrust_coeff();
  if (in_flight) {
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    gv_adapt_run(stateGetAccelNed_i()->z, vertical_thrust, guidance_v_zd_ref);
  } else {
    /* reset estimate while not in_flight */
    gv_adapt_init();
  }

  switch (guidance_v_mode) {

    case GUIDANCE_V_MODE_HELI_INDI:
    case GUIDANCE_V_MODE_RC_DIRECT:
      guidance_v_z_sp = stateGetPositionNed_i()->z; // for display only
      stabilization_cmd[COMMAND_THRUST] = guidance_v_rc_delta_t;
      break;

    case GUIDANCE_V_MODE_RC_CLIMB:
      guidance_v_zd_sp = guidance_v_rc_zd_sp;
      gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
      run_hover_loop(in_flight);
      stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
      break;

    case GUIDANCE_V_MODE_CLIMB:
      gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
      run_hover_loop(in_flight);
#if !NO_RC_THRUST_LIMIT
      /* use rc limitation if available */
      if (radio_control.status == RC_OK) {
        stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
      } else
#endif
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
      break;

    case GUIDANCE_V_MODE_HOVER:
    case GUIDANCE_V_MODE_GUIDED:
      guidance_v_zd_sp = 0;
      gv_update_ref_from_z_sp(guidance_v_z_sp);
      run_hover_loop(in_flight);
#if !NO_RC_THRUST_LIMIT
      /* use rc limitation if available */
      if (radio_control.status == RC_OK) {
        stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
      } else
#endif
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
      break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
    case GUIDANCE_V_MODE_MODULE:
      guidance_v_module_run(in_flight);
      break;
#endif

    case GUIDANCE_V_MODE_NAV: {
      if (vertical_mode == VERTICAL_MODE_ALT) {
        guidance_v_z_sp = -nav_flight_altitude;
        guidance_v_zd_sp = 0;
        gv_update_ref_from_z_sp(guidance_v_z_sp);
        run_hover_loop(in_flight);
      } else if (vertical_mode == VERTICAL_MODE_CLIMB) {
        guidance_v_z_sp = stateGetPositionNed_i()->z;
        guidance_v_zd_sp = -nav_climb;
        gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
        run_hover_loop(in_flight);
      } else if (vertical_mode == VERTICAL_MODE_MANUAL) {
        guidance_v_z_sp = stateGetPositionNed_i()->z;
        guidance_v_zd_sp = stateGetSpeedNed_i()->z;
        GuidanceVSetRef(guidance_v_z_sp, guidance_v_zd_sp, 0);
        guidance_v_z_sum_err = 0;
        guidance_v_delta_t = nav_throttle;
      }
#if !NO_RC_THRUST_LIMIT
      /* use rc limitation if available */
      if (radio_control.status == RC_OK) {
        stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
      } else
#endif
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
      break;
    }

    case GUIDANCE_V_MODE_FLIP:
      break;

    default:
      break;
  }
}

/// get the cosine of the angle between thrust vector and gravity vector
static int32_t get_vertical_thrust_coeff(void)
{
  // cos(30°) = 0.8660254
  static const int32_t max_bank_coef = BFP_OF_REAL(0.8660254f, INT32_TRIG_FRAC);

  struct Int32RMat *att = stateGetNedToBodyRMat_i();
  /* thrust vector:
   *  int32_rmat_vmult(&thrust_vect, &att, &zaxis)
   * same as last colum of rmat with INT32_TRIG_FRAC
   * struct Int32Vect thrust_vect = {att.m[2], att.m[5], att.m[8]};
   *
   * Angle between two vectors v1 and v2:
   *  angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)))
   * since here both are already of unit length:
   *  angle = acos(dot(v1, v2))
   * since we we want the cosine of the angle we simply need
   *  thrust_coeff = dot(v1, v2)
   * also can be simplified considering: v1 is zaxis with (0,0,1)
   *  dot(v1, v2) = v1.z * v2.z = v2.z
   */
  int32_t coef = att->m[8];
  if (coef < max_bank_coef) {
    coef = max_bank_coef;
  }
  return coef;
}


#define FF_CMD_FRAC 18

static void run_hover_loop(bool_t in_flight)
{

  /* convert our reference to generic representation */
  int64_t tmp  = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
  guidance_v_z_ref = (int32_t)tmp;
  guidance_v_zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  guidance_v_zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
  /* compute the error to our reference */
  int32_t err_z  = guidance_v_z_ref - stateGetPositionNed_i()->z;
  Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
  int32_t err_zd = guidance_v_zd_ref - stateGetSpeedNed_i()->z;
  Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

  if (in_flight) {
    guidance_v_z_sum_err += err_z;
    Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
  } else {
    guidance_v_z_sum_err = 0;
  }

  /* our nominal command : (g + zdd)*m   */
  int32_t inv_m;
  if (guidance_v_adapt_throttle_enabled) {
    inv_m =  gv_adapt_X >> (GV_ADAPT_X_FRAC - FF_CMD_FRAC);
  } else {
    /* use the fixed nominal throttle */
    inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);
  }

  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                          (guidance_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

  guidance_v_ff_cmd = g_m_zdd / inv_m;
  /* feed forward command */
  guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / guidance_v_thrust_coeff;

  /* bound the nominal command to 0.9*MAX_PPRZ */
  Bound(guidance_v_ff_cmd, 0, 8640);


  /* our error feed back command                   */
  /* z-axis pointing down -> positive error means we need less thrust */
  guidance_v_fb_cmd = ((-guidance_v_kp * err_z)  >> 7) +
                      ((-guidance_v_kd * err_zd) >> 16) +
                      ((-guidance_v_ki * guidance_v_z_sum_err) >> 16);

  guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

  /* bound the result */
  Bound(guidance_v_delta_t, 0, MAX_PPRZ);

}

/**
 * @brief run_indi_loop
 * Calculates required thrust to follow acceleration setpoint
 * Acceleration setpoint can be set for example by the throttle stick.
 */
static inline void run_indi_loop() {
  struct NedCoor_i *ltp_accel_nedcoor = stateGetAccelNed_i();
  struct Int32Vect3 ltp_accel;
  struct Int32Vect3 body_accel; // Acceleration measurement in body frame
  ltp_accel.x = ltp_accel_nedcoor->x;
  ltp_accel.y = ltp_accel_nedcoor->y;
  ltp_accel.z = ltp_accel_nedcoor->z;
  int32_rmat_vmult(&body_accel, stateGetNedToBodyRMat_i(), &ltp_accel);
  accel_z_raw = body_accel.z;

  // Apply notch filter and IIR to body frame z-axis accelerations
  /* First notch, then IIR filter on MEASURED ACCEL */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_set_filter_frequency(&accel_z_notchfilter, rpm_sensor.motor_frequency);
    notch_filter_update(&accel_z_notchfilter, &body_accel.z, &accel_z_notched);
  } else {
    /* Rotor spinning slowly, don't notch */
    accel_z_notched = body_accel.z;
  }
  /* Always IIR, also if rotor not spinning faster than 25Hz */
  accel_z_filtered = ((accel_z_filtered * (HELI_INDI_ACCEL_Z_FILTSIZE-1)) + accel_z_notched) / HELI_INDI_ACCEL_Z_FILTSIZE;

  /* RADIO throttle stick value */
  int32_t accel_z_sp = (-1)*3*((guidance_v_rc_delta_t - MAX_PPRZ/2) << INT32_ACCEL_FRAC) / (MAX_PPRZ/2);

  accel_z_sp = ((accel_z_sp << INT32_TRIG_FRAC) / guidance_v_thrust_coeff);

  /* Calculate delta z measured */
  int32_t accel_z_err = accel_z_sp - accel_z_filtered;
  int32_t delta_u = (accel_z_err * -1)/4;//was /2 // approx. effectiveness inverse -0.5

  /* Propagate actuator model */
  int32_t thrust_model_output = heli_rate_filter_propagate(&thrust_model, guidance_v_delta_t);
  thrust_model_output_transferred = thrust_model_output;

  /* Notch and IIR on actuator model */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_set_sampling_frequency(&thrust_actuator_inner_notchfilter, rpm_sensor.motor_frequency);
    notch_filter_update(&thrust_actuator_inner_notchfilter, &thrust_model_output, &inner_notch_output);
  } else {
    inner_notch_output = thrust_model_output;
  }
  inner_iir_output = ((inner_iir_output * (HELI_INDI_ACCEL_Z_FILTSIZE-1)) + inner_notch_output) / HELI_INDI_ACCEL_Z_FILTSIZE;

  int32_t new_thrust_setting = inner_iir_output + delta_u;
  global_new_thrust = new_thrust_setting;

  /* verry nice, calculations are practically done now, but we want to make the control signal slower to match the tail rotor */

  /* Different filters for up and down, because tail spins up faster.
    Up:
    0.93028
    0.032866
    1
    0.52857
    Down:
    0.98066
    0.016579
    1
    0.14286

Up:
0.93028
0.032866
1
0.52857
Down:
0.98066
0.016579
1
0.14286
  */

  static int32_t old_thrust_setting = 0;
  static float leadlag_comp_state = 0;

//  // TODO BOUND LEADLAG COMPENSATOR STATE
//  if (new_thrust_setting > old_thrust_setting) {
//    // fast filter
//  leadlag_comp_state = (0.93028f * leadlag_comp_state) + (0.032866f * new_thrust_setting);
//  guidance_v_delta_t = (int32_t)(leadlag_comp_state + (0.52857f * new_thrust_setting));
//  } else {
//    // slow filter
//  }


  leadlag_comp_state = (0.98066f * leadlag_comp_state) + (0.016579f * new_thrust_setting);
  guidance_v_delta_t = (int32_t)(leadlag_comp_state + (0.14286f * new_thrust_setting));
  //guidance_v_delta_t = new_thrust_setting; // use unfiltered thrust

  /* bound the result */
  Bound(guidance_v_delta_t, 1500, MAX_PPRZ);
  old_thrust_setting = guidance_v_delta_t;
  global_old_thrust = old_thrust_setting;
}

bool_t guidance_v_set_guided_z(float z)
{
  if (guidance_v_mode == GUIDANCE_V_MODE_GUIDED) {
    guidance_v_z_sp = POS_BFP_OF_REAL(z);
    return TRUE;
  }
  return FALSE;
}
