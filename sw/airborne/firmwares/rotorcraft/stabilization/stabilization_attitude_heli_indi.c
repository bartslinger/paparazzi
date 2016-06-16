﻿/*
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

/** @file stabilization_attitude_quat_int.c
 * Rotorcraft quaternion attitude stabilization
 */

#include "generated/airframe.h"
#include "autopilot.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_heli_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
//#include "modules/adc_expansion_uart/adc_expansion_uart.h"
//#include "modules/sensors/rpm_sensor.h"
#include "filters/low_pass_filter.h"
#include "subsystems/radio_control.h"

/* This is a beun hack to the max, but that is perfectly in line with the rest of pprz */
#include "firmwares/rotorcraft/guidance/guidance_v.h"
extern int32_t guidance_v_rc_delta_t; // private variable of stabilization_v.c

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"

/* Check order of commands for effectiveness matrix */
#if (COMMAND_ROLL   != 0 | \
     COMMAND_PITCH  != 1 | \
     COMMAND_YAW    != 2 | \
     COMMAND_THRUST != 3)
#warning "Order of commands incorrect"
#endif

#ifndef STABILIZATION_ATTITUDE_STEADY_STATE_ROLL
  #define STABILIZATION_ATTITUDE_STEADY_STATE_ROLL 0
#endif
#ifndef STABILIZATION_ATTITUDE_STEADY_STATE_PITCH
  #define STABILIZATION_ATTITUDE_STEADY_STATE_PITCH 0
#endif

struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_sp_euler;
struct Int32Quat sp_offset;
float sp_offset_roll = STABILIZATION_ATTITUDE_STEADY_STATE_ROLL;
float sp_offset_pitch = STABILIZATION_ATTITUDE_STEADY_STATE_PITCH;

struct HeliIndiGains heli_indi_gains = {
  STABILIZATION_ATTITUDE_HELI_INDI_ROLL_P,
  STABILIZATION_ATTITUDE_HELI_INDI_PITCH_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_D
};


#define HELI_INDI_ROLLRATE_FILTSIZE 16
#define HELI_INDI_PITCHRATE_FILTSIZE 16
#define HELI_INDI_YAWRATE_FILTSIZE 8

struct IndiController_int new_heli_indi;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/* Telemetry messages here */

#endif

void stabilization_attitude_heli_indi_set_steadystate_pitch(float pitch)
{
  sp_offset_pitch = pitch;
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();
}

void stabilization_attitude_heli_indi_set_steadystate_roll(float roll)
{
  sp_offset_roll = roll;
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();
}

void stabilization_attitude_heli_indi_set_steadystate_pitchroll()
{
  /* Pitch roll setpoint not zero */
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  struct FloatQuat q;
  ov.x = -sp_offset_roll * M_PI / 180;
  ov.y = -sp_offset_pitch * M_PI / 180;
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  float_quat_of_orientation_vect(&q, &ov);
  QUAT_BFP_OF_REAL(sp_offset, q);
}

static inline void indi_subtract_vect(int32_t _out[], int32_t _in1[], int32_t _in2[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = _in1[i] - _in2[i];
  }
}

static inline void indi_add_vect(int32_t _out[], int32_t _in1[], int32_t _in2[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = _in1[i] + _in2[i];
  }
}

static inline void indi_matrix_multiply_vector(int32_t _out[], int32_t _matrix[][INDI_DOF], int32_t _vector[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = 0;
    for (uint8_t j = 0; j < INDI_DOF; j++) {
      _out[i] += _matrix[i][j] * _vector[j];
    }
  }
}

static inline void indi_copy_vect(int32_t _out[], int32_t _in[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = _in[i];
  }
}

static inline void indi_set_identity(int32_t _matrix[][INDI_DOF])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    for (uint8_t j = 0; j < INDI_DOF; j++) {
      _matrix[i][j] = (i==j) ? 1 : 0;
    }
  }
}

/* Filter functions referenced to */
struct heli_rate_filter_t actuator_model[INDI_DOF];
#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
struct heli_rate_filter_t fast_dynamics_model[2]; // only pitch and roll
#endif
int32_t alpha_yaw_inc;
int32_t alpha_yaw_dec;
static inline void indi_apply_actuator_models(int32_t _out[], int32_t _in[])
{
  int32_t temp_roll;
  int32_t temp_pitch;
  temp_roll  = heli_rate_filter_propagate(&actuator_model[INDI_ROLL], _in[INDI_ROLL]);
  temp_pitch = heli_rate_filter_propagate(&actuator_model[INDI_PITCH], _in[INDI_PITCH]);

  /* Depending on yaw direction, change filter coefficient */
  int32_t prev = actuator_model[INDI_YAW].buffer[actuator_model[INDI_YAW].idx];
  if(_in[INDI_YAW] - prev > 0) {
    // Tail spinning up
    actuator_model[INDI_YAW].alpha = alpha_yaw_inc;
    //actuator_model[INDI_YAW].alpha = alpha_yaw_dec; // TEMP USE ONLY DEC MODEL
  } else {
    // Tail spinning down
    actuator_model[INDI_YAW].alpha = alpha_yaw_dec;
    //actuator_model[INDI_YAW].alpha = alpha_yaw_inc;  // TEMP USE ONLY INC MODEL
  }
  _out[INDI_YAW] = heli_rate_filter_propagate(&actuator_model[INDI_YAW], _in[INDI_YAW]);

  _out[INDI_THRUST] = heli_rate_filter_propagate(&actuator_model[INDI_THRUST], _in[INDI_THRUST]);

#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
  /* Also apply first order filter that represents fast damping dynamics in pitch and roll rate */
  _out[INDI_ROLL]  = heli_rate_filter_propagate(&fast_dynamics_model[INDI_ROLL], temp_roll);
  _out[INDI_PITCH] = heli_rate_filter_propagate(&fast_dynamics_model[INDI_PITCH], temp_pitch);
  /* For experiment, allow to (temporarily) disable the filter in roll */
  if (!new_heli_indi.use_roll_dyn_filter) {
    _out[INDI_ROLL] = temp_roll;
  }
#else
  _out[INDI_ROLL] = temp_roll;
  _out[INDI_PITCH] = temp_pitch;
#endif
}

static inline void indi_apply_compensator_filters(int32_t _out[], int32_t _in[])
{
  _out[INDI_ROLL]   = _in[INDI_ROLL];
  _out[INDI_PITCH]  = _in[INDI_PITCH];

  /* Delay the tail by 9 samples */
#define YAW_BUFFER_SIZE 9
  static int32_t yaw_output_buffer[YAW_BUFFER_SIZE];
  static uint8_t buf_idx = 0;

  buf_idx %= (YAW_BUFFER_SIZE-1);
  _out[INDI_YAW] = yaw_output_buffer[buf_idx];
  yaw_output_buffer[buf_idx] = _in[INDI_YAW];
  buf_idx++;

  // Disregard, just use input:
  _out[INDI_YAW] = _in[INDI_YAW];

  /* Thrust compensated for slow tail dynamics:
   * Step 1. What would be the next output of the system if the thrust cmd would be applied to the tail dynamics?
   * Step 2. What input is required to obtain this output when assuming dynamics of thrust actuator?
   *
   * We can re-use alpha_yaw_dec and alpha_yaw_inc
   */
  static int32_t prev_thrust_out = 0;
  int32_t alpha;
  if (_in[INDI_THRUST] - prev_thrust_out > 0) {
    alpha = alpha_yaw_inc;
  } else {
    alpha = alpha_yaw_dec;
  }
  int32_t output_target = (alpha*prev_thrust_out + ((1<<14) - alpha)*_in[INDI_THRUST]) >> 14;

  /* Now the target output is known, the collective dynamics is known. What input is required? */
  int32_t alpha_thrust = actuator_model[INDI_THRUST].alpha;
  _out[INDI_THRUST] = ((output_target << 14) - alpha_thrust*prev_thrust_out) / ((1<<14) - alpha_thrust);

  prev_thrust_out =_out[INDI_THRUST];

  //_out[INDI_THRUST] = _in[INDI_THRUST];
}

struct SecondOrderNotchFilter actuator_notchfilter[INDI_DOF];
struct SecondOrderNotchFilter measurement_notchfilter[INDI_DOF];
static inline void indi_apply_actuator_notch_filters(int32_t _out[], int32_t _in[])
{
  if (new_heli_indi.motor_rpm > 1500 && new_heli_indi.enable_notch) {
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_ROLL], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_PITCH], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_YAW], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_THRUST], new_heli_indi.motor_rpm/60.0f);
    notch_filter_update(&actuator_notchfilter[INDI_ROLL], &_in[INDI_ROLL], &_out[INDI_ROLL]);
    notch_filter_update(&actuator_notchfilter[INDI_PITCH], &_in[INDI_PITCH], &_out[INDI_PITCH]);
    notch_filter_update(&actuator_notchfilter[INDI_YAW], &_in[INDI_YAW], &_out[INDI_YAW]);
    notch_filter_update(&actuator_notchfilter[INDI_THRUST], &_in[INDI_THRUST], &_out[INDI_THRUST]);
  } else {
    _out[INDI_ROLL]   = _in[INDI_ROLL];
    _out[INDI_PITCH]  = _in[INDI_PITCH];
    _out[INDI_YAW]    = _in[INDI_YAW];
    _out[INDI_THRUST] = _in[INDI_THRUST];
  }
}

static inline void indi_apply_measurement_notch_filters(int32_t _out[], int32_t _in[])
{
  if (new_heli_indi.motor_rpm > 1500 && new_heli_indi.enable_notch) {
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_ROLL], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_PITCH], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_YAW], new_heli_indi.motor_rpm/60.0f);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_THRUST], new_heli_indi.motor_rpm/60.0f);
    notch_filter_update(&measurement_notchfilter[INDI_ROLL], &_in[INDI_ROLL], &_out[INDI_ROLL]);
    notch_filter_update(&measurement_notchfilter[INDI_PITCH], &_in[INDI_PITCH], &_out[INDI_PITCH]);
    notch_filter_update(&measurement_notchfilter[INDI_YAW], &_in[INDI_YAW], &_out[INDI_YAW]);
    notch_filter_update(&measurement_notchfilter[INDI_THRUST], &_in[INDI_THRUST], &_out[INDI_THRUST]);
  } else {
    _out[INDI_ROLL]   = _in[INDI_ROLL];
    _out[INDI_PITCH]  = _in[INDI_PITCH];
    _out[INDI_YAW]    = _in[INDI_YAW];
    _out[INDI_THRUST] = _in[INDI_THRUST];
  }
}

static inline void indi_apply_actuator_lowpass_filters(int32_t _out[], int32_t _in[])
{
  _out[INDI_ROLL]   = ((_out[INDI_ROLL] * (HELI_INDI_ROLLRATE_FILTSIZE-1)) + _in[INDI_ROLL]) / HELI_INDI_ROLLRATE_FILTSIZE;
  _out[INDI_PITCH]   = ((_out[INDI_PITCH] * (HELI_INDI_ROLLRATE_FILTSIZE-1)) + _in[INDI_PITCH]) / HELI_INDI_ROLLRATE_FILTSIZE;
  _out[INDI_YAW]   = ((_out[INDI_YAW] * (HELI_INDI_YAWRATE_FILTSIZE-1)) + _in[INDI_YAW]) / HELI_INDI_YAWRATE_FILTSIZE;
  _out[INDI_THRUST]   = ((_out[INDI_THRUST] * (HELI_INDI_YAWRATE_FILTSIZE-1)) + _in[INDI_THRUST]) / HELI_INDI_YAWRATE_FILTSIZE;
}

static inline void indi_apply_measurement_lowpass_filters(int32_t _out[], int32_t _in[])
{
  _out[INDI_ROLL]   = ((_out[INDI_ROLL] * (HELI_INDI_ROLLRATE_FILTSIZE-1)) + _in[INDI_ROLL]) / HELI_INDI_ROLLRATE_FILTSIZE;
  _out[INDI_PITCH]   = ((_out[INDI_PITCH] * (HELI_INDI_ROLLRATE_FILTSIZE-1)) + _in[INDI_PITCH]) / HELI_INDI_ROLLRATE_FILTSIZE;
  _out[INDI_YAW]   = ((_out[INDI_YAW] * (HELI_INDI_YAWRATE_FILTSIZE-1)) + _in[INDI_YAW]) / HELI_INDI_YAWRATE_FILTSIZE;
  _out[INDI_THRUST]   = ((_out[INDI_THRUST] * (HELI_INDI_YAWRATE_FILTSIZE-1)) + _in[INDI_THRUST]) / HELI_INDI_YAWRATE_FILTSIZE;
}

Butterworth2LowPass_int actuator_lowpass_filters[INDI_DOF];
Butterworth2LowPass_int measurement_lowpass_filters[INDI_DOF];

static inline void indi_apply_actuator_butterworth_filters(int32_t _out[], int32_t _in[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = update_butterworth_2_low_pass_int(&actuator_lowpass_filters[i], _in[i]);
  }
}

static inline void indi_apply_measurement_butterworth_filters(int32_t _out[], int32_t _in[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = update_butterworth_2_low_pass_int(&measurement_lowpass_filters[i], _in[i]);
  }
}

void stabilization_attitude_heli_indi_set_roll_omega(uint32_t omega)
{
  heli_rate_filter_set_omega(&actuator_model[INDI_ROLL], omega);
}

void stabilization_attitude_heli_indi_set_roll_delay(uint8_t delay)
{
  heli_rate_filter_set_delay(&actuator_model[INDI_ROLL], delay);
}

void stabilization_attitude_heli_indi_set_rollfilter_bw(float bandwidth)
{
  new_heli_indi.rollfilt_bw = bandwidth;
  // Cutoff frequencies are in Hz!!!
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_ROLL], bandwidth, 1.0/PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_ROLL], bandwidth, 1.0/PERIODIC_FREQUENCY, 0);
}

void stabilization_attitude_init(void)
{
  /* Set steady-state pitch and roll values */
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();

  /* Initialization code INDI */
  struct IndiController_int* c = &new_heli_indi;
  c->roll_comp_angle = ANGLE_BFP_OF_REAL(30.0*M_PI/180.0);
  c->pitch_comp_angle = ANGLE_BFP_OF_REAL(11.0*M_PI/180.0);
  c->use_roll_dyn_filter = TRUE;
  c->dist_magnitude = 1000;
  c->add_disturbance = FALSE;
  c->rollfilt_bw = 40.;
  c->enable_notch = TRUE;
  c->motor_rpm = 0;

  /* Initialize model matrices */
  indi_set_identity(c->D);

  c->invG[0][0] =   +11681; c->invG[0][1] =       0; c->invG[0][2] =    0; c->invG[0][3] =       0;
  c->invG[1][0] =        0; c->invG[1][1] =  +17341*.8; c->invG[1][2] =    0; c->invG[1][3] =       0;
  c->invG[2][0] =        0; c->invG[2][1] =       0; c->invG[2][2] =  730; c->invG[2][3] =       0;
  c->invG[3][0] =        0; c->invG[3][1] =       0; c->invG[3][2] =    0; c->invG[3][3] =  -50000;

  /* Actuator filter initialization */
  heli_rate_filter_initialize(&actuator_model[INDI_ROLL], 70, 9, 900);
  heli_rate_filter_initialize(&actuator_model[INDI_PITCH], 70, 9, 900);
  heli_rate_filter_initialize(&actuator_model[INDI_YAW], 37, 0, 9600);
  heli_rate_filter_initialize(&actuator_model[INDI_THRUST], 70, 9, 900/2); /* 450 because dynamic range is only 0-9600 */
  // Different dynamics for up and down
  alpha_yaw_inc = actuator_model[INDI_YAW].alpha;
  alpha_yaw_dec = (PERIODIC_FREQUENCY << 14)/(PERIODIC_FREQUENCY + 13); // OMEGA_DOWN = 13 rad/s, shift = 14

#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
  /* Fast dynamics in roll and pitch model */
  heli_rate_filter_initialize(&fast_dynamics_model[INDI_ROLL], STABILIZATION_ATTITUDE_HELI_INDI_FAST_DYN_ROLL_BW, 0, 9600); // TODO: Fix value
  heli_rate_filter_initialize(&fast_dynamics_model[INDI_PITCH], STABILIZATION_ATTITUDE_HELI_INDI_FAST_DYN_PITCH_BW, 0, 9600); // "       "
#endif

  /* Notch filter initialization */
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    // Sampe parameters in each degree of freedom */
    notch_filter_set_sampling_frequency(&actuator_notchfilter[i], PERIODIC_FREQUENCY);
    notch_filter_set_bandwidth(&actuator_notchfilter[i], 10.0);
    notch_filter_set_sampling_frequency(&measurement_notchfilter[i], PERIODIC_FREQUENCY);
    notch_filter_set_bandwidth(&measurement_notchfilter[i], 10.0);
  }

  notch_filter_set_bandwidth(&actuator_notchfilter[INDI_YAW], 20.0);
  notch_filter_set_bandwidth(&measurement_notchfilter[INDI_YAW], 20.0);

  /* Low pass filter initialization */
  for (uint8_t i = 0; i < INDI_DOF-2; i++) {
    // Cutoff frequencies are in Hz!!!
    init_butterworth_2_low_pass_int(&actuator_lowpass_filters[i], 40, 1.0/PERIODIC_FREQUENCY, 0);
    init_butterworth_2_low_pass_int(&measurement_lowpass_filters[i], 40, 1.0/PERIODIC_FREQUENCY, 0);
  }
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_YAW], 20, 1.0/PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_YAW], 20, 1.0/PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_THRUST], 10, 1.0/PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_THRUST], 10, 1.0/PERIODIC_FREQUENCY, 0);

  /* Assign filter functions: */
  c->apply_actuator_models = &indi_apply_actuator_models;
  c->apply_compensator_filters = &indi_apply_compensator_filters;
  c->apply_measurement_filters[0] = &indi_apply_measurement_notch_filters;
  c->apply_measurement_filters[1] = &indi_apply_measurement_butterworth_filters;
  c->apply_actuator_filters[0] = &indi_apply_actuator_notch_filters;
  c->apply_actuator_filters[1] = &indi_apply_actuator_butterworth_filters;

#if PERIODIC_TELEMETRY
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_<<MSG>>, function);
#endif
}

void stabilization_attitude_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

void stabilization_attitude_run(bool in_flight)
{
  (void) in_flight; // unused variable
  struct IndiController_int *c = &new_heli_indi;

  /* calculate acceleration in body frame */
  struct NedCoor_i *ltp_accel_nedcoor = stateGetAccelNed_i();
  struct Int32Vect3 ltp_accel;
  struct Int32Vect3 body_accel; // Acceleration measurement in body frame
  ltp_accel.x = ltp_accel_nedcoor->x;
  ltp_accel.y = ltp_accel_nedcoor->y;
  ltp_accel.z = ltp_accel_nedcoor->z;
  int32_rmat_vmult(&body_accel, stateGetNedToBodyRMat_i(), &ltp_accel);

  /* attitude error */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();

  /* Add steady-state value to attitude setpoint, because heli has
   * non-zero roll angle by default
   */
  struct Int32Quat corr_att_sp_quat; // Corrected attitude setpoint
  INT32_QUAT_COMP_INV(corr_att_sp_quat, stab_att_sp_quat, sp_offset);

  INT32_QUAT_INV_COMP(att_err, *att_quat, corr_att_sp_quat);
  /* wrap it in the shortest direction */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* rate error (setpoint for rates = 0) */
  struct Int32Rates *body_rate = stateGetBodyRates_i();

  /* Inform INDI about the measurement */
  c->measurement[INDI_ROLL]  = body_rate->p;
  c->measurement[INDI_PITCH] = body_rate->q;
  c->measurement[INDI_YAW]   = body_rate->r;
  c->measurement[INDI_THRUST]= body_accel.z;

  /* Apply actuator dynamics model to previously commanded values
   * input  = actuator command in previous cycle
   * output = actual actuator position right now
   */
  c->apply_actuator_models(c->actuator_out, c->command_out[__k-1]);

  /* Apply a set of filters, both to the actuator and the measurement */
  c->apply_actuator_filters[0](c->filtered_actuator[0], c->actuator_out);
  c->apply_measurement_filters[0](c->filtered_measurement[0], c->measurement);
  for (uint8_t i = 1; i < INDI_NR_FILTERS; i++) {
    c->apply_actuator_filters[i](c->filtered_actuator[i], c->filtered_actuator[i-1]);
    c->apply_measurement_filters[i](c->filtered_measurement[i], c->filtered_measurement[i-1]);
  }

  /* RADIO throttle stick value, for 4dof mode */
  int32_t accel_z_sp = (-1)*3*((guidance_v_rc_delta_t - MAX_PPRZ/2) << INT32_ACCEL_FRAC) / (MAX_PPRZ/2);
  accel_z_sp = ((accel_z_sp << INT32_TRIG_FRAC) / guidance_v_thrust_coeff);

  /* Transform yaw into a delta yaw while keeping filtered yawrate (kinda hacky)*/
  int32_t filtered_measurement_vector[INDI_DOF];
  indi_copy_vect(filtered_measurement_vector, c->filtered_measurement[INDI_NR_FILTERS-1]);
  static int32_t previous_filt_yawrate = 0;
  filtered_measurement_vector[INDI_YAW] = 512*(c->filtered_measurement[INDI_NR_FILTERS-1][INDI_YAW] - previous_filt_yawrate);  // = approximately yaw acceleration error
  previous_filt_yawrate = c->filtered_measurement[INDI_NR_FILTERS-1][INDI_YAW];

  /* Apply model dynamics matrix, is diagonal of ones when model dynamics are neglected. */
  indi_matrix_multiply_vector(c->dynamics_compensated_measurement, c->D, filtered_measurement_vector);

  /* Use the filtered measurements for PID control as well */
  int32_t roll_virtual_control  = (heli_indi_gains.roll_p * att_err.qx)  / 4;
  int32_t pitch_virtual_control = (heli_indi_gains.pitch_p * att_err.qy) / 4;

  /* Try with a cascaded controller */
  int32_t yaw_rate_reference = (heli_indi_gains.yaw_p * att_err.qz / 8);
  int32_t yaw_virtual_control = heli_indi_gains.yaw_d * (yaw_rate_reference - body_rate->r);
  //yaw_virtual_control <<= (16-7);

  /* Run P(D) control to generate references */
  c->reference[INDI_ROLL]   = roll_virtual_control;
  c->reference[INDI_PITCH]  = pitch_virtual_control;
  c->reference[INDI_YAW]    = yaw_virtual_control;
  //c->reference[INDI_THRUST] = accel_z_sp;

  /* Subtract (filtered) measurement from reference to get the error */
  indi_subtract_vect(c->error, c->reference, c->dynamics_compensated_measurement);

  /* Multiply error with inverse of actuator effectiveness, to get delta u (required increment in input) */
  indi_matrix_multiply_vector(c->du, c->invG, c->error);

  /* Bitshift back */
  c->du[INDI_ROLL]  >>= 16;
  c->du[INDI_PITCH] >>= 16;
  c->du[INDI_YAW]   >>= 16;
  c->du[INDI_THRUST]>>= 16;

  /* Take the current (filtered) actuator position and add the incremental value. */
  indi_add_vect(c->u_setpoint, c->filtered_actuator[INDI_NR_FILTERS-1], c->du);
  //c->u_setpoint[INDI_THRUST] = stabilization_cmd[COMMAND_THRUST];

  /* bound the result */
  BoundAbs(c->u_setpoint[INDI_ROLL], MAX_PPRZ);
  BoundAbs(c->u_setpoint[INDI_PITCH], MAX_PPRZ);
  Bound(c->u_setpoint[INDI_YAW], 0, MAX_PPRZ);
  Bound(c->u_setpoint[INDI_THRUST], 0.15*MAX_PPRZ, MAX_PPRZ);

  /* Apply a compensator to the actuator setpoint to obtain actuator command */
  c->apply_compensator_filters(c->command_out[__k], c->u_setpoint);

  /* At the end, set 'previous' output to current output */
  indi_copy_vect(c->command_out[__k-1], c->command_out[__k]);

  /* Two correction angles, don't rotate but just add.
   * sin/cos = tan
   */
  stabilization_cmd[COMMAND_ROLL] = c->command_out[__k][INDI_ROLL]
                                  + c->command_out[__k][INDI_PITCH] * pprz_itrig_sin(c->pitch_comp_angle) / pprz_itrig_cos(c->pitch_comp_angle);
  stabilization_cmd[COMMAND_PITCH] = c->command_out[__k][INDI_PITCH]
                                   - c->command_out[__k][INDI_ROLL] * pprz_itrig_sin(c->roll_comp_angle) / pprz_itrig_cos(c->roll_comp_angle);

  stabilization_cmd[COMMAND_YAW] = c->command_out[__k][INDI_YAW];
  /* Thrust is not applied */

  /* Disable tail when not armed, because this thing goes crazy */
  if(!autopilot_motors_on)
  {
    stabilization_cmd[COMMAND_YAW] = 0;
  }
}

void stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}
