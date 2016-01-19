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

/** @file stabilization_attitude_quat_int.c
 * Rotorcraft quaternion attitude stabilization
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_heli_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "modules/adc_expansion_uart/adc_expansion_uart.h"
#include "subsystems/sensors/rpm_sensor.h"

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


struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_sp_euler;

struct HeliIndiGains heli_indi_gains = {
  STABILIZATION_ATTITUDE_HELI_INDI_ROLL_P,
  STABILIZATION_ATTITUDE_HELI_INDI_PITCH_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_D
};

#define HELI_INDI_ROLLRATE_FILTSIZE 16
#define HELI_INDI_PITCHRATE_FILTSIZE 16
#define HELI_INDI_YAWRATE_FILTSIZE 8

struct HeliIndiStab heli_indi;
int32_t global_delta_u;
int32_t global_pitch_model;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/* Telemetry messages here */

static void send_indi_debug_values(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  int16_t meas_cmd[3];
  meas_cmd[0] = heli_indi.measured_cmd[0];
  meas_cmd[1] = heli_indi.measured_cmd[1];
  meas_cmd[2] = heli_indi.measured_cmd[2];
  pprz_msg_send_STAB_INDI_DEBUG(trans, dev, AC_ID,
                               &meas_cmd[0],
                               &meas_cmd[1],
                               &meas_cmd[2],
                               &stabilization_cmd[COMMAND_PITCH],
                               &stabilization_cmd[COMMAND_ROLL],
                               &stabilization_cmd[COMMAND_THRUST],
                               &body_rate->p,
                               &body_rate->q,
                               &body_rate->r);
}
#endif

void stabilization_attitude_init(void)
{
  /* Initialization code */

  heli_indi.yawrate_setpoint = 0;
  heli_indi.yawrate_err = 0;
  heli_indi.filter_out = 0;
  heli_indi.yaw_incremental_cmd = 0;
  heli_indi.rate_notched.p = 0;
  heli_indi.rate_notched.q = 0;
  heli_indi.rate_notched.r = 0;
  heli_indi.rate_filt.p = 0;
  heli_indi.rate_filt.q = 0;
  heli_indi.rate_filt.r = 0;
  heli_indi.rate_previous.p = 0;
  heli_indi.rate_previous.q = 0;
  heli_indi.rate_previous.r = 0;
  heli_indi.inputmodel_notched.p = 0;
  heli_indi.inputmodel_notched.q = 0;
  heli_indi.inputmodel_notched.r = 0;
  heli_indi.inputmodel_filtered.p = 0;
  heli_indi.inputmodel_filtered.q = 0;
  heli_indi.inputmodel_filtered.r = 0;
  heli_rate_filter_initialize(&heli_indi.tail_model, 37, 0, 9600);
  heli_rate_filter_initialize(&heli_indi.roll_model, 70, 9, 900);
  heli_rate_filter_initialize(&heli_indi.pitch_model, 70, 9, 900);

  /* Initialize notch filters */
  notch_filter_set_sampling_frequency(&heli_indi.p_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.p_filter, 10.0);
  notch_filter_set_sampling_frequency(&heli_indi.p_inner_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.p_inner_filter, 10.0);

  notch_filter_set_sampling_frequency(&heli_indi.q_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.q_filter, 10.0);
  notch_filter_set_sampling_frequency(&heli_indi.q_inner_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.q_inner_filter, 10.0);

  notch_filter_set_sampling_frequency(&heli_indi.r_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.r_filter, 10.0);
  notch_filter_set_sampling_frequency(&heli_indi.r_inner_filter, PERIODIC_FREQUENCY);
  notch_filter_set_bandwidth(&heli_indi.r_inner_filter, 10.0);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_INDI_DEBUG, send_indi_debug_values);
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

void stabilization_attitude_get_measured_commands(){
  /* Convert analog signals to corresponding steady-state PWM values */
  int32_t servo_left  = adc_uart_values[0] * (float)0.54486 + 697;
  int32_t servo_front = adc_uart_values[1] * (float)0.54827 + 703;
  int32_t servo_right = adc_uart_values[2] * (float)0.54983 + 685;

  /* Convert servo deflections back to pitch and roll commands */
  servo_left  -= SERVO_CIC_LEFT_NEUTRAL;
  servo_front -= SERVO_CIC_FRONT_NEUTRAL;
  servo_right -= SERVO_CIC_RIGHT_NEUTRAL;

  servo_left  *= servo_left>0 ?  SERVO_CIC_LEFT_TRAVEL_UP_DEN : SERVO_CIC_LEFT_TRAVEL_DOWN_DEN;
  servo_front *= servo_front>0 ? SERVO_CIC_FRONT_TRAVEL_UP_DEN : SERVO_CIC_FRONT_TRAVEL_DOWN_DEN;
  servo_right *= servo_right>0 ? SERVO_CIC_RIGHT_TRAVEL_UP_DEN : SERVO_CIC_RIGHT_TRAVEL_DOWN_DEN;

  //  B = A*x (matrix multiply)
  //  B = servo_values left, right, front
  //  x = cmd pitch, cmd roll, var_collective
  /* inverse (A) bitshifted <<16 =
  //   24966      -24966      -49932
  //  -51300      -51300           0
  //  -21845       21845      -21845
  */
  heli_indi.measured_cmd[0] = (+ 24966 * servo_left - 24966 * servo_right - 49932 * servo_front) >> 16; // pitch
  heli_indi.measured_cmd[1] = (- 51300 * servo_left - 51300 * servo_right) >> 16;                       // roll
  heli_indi.measured_cmd[2] = (- 21845 * servo_left + 21845 * servo_right - 21845 * servo_front) >> 16; // var_collective
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

void stabilization_attitude_run(bool_t enable_integrator)
{
  (void) enable_integrator;

  /* attitude error */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);
  /* wrap it in the shortest direction */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* rate error */
  struct Int32Rates *body_rate = stateGetBodyRates_i();

  /* First notch, then IIR filter on MEASURED RATES */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_set_filter_frequency(&heli_indi.p_filter, rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&heli_indi.p_inner_filter, rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&heli_indi.q_filter, rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&heli_indi.q_inner_filter, rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&heli_indi.r_filter, rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&heli_indi.r_inner_filter, rpm_sensor.motor_frequency);
    notch_filter_update(&heli_indi.p_filter, &body_rate->p, &heli_indi.rate_notched.p);
    notch_filter_update(&heli_indi.q_filter, &body_rate->q, &heli_indi.rate_notched.q);
    notch_filter_update(&heli_indi.r_filter, &body_rate->r, &heli_indi.rate_notched.r);
  } else {
    /* Rotor spinning slowly, don't notch */
    heli_indi.rate_filt.p = body_rate->p;
    heli_indi.rate_filt.q = body_rate->q;
    heli_indi.rate_filt.r = body_rate->r;
  }
  /* Always IIR, also if rotor not spinning faster than 25Hz */
  heli_indi.rate_filt.p = ((heli_indi.rate_filt.p * (HELI_INDI_ROLLRATE_FILTSIZE-1)) + heli_indi.rate_notched.p) / HELI_INDI_ROLLRATE_FILTSIZE;
  heli_indi.rate_filt.q = ((heli_indi.rate_filt.q * (HELI_INDI_PITCHRATE_FILTSIZE-1)) + heli_indi.rate_notched.q) / HELI_INDI_PITCHRATE_FILTSIZE;
  heli_indi.rate_filt.r = ((heli_indi.rate_filt.r * (HELI_INDI_YAWRATE_FILTSIZE-1)) + heli_indi.rate_notched.r) / HELI_INDI_YAWRATE_FILTSIZE;

  /* Calculate delta pqr measured */
  struct Int32Rates delta_rate_meas;
  delta_rate_meas.p = heli_indi.rate_filt.p;
  delta_rate_meas.q = heli_indi.rate_filt.q;
  delta_rate_meas.r = heli_indi.rate_filt.r - heli_indi.rate_previous.r;
  heli_indi.rate_previous.r = heli_indi.rate_filt.r;

  /* Linear controllers */
  int32_t roll_pid = (GAIN_MULTIPLIER_P * heli_indi_gains.roll_p * att_err.qx) >> 15;
  int32_t pitch_pid = (GAIN_MULTIPLIER_P * heli_indi_gains.pitch_p * att_err.qy) >> 15;
  int32_t roll_virtual_control  = (heli_indi_gains.roll_p * att_err.qx);
  int32_t pitch_virtual_control = (heli_indi_gains.pitch_p * att_err.qy);
  int32_t yaw_virtual_control  = (heli_indi_gains.yaw_p * att_err.qz - heli_indi.rate_filt.r) *
                                 (heli_indi_gains.yaw_d);

  /* ------------------ */

  /* INDI YAW */
  /* Multiply with dt; this integrates virtual control (acceleration) to required change in rate */
  int32_t delta_r_ref = yaw_virtual_control / 512;

  int32_t delta_r_error = delta_r_ref - delta_rate_meas.r;
  int32_t delta_u_yaw = (delta_r_error * 512) / 48; /* equal to multiply with 10.6667, which is inv(B) */
  int32_t yaw_model_output = heli_rate_filter_propagate(&heli_indi.tail_model, stabilization_cmd[COMMAND_YAW]);

  /* Only notch above freq threshold */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_update(&heli_indi.r_inner_filter, &yaw_model_output, &heli_indi.inputmodel_notched.r);
  } else {
    heli_indi.inputmodel_filtered.r = yaw_model_output;
  }
  /* Always IIR */
  heli_indi.inputmodel_filtered.r = ((heli_indi.inputmodel_filtered.r * (HELI_INDI_YAWRATE_FILTSIZE - 1)) + heli_indi.inputmodel_notched.r) / HELI_INDI_YAWRATE_FILTSIZE;
  /* -------- */


  /* INDI PITCH + ROLL */
  /* ----------------- */
  /* Get measured pitch, roll, collective commands from adc measurements */
  stabilization_attitude_get_measured_commands();

  int32_t delta_p_ref = roll_virtual_control;
  int32_t delta_p_error = delta_p_ref - delta_rate_meas.p;
  int32_t roll_model_output = heli_rate_filter_propagate(&heli_indi.roll_model, stabilization_cmd[COMMAND_ROLL]);

  int32_t delta_q_ref = pitch_virtual_control;
  int32_t delta_q_error = delta_q_ref - delta_rate_meas.q;
  int32_t pitch_model_output = heli_rate_filter_propagate(&heli_indi.pitch_model, stabilization_cmd[COMMAND_PITCH]);
  global_pitch_model = pitch_model_output;

  /* from matlab identification procedure
  59558       18324
 -29230       34353
  */

  float delta_u_roll =  (59558.0 * (delta_p_error - 0) + 18324.0 * (delta_q_error + 0)) / 65536 /2;
  float delta_u_pitch = (-29230.0 * (delta_p_error - 0) + 34353.0 * (delta_q_error + 0)) / 65536 /2;

  global_delta_u = (int32_t) delta_u_pitch;

  /* Notch on inner loop pitch and roll */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_update(&heli_indi.p_inner_filter, &roll_model_output, &heli_indi.inputmodel_notched.p);
    notch_filter_update(&heli_indi.q_inner_filter, &pitch_model_output, &heli_indi.inputmodel_notched.q);
  } else {
    heli_indi.inputmodel_filtered.p = roll_model_output;
    heli_indi.inputmodel_filtered.q = pitch_model_output;
  }
  /*Always IIR */
  heli_indi.inputmodel_filtered.p = ((heli_indi.inputmodel_filtered.p * (HELI_INDI_ROLLRATE_FILTSIZE - 1)) + heli_indi.inputmodel_notched.p) / HELI_INDI_ROLLRATE_FILTSIZE;
  heli_indi.inputmodel_filtered.q = ((heli_indi.inputmodel_filtered.q * (HELI_INDI_PITCHRATE_FILTSIZE - 1)) + heli_indi.inputmodel_notched.q) / HELI_INDI_PITCHRATE_FILTSIZE;

  /* set stabilization commands */
  stabilization_cmd[COMMAND_ROLL] = heli_indi.inputmodel_filtered.p + delta_u_roll;
  stabilization_cmd[COMMAND_PITCH] = heli_indi.inputmodel_filtered.q + delta_u_pitch;//(att_err.qy > 0) ? MAX_PPRZ : -MAX_PPRZ;//
  stabilization_cmd[COMMAND_YAW] = heli_indi.inputmodel_filtered.r + delta_u_yaw;

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}
