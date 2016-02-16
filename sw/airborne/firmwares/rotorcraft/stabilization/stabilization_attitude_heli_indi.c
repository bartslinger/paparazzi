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

struct IndiController_int new_heli_indi;
//struct HeliIndiStab heli_indi;
//int32_t global_delta_u;
//int32_t global_pitch_model;
//int32_t global_delta_thrust;
//struct Int32Vect3 global_body_accelerations;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/* Telemetry messages here */

static void send_indi_debug_values(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_INDI_DEBUG(trans, dev, AC_ID,
                                &new_heli_indi.reference[INDI_THRUST],
                                &new_heli_indi.filtered_measurement[1][INDI_THRUST],
                                &stabilization_cmd[COMMAND_THRUST],
                                &new_heli_indi.u_setpoint[INDI_THRUST],
                                &new_heli_indi.command_out[__k][INDI_THRUST],
                                &new_heli_indi.error[INDI_YAW],
                                &new_heli_indi.error[INDI_THRUST],
                                &new_heli_indi.filtered_measurement[1][INDI_YAW],
                                &stabilization_cmd[COMMAND_YAW]);
}
#endif

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
int32_t alpha_yaw_inc;
int32_t alpha_yaw_dec;
static inline void indi_apply_actuator_models(int32_t _out[], int32_t _in[])
{
  _out[INDI_ROLL] = heli_rate_filter_propagate(&actuator_model[INDI_ROLL], _in[INDI_ROLL]);
  _out[INDI_PITCH] = heli_rate_filter_propagate(&actuator_model[INDI_PITCH], _in[INDI_PITCH]);

  /* Depending on yaw direction, change filter coefficient */
  int32_t prev = actuator_model[INDI_YAW].buffer[actuator_model[INDI_YAW].idx];
  if(_in[INDI_YAW] - prev > 0) {
    // Tail spinning up
    actuator_model[INDI_YAW].alpha = alpha_yaw_inc;
  } else {
    // Tail spinning down
    actuator_model[INDI_YAW].alpha = alpha_yaw_dec;
  }
  _out[INDI_YAW] = heli_rate_filter_propagate(&actuator_model[INDI_YAW], _in[INDI_YAW]);

  _out[INDI_THRUST] = heli_rate_filter_propagate(&actuator_model[INDI_THRUST], _in[INDI_THRUST]);
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
  if (rpm_sensor.motor_frequency > 25.0f) {
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_ROLL], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_PITCH], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_YAW], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&actuator_notchfilter[INDI_THRUST], rpm_sensor.motor_frequency);
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
  if (rpm_sensor.motor_frequency > 25.0f) {
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_ROLL], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_PITCH], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_YAW], rpm_sensor.motor_frequency);
    notch_filter_set_filter_frequency(&measurement_notchfilter[INDI_THRUST], rpm_sensor.motor_frequency);
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

void stabilization_attitude_init(void)
{
  /* Initialization code INDI */
  struct IndiController_int* c = &new_heli_indi;

  /* Initialize model matrices */
  indi_set_identity(c->D);
  // matlab new method
  //  47948       14952
  // -31016       27170
  c->invG[0][0] =   +22000; c->invG[0][1] =       0; c->invG[0][2] =    0; c->invG[0][3] =       0;
  c->invG[1][0] =        0; c->invG[1][1] =  +20000; c->invG[1][2] =    0; c->invG[1][3] =       0;
  c->invG[2][0] =        0; c->invG[2][1] =       0; c->invG[2][2] = 2114; c->invG[2][3] = -130000;
  c->invG[3][0] =        0; c->invG[3][1] =       0; c->invG[3][2] =    0; c->invG[3][3] =  -50000;

  /* Actuator filter initialization */
  heli_rate_filter_initialize(&actuator_model[INDI_ROLL], 70, 9, 900);
  heli_rate_filter_initialize(&actuator_model[INDI_PITCH], 70, 9, 900);
  heli_rate_filter_initialize(&actuator_model[INDI_YAW], 37, 0, 9600);
  heli_rate_filter_initialize(&actuator_model[INDI_THRUST], 70, 9, 450);
  // Different dynamics for up and down
  alpha_yaw_inc = actuator_model[INDI_YAW].alpha;
  alpha_yaw_dec = (PERIODIC_FREQUENCY << 14)/(PERIODIC_FREQUENCY + 10); // OMEGA_DOWN = 10 rad/s, shift = 14

  /* Notch filter initialization */
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    // Sampe parameters in each degree of freedom */
    notch_filter_set_sampling_frequency(&actuator_notchfilter[i], PERIODIC_FREQUENCY);
    notch_filter_set_bandwidth(&actuator_notchfilter[i], 10.0);
    notch_filter_set_sampling_frequency(&measurement_notchfilter[i], PERIODIC_FREQUENCY);
    notch_filter_set_bandwidth(&measurement_notchfilter[i], 10.0);
  }

  /* Low pass filter initialization */
  for (uint8_t i = 0; i < INDI_DOF-1; i++) {
    // Cutoff frequencies are in Hz!!!
    init_butterworth_2_low_pass_int(&actuator_lowpass_filters[i], 20, 1.0/PERIODIC_FREQUENCY, 0);
    init_butterworth_2_low_pass_int(&measurement_lowpass_filters[i], 20, 1.0/PERIODIC_FREQUENCY, 0);
  }
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_THRUST], 10, 1.0/PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_THRUST], 10, 1.0/PERIODIC_FREQUENCY, 0);


  /* Assign filter functions: */
  c->apply_actuator_models = &indi_apply_actuator_models;
  c->apply_compensator_filters = &indi_apply_compensator_filters;
  c->apply_measurement_filters[0] = &indi_apply_measurement_notch_filters;
  c->apply_measurement_filters[1] = &indi_apply_measurement_butterworth_filters;
  c->apply_actuator_filters[0] = &indi_apply_actuator_notch_filters;
  c->apply_actuator_filters[1] = &indi_apply_actuator_butterworth_filters;

  // AND ITS GONE (the old stuff)

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

#if 0
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
#endif

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

void stabilization_attitude_run(bool_t in_flight)
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
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);
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

  /* RADIO throttle stick value */
  int32_t accel_z_sp = (-1)*3*((guidance_v_rc_delta_t - MAX_PPRZ/2) << INT32_ACCEL_FRAC) / (MAX_PPRZ/2);
  accel_z_sp = ((accel_z_sp << INT32_TRIG_FRAC) / guidance_v_thrust_coeff);

  /* Transform yaw into a delta yaw while keeping filtered yawrate (kinda hacky)*/
  int32_t filtered_measurement_vector[INDI_DOF];
  indi_copy_vect(filtered_measurement_vector, c->filtered_measurement[INDI_NR_FILTERS-1]);
  static int32_t previous_filt_yawrate = 0;
  filtered_measurement_vector[INDI_YAW] = 512*(c->filtered_measurement[INDI_NR_FILTERS-1][INDI_YAW] - previous_filt_yawrate);  // = approximately yaw acceleration error
  previous_filt_yawrate = c->filtered_measurement[INDI_NR_FILTERS-1][INDI_YAW];

  //static int32_t previous_thrust = 0;
  //int32_t delta_thrust_meas = c->filtered_actuator[0][INDI_THRUST] - previous_thrust;
  //previous_thrust = c->filtered_actuator[0][INDI_THRUST];

  /* Apply model dynamics matrix, is diagonal of ones when model dynamics are neglected. */
  indi_matrix_multiply_vector(c->dynamics_compensated_measurement, c->D, filtered_measurement_vector);

  /* Use the filtered measurements for PID control as well */
  int32_t roll_virtual_control  = (heli_indi_gains.roll_p * att_err.qx)  / 16;
  int32_t pitch_virtual_control = (heli_indi_gains.pitch_p * att_err.qy) / 16;
  int32_t yaw_virtual_control  = (heli_indi_gains.yaw_p * att_err.qz) - (c->filtered_measurement[INDI_NR_FILTERS-1][INDI_YAW] * heli_indi_gains.yaw_d);

  /* Run P(D) control to generate references */
  c->reference[INDI_ROLL]   = roll_virtual_control;
  c->reference[INDI_PITCH]  = pitch_virtual_control;
  c->reference[INDI_YAW]    = yaw_virtual_control;
  c->reference[INDI_THRUST] = accel_z_sp;

  /* Subtract (filtered) measurement from reference to get the error */
  indi_subtract_vect(c->error, c->reference, c->dynamics_compensated_measurement);

  //c->error[INDI_THRUST] = delta_thrust_meas;

  /* Multiply error with inverse of actuator effectiveness, to get delta u (required increment in input) */
  indi_matrix_multiply_vector(c->du, c->invG, c->error);

  /* Bitshift back */
  c->du[INDI_ROLL]  >>= 16;
  c->du[INDI_PITCH] >>= 16;
  c->du[INDI_YAW]   >>= 16;
  c->du[INDI_THRUST]>>= 16;  // to get -1/4
  //c->du[INDI_YAW] = (c->error[INDI_YAW] + 69*delta_thrust_meas) / 31;

  /* Take the current (filtered) actuator position and add the incremental value. */
  indi_add_vect(c->u_setpoint, c->filtered_actuator[INDI_NR_FILTERS-1], c->du);
  //c->u_setpoint[INDI_THRUST] = stabilization_cmd[COMMAND_THRUST];

  /* bound the result */
  BoundAbs(c->u_setpoint[INDI_ROLL], MAX_PPRZ);
  BoundAbs(c->u_setpoint[INDI_PITCH], MAX_PPRZ);
  BoundAbs(c->u_setpoint[INDI_YAW], MAX_PPRZ);
  Bound(c->u_setpoint[INDI_THRUST], 2000, MAX_PPRZ);

  /* Apply a compensator to the actuator setpoint to obtain actuator command */
  c->apply_compensator_filters(c->command_out[__k], c->u_setpoint);

  /* At the end, set 'previous' output to current output */
  indi_copy_vect(c->command_out[__k-1], c->command_out[__k]);

  /* Set stabilization commands to output values of the INDI controller */
  stabilization_cmd[COMMAND_ROLL] = c->command_out[__k][INDI_ROLL];
  stabilization_cmd[COMMAND_PITCH] = c->command_out[__k][INDI_PITCH];
  stabilization_cmd[COMMAND_YAW] = c->command_out[__k][INDI_YAW];
  // Do not overwrite thrust unless when in 4DOF mode
  if (guidance_v_mode == GUIDANCE_V_MODE_HELI_INDI_4DOF) {
    stabilization_cmd[COMMAND_THRUST] = c->command_out[__k][INDI_THRUST];
  }
}

#if 0
static inline void stabilization_attitude_run_old(bool_t enable_integrator)
{
  (void) enable_integrator;
  //struct HeliIndiStab *c = &heli_indi;

  /* calculate acceleration in body frame */
  //ins_int.ltp_accel.x = accel_meas_ltp.x;
  //ins_int.ltp_accel.y = accel_meas_ltp.y;
  struct NedCoor_i *ltp_accel_nedcoor = stateGetAccelNed_i();
  struct Int32Vect3 ltp_accel;
  ltp_accel.x = ltp_accel_nedcoor->x;
  ltp_accel.y = ltp_accel_nedcoor->y;
  ltp_accel.z = ltp_accel_nedcoor->z;
  int32_rmat_vmult(&global_body_accelerations, stateGetNedToBodyRMat_i(), &ltp_accel);

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
    heli_indi.rate_notched.p = body_rate->p;
    heli_indi.rate_notched.q = body_rate->q;
    heli_indi.rate_notched.r = body_rate->r;
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
  //int32_t roll_pid = (GAIN_MULTIPLIER_P * heli_indi_gains.roll_p * att_err.qx) >> 15;
  //int32_t pitch_pid = (GAIN_MULTIPLIER_P * heli_indi_gains.pitch_p * att_err.qy) >> 15;
  int32_t roll_virtual_control  = (heli_indi_gains.roll_p * att_err.qx) / 16;
  int32_t pitch_virtual_control = (heli_indi_gains.pitch_p * att_err.qy) / 16;
  int32_t yaw_virtual_control  = (heli_indi_gains.yaw_p * att_err.qz) - (heli_indi.rate_filt.r * heli_indi_gains.yaw_d);

  /* ------------------ */

  /* INDI YAW */
  /* Multiply with dt; this integrates virtual control (acceleration) to required change in rate */
  int32_t delta_thrust_meas = thrust_model_output_transferred - heli_indi.previous_thrust;
  heli_indi.previous_thrust = thrust_model_output_transferred;
  global_delta_thrust = delta_thrust_meas;

  int32_t delta_r_ref = yaw_virtual_control * 512/512;

  int32_t delta_r_error = delta_r_ref - (delta_rate_meas.r*512);
  int32_t delta_u_yaw = (delta_r_error * 512/512 + 69*delta_thrust_meas) / 31;

  /* Depending on direction, change filter coefficient */
  int32_t prev = heli_indi.tail_model.buffer[heli_indi.tail_model.idx];
  if(stabilization_cmd[COMMAND_YAW] - prev > 0) {
    // Tail spinning up
    heli_indi.tail_model.alpha = heli_indi.alpha_tail_inc;
  } else {
    // Tail spinning down
    heli_indi.tail_model.alpha = heli_indi.alpha_tail_dec;
  }
  int32_t yaw_model_output = heli_rate_filter_propagate(&heli_indi.tail_model, stabilization_cmd[COMMAND_YAW]);

  /* Only notch above freq threshold */
  if (rpm_sensor.motor_frequency > 25) {
    notch_filter_update(&heli_indi.r_inner_filter, &yaw_model_output, &heli_indi.inputmodel_notched.r);
  } else {
    heli_indi.inputmodel_notched.r = yaw_model_output;
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

  float delta_u_roll =  (59558.0 * (delta_p_error - 0) + 18324.0 / 4 * (delta_q_error + 0)) / 65536 /2;
  float delta_u_pitch = (-29230.0 / 4 * (delta_p_error - 0) + 34353.0 * (delta_q_error + 0)) / 65536 /2;

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
#endif

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
