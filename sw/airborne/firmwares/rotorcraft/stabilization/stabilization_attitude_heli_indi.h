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

#ifndef STABILIZATION_ATTITUDE_QUAT_INT_H
#define STABILIZATION_ATTITUDE_QUAT_INT_H

#include "math/pprz_algebra_int.h"

#include "filters/notch_filter.h"
#include "filters/heli_rate_filter.h"

#define GAIN_MULTIPLIER_P 12
#define GAIN_MULTIPLIER_D 3

struct HeliIndiGains {
  int32_t roll_p;
  int32_t pitch_p;
  int32_t yaw_p;
  int32_t yaw_d;
};

struct HeliIndiStab {
  float heli_roll_effectiveness_inv;
  int32_t yawrate_setpoint;
  int32_t yawrate_err;
  int32_t filter_out;
  int32_t yaw_incremental_cmd;
  int32_t previous_thrust;
  struct Int32Rates rate_notched;
  struct Int32Rates rate_filt;
  struct Int32Rates rate_previous;
  struct heli_rate_filter_t tail_model;
  int32_t alpha_tail_inc;
  int32_t alpha_tail_dec;
  struct heli_rate_filter_t roll_model; // haha woordgrapje
  struct heli_rate_filter_t pitch_model;
  struct SecondOrderNotchFilter p_filter;
  struct SecondOrderNotchFilter q_filter;
  struct SecondOrderNotchFilter r_filter;
  struct SecondOrderNotchFilter p_inner_filter;
  struct SecondOrderNotchFilter q_inner_filter;
  struct SecondOrderNotchFilter r_inner_filter;
  struct Int32Rates inputmodel_notched;
  struct Int32Rates inputmodel_filtered;
  int32_t measured_cmd[3];
};


#define __k 1
#define INDI_NR_FILTERS 2
#define INDI_DOF 4
#define INDI_ROLL 0
#define INDI_PITCH 1
#define INDI_YAW 2
#define INDI_THRUST 3

/* All these values are in the struct to make it easier for logging */
struct IndiController_int {
  int32_t reference[INDI_DOF];                              ///< Range -MAX_PPRZ:MAX_PPRZ
  int32_t dynamics_compensated_measurement[INDI_DOF];
  int32_t error[INDI_DOF];                                  ///<
  int32_t invG[INDI_DOF][INDI_DOF];                         ///< Inverse control effectiveness matrix
  int32_t D[INDI_DOF][INDI_DOF];                            ///< Dynamics matrix, use identity matrix if not compensating for dynamics
  int32_t du[INDI_DOF];
  int32_t u_setpoint[INDI_DOF];                             ///< Actuator setpoint without compensator
  void (*apply_compensator_filters)(int32_t _out[], int32_t _in[]);
  int32_t command_out[2][INDI_DOF];                         ///< Command and command from previous measurement
  void (*apply_actuator_models)(int32_t _out[], int32_t _in[]);
  void (*apply_actuator_filters[INDI_NR_FILTERS])(int32_t _out[], int32_t _in[]);
  int32_t actuator_out[INDI_DOF];
  void (*apply_measurement_filters[INDI_NR_FILTERS])(int32_t _out[], int32_t _in[]);
  int32_t filtered_actuator[INDI_NR_FILTERS][INDI_DOF];
  int32_t measurement[INDI_DOF];
  int32_t filtered_measurement[INDI_NR_FILTERS][INDI_DOF];
  int32_t roll_comp_angle;                                        ///< Angle to rotate pitch/roll commands with INT32_ANGLE_FRAC
  int32_t pitch_comp_angle;                                        ///< Angle to rotate pitch/roll commands with INT32_ANGLE_FRAC
  uint32_t roll_omega;
  uint32_t roll_delay;
  bool_t use_roll_dyn_filter;
  uint16_t dist_magnitude;                                  ///< Magnitude of roll disturbance
  bool_t add_disturbance;                                   ///< Wether or not to add this disturbance, set by external module
  float rollfilt_bw;                                        ///< Bandwidth of the roll measurement filter
  bool_t enable_notch;                                      ///< Use notch filters
};

extern struct IndiController_int new_heli_indi;

extern struct heli_rate_filter_t actuator_model[INDI_DOF];
extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern float sp_offset_roll;
extern float sp_offset_pitch;
extern struct HeliIndiStab heli_indi;
extern struct HeliIndiGains heli_indi_gains;

extern void stabilization_attitude_heli_indi_set_steadystate_pitch(float pitch);
extern void stabilization_attitude_heli_indi_set_steadystate_roll(float roll);
extern void stabilization_attitude_heli_indi_set_steadystate_pitchroll(void);

extern void stabilization_attitude_heli_indi_set_roll_omega(uint32_t omega);
extern void stabilization_attitude_heli_indi_set_roll_delay(uint8_t delay);
extern void stabilization_attitude_heli_indi_set_rollfilter_bw(float bandwidth);

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
