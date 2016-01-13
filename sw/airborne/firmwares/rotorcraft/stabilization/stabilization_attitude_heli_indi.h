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
  struct Int32Rates rate_filt;
  struct Int32Rates rate_previous;
  struct heli_rate_filter_t tail_model;
  struct SecondOrderNotchFilter p_filter;
  struct SecondOrderNotchFilter q_filter;
  int32_t yawmodel_filtered;
  int16_t measured_cmd[3];
};

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern struct HeliIndiStab heli_indi;
extern struct HeliIndiGains heli_indi_gains;

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
