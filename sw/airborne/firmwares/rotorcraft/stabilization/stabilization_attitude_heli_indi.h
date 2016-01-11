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

#define GAIN_DIVIDER_P 256
#define GAIN_DIVIDER_D 256

struct HeliIndiGains {
  uint32_t roll_p;
  uint32_t pitch_p;
  uint32_t yaw_p;
  uint32_t yaw_d;
};

struct HeliIndiStab {

};

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern struct HeliIndiStab heli_indi_controller;
extern struct HeliIndiGains heli_indi_gains;

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
