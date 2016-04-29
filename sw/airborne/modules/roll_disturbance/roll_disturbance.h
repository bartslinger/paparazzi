/*
 * Copyright (C) Bart Slinger
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/roll_disturbance/roll_disturbance.h"
 * @author Bart Slinger
 * disturbance roll command
 */

#ifndef ROLL_DISTURBANCE_H
#define ROLL_DISTURBANCE_H

#include "stabilization/stabilization_attitude_heli_indi.h"


extern void roll_disturbance_periodic(void);
extern bool_t roll_disturbance_restart(void);
#endif

