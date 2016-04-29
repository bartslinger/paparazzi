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
 * @file "modules/roll_disturbance/roll_disturbance.c"
 * @author Bart Slinger
 * disturbance roll command
 */

#include "modules/roll_disturbance/roll_disturbance.h"

uint32_t dist_counter = 0;
bool_t dist_is_counting = FALSE;

void roll_disturbance_periodic()
{
  if (dist_is_counting) {
    if (dist_counter > 2*512) {
      new_heli_indi.add_disturbance = FALSE;
      dist_is_counting = FALSE;
    }
    else if (dist_counter > 256) {
      new_heli_indi.add_disturbance = TRUE;
    }
    else {
      new_heli_indi.add_disturbance = FALSE;
    }
    dist_counter++;
  }
}


bool_t roll_disturbance_restart()
{
  dist_counter = 0;
  dist_is_counting = TRUE;
  return FALSE;
}
