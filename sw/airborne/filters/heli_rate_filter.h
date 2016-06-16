/*
 * Copyright (C) 2015 Bart Slinger
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

/** @file filters/heli_rate_filter.h
 *  @brief First order low-pass filter with delay
 *
 */

#ifndef HELI_RATE_FILTER_H
#define HELI_RATE_FILTER_H

#include "paparazzi.h"

#define HELI_RATE_FILTER_BUFFER_SIZE 20

struct heli_rate_filter_t {
  uint32_t omega;
  uint8_t delay;
  int32_t alpha;
  uint16_t max_inc;
  int32_t buffer[HELI_RATE_FILTER_BUFFER_SIZE];
  uint8_t idx;
};

extern void heli_rate_filter_initialize(struct heli_rate_filter_t *f, uint32_t omega, uint8_t delay, uint16_t max_inc);
extern int32_t heli_rate_filter_propagate(struct heli_rate_filter_t *f, int32_t input);
extern void heli_rate_filter_set_omega(struct heli_rate_filter_t *f, uint32_t omega);
extern void heli_rate_filter_set_delay(struct heli_rate_filter_t *f, uint8_t delay);
#endif
