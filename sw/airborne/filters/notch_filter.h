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

/** @file filters/notch_filter.h
 *  @brief Second order notch filter
 *
 */

#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include "std.h"

struct SecondOrderNotchFilter {
  float Ts;
  float d2;
  float costheta;
  int32_t xn1;
  int32_t xn2;
  int32_t yn1;
  int32_t yn2;
};

extern void notch_filter_set_sampling_frequency(struct SecondOrderNotchFilter *filter, uint16_t frequency);
extern void notch_filter_set_bandwidth(struct SecondOrderNotchFilter *filter, float bandwidth);
extern void notch_filter_set_filter_frequency(struct SecondOrderNotchFilter *filter, float frequency);
extern void notch_filter_update(struct SecondOrderNotchFilter *filter, int32_t *input_signal, int32_t *output_signal);

#endif
