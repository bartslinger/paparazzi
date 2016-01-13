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

/** @file filters/notch_filter.c
 *  @brief Second order notch filter
 *
 */

#include "filters/notch_filter.h"

void notch_filter_set_sampling_frequency(struct SecondOrderNotchFilter *filter, uint16_t frequency)
{
  filter->Ts = 1.0/frequency;
}

void notch_filter_set_bandwidth(struct SecondOrderNotchFilter *filter, float bandwidth)
{
  float d = exp(-M_PI*bandwidth*filter->Ts);
  filter->d2 = d*d;
}

void notch_filter_set_filter_frequency(struct SecondOrderNotchFilter *filter, float frequency)
{
  float theta = 2.0*M_PI*frequency*filter->Ts;
  filter->costheta = cos(theta);
}


/*
 * Notch filter update
 * y[n] = b * y[n-1] - d^2 * y[n-2] + a * x[n] - b * x[n-1] + a * x[n-2]
 *
 */
void notch_filter_update(struct SecondOrderNotchFilter *filter, int32_t *input_signal, int32_t *output_signal)
{
  float a = (1 + filter->d2) * 0.5;
  float b = (1 + filter->d2) * filter->costheta;
  *output_signal = (b * filter->yn1) - (filter->d2 * filter->yn2) + (a * *input_signal) - (b * filter->xn1) + (a * filter->xn2);

  /* Update values for next update */
  filter->xn2 = filter->xn1;
  filter->xn1 = *input_signal;
  filter->yn2 = filter->yn1;
  filter->yn1 = *output_signal;
}
