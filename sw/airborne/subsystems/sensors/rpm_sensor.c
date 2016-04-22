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
 * @file "modules/rpm_sensor/rpm_sensor.c"
 * @author Bart Slinger
 * Measure the ppm signal of the RPM sensor
 */

// Telemetry to test values
#include "subsystems/datalink/telemetry.h"

#ifdef TEST
/* The original messages.h uses inline functions, which are incompatible with cmock. TEST is defined by the unittest framework */
#include "messages_testable.h"
#endif // TEST

#include "subsystems/sensors/rpm_sensor.h"

struct RpmSensor rpm_sensor;
int32_t thrust_model_output_transferred;

/* Re-use TCAS_DEBUG message for logging RPM which is also float */
static void send_rpm(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TCAS_DEBUG(trans, dev, AC_ID, 0, &rpm_sensor.motor_frequency);
}

void rpm_sensor_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TCAS_DEBUG, send_rpm);
  rpm_sensor_arch_init();
}

#define MIN_DIFF_MEASURE 3000
void rpm_sensor_process_pulse(uint16_t cnt, uint8_t overflow_cnt)
{
  uint16_t diff = cnt - rpm_sensor.previous_cnt;

  if ((cnt > rpm_sensor.previous_cnt && overflow_cnt > 0) || (overflow_cnt > 1)) {
    rpm_sensor.motor_frequency = 0.0f;
  } else {
    rpm_sensor.motor_frequency = (8*200000.0/diff)/18;
  }

  /* Spike filter (spikes in rpm measurement removed by limiting maximum inc/dec) */
  static float previous_motor_frequency = 0.0f;
  float freqdiff = rpm_sensor.motor_frequency - previous_motor_frequency;
  /* Difference of 0.25 corresponds to 128 Hz / second */
  /* Hard-coded for 512 Hz */
  /* Yes this is beuned */

  if (freqdiff > 0.25) {
    rpm_sensor.motor_frequency = previous_motor_frequency + 0.25;
  }
  else if (freqdiff < -0.25) {
    rpm_sensor.motor_frequency = previous_motor_frequency - 0.25;
  }
  previous_motor_frequency = rpm_sensor.motor_frequency;

  /* Remember count */
  rpm_sensor.previous_cnt = cnt;
}


