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
 * @file "modules/adc_expansion_uart/adc_expansion_uart.c"
 * @author Bart Slinger
 * Get analog signals from expansion board to main autopilot over uart
 */

#include "modules/adc_expansion_uart/adc_expansion_uart.h"
#include "subsystems/datalink/datalink.h"

uint16_t adc_uart_values[3];
uint16_t adc_uart_buffer[3];

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_adc_uart_values(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ADC_UART_DEBUG(trans, dev, AC_ID,
                               &adc_uart_values[0],
                               &adc_uart_values[1],
                               &adc_uart_values[2]);
}
#endif

void adc_expansion_uart_init() {
  adc_uart_values[0] = 0;
  adc_uart_values[1] = 0;
  adc_uart_values[2] = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ADC_UART_DEBUG, send_adc_uart_values);
#endif
}

/* Process message with ADC values */
void adc_expansion_uart_process_msg() {
  uint16_t new_data[3];

  new_data[0] = DL_ADC_DATA_adc_1(dl_buffer);
  new_data[1] = DL_ADC_DATA_adc_2(dl_buffer);
  new_data[2] = DL_ADC_DATA_adc_3(dl_buffer);

  for (uint8_t i = 0; i < 3; i++) {
    adc_uart_values[i] = new_data[i];
  }

  /* Filter spikes, only positive spikes exist */
  /*
  for (uint8_t i = 0; i < 3; i++) {
    int16_t diff1 = adc_uart_buffer[i] - adc_uart_values[i];
    int16_t diff2 = new_data[i] - adc_uart_buffer[i];
    if (diff1 > 40 && diff2 < 0) {
      // spike detected
    }
    else {
      // pass on values
      adc_uart_values[i] = adc_uart_buffer[i];
      adc_uart_buffer[i] = new_data[i];
    }
  }
  */

  /*
  for (uint8_t i=0; i<3;i++){
    if (diff > 40) {
      adc_uart_values[i] += 5;
    } else if (diff < -100) {
      adc_uart_values[i] -= 5;
    } else {
      adc_uart_values[i] = new_data[i];
    }
  }
  */
}


