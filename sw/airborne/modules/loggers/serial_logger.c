/*
 * Copyright (C) 2015 Bart Slinger <bartslinger@gmail.com>
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
 *
 */

/** @file modules/loggers/serial_logger.c
 *  @brief Basic serial logger.
 */

#include "serial_logger.h"
#include "mcu_periph/uart.h"

// Struct with all the variables for this module
struct serial_logger_struct serial_logger = {
  STATE_SENDING_DATA
};

void serial_logger_start(void)
{

}

void serial_logger_periodic(void)
{
  switch(serial_logger.state) {
    case STATE_NOT_SENDING:
      // do nothing
      break;
    case STATE_SENDING_DATA:
      if(uart_check_free_space(&SERIAL_LOG_UART, 13)){
        serial_logger_send_data();
      }
      else{
        serial_logger.state = STATE_TOO_MUCH_DATA;
      }
      break;
    case STATE_TOO_MUCH_DATA:
      if(uart_check_free_space(&SERIAL_LOG_UART, 5)){
        serial_logger_send_error();
        serial_logger.state = STATE_SENDING_DATA;
      }
      //
      break;
    default:
      break;
  }
}

void serial_logger_stop(void)
{

}

void serial_logger_send_data(void)
{
  // Send startbyte
  uart_transmit(&SERIAL_LOG_UART, 0xFF);

  // Send first 32-bit datafield
  for(uint8_t i=0; i<4; i++){
    uart_transmit(&SERIAL_LOG_UART, (imu.accel_unscaled.x >> 8*i) & 0xFF);
  }

  // Send second 32-bit datafield
  for(uint8_t i=0; i<4; i++){
    uart_transmit(&SERIAL_LOG_UART, (imu.accel_unscaled.y >> 8*i) & 0xFF);
  }

  // Send third 32-bit datafield
  for(uint8_t i=0; i<4; i++){
    uart_transmit(&SERIAL_LOG_UART, (imu.accel_unscaled.z >> 8*i) & 0xFF);
  }
}

void serial_logger_send_error(void)
{
  uart_transmit(&SERIAL_LOG_UART, 0xF0);
  for (uint8_t i=0; i<4; i++)
  {
    uart_transmit(&SERIAL_LOG_UART, 0x00);
  }
}
