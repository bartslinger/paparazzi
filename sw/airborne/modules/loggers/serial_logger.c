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

void serial_logger_start(void){
  uart_periph_init(&SERIAL_LOG_UART); // defined from xml
  uart_periph_set_bits_stop_parity(&SERIAL_LOG_UART, 8, 1, 0);
}

void serial_logger_periodic(void){
  uart_transmit(&SERIAL_LOG_UART, 75);
  uart_transmit(&SERIAL_LOG_UART, 111);
  uart_transmit(&SERIAL_LOG_UART, 110);
  uart_transmit(&SERIAL_LOG_UART, 105);
  uart_transmit(&SERIAL_LOG_UART, 110);
  uart_transmit(&SERIAL_LOG_UART, 103);
  uart_transmit(&SERIAL_LOG_UART, 115);
  uart_transmit(&SERIAL_LOG_UART, 33);
  uart_transmit(&SERIAL_LOG_UART, 10);
}

void serial_logger_stop(void){

}
