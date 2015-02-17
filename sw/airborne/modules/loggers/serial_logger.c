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
struct serial_logger_struct serial_logger;

void serial_logger_start(void)
{

}

void serial_logger_periodic(void)
{
    if(uart_check_free_space(&SERIAL_LOG_UART, 13*8))
    {
        if(serial_logger.bufferOverrun)
        {
            // Send error message
            uart_transmit(&SERIAL_LOG_UART, 0xF0);
            for (uint8_t i=0; i<12; i++)
            {
                uart_transmit(&SERIAL_LOG_UART, 0x00);
            }
            // Erorr has been sent, can send new data from now on
            serial_logger.bufferOverrun = FALSE;
        }
        else{
            // Send startbyte
            uart_transmit(&SERIAL_LOG_UART, 0xFF);

            // Send first 32-bit datafield
            uart_transmit(&SERIAL_LOG_UART, 0x00);
            uart_transmit(&SERIAL_LOG_UART, 0x01);
            uart_transmit(&SERIAL_LOG_UART, 0x10);
            uart_transmit(&SERIAL_LOG_UART, 0x20);

            // Send second 32-bit datafield
            uart_transmit(&SERIAL_LOG_UART, 0x05);
            uart_transmit(&SERIAL_LOG_UART, 0x11);
            uart_transmit(&SERIAL_LOG_UART, 0x32);
            uart_transmit(&SERIAL_LOG_UART, 0x33);

            // Send third 32-bit datafield
            uart_transmit(&SERIAL_LOG_UART, 0x66);
            uart_transmit(&SERIAL_LOG_UART, 0x44);
            uart_transmit(&SERIAL_LOG_UART, 0x0F);
            uart_transmit(&SERIAL_LOG_UART, 0xFF);
        }
    }
    else
    {
        serial_logger.bufferOverrun = TRUE;
    }
}

void serial_logger_stop(void)
{

}
