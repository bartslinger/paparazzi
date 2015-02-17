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

/** @file modules/loggers/serial_logger.h
 *  @brief Basic serial logger.
 */

#ifndef SERIAL_LOGGER_H_
#define SERIAL_LOGGER_H_

// Translates SERIAL_LOGGER_USE_UART to USE_UART1 as defined in serial_logger.xml
#define __SERIAL_LOGGER_USE_UART(port) USE_##port
#define _SERIAL_LOGGER_USE_UART(port) __SERIAL_LOGGER_USE_UART(port)
#define SERIAL_LOGGER_USE_UART _SERIAL_LOGGER_USE_UART(SERIAL_LOG_UART_UPPER)

// Translates SERIAL_LOGGER_BAUD to B115200 as defined in serial_logger.xml
#define __SERIAL_LOGGER_BAUD(port) port##_BAUD
#define _SERIAL_LOGGER_BAUD(port) __SERIAL_LOGGER_BAUD(port)
#define SERIAL_LOGGER_BAUD _SERIAL_LOGGER_BAUD(SERIAL_LOG_UART_UPPER)

#include "std.h"

struct serial_logger_struct {
    bool_t bufferOverrun;
};
extern struct serial_logger_struct serial_logger;

extern void serial_logger_start(void);
extern void serial_logger_periodic(void);
extern void serial_logger_stop(void);

#endif /* SERIAL_LOGGER_H_ */
