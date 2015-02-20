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

#define SERIAL_LOGGER_NOT_SENDING 0
#define SERIAL_LOGGER_SENDING_DATA 1
#define SERIAL_LOGGER_TOO_MUCH_DATA 2

#include "std.h"
#include "subsystems/imu.h"

enum serial_logger_state{
  STATE_NOT_SENDING = 1,
  STATE_SENDING_DATA = 2,
  STATE_TOO_MUCH_DATA = 3
};

struct serial_logger_struct {
  enum serial_logger_state state;
};
extern struct serial_logger_struct serial_logger;

extern void serial_logger_start(void);
extern void serial_logger_periodic(void);
extern void serial_logger_stop(void);

extern void serial_logger_send_data(void);
extern void serial_logger_send_error(void);

#endif /* SERIAL_LOGGER_H_ */
