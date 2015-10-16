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
 * @file "modules/loggers/sdlogger_spi_direct.h"
 * @author Bart Slinger
 * SPI SD Logger that saves pprzlog messages to SD Card.
 */

#ifndef SDLOGGER_SPI_H
#define SDLOGGER_SPI_H

#include "mcu_periph/link_device.h"

struct sdlogger_spi_periph{
    struct link_device device;
};

extern struct sdlogger_spi_periph sdlogger_spi;

extern void sdlogger_spi_direct_init(void);
extern void sdlogger_spi_direct_periodic(void);

extern void sdlogger_spi_direct_start(void);
extern void sdlogger_spi_direct_stop(void);

extern int sdlogger_spi_direct_check_free_space(void *p, uint8_t len);
extern void sdlogger_spi_direct_put_byte(void *p, uint8_t data);
extern void sdlogger_spi_direct_send_message(void *p);
extern int sdlogger_spi_direct_char_available(void *p);
extern uint8_t sdlogger_spi_direct_get_byte(void *p);

#endif

