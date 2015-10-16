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
 * @file "modules/loggers/sdlogger_spi_direct.c"
 * @author Bart Slinger
 * SPI SD Logger that saves pprzlog messages to SD Card.
 */

#include "modules/loggers/sdlogger_spi_direct.h"


struct sdlogger_spi_periph sdlogger_spi;

void sdlogger_spi_direct_init(void) {
  sdlogger_spi.device.check_free_space = &sdlogger_spi_direct_check_free_space;
  sdlogger_spi.device.put_byte = &sdlogger_spi_direct_put_byte;
  sdlogger_spi.device.send_message = &sdlogger_spi_direct_send_message;
  sdlogger_spi.device.char_available = &sdlogger_spi_direct_char_available;
  sdlogger_spi.device.get_byte = &sdlogger_spi_direct_get_byte;
  sdlogger_spi.device.periph = &sdlogger_spi;
}
void sdlogger_spi_direct_periodic(void) {}

void sdlogger_spi_direct_start(void) {}
void sdlogger_spi_direct_stop(void) {}

int sdlogger_spi_direct_check_free_space(void *p, uint8_t len)
{
  (void) p, (void) len;
  return 0;
}

void sdlogger_spi_direct_put_byte(void *p, uint8_t data)
{
  (void) p, (void) data;
}

void sdlogger_spi_direct_send_message(void *p)
{
  (void) p;
}

int sdlogger_spi_direct_char_available(void *p){
  (void) p;
  return 0;
}

uint8_t sdlogger_spi_direct_get_byte(void *p)
{
  (void) p;
  return 0;
}
