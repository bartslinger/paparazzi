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

/** @file modules/loggers/sd_logger.c
 *  @brief Basic sd logger.
 */

#include "peripherals/sdcard.h"
#include "sd_logger.h"

extern struct SdCard sdcard_logger;

bool_t uglyGlobalVariable = FALSE;

void sd_logger_start(void)
{
  sdcard_init(&sdcard_logger, &spi2, SPI_SLAVE3);
}

void sd_logger_periodic(void)
{
  sdcard_periodic(&sdcard_logger);

  if (sdcard_logger.status == SdCard_Idle && uglyGlobalVariable == FALSE) {
    uglyGlobalVariable = TRUE;
    for (uint16_t i=0; i<512; i++) {
      sdcard_logger.output_buf[i+1] = i;
    }
    sdcard_write_block(&sdcard_logger, 0x00000020);
  }
}

void sd_logger_stop(void)
{

}

