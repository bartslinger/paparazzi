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
#include "subsystems/datalink/telemetry.h"
#include "subsystems/imu.h"
#include "sd_logger.h"

// defined in sdcard.c
extern struct SdCard sdcard1;

uint8_t uglyGlobalVariable = 0;

void sd_logger_start(void)
{
  sdcard_init(&sdcard1, &spi2, SPI_SLAVE3);
}

void sd_logger_periodic(void)
{
  sdcard_periodic(&sdcard1);

  if (sdcard1.status == SdCard_Idle && uglyGlobalVariable == 0) {
    uglyGlobalVariable++;
    for (uint16_t i=0; i<512; i++) {
      sdcard1.output_buf[i+1] = i;
    }
    sdcard_write_block(&sdcard1, 0x00000020);
  }
  if (sdcard1.status == SdCard_Idle && uglyGlobalVariable == 1) {
    uglyGlobalVariable++;

    sdcard_read_block(&sdcard1, 0x00000020);
  }
  if (sdcard1.status == SdCard_Idle && uglyGlobalVariable == 2) {
    uglyGlobalVariable++;

    sdcard_read_block(&sdcard1, 0x00000020);
  }
  RunOnceEvery(30, DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r));
}

void sd_logger_stop(void)
{
}

