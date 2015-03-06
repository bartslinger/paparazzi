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

#include "sd_logger.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

// Struct with all the variables for this module
struct SdLogger sd_logger;

void sd_logger_start(void)
{
  sd_logger_spi_init(&sd_logger, &(SD_LOGGER_SPI_LINK_DEVICE), SD_LOGGER_SPI_LINK_SLAVE_NUMBER);
  sd_logger.output_buf[0] = 0xFF;
  sd_logger.output_buf[1] = 0xFF;
  sd_logger.output_buf[2] = 0xFF;
  sd_logger.output_buf[3] = 0xFF;
  sd_logger.output_buf[4] = 0xFF;
  sd_logger.output_buf[5] = 0xFF;
  sd_logger.output_buf[6] = 0xFF;
  sd_logger.output_buf[7] = 0xFF;
  sd_logger.output_buf[8] = 0xFF;
  sd_logger.output_buf[9] = 0xFF;
  sd_logger.spi_t.input_length = 10;
  sd_logger.spi_t.output_length = 0;
  sd_logger.spi_t.select = SPINoSelect;
  spi_submit(&(SD_LOGGER_SPI_LINK_DEVICE), &sd_logger.spi_t);
}

void sd_logger_periodic(void)
{
  if(sd_logger.alternator){
#if 1
    sd_logger.output_buf[0] = 0x40 | 17;
    sd_logger.output_buf[1] = 0x00;
    sd_logger.output_buf[2] = 0x00;
    sd_logger.output_buf[3] = 0x00;
    sd_logger.output_buf[4] = 0x00;
    sd_logger.output_buf[5] = 0x95;
    sd_logger.spi_t.input_length = 700;
    sd_logger.spi_t.output_length = 6;
    sd_logger.spi_t.select = SPISelectUnselect;
#endif
  }
  else{
    // CMD0
    sd_logger.output_buf[0] = (1 << 6);
    sd_logger.output_buf[1] = 0x00;
    sd_logger.output_buf[2] = 0x00;
    sd_logger.output_buf[3] = 0x00;
    sd_logger.output_buf[4] = 0x00;
    sd_logger.output_buf[5] = 0x95;
    sd_logger.spi_t.input_length = 8;
    sd_logger.spi_t.output_length = 6;
    sd_logger.spi_t.select = SPISelectUnselect;
  }
  spi_submit(&(SD_LOGGER_SPI_LINK_DEVICE), &sd_logger.spi_t);
  sd_logger.alternator = !sd_logger.alternator;
}

void sd_logger_stop(void)
{

}

void sd_logger_serial_println(const char text[])
{
  uint8_t i = 0;
  while(text[i] != 0x00){
    uart_transmit(&SD_LOG_UART, text[i]);
    i++;
  }
  uart_transmit(&SD_LOG_UART, 0x0A); // send "\n" character
}

void sd_logger_spi_init(struct SdLogger *sdlog, struct spi_periph *spi_p, uint8_t slave_idx)
{
  sdlog->alternator = TRUE;
  /* Set spi_peripheral */
  sdlog->spi_p = spi_p;

  /* Set the spi transaction */
  sdlog->spi_t.select    = SPINoSelect;
  sdlog->spi_t.status    = SPITransDone;
  sdlog->spi_t.cpol      = SPICpolIdleLow;
  sdlog->spi_t.cpha      = SPICphaEdge1;
  sdlog->spi_t.dss       = SPIDss8bit;
  sdlog->spi_t.bitorder  = SPIMSBFirst;
  sdlog->spi_t.cdiv      = SPIDiv32;
  sdlog->spi_t.slave_idx = slave_idx;

  sdlog->spi_t.input_length  = 10;
  sdlog->spi_t.output_length = 10;
  sdlog->spi_t.input_buf     = sdlog->input_buf;
  sdlog->spi_t.output_buf    = sdlog->output_buf;
}
