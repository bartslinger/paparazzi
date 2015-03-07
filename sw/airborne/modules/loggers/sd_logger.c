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
  sd_logger_setup_spi();
  sd_logger.spi_t.input_length = 0;
  sd_logger.spi_t.output_length = 10;

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

  sd_logger.spi_t.after_cb = &sd_logger_send_CMD0;

  spi_submit(sd_logger.spi_p, &sd_logger.spi_t);
}

void sd_logger_periodic(void)
{

}

void sd_logger_stop(void)
{

}

void sd_logger_setup_spi(void)
{

  /* Set spi_peripheral */
  sd_logger.spi_p = &(SD_LOGGER_SPI_LINK_DEVICE);

  /* Set the spi transaction */
  sd_logger.spi_t.select    = SPINoSelect;
  sd_logger.spi_t.status    = SPITransDone;
  sd_logger.spi_t.cpol      = SPICpolIdleLow;
  sd_logger.spi_t.cpha      = SPICphaEdge1;
  sd_logger.spi_t.dss       = SPIDss8bit;
  sd_logger.spi_t.bitorder  = SPIMSBFirst;
  sd_logger.spi_t.cdiv      = SPIDiv64;
  sd_logger.spi_t.slave_idx = SD_LOGGER_SPI_LINK_SLAVE_NUMBER;

  sd_logger.spi_t.input_length = 0;
  sd_logger.spi_t.output_length = 0;
  sd_logger.spi_t.input_buf = sd_logger.input_buf;
  sd_logger.spi_t.output_buf = sd_logger.output_buf;
}

void sd_logger_send_CMD0(struct spi_transaction *t)
{
  t->after_cb = NULL;
  t->select = SPISelectUnselect;
  t->output_length = 6;
  t->input_length = 15;
  t->after_cb = &sd_logger_get_CMD0_response;

  sd_logger.output_buf[0] = 0x40;
  sd_logger.output_buf[1] = 0x00;
  sd_logger.output_buf[2] = 0x00;
  sd_logger.output_buf[3] = 0x00;
  sd_logger.output_buf[4] = 0x00;
  sd_logger.output_buf[5] = 0x95;

  spi_submit(sd_logger.spi_p, t);
}

void sd_logger_get_CMD0_response(struct spi_transaction *t)
{
  for(uint8_t i=6; i<15; i++){
    if(t->input_buf[i] != 0xFF){

      // Correct response from CMD0, continue with CMD8
      sd_logger.spi_t.output_length = 6;
      sd_logger.spi_t.input_length = 19;
      sd_logger.output_buf[0] = 0x40 + 8;
      sd_logger.output_buf[1] = 0x00;
      sd_logger.output_buf[2] = 0x00;
      sd_logger.output_buf[3] = 0x01;
      sd_logger.output_buf[4] = 0xAA;
      sd_logger.output_buf[5] = 0x87;

      spi_submit(sd_logger.spi_p, t);

      return;
    }
  }
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



