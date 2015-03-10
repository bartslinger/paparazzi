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
  sd_logger.initialization_counter = 0;
  sd_logger.state = SdLoggerStateInitializing;
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

  if(!spi_submit(sd_logger.spi_p, &sd_logger.spi_t)){
    sd_logger_serial_println("SDinit failed.");
  }
}

void sd_logger_periodic(void)
{
  switch(sd_logger.state) {
    case SdLoggerStateInitializing:
      if (sd_logger.initialization_counter < 10){
        sd_logger.initialization_counter++;

        switch(sd_logger.try_card_type) {
          case SDTryV2:
            sd_logger_send_app_cmd(41, 0x40000000, SdResponseR1, &sd_logger_process_ACMD41_SDv2);
            break;
          case SDTryV1:
            // falltrough
          default:
            sd_logger_send_app_cmd(41, 0x00000000, SdResponseR1, &sd_logger_process_ACMD41_SDv1);
            break;
        }
      }
      break;
    case SdLoggerStateReady:
      sd_logger.state = SdLoggerStateIdle;
      break;
    case SdLoggerStateIdle:
      if(uart_char_available(&SD_LOG_UART) > 0) {
        if(uart_getch(&SD_LOG_UART) == 0x41) {
          sd_logger.state = SdLoggerStateRequestingData;
        }
      }
      break;
    case SdLoggerStateRequestingData:
      if(sd_logger.spi_t.status == SPITransSuccess || sd_logger.spi_t.status == SPITransDone){
        sd_logger_send_cmd(17, 0x00000000, SdResponseNone, &sd_logger_process_CMD17_request_single_byte);
      }
      break;
    case SdLoggerStateReadingData:
      if(sd_logger.spi_t.status == SPITransSuccess || sd_logger.spi_t.status == SPITransDone){
        sd_logger.spi_t.select = SPISelectUnselect;
        sd_logger.spi_t.output_length = 1;
        sd_logger.spi_t.input_length = 1;
        sd_logger.spi_t.output_buf[0] = 0xFF;
        sd_logger.spi_t.after_cb = &sd_logger_process_datarequest_single_byte;
        spi_submit(sd_logger.spi_p, &sd_logger.spi_t);
      }
      break;
    case SdLoggerStateFailed:
      // falltrough
    case SdLoggerStateDisabled:
      // falltrough
    default:
      // do nothing
      break;
  }

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

void sd_logger_send_cmd(uint8_t cmd, uint32_t arg, enum SdResponseType response_type, SPICallback after_cb)
{
  sd_logger.spi_t.select = SPISelectUnselect;
  switch(response_type) {
    case SdResponseNone:
      sd_logger.spi_t.input_length = 6;
      sd_logger.spi_t.output_length = 6;
      break;
    case SdResponseR3:
      // falltrough
    case SdResponseR7:
      sd_logger.spi_t.input_length = 19;
      sd_logger.spi_t.output_length = 19;
      break;
    default:
      // falltrough
    case SdResponseR1:
      sd_logger.spi_t.input_length = 15;
      sd_logger.spi_t.output_length = 15;
      break;
  }

  sd_logger.output_buf[0] = 0x40 | cmd;
  sd_logger.output_buf[1] = arg >> 24;
  sd_logger.output_buf[2] = arg >> 16;
  sd_logger.output_buf[3] = arg >> 8;
  sd_logger.output_buf[4] = arg;
  switch(cmd) {
    case 0:
      sd_logger.output_buf[5] = 0x95;
      break;
    case 8:
      sd_logger.output_buf[5] = 0x87;
      break;
    default:
      sd_logger.output_buf[5] = 0x01;
  }

  for(uint8_t i=6; i<19; i++){
    sd_logger.output_buf[i] = 0xFF;
  }

  sd_logger.spi_t.after_cb = after_cb;
  if(!spi_submit(sd_logger.spi_p, &sd_logger.spi_t)){
    sd_logger_serial_println("SDcmd failed.");
  }
}

void sd_logger_send_app_cmd(uint8_t cmd, uint32_t arg, enum SdResponseType response_type, SPICallback after_cb)
{
  sd_logger.spi_t.select = SPISelectUnselect;

  switch(response_type) {
    case SdResponseR7:
      sd_logger.spi_t.input_length = 34;
      sd_logger.spi_t.output_length = 34;
      break;
    default:
      // falltrough
    case SdResponseR1:
      sd_logger.spi_t.input_length = 30;
      sd_logger.spi_t.output_length = 30;
      break;
  }

  sd_logger.output_buf[0] = 0x77; // CMD55
  sd_logger.output_buf[1] = 0x00;
  sd_logger.output_buf[2] = 0x00;
  sd_logger.output_buf[3] = 0x00;
  sd_logger.output_buf[4] = 0x00;
  sd_logger.output_buf[5] = 0x01; // End CMD55
  sd_logger.output_buf[6] = 0xFF;
  sd_logger.output_buf[7] = 0xFF;
  sd_logger.output_buf[8] = 0xFF;
  sd_logger.output_buf[9] = 0xFF;
  sd_logger.output_buf[10] = 0xFF;
  sd_logger.output_buf[11] = 0xFF;
  sd_logger.output_buf[12] = 0xFF;
  sd_logger.output_buf[13] = 0xFF;
  sd_logger.output_buf[14] = 0xFF;

  sd_logger.output_buf[15] = 0x40 + cmd;
  sd_logger.output_buf[16] = arg >> 24;
  sd_logger.output_buf[17] = arg >> 16;
  sd_logger.output_buf[18] = arg >> 8;
  sd_logger.output_buf[19] = arg;
  sd_logger.output_buf[20] = 0x01;

  for(uint8_t i=21; i<34; i++){
    sd_logger.output_buf[i] = 0xFF;
  }
  sd_logger.spi_t.after_cb = after_cb;
  if(!spi_submit(sd_logger.spi_p, &sd_logger.spi_t)){
    sd_logger_serial_println("SDappcmd failed.");
  }

}

uint8_t sd_logger_get_response_idx(void)
{
  for(uint8_t i=6; i<15; i++){
    if(sd_logger.spi_t.input_buf[i] != 0xFF){
      return i;
      break;
    }
  }
  return 6; // return 0xFF
}

uint8_t sd_logger_get_R1(void)
{
  return sd_logger.spi_t.input_buf[sd_logger_get_response_idx()];
}

uint32_t sd_logger_get_R7(void)
{
  uint8_t response_idx = sd_logger_get_response_idx();
  return sd_logger.spi_t.input_buf[response_idx+1] << 24 |
         sd_logger.spi_t.input_buf[response_idx+2] << 16 |
         sd_logger.spi_t.input_buf[response_idx+3] << 8  |
         sd_logger.spi_t.input_buf[response_idx+4];
}

uint32_t sd_logger_get_R3(void)
{
  return sd_logger_get_R7();
}

uint8_t sd_logger_get_ACMD_R1(void)
{
  uint8_t response = 0xFF;
  for (uint8_t i=21; i<30; i++){
    if(sd_logger.spi_t.input_buf[i] != 0xFF){
      response = sd_logger.spi_t.input_buf[i];
      break;
    }
  }
  return response;
}

void sd_logger_send_CMD0(struct spi_transaction *t)
{
  (void) t; // ignore unused warning
  sd_logger_send_cmd(0, 0x00000000, SdResponseR1, &sd_logger_get_CMD0_response);
}

void sd_logger_get_CMD0_response(struct spi_transaction *t)
{
  (void) t; // ignore unused warning
  if(sd_logger_get_R1() == 0x01){
    // Correct response from CMD0, continue with CMD8
    sd_logger_send_cmd(8, 0x000001AA, SdResponseR7, &sd_logger_process_CMD8);
  }
  else {
    sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_CMD8(struct spi_transaction *t)
{
  (void) t; // ignore unused warning
  if(sd_logger_get_R1() != 0x01){
    sd_logger.try_card_type = SDTryV1;
    sd_logger.state = SdLoggerStateInitializing;
  }
  else if(sd_logger_get_R7() == 0x000001AA){
    sd_logger.try_card_type = SDTryV2;
    sd_logger.state = SdLoggerStateInitializing;
  }
  else{
    sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_CMD16(struct spi_transaction *t)
{
  (void) t; // ignore unused warning
  if(sd_logger_get_R1() == 0x00) {
    sd_logger.state = SdLoggerStateReady;
  }
  else {
    sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_ACMD41_SDv2(struct spi_transaction *t)
{
  (void) t;
  uint8_t response = sd_logger_get_ACMD_R1();
  switch(response) {
    case 0x00:
      sd_logger.state = SdLoggerStateInitialized;
      sd_logger_send_cmd(58, 0x00000000, SdResponseR3, &sd_logger_process_CMD58);
      break;
    case 0x01:
      break;
    default:
      sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_ACMD41_SDv1(struct spi_transaction *t)
{
  (void) t; // ignore unused
  uint8_t response = sd_logger_get_ACMD_R1();
  switch(response) {
    case 0x00:
      sd_logger.state = SdLoggerStateInitialized;
      sd_logger.card_type = CardSdV1;
      sd_logger_send_cmd(16, 0x00000200, SdResponseR1, &sd_logger_process_CMD16);
      break;
    case 0x01:
      break;
    default:
      sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_CMD58(struct spi_transaction *t)
{
  (void) t; // ignore unused
  uint32_t response = sd_logger_get_R3();
  if(response & (1 << 31)){
    if(response & (1 << 30)){
      sd_logger.card_type = CardSdV2block;
      sd_logger.state = SdLoggerStateReady;
    }
    else{
      sd_logger.card_type = CardSdV2byte;
      sd_logger_send_cmd(16, 0x00000200, SdResponseR1, &sd_logger_process_CMD16);
    }
  }
  else{
    sd_logger.card_type = CardUnknown;
    sd_logger.state = SdLoggerStateFailed;
  }
}

void sd_logger_process_CMD17_request_single_byte(struct spi_transaction *t)
{
  (void) t; // ignore unused warning
  sd_logger.spi_t.select = SPISelectUnselect;
  sd_logger.spi_t.output_length = 1;
  sd_logger.spi_t.input_length = 1;
  sd_logger.spi_t.output_buf[0] = 0xFF;

  sd_logger.spi_t.after_cb = &sd_logger_process_CMD17_single_byte;

  spi_submit(sd_logger.spi_p, &sd_logger.spi_t);
}

void sd_logger_process_CMD17_single_byte(struct spi_transaction *t)
{
  if (sd_logger.try_counter < 9) {
    sd_logger.try_counter++;
    switch(t->input_buf[0]) {
      case 0xFF:
        sd_logger_process_CMD17_request_single_byte(t);
        break;
      case 0x00:
        sd_logger.state = SdLoggerStateReadingData;
        break;
      default:
        sd_logger_serial_println("CMD17 wrong response.");
    }
  }
  else {
    sd_logger_serial_println("CMD17 no response.");
  }
}

void sd_logger_process_datarequest_single_byte(struct spi_transaction *t)
{
  (void) t; // ignore unused parameter
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
