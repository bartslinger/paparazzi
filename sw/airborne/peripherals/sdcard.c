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

/** @file peripherals/sdcard->c
 *  @brief TODO.
 */

#include "sdcard.h"

struct SdCard sdcard;

void sdcard_init(struct SdCard *sdcard, struct spi_periph *spi_p, const uint8_t slave_idx)
{
  sdcard->spi_p = spi_p;
  sdcard->spi_t.slave_idx = slave_idx;
  sdcard->spi_t.select = SPISelectUnselect;
  sdcard->spi_t.status = SPITransDone;
  sdcard->spi_t.cpol = SPICpolIdleLow;
  sdcard->spi_t.cpha = SPICphaEdge1;
  sdcard->spi_t.dss = SPIDss8bit;
  sdcard->spi_t.bitorder = SPIMSBFirst;
  sdcard->spi_t.cdiv = SPIDiv64;
  sdcard->spi_t.input_buf = sdcard->input_buf;
  sdcard->spi_t.output_buf = sdcard->output_buf;
  sdcard->spi_t.input_length = 0;
  sdcard->spi_t.output_length = 0;

  sdcard->status = SdCard_BeforeDummyClock;
}


void sdcard_periodic(struct SdCard *sdcard)
{

  /* Do nothing if spi transaction is in progress */
  if (sdcard->spi_t.status == SPITransPending || sdcard->spi_t.status == SPITransRunning) {
    return;
  }

  switch(sdcard->status) {

    /* Send dummy clock to set SD card in SPI mode */
    case SdCard_BeforeDummyClock:
      sdcard->spi_t.select = SPINoSelect;
      sdcard->spi_t.output_length = 10;
      sdcard->spi_t.input_length = 0;
      for (uint8_t i=0; i<10; i++) {
        sdcard->output_buf[i] = 0xFF;
      }
      sdcard->spi_t.after_cb = &sdcard_spicallback;
      spi_submit(sdcard->spi_p, &sdcard->spi_t);
      sdcard->status = SdCard_SendingDummyClock;
      break;

    /* Sending ACMD41 */
    case SdCard_SendingACMD41v2:
      sdcard_send_app_cmd(sdcard, 41, 0x40000000);
      break;

    /* Should not reach this */
    default:
      break;
  }

}


void sdcard_spicallback(struct spi_transaction *t)
{
  (void) t; // ignore unused warning

  switch (sdcard.status) {

    /* Ready with dummy clock, proceed with CMD0 */
    case SdCard_SendingDummyClock:
      sdcard_send_cmd(&sdcard, 0, 0x00000000);
      sdcard.status = SdCard_SendingCMD0;
      break;

    /* Ready sending CMD0, start polling result */
    case SdCard_SendingCMD0:
      sdcard.response_counter = 0;
      sdcard_request_bytes(&sdcard, 1);
      sdcard.status = SdCard_ReadingCMD0Resp;
      break;

    /* Continue polling bytes until there is a response or not */
    case SdCard_ReadingCMD0Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_send_cmd(&sdcard, 8, 0x000001AA);
        sdcard.status = SdCard_SendingCMD8;
      }
      else if (sdcard.response_counter >= 9) {
        sdcard.status = SdCard_Error;
      }
      else{
        sdcard_request_bytes(&sdcard, 1);
      }
      break;

    /* Ready sending CMD8, start polling result */
    case SdCard_SendingCMD8:
      sdcard.response_counter = 0;
      sdcard_request_bytes(&sdcard, 1);
      sdcard.status = SdCard_ReadingCMD8Resp;
      break;

    /* Continue polling bytes until there is a response or not */
    case SdCard_ReadingCMD8Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_request_bytes(&sdcard, 4);
        sdcard.status = SdCard_ReadingCMD8Parameter;
      }
      else if (sdcard.response_counter >= 9) {
        sdcard.status = SdCard_Error;
      }
      else {
        sdcard_request_bytes(&sdcard, 1);
      }
      break;

    /* Process parameter from CMD8 response (after 0x01 was received) */
    case SdCard_ReadingCMD8Parameter:
      if (sdcard.input_buf[0] == 0x00 && sdcard.input_buf[1] == 0x00 && sdcard.input_buf[2] == 0x01 && sdcard.input_buf[3] == 0xAA) {
        sdcard.status = SdCard_SendingACMD41v2;
      }
      else {
        sdcard.status = SdCard_Error;
      }
      break;

    /* Should not reach this */
    default:
      break;
  }

}

void sdcard_send_cmd(struct SdCard *sdcard, uint8_t cmd, uint32_t arg)
{
  (void) cmd; (void) arg;
  sdcard->spi_t.input_length = 6;
  sdcard->spi_t.output_length = 6;
  sdcard->output_buf[0] = 0x40 | cmd;
  sdcard->output_buf[1] = arg >> 24;
  sdcard->output_buf[2] = arg >> 16;
  sdcard->output_buf[3] = arg >> 8;
  sdcard->output_buf[4] = arg;
  switch(cmd) {
    case 0:
      sdcard->output_buf[5] = 0x95;
      break;
    case 8:
      sdcard->output_buf[5] = 0x87;
      break;
    default:
      sdcard->output_buf[5] = 0x01;
  }
  sdcard->spi_t.after_cb = &sdcard_spicallback;

  spi_submit(sdcard->spi_p, &sdcard->spi_t);
}

void sdcard_send_app_cmd(struct SdCard *sdcard, uint8_t cmd, uint32_t arg)
{
  sdcard->spi_t.output_length = 21;
  sdcard->spi_t.input_length = 21;

  sdcard->output_buf[0] = 0x77; // CMD55
  sdcard->output_buf[1] = 0x00;
  sdcard->output_buf[2] = 0x00;
  sdcard->output_buf[3] = 0x00;
  sdcard->output_buf[4] = 0x00;
  sdcard->output_buf[5] = 0x01; // End CMD55
  sdcard->output_buf[6] = 0xFF;
  sdcard->output_buf[7] = 0xFF;
  sdcard->output_buf[8] = 0xFF;
  sdcard->output_buf[9] = 0xFF;
  sdcard->output_buf[10] = 0xFF;
  sdcard->output_buf[11] = 0xFF;
  sdcard->output_buf[12] = 0xFF;
  sdcard->output_buf[13] = 0xFF;
  sdcard->output_buf[14] = 0xFF;

  sdcard->output_buf[15] = 0x40 + cmd;
  sdcard->output_buf[16] = arg >> 24;
  sdcard->output_buf[17] = arg >> 16;
  sdcard->output_buf[18] = arg >> 8;
  sdcard->output_buf[19] = arg;
  sdcard->output_buf[20] = 0x01;

  sdcard->spi_t.after_cb = &sdcard_spicallback;
  spi_submit(sdcard->spi_p, &sdcard->spi_t);
}

void sdcard_request_bytes(struct SdCard *sdcard, uint8_t len)
{
  sdcard->spi_t.input_length = len;
  sdcard->spi_t.output_length = len;
  for (uint8_t i=0; i<len; i++) {
    sdcard->output_buf[i] = 0xFF;
  }
  sdcard->spi_t.after_cb = &sdcard_spicallback;
  spi_submit(sdcard->spi_p, &sdcard->spi_t);
  sdcard->response_counter++;
}
