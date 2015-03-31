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

struct SdCard sdcard_logger;

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
  sdcard->card_type = SdCardType_Unknown;
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
      sdcard->timeout_counter++;
      break;

    /* While busy, keep checking if it is still busy */
    case SdCard_Busy:
      sdcard_request_bytes(sdcard, 1);
      break;

    /* Should not reach this */
    default:
      break;
  }

}


void sdcard_spicallback(struct spi_transaction *t)
{
  (void) t; // ignore unused warning

  switch (sdcard_logger.status) {

    /* Ready with dummy clock, proceed with CMD0 */
    case SdCard_SendingDummyClock:
      sdcard_logger.spi_t.select = SPISelectUnselect;
      sdcard_send_cmd(&sdcard_logger, 0, 0x00000000);
      sdcard_logger.status = SdCard_SendingCMD0;
      break;

    /* Ready sending CMD0, start polling result */
    case SdCard_SendingCMD0:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingCMD0Resp;
      break;

    /* Continue polling bytes until there is a response or not */
    case SdCard_ReadingCMD0Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_send_cmd(&sdcard_logger, 8, 0x000001AA);
        sdcard_logger.status = SdCard_SendingCMD8;
      }
      else if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      }
      else{
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Ready sending CMD8, start polling result */
    case SdCard_SendingCMD8:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingCMD8Resp;
      break;

    /* Continue polling bytes until there is a response or not */
    case SdCard_ReadingCMD8Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_request_bytes(&sdcard_logger, 4);
        sdcard_logger.status = SdCard_ReadingCMD8Parameter;
      }
      else if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      }
      else {
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Process parameter from CMD8 response (after 0x01 was received) */
    case SdCard_ReadingCMD8Parameter:
      if (sdcard_logger.input_buf[0] == 0x00 && sdcard_logger.input_buf[1] == 0x00 && sdcard_logger.input_buf[2] == 0x01 && sdcard_logger.input_buf[3] == 0xAA) {
        sdcard_logger.status = SdCard_SendingACMD41v2;
        sdcard_logger.timeout_counter = 0;
      }
      else {
        sdcard_logger.status = SdCard_Error;
      }
      break;

    /* Ready sending the ACMDv2 command, start polling bytes for response */
    case SdCard_SendingACMD41v2:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingACMD41v2Resp;
      break;

    /* Grabbing bytes in response to the ACMD41v2 command */
    case SdCard_ReadingACMD41v2Resp:
      if (t->input_buf[0] == 0x01) {
        if (sdcard_logger.timeout_counter < 500-1) {
          sdcard_logger.status = SdCard_SendingACMD41v2;
        }
        else {
          sdcard_logger.status = SdCard_Error;
        }
      }
      else if (t->input_buf[0] == 0x00) {
        sdcard_send_cmd(&sdcard_logger, 58, 0x00000000);
        sdcard_logger.status = SdCard_SendingCMD58;
      }
      else if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      }
      else {
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Ready sending CMD58, request the first byte of R3 response */
    case SdCard_SendingCMD58:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingCMD58Resp;
      break;

    /* Continue polling bytes until there is a response to CMD58 */
    case SdCard_ReadingCMD58Resp:
      if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      }
      else if (sdcard_logger.input_buf[0] == 0x00){
        sdcard_request_bytes(&sdcard_logger, 4);
        sdcard_logger.status = SdCard_ReadingCMD58Parameter;
      }
      else{
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Parameter of CMD58 ready, processing it */
    case SdCard_ReadingCMD58Parameter:
      if (sdcard_logger.input_buf[0] & 0x80) { // bit 31 set, CCS bit is valid
        if (sdcard_logger.input_buf[0] & 0x40) { // bit 30 set
          sdcard_logger.card_type = SdCardType_SdV2block;
          sdcard_logger.status = SdCard_Idle;
        }
        else { // bit 30 not set
          sdcard_send_cmd(&sdcard_logger, 16, 0x00000200);
          sdcard_logger.card_type = SdCardType_SdV1;
          sdcard_logger.status = SdCard_SendingCMD16;
        }
      }
      else { // bit 31 not set, CCS bit is unvalid
        sdcard_logger.status = SdCard_Error;
      }
      break;

    /* Ready sending CMD16, request first byte */
    case SdCard_SendingCMD16:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingCMD16Resp;
      break;

    /* Continue polling bytes until there is a response to CMD16 */
    case SdCard_ReadingCMD16Resp:
      if (sdcard_logger.input_buf[0] == 0x00) {
        sdcard_logger.status = SdCard_Idle;
      }
      else if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      } else {
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Ready sending CMD24, request first response byte */
    case SdCard_SendingCMD24:
      sdcard_logger.response_counter = 0;
      sdcard_request_bytes(&sdcard_logger, 1);
      sdcard_logger.status = SdCard_ReadingCMD24Resp;
      break;

    /* Request additional bytes until response or timeout */
    case SdCard_ReadingCMD24Resp:
      if (sdcard_logger.input_buf[0] == 0x00) {
        sdcard_request_bytes(&sdcard_logger, 1);
        sdcard_logger.status = SdCard_BeforeSendingDataBlock;
      }
      else if (sdcard_logger.response_counter >= 9) {
        sdcard_logger.status = SdCard_Error;
      }
      else {
        sdcard_request_bytes(&sdcard_logger, 1);
      }
      break;

    /* Send the data block */
    case SdCard_BeforeSendingDataBlock:
      sdcard_logger.spi_t.input_length = 516;
      sdcard_logger.spi_t.output_length = 516;
      sdcard_logger.output_buf[0] = 0xFE; // data token
      sdcard_logger.output_buf[513] = 0xFF; // CRC byte 1
      sdcard_logger.output_buf[514] = 0xFF; // CRC byte 2
      sdcard_logger.output_buf[515] = 0xFF; // to request data response
      sdcard_logger.spi_t.after_cb = &sdcard_spicallback;
      spi_submit(sdcard_logger.spi_p, &sdcard_logger.spi_t);
      sdcard_logger.status = SdCard_SendingDataBlock;
      break;

    /* Finished sending the data block */
    case SdCard_SendingDataBlock:
      if ((sdcard_logger.input_buf[515] & 0x0F) == 0x05) {
        sdcard_logger.status = SdCard_Busy;
      }
      else {
        sdcard_logger.status = SdCard_Error;
      }
      break;

    case SdCard_Busy:
      if (sdcard_logger.input_buf[0] == 0xFF) {
        sdcard_logger.status = SdCard_Idle;
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

void sdcard_write_block(struct SdCard *sdcard, uint32_t addr)
{
  /* Do not write data if not in idle state */
  if (sdcard->status != SdCard_Idle) {
    return;
  }

  sdcard->spi_t.cdiv = SPIDiv2;

  /* Translate block address to byte address */
  if (sdcard->card_type != SdCardType_SdV2block) {
    addr = addr * 512;
  }

  sdcard_send_cmd(&sdcard_logger, 24, addr);
  sdcard->status = SdCard_SendingCMD24;
}
