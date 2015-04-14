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

/** @file peripherals/sdcard.h
 *  @brief TODO.
 */

#ifndef SDCARD_H_
#define SDCARD_H_

#include "std.h"
#include "mcu_periph/spi.h"

/** SDCard Callback function.
 * If not NULL (or 0), call function
 */
typedef void (*SdCardCallback)(void);

#define SD_BLOCK_SIZE 512

enum SdResponseType {
  SdResponseNone,
  SdResponseR1,
  SdResponseR3,
  SdResponseR7
};

enum SdCardType {
  SdCardType_Unknown,
  SdCardType_MmcV3,
  SdCardType_SdV1,
  SdCardType_SdV2byte,
  SdCardType_SdV2block
};

enum SdTryCardInitialize {
  SDTryV2,
  SDTryV1,
  SDTryMMC
};

enum SdCardStatus {
  SdCard_UnInit,                            /**< SD card is not initialized */
  SdCard_Error,                             /**< An error has occured, sending debug message */
  SdCard_Idle,                              /**< Initialization sequence succesful */
  SdCard_Busy,                              /**< SD card is busy with internal process */
  SdCard_BeforeDummyClock,                  /**< About to send dummy clock cycles to initialize spi mode */
  SdCard_SendingDummyClock,                 /**< Busy sending dummy clock cycles */
  SdCard_SendingCMD0,                       /**< Busy sending CMD0 */
  SdCard_ReadingCMD0Resp,                   /**< Reading R1 response to CMD0 byte by byte */
  SdCard_SendingCMD8,                       /**< Busy sending CMD8 */
  SdCard_ReadingCMD8Resp,                   /**< Reading R7 response to CMD8 byte by byte */
  SdCard_ReadingCMD8Parameter,              /**< Reading the 32-bit parameter after receiving 0x01 */
  SdCard_SendingACMD41v2,                   /**< Busy sending ACMD41 */
  SdCard_ReadingACMD41v2Resp,               /**< Reading R1 response to ACMD41 byte by byte */
  SdCard_SendingCMD58,                      /**< Busy sending CMD58 */
  SdCard_ReadingCMD58Resp,                  /**< Reading R3 response to CMD58 byte by byte */
  SdCard_ReadingCMD58Parameter,             /**< Reading the 32-bit parameter after receiving 0x00 from CMD58 */
  SdCard_SendingCMD16,                      /**< Busy sending CMD16 */
  SdCard_ReadingCMD16Resp,                  /**< Reading R1 response to CMD16 byte by byte */
  SdCard_SendingCMD24,                      /**< Busy sending CMD24 */
  SdCard_ReadingCMD24Resp,                  /**< Reading R1 response to CMD24 byte by byte */
  SdCard_BeforeSendingDataBlock,            /**< Start data block transfer */
  SdCard_SendingDataBlock,                  /**< Busy sending data block */
  SdCard_SendingCMD17,                      /**< Busy sending CMD17 (block read request) */
  SdCard_ReadingCMD17Resp,                  /**< Reading R1 response to CMD17 byte by byte */
  SdCard_WaitingForDataToken,               /**< Reading a byte each period until there is a data token or error */
  SdCard_ReadingDataBlock,                  /**< Busy reading data block */
  SdCard_SendingCMD25,                      /**< Busy sending CMD25 (multiwrite start command) */
  SdCard_ReadingCMD25Resp,                  /**< Reading R1 response to CMD25 byte by byte */
  SdCard_MultiWriteIdle,                    /**< CMD25 complete, ready to sent blocks */
  SdCard_MultiWriteWriting,                 /**< Busy with the SPI transfer in multiwrite */
  SdCard_MultiWriteBusy,                    /**< Busy flag after sending data block in multiwrite */
  SdCard_MultiWriteStopping,                /**< Busy sending the stop token */
  bladiebla
};

struct SdCard{
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  volatile enum SdCardStatus status;        /**< The status of the SD card */
  uint8_t input_buf[SD_BLOCK_SIZE+10];      /**< The input buffer for the SPI transaction */
  uint8_t output_buf[SD_BLOCK_SIZE+10];     /**< The output buffer for the SPI transaction */
  uint8_t response_counter;                 /**< Response counter used at various locations */
  uint32_t timeout_counter;                 /**< Timeout counter used for initializatino checks with ACMD41 */
  enum SdCardType card_type;                /**< Type of SD card */
  SdCardCallback read_callback;             /**< Callback to call when read operation finishes */
};

extern struct SdCard sdcard1;

//! Public functions
extern void sdcard_init(struct SdCard *sdcard, struct spi_periph *spi_p, const uint8_t slave_idx);
extern void sdcard_periodic(struct SdCard *sdcard);
extern void sdcard_write_block(struct SdCard *sdcard, uint32_t addr);
extern void sdcard_read_block(struct SdCard *sdcard, uint32_t addr, SdCardCallback callback);
extern void sdcard_multiwrite_start(struct SdCard *sdcard, uint32_t addr);
extern void sdcard_multiwrite_next(struct SdCard *sdcard);
extern void sdcard_multiwrite_stop(struct SdCard *sdcard);

//! Private functions
extern void sdcard_spicallback(struct spi_transaction *t);
extern void sdcard_process_callback(struct SdCard *sdcard, struct spi_transaction *t);
extern void sdcard_send_cmd(struct SdCard *sdcard, uint8_t cmd, uint32_t arg);
extern void sdcard_send_app_cmd(struct SdCard *sdcard, uint8_t cmd, uint32_t arg);
extern void sdcard_request_bytes(struct SdCard *sdcard, uint8_t len);


#endif // SDCARD_H_
