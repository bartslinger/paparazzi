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

/** @file modules/loggers/sd_logger.h
 *  @brief Basic sd logger.
 */

#ifndef SD_LOGGER_H_
#define SD_LOGGER_H_


#include "std.h"
#include "subsystems/imu.h"

enum SdCardType{
  CardUnknown,
  CardMmcV3,
  CardSdV1,
  CardSdV2byte,
  CardSdV2block
};

enum SdResponseType{
  SdResponseR1,
  SdResponseR3,
  SdResponseR7
};

enum SdTryCardInitialize{
  SDTryV2,
  SDTryV1,
  SDTryMMC
};

struct SdLogger{
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  uint8_t input_buf[512];                   /**< The input buffer for the SPI transaction */
  uint8_t output_buf[512];                  /**< The output buffer for the SPI transaction */
  enum SdTryCardInitialize try_card_type;
  bool failed;
  uint8_t sd_response;
  enum SdCardType card_type;
  uint8_t initialization_counter;
  bool initialized;
  bool ready;
};

extern struct SdLogger sd_logger;

extern void sd_logger_start(void);
extern void sd_logger_periodic(void);
extern void sd_logger_stop(void);

/* Helper functions */
extern void sd_logger_setup_spi(void);
extern void sd_logger_send_cmd(uint8_t cmd, uint32_t arg, enum SdResponseType response_type, SPICallback after_cb);
extern void sd_logger_send_app_cmd(uint8_t cmd, uint32_t arg, enum SdResponseType response_type, SPICallback after_cb);
extern uint8_t sd_logger_get_response_idx(void);
extern uint8_t sd_logger_get_R1(void);
extern uint32_t sd_logger_get_R7(void);
extern uint32_t sd_logger_get_R3(void);
extern uint8_t sd_logger_get_ACMD_R1(void);

/* SPI callback functions */
extern void sd_logger_send_CMD0(struct spi_transaction *t);
extern void sd_logger_get_CMD0_response(struct spi_transaction *t);
extern void sd_logger_process_CMD8(struct spi_transaction *t);
extern void sd_logger_process_ACMD41_SDv2(struct spi_transaction *t);
extern void sd_logger_process_ACMD41_SDv1(struct spi_transaction *t);
extern void sd_logger_process_CMD58(struct spi_transaction *t);
extern void sd_logger_process_CMD16(struct spi_transaction *t);

extern void sd_logger_serial_println(const char text[]);
extern void sd_logger_spi_init(struct SdLogger *sdlog, struct spi_periph *spi_p, uint8_t slave_idx);

#endif /* SD_LOGGER_H_ */
