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

#ifdef LOGGER_LED
#define LOGGER_LED_ON LED_ON(LOGGER_LED);
#else
#define LOGGER_LED_ON {}
#endif

struct sdlogger_spi_periph sdlogger_spi;

/**
 * @brief sdlogger_spi_direct_init
 * Initialize the logger and SD Card.
 */
void sdlogger_spi_direct_init(void)
{
  /* Initialize the SD Card */
  sdcard_spi_init(&sdcard1, &(SDLOGGER_SPI_LINK_DEVICE),
                  SDLOGGER_SPI_LINK_SLAVE_NUMBER);

  /* Set values in the struct to their defaults */
  sdlogger_spi.status = SDLogger_Initializing;
  sdlogger_spi.next_available_address = 0;
  sdlogger_spi.last_completed = 0;
  sdlogger_spi.sdcard_buf_idx = 1;

  /* Fill internal buffer with zeros */
  for (uint8_t i = 0; i < sizeof(sdlogger_spi.buffer); i++) {
    sdlogger_spi.buffer[i] = 0;
  }
  sdlogger_spi.idx = 0;

  /* Set function pointers in link_device to the logger functions */
  sdlogger_spi.device.check_free_space = (check_free_space_t)sdlogger_spi_direct_check_free_space;
  sdlogger_spi.device.put_byte = (put_byte_t)sdlogger_spi_direct_put_byte;
  sdlogger_spi.device.send_message = (send_message_t)sdlogger_spi_direct_send_message;
  sdlogger_spi.device.char_available = (char_available_t)sdlogger_spi_direct_char_available;
  sdlogger_spi.device.get_byte = (get_byte_t)sdlogger_spi_direct_get_byte;
  sdlogger_spi.device.periph = &sdlogger_spi;
}

/**
 * @brief sdlogger_spi_direct_periodic
 * Periodic function called at module frequency
 */
void sdlogger_spi_direct_periodic(void)
{
  sdcard_spi_periodic(&sdcard1);

  switch (sdlogger_spi.status) {
    case SDLogger_Initializing:
      if (sdcard1.status == SDCard_Idle) {
        sdcard_spi_read_block(&sdcard1, 0x00002000, &sdlogger_spi_direct_index_received);
        sdlogger_spi.status = SDLogger_RetreivingIndex;
      }
      break;

    case SDLogger_Ready:
      if (radio_control.values[SDLOGGER_CONTROL_SWITCH] > 0 &&
          sdcard1.status == SDCard_Idle) {
        LOGGER_LED_ON;
        sdcard_spi_multiwrite_start(&sdcard1, sdlogger_spi.next_available_address);
        sdlogger_spi.status = SDLogger_Logging;
      }
      break;

    case SDLogger_Logging:
      /* Check if SD Card buffer is full and SD Card is ready for new data */
      if (sdlogger_spi.sdcard_buf_idx > 512 &&
          sdcard1.status == SDCard_MultiWriteIdle) {
        sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
      }
      break;

    default:
      break;
  }
}

void sdlogger_spi_direct_start(void) {}
void sdlogger_spi_direct_stop(void) {}

/**
 * @brief sdlogger_spi_direct_index_received
 * Callback from SD Card when block at index location is received.
 */
void sdlogger_spi_direct_index_received(void)
{
  /* Save address and last completed log for later use */
  sdlogger_spi.next_available_address = (sdcard1.input_buf[0] << 24) |
                                        (sdcard1.input_buf[1] << 16) |
                                        (sdcard1.input_buf[2] << 8) |
                                        (sdcard1.input_buf[3]);
  sdlogger_spi.last_completed = sdcard1.input_buf[4];

  /* Ready to start logging */
  sdlogger_spi.status = SDLogger_Ready;
}

/**
 * @brief sdlogger_spi_direct_multiwrite_written
 * Called when a multiwrite is complete. Data stored in the logger buffer is
 * then moved to the SD Card buffer, which is now available again.
 */
void sdlogger_spi_direct_multiwrite_written(void)
{
  /* Copy data from logger buffer to SD Card buffer */
  for (uint8_t i = 0; i < sdlogger_spi.idx; i++) {
    sdcard1.output_buf[i+1] = sdlogger_spi.buffer[i];
  }
  /* Set sdcard buffer index to new value */
  sdlogger_spi.sdcard_buf_idx = sdlogger_spi.idx + 1;
  /* And reset the logger buffer index */
  sdlogger_spi.idx = 0;
}

bool_t sdlogger_spi_direct_check_free_space(struct sdlogger_spi_periph *p, uint8_t len)
{
  (void) len;
  if (p->status == SDLogger_Logging) {
    return TRUE;
  }
  return FALSE;
}

void sdlogger_spi_direct_put_byte(struct sdlogger_spi_periph *p, uint8_t data)
{
  /* SD Buffer full, write in logger buffer */
  if (p->sdcard_buf_idx > 512) {
    if (p->idx < SDLOGGER_BUFFER_SIZE) {
      p->buffer[p->idx++] = data;
    }
  }
  /* Writing directly to SD Card buffer */
  else {
    sdcard1.output_buf[p->sdcard_buf_idx++] = data;

    /* Flush buffer */
    if (p->sdcard_buf_idx > 512 && sdcard1.status == SDCard_MultiWriteIdle) {
      sdcard_spi_multiwrite_next(&sdcard1, &sdlogger_spi_direct_multiwrite_written);
    }
  }
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

