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

#ifdef TEST
#include "messages_testable.h"
#endif // TEST

#include "subsystems/imu.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "sd_logger.h"

// defined in sdcard.c
extern struct SdCard sdcard1;

struct SdLogger sdlogger;

void sd_logger_start(void)
{
  sdcard_init(&sdcard1, &spi2, SPI_SLAVE3);
  sdlogger.status = SdLogger_Initializing;
}

void sd_logger_periodic(void)
{
  sdcard_periodic(&sdcard1);

  switch (sdlogger.status) {

    /* SD card is initializing, check to see if it is ready */
    case SdLogger_Initializing:
      if (sdcard1.status == SdCard_Error) {
        sdlogger.status = SdLogger_Error;
      }
      else if (sdcard1.status == SdCard_Idle) {
        sdlogger.status = SdLogger_Idle;
      }
      // else state unchanged
      break;

    /* Logging data, write measurement to buffer */
    case SdLogger_Logging:
      sdlogger.packet_count++;
      sd_logger_uint32_to_buffer(sdlogger.packet_count, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.x, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+4]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.y, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+8]);
      sd_logger_int32_to_buffer(imu.accel_unscaled.z, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+12]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.p, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+16]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.q, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+20]);
      sd_logger_int32_to_buffer(imu.gyro_unscaled.r, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+24]);
      sd_logger_int32_to_buffer(actuators_pwm_values[0], &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+28]);
      sd_logger_int32_to_buffer(actuators_pwm_values[2], &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+32]);
      sd_logger_int32_to_buffer(actuators_pwm_values[3], &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+36]);
      sd_logger_int32_to_buffer(actuators_pwm_values[4], &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+40]);
      sd_logger_int32_to_buffer(actuators_pwm_values[5], &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+44]);
      sd_logger_int32_to_buffer(0, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+sdlogger.buffer_addr+48]); // reserved for something
      sdlogger.buffer_addr += SD_LOGGER_PACKET_SIZE;

      // Check if the buffer is now full. If so, write to SD card
      if ((SD_BLOCK_SIZE - sdlogger.buffer_addr) < SD_LOGGER_PACKET_SIZE ) {
        if (sdcard1.status != SdCard_MultiWriteIdle) {
          sdlogger.error_count++;
        }
        sd_logger_uint32_to_buffer(sdlogger.unique_id, &sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET]);
        sdcard_multiwrite_next(&sdcard1);
        sdlogger.buffer_addr = SD_LOGGER_BLOCK_PREAMBLE_SIZE;
        sdlogger.block_addr++;
      }
      break;

    /* Write summary block if sdcard is ready with writing previous block */
    case SdLogger_WriteStatusPacket:
      if (sdcard1.status != SdCard_Idle) {
        break;
      }
      sd_logger_uint32_to_buffer(sdlogger.packet_count, &sdcard1.output_buf[6]);
      sd_logger_uint32_to_buffer(sdlogger.error_count, &sdcard1.output_buf[6+4]);
      sd_logger_uint32_to_buffer(sdlogger.unique_id, &sdcard1.output_buf[6+8]);
      for (uint16_t i=12; i<SD_BLOCK_SIZE; i++) {
        sdcard1.output_buf[6+i] = 0x00;
      }
      sdcard_write_block(&sdcard1, 0x00000000);
      sdlogger.status = SdLogger_Idle;
      break;

    case SdLogger_BeforeLogging:
      if (sdcard1.status == SdCard_MultiWriteIdle) {
          sdlogger.status = SdLogger_Logging;
      }
      break;

    case SdLogger_StopLogging:
      if (sdcard1.status != SdCard_MultiWriteIdle) {
        break;
      }
      sdcard_multiwrite_stop(&sdcard1);
      sdlogger.status = SdLogger_WriteStatusPacket;
      break;

    case SdLogger_Idle:
      break;

    case SdLogger_ReadingBlock:
      sdlogger.timeout_counter++;
      if (sdlogger.timeout_counter > 199) {
        sdlogger.status = SdLogger_Idle;
        sdlogger.timeout_counter = 0;
      }
      break;

    default:

      break;

  }
}

void sd_logger_command(void)
{

  switch (sdlogger.cmd) {

    /* Start logging */
    case SdLoggerCmd_StartLogging:
      if (sdcard1.status != SdCard_Idle) {
        break;
      }
      sdcard_multiwrite_start(&sdcard1, 0x00000001);
      sdlogger.status = SdLogger_BeforeLogging;
      sdlogger.buffer_addr = SD_LOGGER_BLOCK_PREAMBLE_SIZE;
      sdlogger.packet_count = 0;
      sdlogger.error_count = 0;
      break;


    /* Stop logging, write summary block0 */
    case SdLoggerCmd_StopLogging:
      // Cannot stop if not logging
      if (sdlogger.status != SdLogger_Logging) {
        break;
      }

      // Fill rest of block with trailing zeros
      for (uint16_t i = sdlogger.buffer_addr; i< (SD_LOGGER_BUFFER_OFFSET + SD_BLOCK_SIZE); i++) {
        sdcard1.output_buf[SD_LOGGER_BUFFER_OFFSET+i] = 0x00;
      }
      sdcard_multiwrite_next(&sdcard1);
      sdlogger.status = SdLogger_StopLogging;
      break;

    /* Request status byte */
    case SdLoggerCmd_RequestStatusPacket:
      if (sdlogger.status != SdLogger_Idle && sdlogger.status != SdLogger_DataAvailable) {
        break;
      }

      if (sdlogger.block_addr == 0x00000000 && sdlogger.status == SdLogger_DataAvailable) {
        sd_logger_send_packet_from_buffer(0);
        break;
      }
      else {
        sdcard_read_block(&sdcard1, 0x00000000, &sd_logger_statusblock_ready);
      }
      break;

    /* No command, but maybe the request_id was set to arrive here */
    case SdLoggerCmd_Nothing:
      if (sdlogger.request_id == 0) {
        break;
      }
      if (sdlogger.status != SdLogger_Idle && sdlogger.status != SdLogger_DataAvailable) {
        break;
      }
      // check if block address is correct
      if (sdlogger.status == SdLogger_DataAvailable && sdlogger.block_addr == ( ( sdlogger.request_id - 1) / SD_LOGGER_PACKETS_PER_BLOCK) + 1 ) {
        sd_logger_packetblock_ready();
      }
      else {
        sdlogger.block_addr = ((( sdlogger.request_id - 1) / SD_LOGGER_PACKETS_PER_BLOCK) + 1);
        sdcard_read_block(&sdcard1, sdlogger.block_addr, &sd_logger_packetblock_ready);
        sdlogger.status = SdLogger_ReadingBlock;
        sdlogger.timeout_counter = 0;
      }
      break;

    default:
      break;

  }

  sdlogger.cmd = SdLoggerCmd_Nothing;
}

void sd_logger_stop(void)
{

}

void sd_logger_statusblock_ready(void)
{
  sdlogger.block_addr = 0x00000000;
  sdlogger.status = SdLogger_DataAvailable;
  sd_logger_send_packet_from_buffer(0);
}

void sd_logger_packetblock_ready(void)
{
  // If unique id is not matchin with the one in the data block
  if (sdlogger.unique_id != sd_logger_get_uint32(&sdcard1.input_buf[0])) {
    sdlogger.packet.time = 0;
    sdlogger.packet.data_1 = 0;
    sdlogger.packet.data_2 = 0;
    sdlogger.packet.data_3 = 0;
    sdlogger.packet.data_4 = 0;
    sdlogger.packet.data_5 = 0;
    sdlogger.packet.data_6 = 0;
    sdlogger.packet.data_7 = 0;
    sdlogger.packet.data_8 = 0;
    sdlogger.packet.data_9 = 0;
    sdlogger.packet.data_10 = 0;
    sdlogger.packet.data_11 = 0;
    sdlogger.packet.data_12 = 0;
    DOWNLINK_SEND_LOG_DATAPACKET(DefaultChannel, DefaultDevice, &sdlogger.packet.time, &sdlogger.packet.data_1, &sdlogger.packet.data_2, &sdlogger.packet.data_3, &sdlogger.packet.data_4, &sdlogger.packet.data_5, &sdlogger.packet.data_6, &sdlogger.packet.data_7, &sdlogger.packet.data_8, &sdlogger.packet.data_9, &sdlogger.packet.data_10, &sdlogger.packet.data_11, &sdlogger.packet.data_12);
  }
  else {
    sd_logger_send_packet_from_buffer( ((sdlogger.request_id - 1) % SD_LOGGER_PACKETS_PER_BLOCK) * SD_LOGGER_PACKET_SIZE + SD_LOGGER_BLOCK_PREAMBLE_SIZE );
  }
  sdlogger.status = SdLogger_DataAvailable;
}

void sd_logger_send_packet_from_buffer(uint16_t buffer_idx)
{
  sdlogger.packet.time = sd_logger_get_uint32(&sdcard1.input_buf[buffer_idx+0]);
  sdlogger.packet.data_1 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+4]);
  sdlogger.packet.data_2 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+8]);
  sdlogger.packet.data_3 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+12]);
  sdlogger.packet.data_4 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+16]);
  sdlogger.packet.data_5 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+20]);
  sdlogger.packet.data_6 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+24]);
  sdlogger.packet.data_7 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+28]);
  sdlogger.packet.data_8 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+32]);
  sdlogger.packet.data_9 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+36]);
  sdlogger.packet.data_10 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+40]);
  sdlogger.packet.data_11 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+44]);
  sdlogger.packet.data_12 = sd_logger_get_int32(&sdcard1.input_buf[buffer_idx+48]);
  DOWNLINK_SEND_LOG_DATAPACKET(DefaultChannel, DefaultDevice, &sdlogger.packet.time, &sdlogger.packet.data_1, &sdlogger.packet.data_2, &sdlogger.packet.data_3, &sdlogger.packet.data_4, &sdlogger.packet.data_5, &sdlogger.packet.data_6, &sdlogger.packet.data_7, &sdlogger.packet.data_8, &sdlogger.packet.data_9, &sdlogger.packet.data_10, &sdlogger.packet.data_11, &sdlogger.packet.data_12);
}

void sd_logger_int32_to_buffer(const int32_t value, uint8_t *target)
{
  target[0] = value >> 24;
  target[1] = value >> 16;
  target[2] = value >> 8;
  target[3] = value;
}

void sd_logger_uint32_to_buffer(const uint32_t value, uint8_t *target)
{
  target[0] = value >> 24;
  target[1] = value >> 16;
  target[2] = value >> 8;
  target[3] = value;
}

int32_t sd_logger_get_int32(uint8_t *target)
{
  return target[0] << 24 | target[1] << 16 | target[2] << 8 | target[3];
}

uint32_t sd_logger_get_uint32(uint8_t *target)
{
  return target[0] << 24 | target[1] << 16 | target[2] << 8 | target[3];
}
