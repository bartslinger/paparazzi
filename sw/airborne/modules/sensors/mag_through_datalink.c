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
 * @file "modules/sensors/mag_through_datalink.c"
 * @author Bart Slinger
 * gets magneto data through datalink
 */

#include "modules/sensors/mag_through_datalink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/imu.h"
#include "state.h"
#include "abi_messages.h"

struct OrientationReps imu_to_mag;
struct Int32Vect3 raw_mag;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void mag_through_datalink_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID, &imu.mag_unscaled.x, &imu.mag_unscaled.y,
                             &imu.mag_unscaled.z);
}
#endif

void mag_through_datalink_init() {
  /* Set imu to magneto rotation */
  struct FloatEulers imu_to_mag_eulers =
    {IMU_TO_MAG_PHI, IMU_TO_MAG_THETA, IMU_TO_MAG_PSI};
  orientationSetEulers_f(&imu_to_mag, &imu_to_mag_eulers);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, mag_through_datalink_raw_downlink);
#endif
}

void mag_through_datalink_parse_msg() {
  uint32_t now_ts = get_sys_time_usec();
  /* (Ab)used HITL_INFRARED message for this */
  raw_mag.x = (int16_t) (dl_buffer[2] << 8) | (int16_t) (dl_buffer[3]);
  raw_mag.y = (int16_t) (dl_buffer[4] << 8) | (int16_t) (dl_buffer[5]);
  raw_mag.z = (int16_t) (dl_buffer[6] << 8) | (int16_t) (dl_buffer[7]);

  /* Rotate the magneto */
  struct Int32RMat *imu_to_mag_rmat = orientationGetRMat_i(&imu_to_mag);
  int32_rmat_vmult(&imu.mag_unscaled, imu_to_mag_rmat, &raw_mag);

  // Send and set the magneto IMU data
  imu_scale_mag(&imu);
  AbiSendMsgIMU_MAG_INT32(MAG_HMC58XX_SENDER_ID, now_ts, &imu.mag);
}
