/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "std.h"
#include "sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

//#include "ops_app.h"   
//#include "ops_app_if.h" 
#include "subsystems/ops/ops_msg_if.h"
#include "subsystems/ops/ops_app_if.h" 

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/logger/000
#endif

/** The file pointer */
static FIL flog;
static FRESULT res;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
	ops_info.ops_debug = TRUE;
	ops_msg_start_spraying();
	#if 0
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  #if 0
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  res = f_open(&flog, filename, "r");
  if(res != FR_OK)
  {
    f_close(&flog);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }
  #endif

  f_open(&flog, "logger.csv", FA_CREATE_NEW);
  f_close(&flog);


  res = f_open(&flog, "logger.csv", FA_WRITE);

  if (res == FR_OK) 
  {
    f_printf(
      &flog,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );
  }
  #endif
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
	ops_info.ops_debug = FALSE;
	ops_msg_stop_spraying();

	#if 0
  if (res == FR_OK) 
  {
    f_close(&flog);
  }
  #endif
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{

	
	#if 0
  if (res != FR_OK) 
  {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  f_printf(&flog, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,
          imu.mag_unscaled.z,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz
         );
  counter++;
  #endif
}
