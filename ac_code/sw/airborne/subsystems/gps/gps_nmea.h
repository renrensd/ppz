/*
 * Copyright (C) 2004-2011 The Paparazzi Team
 *               2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file gps_nmea.h
 * NMEA protocol specific code.
 *
*/


#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "mcu_periph/uart.h"
#include "filters/median_filter.h"

#define USE_XYZA 1

#define GPS_NB_CHANNELS 12

#define NMEA_MAXLEN 510

/*information form RTK datasheet, page 84*/
enum Xyzmsg_Pos_Type
{
	NONE = 0,
	FIXEDPOS = 1,
	DOPPLER_VELOCITY = 8,
	SINGLE = 16,
	PSRDIFF = 17,
	SBAS = 18,
	NARROW_FLOAT = 34,
	WIDE_INT = 49,
	NARROW_INT = 50,
	SUPER_WIDE_LANE = 51,
	PPP = 69,
	TIME_OUT = 2  //whp add, once update need check the value
};

struct GpsNmea {
  bool_t msg_available;
  bool_t pos_available;
 #if USE_XYZA
  bool_t pos_xyz_available;
 #endif
 #ifdef USE_GPS_HEADING
  bool_t heading_available;
 #endif
  bool_t is_configured;       ///< flag set to TRUE if configuration is finished
  bool_t have_gsv;            ///< flag set to TRUE if GPGSV message received
  uint8_t gps_nb_ovrn;        ///< number if incomplete nmea-messages
  char msg_buf[NMEA_MAXLEN];  ///< buffer for storing one nmea-line
  
  uint8_t status;             ///< line parser status
  uint8_t gps_qual;           ///< RTK FIX(0x04) OR RTK FLOAT(0x05) OR SINGLE(0x01)
  enum Xyzmsg_Pos_Type pos_type;
  
  uint8_t sol_tatus;          ///< gps heading status
  uint8_t num_sta_use;        ///< number of sta in used
  int32_t msg_len;
  float heading;
  float pitch;

  uint32_t last_xyzmsg_time; ///< last available msg cpu time(ms)
  uint32_t last_tramsg_time; ///< last available msg cpu time(ms)

  //median filter
  struct MedianFilter3Int ecef_pos_filter;
  struct MedianFilter3Int ecef_vel_filter;
};

extern struct GpsNmea gps_nmea;


/*
 * This part is used by the autopilot to read data from a uart
 */

/** The function to be called when a characted from the device is available */
#include "mcu_periph/link_device.h"

extern void nmea_configure(void);
extern void nmea_parse_char(uint8_t c);
extern void nmea_parse_msg(void);
extern uint8_t nmea_calc_crc(const char *buff, int buff_sz);
extern void nmea_parse_prop_init(void);
extern void nmea_parse_prop_msg(void);
extern void gps_nmea_msg(void);
extern void get_gps_pos_stable(void);
extern void get_gps_heading_stable(void);
extern void gps_nmea_msg_outtime_check(void);

static inline void GpsEvent(void)
{
  struct link_device *dev = &((GPS_LINK).device);

  if (!gps_nmea.is_configured) {
    nmea_configure();
    return;
  }
  while (dev->char_available(dev->periph)) {
    nmea_parse_char(dev->get_byte(dev->periph));
    if (gps_nmea.msg_available) {
      gps_nmea_msg();
    }
  }
}

/** Read until a certain character, placed here for proprietary includes */
static inline void nmea_read_until(int *i)
{
  while (gps_nmea.msg_buf[(*i)++] != ',') {
    if (*i >= gps_nmea.msg_len) {
      return;
    }
  }
}

#endif /* GPS_NMEA_H */
