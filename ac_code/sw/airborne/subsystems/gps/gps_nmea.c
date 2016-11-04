/*
 *
 * Copyright (C) 2008-2011 The Paparazzi Team
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

/**
 * @file gps_nmea.c
 * Basic parser for the NMEA protocol.
 *
 * Status:
 *  Parsing GGA, RMC, GSA and GSV.
 */

#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "led.h"
#include "filters/median_filter.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#endif
#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef NMEA_PRINT
#define NMEA_PRINT(...) {};
#endif

#if NMEA_PRINT == printf
#include <stdio.h>
#endif

/* line parser status */
#define WAIT          0
#define GOT_START     1
#define GOT_CHECKSUM  2
#define GOT_END       3

#if USE_XYZA
 #define CRC32_POLYNOMIAL 0xEDB88320
#endif

struct GpsNmea gps_nmea;

static void nmea_parse_GSA(void);
static void nmea_parse_RMC(void);
static void nmea_parse_GGA(void);
static void nmea_parse_GSV(void);
#ifdef USE_GPS_HEADING
static void nmea_parse_TRA(void);
#endif

#if USE_XYZA
uint32_t CRC32Value(int32_t i);
uint32_t CalculateXYZACRC32(uint16_t ulCount,unsigned char *ucBuffer);
static void nmea_parse_XYZ(void);
#endif

/*double my_strtod(const char* s, char** endptr);*/


#if USE_XYZA
uint32_t CRC32Value(int32_t i)
{
	uint8_t j;
	uint32_t ulCRC;
	ulCRC = (uint32_t)i;
	for (j=8; j>0; j--)
	{
		if (ulCRC & 1)
		ulCRC = (ulCRC >> 1)^CRC32_POLYNOMIAL;
		else ulCRC >>= 1;
	}
	return ulCRC;
}

uint32_t CalculateXYZACRC32(uint16_t ulCount,unsigned char *ucBuffer)
{
	uint32_t ulTemp1;
	uint32_t ulTemp2;
	uint32_t ulCRC = 0;
	
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		//ulTemp2 = CRC32Value( ( (int32_t)ulCRC^*ucBuffer++ )&0xff );
		ulTemp2 = CRC32Value( ( ulCRC^(uint32_t)(*ucBuffer) )&0x000000FFL );
		ucBuffer++;
		ulCRC = ulTemp1^ulTemp2;
	}
	return ulCRC;
}
#endif

static void gps_nmea_data_filter_ini(void)
{
	InitMedianFilterVect3Int(gps_nmea.ecef_pos_filter);
	InitMedianFilterVect3Int(gps_nmea.ecef_vel_filter);
}
static void gps_nmea_data_filter_update(struct GpsState *gps)
{
	UpdateMedianFilterVect3Int(gps_nmea.ecef_pos_filter, gps->ecef_pos);
	UpdateMedianFilterVect3Int(gps_nmea.ecef_vel_filter, gps->ecef_vel);
}

void gps_impl_init(void)
{
  gps.nb_channels = GPS_NB_CHANNELS;
  gps_nmea.is_configured = FALSE;
  gps_nmea.msg_available = FALSE;
  gps_nmea.pos_available = FALSE;
 #if USE_XYZA
  gps_nmea.pos_xyz_available = FALSE;
 #endif
  gps_nmea.have_gsv = FALSE;
  gps_nmea.gps_nb_ovrn = 0;
  gps_nmea.msg_len = 0;
  nmea_parse_prop_init();
  nmea_configure();

  gps_nmea_data_filter_ini();
}

void gps_nmea_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  nmea_parse_msg();
  if( gps_nmea.pos_available
     #if USE_XYZA
	  && gps_nmea.pos_xyz_available
	 #endif
                                    ) 
  {
	 	if (gps.fix == GPS_FIX_3D) 
		{
			gps.last_3dfix_ticks = sys_time.nb_sec_rem;
			gps.last_3dfix_time = sys_time.nb_sec;
		}
	 	//median filter
	 	#if USE_XYZA
	 	//gps_nmea_data_filter_update(&gps);
		#endif
		AbiSendMsgGPS(GPS_NMEA_ID, now_ts, &gps);
  }
  gps_nmea.msg_available = FALSE;
 #if USE_XYZA
  gps_nmea.pos_xyz_available = FALSE;  /*xyza msg finished,reset to false*/
 #endif
}

void WEAK nmea_configure(void)
{
  gps_nmea.is_configured = TRUE;
}

void WEAK nmea_parse_prop_init(void)
{
}

void WEAK nmea_parse_prop_msg(void)
{
}

/**
 * nmea_parse_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
void nmea_parse_msg(void)
{

  if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "RMC", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("RMC: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_RMC();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GGA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GGA: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GGA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GSA: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GSA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSV", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    gps_nmea.have_gsv = TRUE;
    NMEA_PRINT("GSV: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GSV();
  } 
  #ifdef USE_GPS_HEADING
    else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "TRA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("TRA: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_TRA();
  } 
  #endif
  #if USE_XYZA  //add BESTXYZA data to get ecef information
    else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[4] , "XYZA", 4)) {   
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("XYZ: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_XYZ();
  } 
  #endif
    else {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("Other/propriarty message: len=%i \n\r \"%s\" \n\r", gps_nmea.msg_len, gps_nmea.msg_buf);
    nmea_parse_prop_msg();
  }

  // reset line parser
  gps_nmea.status = WAIT;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_nmea.msg_available to TRUE
 * after a full line.
 */
void nmea_parse_char(uint8_t c)
{
  static uint8_t header_type = 0; /*0:none, 1:nmea, 2:bestxyza*/
  uint32_t crc_sum,cal;
  switch (gps_nmea.status) {
    case WAIT:
      gps_nmea.msg_len = 0;    //reset buf length
      
      if (c == '$') /* nmea valid message needs to start with $ sign */
	  { 
        header_type = 1;
        gps_nmea.status = GOT_START;
      }
	  else if(c == '#') /* xyza valid message needs to start with # sign */
	  {
	  	header_type = 2;
        gps_nmea.status = GOT_START;
	  }
      break;

    case GOT_START:
      switch (c) {
        case '\r':
        case '\n':
          if (gps_nmea.msg_len != 0)  /*do crc verify*/
		  {
		  	if(1==header_type)
		  	{
				char crc_char[2] = { gps_nmea.msg_buf[gps_nmea.msg_len-2], gps_nmea.msg_buf[gps_nmea.msg_len-1] };
				crc_sum = (uint32_t)(strtoul(crc_char, NULL,16));
				cal = (uint32_t)nmea_calc_crc(gps_nmea.msg_buf, gps_nmea.msg_len-3);				
		  	}
		   #if USE_XYZA
			else if(2==header_type)
			{
				char crc_char[9];
				for(uint8_t i=0; i<8; i++)
				{
					crc_char[i] = gps_nmea.msg_buf[gps_nmea.msg_len-8+i];
				}
				crc_char[8] = 0x2C;  /*set last char "," for get exact data*/

				crc_sum = (uint32_t)(strtoul(crc_char, NULL,16));
				cal = CalculateXYZACRC32((unsigned long)(gps_nmea.msg_len-9), (unsigned char*)gps_nmea.msg_buf);
			}
		   #endif
			
			if(cal==crc_sum)
			{
				gps_nmea.status = GOT_END;
		                gps_nmea.msg_available = TRUE;
				break;
			}			

          }

		  //reject empty lines
          gps_nmea.status = WAIT;
          break;

		case '$':
		case '#':
          // got another dollar sign, msg incomplete: reset
          gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
          NMEA_PRINT("nmea_parse_char: skipping incomplete msg: len=%i, \"%s\"\n\r",
                     gps_nmea.msg_len, gps_nmea.msg_buf);
          gps_nmea.status = WAIT;
          break;

        default:
           // fill the buffer, unless it's full
          if (gps_nmea.msg_len < NMEA_MAXLEN - 1) {
            gps_nmea.msg_buf[gps_nmea.msg_len] = c;
            gps_nmea.msg_len++;
          }
          else {
            gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
            NMEA_PRINT("nmea_parse_char: msg too long, len=%i, \"%s\"\n\r",
                       gps_nmea.msg_len, gps_nmea.msg_buf);
            gps_nmea.status = WAIT;
          }
          break;
      }
      break;

    case GOT_CHECKSUM:
      // TODO
      break;

    case GOT_END:
      // shouldn't really happen, msg should be parsed and state reset before the next char
      NMEA_PRINT("nmea_parse_char: this should not happen!");
      break;

    default:
      break;
  }
}

/**
 * Calculate control sum of binary buffer
 */
uint8_t nmea_calc_crc(const char *buff, int buff_sz)
{
  uint8_t chsum = 0, it;

  for (it = 0; it < buff_sz; ++it) {
    chsum ^= buff[it];
  }
  return chsum;
}

/**
 * Parse GSA NMEA messages.
 * GPS DOP and active satellites.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GSA(void)
{
  int i = 6;     // current position in the message, start after: GPGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GSA() - skipping empty message\n\r");
    return;
  }

  // get auto2D/3D
  // ignored
  nmea_read_until(&i);

  // get 2D/3D-fix
  // set gps_mode=3=3d, 2=2d, 1=no fix or 0
  gps.fix = atoi(&gps_nmea.msg_buf[i]);
  if (gps.fix == 1) {
    gps.fix = 0;
  }
  #if 0
  gps.fix=GPS_FIX_3D;       
  gps.pacc=300;
  #endif
  NMEA_PRINT("p_GSA() - gps.fix=%i (3=3D)\n\r", gps.fix);
  nmea_read_until(&i);

  // up to 12 PRNs of satellites used for fix
  int satcount = 0;
  int prn_cnt;
  for (prn_cnt = 0; prn_cnt < 12; prn_cnt++) {
    if (gps_nmea.msg_buf[i] != ',') {
      int prn = atoi(&gps_nmea.msg_buf[i]);
      NMEA_PRINT("p_GSA() - PRN %i=%i\n\r", satcount, prn);
      if (!gps_nmea.have_gsv) {
        gps.svinfos[prn_cnt].svid = prn;
      }
      satcount++;
    }
    else {
      if (!gps_nmea.have_gsv) {
        gps.svinfos[prn_cnt].svid = 0;
      }
    }
    nmea_read_until(&i);
  }

  // PDOP
  float pdop = (float)strtod(&gps_nmea.msg_buf[i], NULL);
  gps.pdop = pdop * 100;
  NMEA_PRINT("p_GSA() - pdop=%f\n\r", pdop);
  nmea_read_until(&i);

  // HDOP
  float hdop __attribute__((unused)) = (float)strtod(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GSA() - hdop=%f\n\r", hdop);
  nmea_read_until(&i);

  // VDOP
  float vdop __attribute__((unused)) = (float)strtod(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GSA() - vdop=%f\n\r", vdop);
  nmea_read_until(&i);

}

/**
 * Parse TRA NMEA messages.
 * GPS HEADING and PITCH.
 * Msg stored in gps_nmea.msg_buf.
 */
#ifdef USE_GPS_HEADING

#define GPS_HEADING_OFFSET  90.0
#define Course360(x) { \
    if(x < 0.0) x += 360.0; \
    else if(x >= 360.0) x -= 360.0; \
  }
static void nmea_parse_TRA(void)
{
  int i = 6;     // current position in the message, start after: GPTRA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    //NMEA_PRINT("p_TRA() - skipping empty message\n\r");
    return;
  }

  // ignored
  //nmea_read_until(&i);
	//i++;
  // get heading
  nmea_read_until(&i);
  gps_nmea.heading = (float)strtod(&gps_nmea.msg_buf[i],NULL);
  if(gps_nmea.heading < 0.0 || gps_nmea.heading > 360.0)
  {
  	   gps_nmea.sol_tatus = 0;
  	   return;
  }
  gps.heading = gps_nmea.heading + GPS_HEADING_OFFSET;
  Course360(gps.heading);  
  
  NMEA_PRINT("p_TRA() - gps_nmea.heading=%f\n\r", gps_nmea.heading);
  nmea_read_until(&i);
  // get pitch
  gps_nmea.pitch= (float)strtod(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_TRA() - gps_nmea.pitch=%f\n\r", gps_nmea.pitch);
  nmea_read_until(&i);
  nmea_read_until(&i);
  // get status
  gps_nmea.sol_tatus= (float)strtod(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_TRA() - gps_nmea.pitch=%f\n\r", gps_nmea.sol_tatus);
  nmea_read_until(&i);
  gps.heading_sv_num=(uint8_t)strtod(&gps_nmea.msg_buf[i], NULL);
  //gps_nmea.num_sta_use = (float)strtoi(&gps_nmea.msg_buf[i], NULL);
}
#endif

/**
 * Parse RMC NMEA messages.
 * Recommended minimum GPS sentence.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_RMC(void)
{
  int i = 6;     // current position in the message, start after: GPRMC,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_RMC() - skipping empty message\n\r");
    return;
  }
  // First read time (ignored)

  // get warning
  nmea_read_until(&i);

  // get lat
  nmea_read_until(&i);

  // get North/South
  nmea_read_until(&i);

  // get lon
  nmea_read_until(&i);

  // get eath/west
  nmea_read_until(&i);

  // get speed
  nmea_read_until(&i);
  double speed = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.gspeed = speed * 1.852 * 100 / (60 * 60);
  NMEA_PRINT("p_RMC() - ground-speed=%f knot = %d cm/s\n\r", speed, (gps.gspeed * 1000));

  // get course
  nmea_read_until(&i);
  double course = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.course = RadOfDeg(course) * 1e7;
  NMEA_PRINT("p_RMC() - course: %f deg\n\r", course);
}


/**
 * Parse GGA NMEA messages.
 * GGA has essential fix data providing 3D location and HDOP.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GGA(void)
{
  int i = 6;     // current position in the message, start after: GPGGA,
  double degrees, minutesfrac;
  struct LlaCoor_f lla_f;

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GGA() - skipping empty message\n\r");
    return;
  }

  // get UTC time [hhmmss.sss]
  // ignored GpsInfo.PosLLA.TimeOfFix.f = strtod(&packet[i], NULL);
  // FIXME: parse UTC time correctly
  double time = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.tow = (uint32_t)((time + 1) * 1000);

  // get latitude [ddmm.mmmmm]
  nmea_read_until(&i);
  double lat = strtod(&gps_nmea.msg_buf[i], NULL);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lat / 100, &degrees);
  lat = degrees + (minutesfrac * 100) / 60;

  // get latitute N/S
  nmea_read_until(&i);
  if (gps_nmea.msg_buf[i] == 'S') {
    lat = -lat;
  }

  // convert to radians
  lla_f.lat = RadOfDeg(lat);
  gps.lla_pos.lat = lat * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GGA() - lat=%f gps_lat=%f\n\r", (lat * 1000), lla_f.lat);


  // get longitude [ddmm.mmmmm]
  nmea_read_until(&i);
  double lon = strtod(&gps_nmea.msg_buf[i], NULL);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lon / 100, &degrees);
  lon = degrees + (minutesfrac * 100) / 60;

  // get longitude E/W
  nmea_read_until(&i);
  if (gps_nmea.msg_buf[i] == 'W') {
    lon = -lon;
  }

  // convert to radians
  lla_f.lon = RadOfDeg(lon);
  gps.lla_pos.lon = lon * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GGA() - lon=%f gps_lon=%f time=%u\n\r", (lon * 1000), lla_f.lon, gps.tow);

  // get position fix status
  nmea_read_until(&i);
  // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
  // check for good position fix
  if ((gps_nmea.msg_buf[i] != '0') && (gps_nmea.msg_buf[i] != ','))  {
    gps_nmea.pos_available = TRUE;
	gps_nmea.gps_qual = (uint8_t)gps_nmea.msg_buf[i];
    NMEA_PRINT("p_GGA() - POS_AVAILABLE == TRUE\n\r");
  } else {
    gps_nmea.pos_available = FALSE;
	gps_nmea.gps_qual = 0;
    NMEA_PRINT("p_GGA() - gps_pos_available == false\n\r");
  }

  // get number of satellites used in GPS solution
  nmea_read_until(&i);
  gps.num_sv = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GGA() - gps_numSatlitesUsed=%i\n\r", gps.num_sv);

  // get HDOP, but we use PDOP from GSA message
  nmea_read_until(&i);
  //float hdop = strtof(&gps_nmea.msg_buf[i], NULL);
  //gps.pdop = hdop * 100;

  // get altitude (in meters) above geoid (MSL)
  nmea_read_until(&i);
  float hmsl = (float)strtod(&gps_nmea.msg_buf[i], NULL);
  gps.hmsl = hmsl * 1000;
  NMEA_PRINT("p_GGA() - gps.hmsl=%i\n\r", gps.hmsl);

  // get altitude units (always M)
  nmea_read_until(&i);

  // get geoid seperation
  nmea_read_until(&i);
  float geoid = (float)strtod(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GGA() - geoid alt=%f\n\r", geoid);
  // height above ellipsoid
  lla_f.alt = hmsl + geoid;
  gps.lla_pos.alt = lla_f.alt * 1000;
  NMEA_PRINT("p_GGA() - gps.alt=%i\n\r", gps.lla_pos.alt);

  // get seperations units
  nmea_read_until(&i);
  // get DGPS age
  nmea_read_until(&i);
  // get DGPS station ID

#if GPS_USE_LATLONG
  /* convert to utm */
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  utm_of_lla_f(&utm_f, &lla_f);

  /* copy results of utm conversion */
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  /* convert to ECEF */
#if !USE_XYZA
  struct EcefCoor_f ecef_f;
  ecef_of_lla_f(&ecef_f, &lla_f);
  gps.ecef_pos.x = ecef_f.x * 100;
  gps.ecef_pos.y = ecef_f.y * 100;
  gps.ecef_pos.z = ecef_f.z * 100;
#endif
}

/**
 * Parse GSV-nmea-messages.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GSV(void)
{
  int i = 6;     // current position in the message, start after: GxGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GSV() - skipping empty message\n\r");
    return;
  }

  // check what satellites this messages contains
  // GPGSV -> GPS
  // GLGSV -> GLONASS
  bool_t is_glonass = FALSE;
  if (!strncmp(&gps_nmea.msg_buf[0] , "GL", 2)) {
    is_glonass = TRUE;
  }

  // total sentences
  int nb_sen __attribute__((unused)) = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - %i sentences\n\r", nb_sen);
  nmea_read_until(&i);

  // current sentence
  int cur_sen = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - sentence=%i\n\r", cur_sen);
  nmea_read_until(&i);

  // num satellites in view
  int num_sat __attribute__((unused)) = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - num_sat=%i\n\r", num_sat);
  nmea_read_until(&i);

  // up to 4 sats per sentence
  int sat_cnt;
  for (sat_cnt = 0; sat_cnt < 4; sat_cnt++) {
    if (gps_nmea.msg_buf[i] == ',') break;
    // 4 fields per sat: PRN, elevation (deg), azimuth (deg), SNR
    int prn = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int elev = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int azim = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int snr = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);

    int ch_idx = (cur_sen - 1) * 4 + sat_cnt;
    // don't populate svinfos with GLONASS sats for now
    if (!is_glonass && ch_idx > 0 && ch_idx < 12) {
      gps.svinfos[ch_idx].svid = prn;
      gps.svinfos[ch_idx].cno = snr;
      gps.svinfos[ch_idx].elev = elev;
      gps.svinfos[ch_idx].azim = azim;
    }
    if (is_glonass) {
      NMEA_PRINT("p_GSV() - GLONASS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
    else {
      NMEA_PRINT("p_GSV() - GPS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
  }
}

#if USE_XYZA
/**
 * Parse BESTXYZA messages.
 * Get ecef coor pos/speed information
 */
static void nmea_parse_XYZ(void)
{
  //get ecef_pos
  int i=1;
  for(uint8_t n=0;n<10;n++)
  {
  	nmea_read_until(&i);
  }
  if( !strncmp(&gps_nmea.msg_buf[i] , "NONE", 4) )
  {
  	gps_nmea.pos_xyz_available = FALSE;
	return;
  }
  gps_nmea.pos_xyz_available = TRUE;
  nmea_read_until(&i);  
  gps.ecef_pos.x = (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  nmea_read_until(&i);
  gps.ecef_pos.y = (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  nmea_read_until(&i);
  gps.ecef_pos.z = (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  
  nmea_read_until(&i);
  gps.ecef_pos_sd.x = (int32_t)(strtof(&gps_nmea.msg_buf[i], NULL)*10000);
  nmea_read_until(&i);
  gps.ecef_pos_sd.y = (int32_t)(strtof(&gps_nmea.msg_buf[i], NULL)*10000);
  nmea_read_until(&i);
  gps.ecef_pos_sd.z = (int32_t)(strtof(&gps_nmea.msg_buf[i], NULL)*10000);
  
  for(uint8_t m=0;m<3;m++)
  {
  	nmea_read_until(&i);
  }
  //get ecef_speed
  gps.ecef_vel.x= (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  nmea_read_until(&i);
  gps.ecef_vel.y= (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  nmea_read_until(&i);
  gps.ecef_vel.z= (int32_t)(strtod(&gps_nmea.msg_buf[i], NULL)*100);
  
}

#endif

/*run 20hz,use 2s time no fix pos set unstable*/
void get_gps_pos_stable(void)
{
	static uint8_t counter_nmea_qual = 0;
	if(gps_nmea.gps_qual != 52) //not fix pos
	{
		gps.p_stable = FALSE;
		counter_nmea_qual = 0;
	}
	else
	{
		counter_nmea_qual++;
	}

	if(counter_nmea_qual > 40)
	{
		gps.p_stable = TRUE;
		counter_nmea_qual = 41;  /*avoid overflow*/
	}	
}

/*run 20hz,use 2s time no fix heading set unstable*/
void get_gps_heading_stable(void)
{
	static uint8_t counter_heading = 0;
	if(gps_nmea.sol_tatus!= 4) //not fix pos
	{
		gps.h_stable = FALSE;
		counter_heading = 0;
	}
	else
	{
		counter_heading++;
	}

	if( ((counter_heading>100)&&(get_sys_time_float()<300.0))
		||((counter_heading>40)&&(get_sys_time_float()>300.0)) )
	{
		gps.h_stable = TRUE;
		counter_heading = 101;  /*avoid overflow*/
	}	
}
/*
double my_strtod(const char* s, char** endptr)

{

   register const char*  p     = s;

    register long double  value = 0.L;

    int                   sign  = 0;

    long double           factor;

    unsigned int          expo;

  

    while ( isspace(*p) )//\u8df3\u8fc7\u524d\u9762\u7684\u7a7a\u683c

      p++;

 

    if(*p == '-' || *p == '+')

      sign = *p++;//\u628a\u7b26\u53f7\u8d4b\u7ed9\u5b57\u7b26sign\uff0c\u6307\u9488\u540e\u79fb\u3002

  

   //\u5904\u7406\u6570\u5b57\u5b57\u7b26

 

    while ( (unsigned int)(*p - '0') < 10u )//\u8f6c\u6362\u6574\u6570\u90e8\u5206

      value = value*10 + (*p++ - '0');

   //\u5982\u679c\u662f\u6b63\u5e38\u7684\u8868\u793a\u65b9\u5f0f\uff08\u5982\uff1a1234.5678\uff09

   if ( *p == '.' )

   {

        factor = 1.;

        p++;

        while ( (unsigned int)(*p - '0') < 10u )

      {

         factor *= 0.1;

            value  += (*p++ - '0') * factor;

        }

    }

   //\u5982\u679c\u662fIEEE754\u6807\u51c6\u7684\u683c\u5f0f\uff08\u5982\uff1a1.23456E+3\uff09

    if ( (*p | 32) == 'e' )

   {

      expo   = 0;

        factor = 10.L;

        switch (*++p)

      {

        case '-':

           factor = 0.1;

        case '+':

           p++;

           break;

        case '0':

        case '1':

        case '2':

        case '3':

        case '4':

        case '5':

        case '6':

        case '7':

        case '8':

        case '9':

           break;

        default :

           value = 0.L;

           p     = s;

           goto done;

        }

        while ( (unsigned int)(*p - '0') < 10u )

            expo = 10 * expo + (*p++ - '0');

        while ( 1 )

      {

        if ( expo & 1 )

           value *= factor;

            if ( (expo >>= 1) == 0 )

                break;

            factor *= factor;

        }

    }

done:

    if ( endptr != 0 )

        *endptr = (char*)p;

 

    return (sign == '-' ? -value : value);

}
*/
