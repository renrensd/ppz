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

#ifdef USE_BESTXYZ
#define CRC32_POLYNOMIAL 0xEDB88320
#endif

struct GpsNmea gps_nmea;

static bool_t nmea_read_empty(int i);
static void nmea_read_until_char(int *i, char c);
static void nmea_read_until(int *i);

static void nmea_parse_msg(void);
static void nmea_parse_GSA(void);
static void nmea_parse_RMC(void);
static void nmea_parse_GGA(void);
static void nmea_parse_GSV(void);
#ifdef USE_GPS_HEADING
static void nmea_parse_TRA(void);
#endif

#ifdef USE_BESTXYZ
uint32_t CRC32Value(int32_t i);
uint32_t CalculateXYZACRC32(uint16_t ulCount, unsigned char *ucBuffer);
static void nmea_parse_XYZ(void);
#endif

/*double my_strtod(const char* s, char** endptr);*/

#ifdef USE_BESTXYZ
uint32_t CRC32Value(int32_t i)
{
	uint8_t j;
	uint32_t ulCRC;
	ulCRC = (uint32_t) i;
	for (j = 8; j > 0; j--)
	{
		if( ulCRC & 1 )
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

uint32_t CalculateXYZACRC32(uint16_t ulCount, unsigned char *ucBuffer)
{
	uint32_t ulTemp1;
	uint32_t ulTemp2;
	uint32_t ulCRC = 0;

	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		//ulTemp2 = CRC32Value( ( (int32_t)ulCRC^*ucBuffer++ )&0xff );
		ulTemp2 = CRC32Value((ulCRC ^ (uint32_t) (*ucBuffer)) & 0x000000FFL);
		ucBuffer++;
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}
#endif

void gps_impl_init(void)
{
	gps.nb_channels = GPS_NB_CHANNELS;
	gps_nmea.is_configured = FALSE;
	gps_nmea.msg_available = FALSE;
	gps_nmea.pos_available = FALSE;
#ifdef USE_BESTXYZ
	gps_nmea.pos_xyz_available = FALSE;
	gps_nmea.BESTXYZ.error.PV_type_empty = TRUE;
	gps_nmea.BESTXYZ.error.PV_empty = TRUE;
#endif
#ifdef USE_GPS_HEADING
	gps_nmea.heading_available = FALSE;
	gps_nmea.GPTRA.error.sol_empty = TRUE;
	gps_nmea.GPTRA.error.heading_empty = TRUE;
#endif
	gps_nmea.have_gsv = FALSE;
	gps_nmea.gps_nb_ovrn = 0;
	gps_nmea.msg_len = 0;
	nmea_parse_prop_init();
	nmea_configure();

	//TEST_CASE

	char *test_string1 =
			"#BESTXYZA,COM1,0,55.0,FINESTEERING,1419,340033.000,00000040,D821,\
2724;SOL_COMPUTED,IONOFREE_FLOAT,-1634531.5683,,\
4942496.3270,0.0099,0.0219,0.0115,SOL_COMPUTED,NARROW_INT,0.0011,\
-0.0049,-0.0001,0.0199,0.0439,0.0230,\"AAAA\",0.250,1.000,0.000,\
12,11,11,11,0,01,0,33*E9EAFECA";
	char *test_string2 = "$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20";
	char *test_string3 = "$GPTRA,063027.30,101.78,071.19,-00.00,2,10,0.00,0004*51";
	/*
	 char *test_string =
	 "#BESTXYZA,COM1,0,55.0,FINESTEERING,1419,340033.000,00000040,D821,\
2724;SOL_COMPUTED,NARROW_INT,,-3664618.0326,\
4942496.3270,0.0099,0.0219,0.0115,SOL_COMPUTED,NARROW_INT,0.0011,\
-0.0049,-0.0001,0.0199,0.0439,0.0230,\"AAAA\",0.250,1.000,0.000,\
12,11,11,11,0,01,0,33*E9EAFECA";
	 */
	gps_nmea.msg_len = strlen(test_string1);
	for (int n = 0; n < gps_nmea.msg_len; ++n)
	{
		gps_nmea.msg_buf[n] = test_string1[n];
	}
	nmea_parse_XYZ();

	gps_nmea.msg_len = strlen(test_string2);
	for (int n = 0; n < gps_nmea.msg_len; ++n)
	{
		gps_nmea.msg_buf[n] = test_string2[n];
	}
	nmea_parse_RMC();

	gps_nmea.msg_len = strlen(test_string3);
	for (int n = 0; n < gps_nmea.msg_len; ++n)
	{
		gps_nmea.msg_buf[n] = test_string3[n];
	}
	nmea_parse_TRA();
}

void gps_nmea_msg(void)
{
	// current timestamp
	uint32_t now_ts = get_sys_time_usec();
	gps.last_msg_ticks = sys_time.nb_sec_rem;
	gps.last_msg_time = sys_time.nb_sec;
	nmea_parse_msg();

	/*abi send pos info*/
	if(
	#ifdef USE_BESTXYZ
	gps_nmea.pos_xyz_available
	#else
	gps_nmea.pos_available
#endif
	)
	{
		if( gps.fix == GPS_FIX_3D )
		{
			gps.last_3dfix_ticks = sys_time.nb_sec_rem;
			gps.last_3dfix_time = sys_time.nb_sec;
		}
		AbiSendMsgGPS_POS(GPS_NMEA_ID, now_ts, &gps);
	}
#ifdef USE_BESTXYZ
	gps_nmea.pos_xyz_available = FALSE; /*xyza msg finished,reset to false*/
#else
	gps_nmea.pos_available = FALSE;
#endif

	/*abi send heading info*/
#ifdef USE_GPS_HEADING
	if( gps_nmea.heading_available )
	{
		AbiSendMsgGPS_HEADING(GPS_NMEA_ID, now_ts, &gps);
	}
	gps_nmea.heading_available = FALSE;
#endif

	gps_nmea.msg_available = FALSE;

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

static bool_t nmea_read_empty(int i)
{
	if( (gps_nmea.msg_buf[i] == ',') )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

static void nmea_read_until_char(int *i, char c)
{
	while (gps_nmea.msg_buf[(*i)++] != c)
	{
		if( *i >= gps_nmea.msg_len )
		{
			return;
		}
	}
}
/** Read until a certain character, placed here for proprietary includes */
static void nmea_read_until(int *i)
{
	nmea_read_until_char(i, ',');
}

/**
 * nmea_parse_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
static void nmea_parse_msg(void)
{

	if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "RMC", 3) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		NMEA_PRINT("RMC: \"%s\" \n\r", gps_nmea.msg_buf);
		nmea_parse_RMC();
	}
	else if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "GGA", 3) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		NMEA_PRINT("GGA: \"%s\" \n\r", gps_nmea.msg_buf);
		//nmea_parse_GGA();
	}
	else if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "GSA", 3) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		NMEA_PRINT("GSA: \"%s\" \n\r", gps_nmea.msg_buf);
		//nmea_parse_GSA();
	}
	else if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "GSV", 3) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		gps_nmea.have_gsv = TRUE;
		NMEA_PRINT("GSV: \"%s\" \n\r", gps_nmea.msg_buf);
		//nmea_parse_GSV();
	}
#ifdef USE_GPS_HEADING
	else if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "TRA", 3) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		NMEA_PRINT("TRA: \"%s\" \n\r", gps_nmea.msg_buf);
		nmea_parse_TRA();
	}
#endif
#ifdef USE_BESTXYZ  //add BESTXYZA data to get ecef information
	else if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[4], "XYZA", 4) )
	{
		gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
		NMEA_PRINT("XYZ: \"%s\" \n\r", gps_nmea.msg_buf);
		nmea_parse_XYZ();
	}
#endif
	else
	{
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
	uint32_t crc_sum, cal;
	switch (gps_nmea.status)
	{
		case WAIT:
			gps_nmea.msg_len = 0;    //reset buf length

			if( c == '$' ) /* nmea valid message needs to start with $ sign */
			{
				header_type = 1;
				gps_nmea.status = GOT_START;
			}
			else if( c == '#' ) /* xyza valid message needs to start with # sign */
			{
				header_type = 2;
				gps_nmea.status = GOT_START;
			}
			break;

		case GOT_START:
			switch (c)
			{
				case '\r':
					case '\n':
					if( gps_nmea.msg_len != 0 ) /*do crc verify*/
					{
						if( 1 == header_type )
						{
							char crc_char[2] = { gps_nmea.msg_buf[gps_nmea.msg_len - 2], gps_nmea.msg_buf[gps_nmea.msg_len - 1] };
							crc_sum = (uint32_t) (strtoul(crc_char, NULL, 16));
							cal = (uint32_t) nmea_calc_crc(gps_nmea.msg_buf, gps_nmea.msg_len - 3);
						}
#ifdef USE_BESTXYZ
						else if( 2 == header_type )
						{
							char crc_char[9];
							for (uint8_t i = 0; i < 8; i++)
							{
								crc_char[i] = gps_nmea.msg_buf[gps_nmea.msg_len - 8 + i];
							}
							crc_char[8] = 0x2C; /*set last char "," for get exact data*/

							crc_sum = (uint32_t) (strtoul(crc_char, NULL, 16));
							cal = CalculateXYZACRC32((unsigned long) (gps_nmea.msg_len - 9), (unsigned char*) gps_nmea.msg_buf);
						}
#endif

						if( cal == crc_sum )
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
							gps_nmea.msg_len, gps_nmea.msg_buf)
					;
					gps_nmea.status = WAIT;
					break;

				default:
					// fill the buffer, unless it's full
					if( gps_nmea.msg_len < NMEA_MAXLEN - 1 )
					{
						gps_nmea.msg_buf[gps_nmea.msg_len] = c;
						gps_nmea.msg_len++;
					}
					else
					{
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
			NMEA_PRINT("nmea_parse_char: this should not happen!")
			;
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

	for (it = 0; it < buff_sz; ++it)
	{
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
#if 0
	int i = 6;     // current position in the message, start after: GPGSA,

	// set gps_mode=3=3d, 2=2d, 1=no fix or 0
	nmea_read_until(&i);
	if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',')
	{
		return;
	}
	uint8_t fix = atoi(&gps_nmea.msg_buf[i]);
	if (fix < 3)
	{
		if(gps_nmea.pos_type >= SINGLE)
		{
			fix = 3;   //force change to 3D
		}
	}
	gps.fix = fix;
	NMEA_PRINT("p_GSA() - gps.fix=%i (3=3D)\n\r", gps.fix);
	nmea_read_until(&i);

	// up to 12 PRNs of satellites used for fix
	int satcount = 0;
	int prn_cnt;
	for (prn_cnt = 0; prn_cnt < 12; prn_cnt++)
	{
		if (gps_nmea.msg_buf[i] != ',')
		{
			int prn = atoi(&gps_nmea.msg_buf[i]);
			NMEA_PRINT("p_GSA() - PRN %i=%i\n\r", satcount, prn);
			if (!gps_nmea.have_gsv)
			{
				gps.svinfos[prn_cnt].svid = prn;
			}
			satcount++;
		}
		else
		{
			if (!gps_nmea.have_gsv)
			{
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
	*/
#endif
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

bool_t GPTRA_heading_valid(void)
{
	if( gps_nmea.GPTRA.error.sol_empty || gps_nmea.GPTRA.error.heading_empty )
	{
		return FALSE;
	}

	if( gps_nmea.GPTRA.sol != 4 )
	{
		return FALSE;
	}

	if( (gps_nmea.GPTRA.heading < 0.0) || (gps_nmea.GPTRA.heading > 360.0) )
	{
		return FALSE;
	}

	return TRUE;
}

static void nmea_parse_TRA(void)
{
	int i = 0;

	gps_nmea.heading_available = FALSE;
	gps_nmea.GPTRA.error.heading_empty = FALSE;
	gps_nmea.GPTRA.error.sol_empty = FALSE;

	nmea_read_until(&i);
	nmea_read_until(&i);
	if( !nmea_read_empty(i) )
	{
		gps_nmea.GPTRA.heading = (float) strtod(&gps_nmea.msg_buf[i], NULL);
	}
	else
	{
		gps_nmea.GPTRA.error.heading_empty = TRUE;
	}
	nmea_read_until(&i);
	nmea_read_until(&i);
	nmea_read_until(&i);
	if( !nmea_read_empty(i) )
	{
		gps_nmea.GPTRA.sol = (uint8_t) strtod(&gps_nmea.msg_buf[i], NULL);
	}
	else
	{
		gps_nmea.GPTRA.error.sol_empty = TRUE;
	}

	nmea_read_until(&i);
	gps_nmea.GPTRA.SV_used = (uint8_t) strtod(&gps_nmea.msg_buf[i], NULL);

	if( GPTRA_heading_valid() )
	{
		gps.heading = gps_nmea.GPTRA.heading + GPS_HEADING_OFFSET;
		Course360(gps.heading);
		gps_nmea.heading_available = TRUE;
	}
	gps.head_sv = gps_nmea.GPTRA.SV_used;

	gps_nmea.last_tramsg_time = get_sys_time_msec();
}
#endif

/**
 * Parse RMC NMEA messages.
 * Recommended minimum GPS sentence.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_RMC(void)
{
	int i = 0;

	nmea_read_until(&i);
	if( !nmea_read_empty(i) )
	{
		gps_nmea.GPRMC.utc = strtoul(&gps_nmea.msg_buf[i], NULL, 10);
	}
	for (uint8_t n = 0; n < 8; ++n)
	{
		nmea_read_until(&i);
	}
	if( !nmea_read_empty(i) )
	{
		gps_nmea.GPRMC.date = strtoul(&gps_nmea.msg_buf[i], NULL, 10);
	}

	gps.gps_time.hour = gps_nmea.GPRMC.utc / 10000;
	gps.gps_time.minute = (gps_nmea.GPRMC.utc - gps.gps_time.hour * 10000) / 100;
	gps.gps_time.second = gps_nmea.GPRMC.utc % 100;

	gps.gps_time.day = gps_nmea.GPRMC.date / 10000;
	gps.gps_time.month = (gps_nmea.GPRMC.date - gps.gps_time.day * 10000) / 100;
	gps.gps_time.year = gps_nmea.GPRMC.date % 100;
}

/**
 * Parse GGA NMEA messages.
 * GGA has essential fix data providing 3D location and HDOP.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GGA(void)
{
#if 0
	int i = 6;     // current position in the message, start after: GPGGA,
	double degrees, minutesfrac;
	struct LlaCoor_f lla_f;

	// attempt to reject empty packets right away
	if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',')
	{
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
	if (gps_nmea.msg_buf[i] == 'S')
	{
		lat = -lat;
	}

	// convert to radians
	lla_f.lat = RadOfDeg(lat);
	gps.lla_pos.lat = lat * 1e7;// convert to fixed-point
	NMEA_PRINT("p_GGA() - lat=%f gps_lat=%f\n\r", (lat * 1000), lla_f.lat);

	// get longitude [ddmm.mmmmm]
	nmea_read_until(&i);
	double lon = strtod(&gps_nmea.msg_buf[i], NULL);
	// convert to pure degrees [dd.dddd] format
	minutesfrac = modf(lon / 100, &degrees);
	lon = degrees + (minutesfrac * 100) / 60;

	// get longitude E/W
	nmea_read_until(&i);
	if (gps_nmea.msg_buf[i] == 'W')
	{
		lon = -lon;
	}

	// convert to radians
	lla_f.lon = RadOfDeg(lon);
	gps.lla_pos.lon = lon * 1e7;// convert to fixed-point
	NMEA_PRINT("p_GGA() - lon=%f gps_lon=%f time=%u\n\r", (lon * 1000), lla_f.lon, gps.tow);

	// get position fix status
	nmea_read_until(&i);
	if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',')
	{
		return;
	}
	// 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
	// check for good position fix
	if ((gps_nmea.msg_buf[i] != '0') && (gps_nmea.msg_buf[i] != ','))
	{
		gps_nmea.pos_available = TRUE;
		gps_nmea.gps_qual = (uint8_t)gps_nmea.msg_buf[i];
	}
	else
	{
		gps_nmea.pos_available = FALSE;
		gps_nmea.gps_qual = 0;
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
#ifndef USE_BESTXYZ
	struct EcefCoor_f ecef_f;
	ecef_of_lla_f(&ecef_f, &lla_f);
	gps.ecef_pos.x = ecef_f.x * 100;
	gps.ecef_pos.y = ecef_f.y * 100;
	gps.ecef_pos.z = ecef_f.z * 100;
#endif
#endif
}

/**
 * Parse GSV-nmea-messages.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GSV(void)
{
#if 0
	int i = 6;     // current position in the message, start after: GxGSA,

	// attempt to reject empty packets right away
	if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',')
	{
		NMEA_PRINT("p_GSV() - skipping empty message\n\r");
		return;
	}

	// check what satellites this messages contains
	// GPGSV -> GPS
	// GLGSV -> GLONASS
	bool_t is_glonass = FALSE;
	if (!strncmp(&gps_nmea.msg_buf[0] , "GL", 2))
	{
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
	for (sat_cnt = 0; sat_cnt < 4; sat_cnt++)
	{
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
		if (!is_glonass && ch_idx > 0 && ch_idx < 12)
		{
			gps.svinfos[ch_idx].svid = prn;
			gps.svinfos[ch_idx].cno = snr;
			gps.svinfos[ch_idx].elev = elev;
			gps.svinfos[ch_idx].azim = azim;
		}
		if (is_glonass)
		{
			NMEA_PRINT("p_GSV() - GLONASS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
		}
		else
		{
			NMEA_PRINT("p_GSV() - GPS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
		}
	}
#endif
}

#ifdef USE_BESTXYZ

static enum _e_BESTXYZ_sol nmea_parse_XYZ_sol(int i)
{
	uint8_t n;
	enum _e_BESTXYZ_sol sol = 0xFF;

	for (n = 0; n < BESTXYZ_sol_NUM; ++n)
	{
		if( strncmp(&gps_nmea.msg_buf[i], BESTXYZ_sol_strings[n], strlen(BESTXYZ_sol_strings[n]) - 1) == 0 )
		{
			sol = BESTXYZ_sol_num[n];
			break;
		}
	}

	return sol;
}

static enum _e_BESTXYZ_type nmea_parse_XYZ_type(int i)
{
	uint8_t n;
	enum _e_BESTXYZ_type type = 0xFF;

	for (n = 0; n < BESTXYZ_type_NUM; ++n)
	{
		if( strncmp(&gps_nmea.msg_buf[i], BESTXYZ_type_strings[n], strlen(BESTXYZ_type_strings[n]) - 1) == 0 )
		{
			type = BESTXYZ_type_num[n];
			break;
		}
	}

	return type;
}

static double nmea_parse_BESTXYZ_double(int *i)
{
	if( !nmea_read_empty(*i) )
	{
		return strtod(&gps_nmea.msg_buf[*i], NULL);
	}
	else
	{
		gps_nmea.BESTXYZ.error.PV_empty = TRUE;
		return 0;
	}
}

bool_t BESTXYZ_data_valid(void)
{
	if( gps_nmea.BESTXYZ.error.PV_type_empty || gps_nmea.BESTXYZ.error.PV_empty )
	{
		return FALSE;
	}

	if( (gps_nmea.BESTXYZ.P_type <= NARROW_FLOAT) || (gps_nmea.BESTXYZ.V_type <= NARROW_FLOAT) )
	{
		return FALSE;
	}

	return TRUE;
}

static void nmea_parse_XYZ(void)
{
	int i = 0;

	gps_nmea.pos_xyz_available = FALSE;   //msg enter, set default false
	gps_nmea.BESTXYZ.error.PV_type_empty = FALSE;
	gps_nmea.BESTXYZ.error.PV_empty = FALSE;

	nmea_read_until_char(&i, ';');
	gps_nmea.BESTXYZ.P_sol = nmea_parse_XYZ_sol(i);
	nmea_read_until(&i);
	if( !nmea_read_empty(i) )
	{
		gps_nmea.BESTXYZ.P_type = nmea_parse_XYZ_type(i);
	}
	else
	{
		gps_nmea.BESTXYZ.error.PV_type_empty = TRUE;
	}
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Px = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Py = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Pz = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Px_sd = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Py_sd = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Pz_sd = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.V_sol = nmea_parse_XYZ_sol(i);
	nmea_read_until(&i);
	if( !nmea_read_empty(i) )
	{
		gps_nmea.BESTXYZ.V_type = nmea_parse_XYZ_type(i);
	}
	else
	{
		gps_nmea.BESTXYZ.error.PV_type_empty = TRUE;
	}
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vx = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vy = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vz = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vx_sd = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vy_sd = nmea_parse_BESTXYZ_double(&i);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.Vz_sd = nmea_parse_BESTXYZ_double(&i);
	for (uint8_t n = 0; n < 5; ++n)
	{
		nmea_read_until(&i);
	}
	gps_nmea.BESTXYZ.SV_tracked = strtod(&gps_nmea.msg_buf[i], NULL);
	nmea_read_until(&i);
	gps_nmea.BESTXYZ.SV_used = strtod(&gps_nmea.msg_buf[i], NULL);

	if( BESTXYZ_data_valid() )
	{
		gps.ecef_pos.x = gps_nmea.BESTXYZ.Px * 100.0;
		gps.ecef_pos.y = gps_nmea.BESTXYZ.Py * 100.0;
		gps.ecef_pos.z = gps_nmea.BESTXYZ.Pz * 100.0;
		gps.ecef_pos_sd.x = gps_nmea.BESTXYZ.Px_sd * 10000.0f;
		gps.ecef_pos_sd.y = gps_nmea.BESTXYZ.Py_sd * 10000.0f;
		gps.ecef_pos_sd.z = gps_nmea.BESTXYZ.Pz_sd * 10000.0f;

		gps.ecef_vel.x = gps_nmea.BESTXYZ.Vx * 100.0;
		gps.ecef_vel.y = gps_nmea.BESTXYZ.Vy * 100.0;
		gps.ecef_vel.z = gps_nmea.BESTXYZ.Vz * 100.0;
		gps.ecef_vel_sd.x = gps_nmea.BESTXYZ.Vx_sd * 10000.0f;
		gps.ecef_vel_sd.y = gps_nmea.BESTXYZ.Vy_sd * 10000.0f;
		gps.ecef_vel_sd.z = gps_nmea.BESTXYZ.Vz_sd * 10000.0f;

		gps_nmea.pos_xyz_available = TRUE;
	}
	gps.pos_sv = gps_nmea.BESTXYZ.SV_used;

	gps_nmea.last_xyzmsg_time = get_sys_time_msec();
}

#endif

