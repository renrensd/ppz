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

#define GPS_NB_CHANNELS 12

#define NMEA_MAXLEN 510

/*
 * BESTXYZ
 */
#define BESTXYZ_sol_NUM	14
enum _e_BESTXYZ_sol
{
	SOL_COMPUTED = 0, //Solution computed
	INSUFFICIENT_OBS = 1, //Insufficient observations
	NO_CONVERGENCE = 2, //No convergence
	SINGULARITY = 3, //Singularity at parameters matrix
	COV_TRACE = 4, //Covariance trace exceeds maximum (trace > 1000 m)
	TEST_DIST = 5, //Test distance exceeded (maximum of 3 rejections if distance >10 km)
	COLD_START = 6, //Not yet converged from cold start
	V_H_LIMIT = 7, //Height or velocity limits exceeded (in accordance with export licensing restrictions)
	VARIANCE = 8, //Variance exceeds limits
	RESIDUALS = 9, //Residuals are too large
	INTEGRITY = 13, //WARNING Large residuals make position unreliable
	PENDING = 18, //When a FIX POSITION command is entered, the receiver computes its own position and determines if the fixed position is valid a
	INVALID_FIX = 19, //The fixed position, entered using the FIX POSITION command, is not valid
	UNAUTHORIZED = 20 //Position type is unauthorized - HP or XP on a receiver not authorized for it
};

static const char *BESTXYZ_sol_strings[BESTXYZ_sol_NUM] =
		{
			"SOL_COMPUTED",
			"INSUFFICIENT_OBS",
			"NO_CONVERGENCE",
			"SINGULARITY",
			"COV_TRACE",
			"TEST_DIST",
			"COLD_START",
			"V_H_LIMIT",
			"VARIANCE",
			"RESIDUALS",
			"INTEGRITY",
			"PENDING",
			"INVALID_FIX",
			"UNAUTHORIZED"
		};

static const uint8_t BESTXYZ_sol_num[BESTXYZ_sol_NUM] =
		{
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 18, 19, 20
		};

#define BESTXYZ_type_NUM	18
enum _e_BESTXYZ_type
{
	NONE = 0,
	FIXEDPOS = 1,
	FIXEDHEIGHT = 2,
	DOPPLER_VELOCITY = 8,
	SINGLE = 16,
	PSRDIFF = 17,
	WAAS = 18,
	PROPAGATED = 19,
	OMNISTAR = 20,
	L1_FLOAT = 32,
	IONOFREE_FLOAT = 33,
	NARROW_FLOAT = 34,
	L1_INT = 48,
	NARROW_INT = 50,
	OMNISTAR_HP = 64,
	OMNISTAR_XP = 65,
	PPP_CONVERGING = 68,
	PPP = 69
};

static const char *BESTXYZ_type_strings[BESTXYZ_type_NUM] =
		{
			"NONE",
			"FIXEDPOS",
			"FIXEDHEIGHT",
			"DOPPLER_VELOCITY",
			"SINGLE",
			"PSRDIFF",
			"WAAS",
			"PROPAGATED",
			"OMNISTAR",
			"L1_FLOAT",
			"IONOFREE_FLOAT",
			"NARROW_FLOAT",
			"L1_INT",
			"NARROW_INT",
			"OMNISTAR_HP",
			"OMNISTAR_XP",
			"PPP_CONVERGING",
			"PPP"
		};

static const uint8_t BESTXYZ_type_num[BESTXYZ_type_NUM] =
		{
			0, 1, 2, 8, 16, 17, 18, 19, 20, 32, 33, 34, 48, 50, 64, 65, 68, 69
		};

struct _s_nmea_BESTXYZ
{
	enum _e_BESTXYZ_sol P_sol;
	enum _e_BESTXYZ_type P_type;
	double Px;
	double Py;
	double Pz;
	float Px_sd;
	float Py_sd;
	float Pz_sd;

	enum _e_BESTXYZ_sol V_sol;
	enum _e_BESTXYZ_type V_type;
	double Vx;
	double Vy;
	double Vz;
	float Vx_sd;
	float Vy_sd;
	float Vz_sd;

	uint8_t SV_tracked;
	uint8_t SV_used;

	struct _s_nmea_BESTXYZ_error
	{
		bool_t PV_type_empty;
		bool_t PV_empty;
	} error;
};

/*
 * END OF BESTXYZ
 */

/*
 * GPTRA
 */
struct _s_nmea_GPTRA
{
	float heading;
	uint8_t sol;
	uint8_t SV_used;

	struct _s_nmea_GPTRA_error
	{
		bool_t heading_empty;
		bool_t sol_empty;
	} error;
};

/*
 * END OF GPTRA
 */

/*
 * GPRMC
 */
struct _s_nmea_GPRMC
{
	uint32_t utc;
	uint32_t date;
};

/*
 * END OF GPRMC
 */

struct GpsNmea
{
	bool_t msg_available;
	bool_t pos_available;
	#ifdef USE_BESTXYZ
	bool_t pos_xyz_available;
	#endif
#ifdef USE_GPS_HEADING
	bool_t heading_available;
#endif
	bool_t is_configured;       ///< flag set to TRUE if configuration is finished
	bool_t have_gsv;            ///< flag set to TRUE if GPGSV message received
	uint8_t gps_nb_ovrn;        ///< number if incomplete nmea-messages
	char msg_buf[NMEA_MAXLEN];  ///< buffer for storing one nmea-line
	int32_t msg_len;
	uint8_t status;             ///< line parser status

	struct _s_nmea_BESTXYZ BESTXYZ;
	struct _s_nmea_GPTRA GPTRA;
	struct _s_nmea_GPRMC GPRMC;

	uint32_t last_xyzmsg_time; ///< last available msg cpu time(ms)
	uint32_t last_tramsg_time; ///< last available msg cpu time(ms)
};

extern struct GpsNmea gps_nmea;

/*
 * This part is used by the autopilot to read data from a uart
 */

/** The function to be called when a characted from the device is available */
#include "mcu_periph/link_device.h"

extern void nmea_configure(void);
extern void nmea_parse_char(uint8_t c);
extern uint8_t nmea_calc_crc(const char *buff, int buff_sz);
extern void nmea_parse_prop_init(void);
extern void nmea_parse_prop_msg(void);
extern void gps_nmea_msg(void);

extern bool_t BESTXYZ_data_valid(void);
extern bool_t GPTRA_heading_valid(void);

static inline void GpsEvent(void)
{
	struct link_device *dev = &((GPS_LINK).device);

	if( !gps_nmea.is_configured )
	{
		nmea_configure();
		return;
	}
	while (dev->char_available(dev->periph))
	{
		nmea_parse_char(dev->get_byte(dev->periph));
		if( gps_nmea.msg_available )
		{
			gps_nmea_msg();
		}
	}
}

#endif /* GPS_NMEA_H */
