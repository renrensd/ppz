/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 */

/** @file gps.c
 *  @brief Device independent GPS code
 *
 */

#include "subsystems/gps.h"
#include "led.h"
#ifdef USE_GPS_NMEA
#include "subsystems/gps/gps_nmea.h"
#endif

#ifdef GPS_POWER_GPIO
#include "mcu_periph/gpio.h"

#ifndef GPS_POWER_GPIO_ON
#define GPS_POWER_GPIO_ON gpio_set
#endif
#endif

#define MSEC_PER_WEEK (1000*60*60*24*7)

struct GpsState gps;

struct GpsTimeSync gps_time_sync;

//static void get_gps_pos_stable(void);
//static void get_gps_heading_stable(void);
static void gps_pos_state_update(void);
static void gps_head_state_update(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_svinfo_id(struct transport_tx *trans, struct link_device *dev,
													 uint8_t svid)
{
	if (svid < GPS_NB_CHANNELS)
	{
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		pprz_msg_send_SVINFO(trans, dev, AC_ID, &svid,
												 &gps.svinfos[svid].svid, &gps.svinfos[svid].flags,
												 &gps.svinfos[svid].qi, &gps.svinfos[svid].cno,
												 &gps.svinfos[svid].elev, &gps.svinfos[svid].azim);
	}
}

/** send SVINFO message (regardless of state) */
static void send_svinfo(struct transport_tx *trans, struct link_device *dev)
{
	static uint8_t i = 0;
	if (i == gps.nb_channels)
	{
		i = 0;
	}
	send_svinfo_id(trans, dev, i);
	i++;
}

/** send SVINFO message if updated.
 * send SVINFO for all satellites while no GPS fix,
 * after 3D fix, send avialable sats only when there is new information
 */
static inline void send_svinfo_available(struct transport_tx *trans, struct link_device *dev)
{
	static uint8_t i = 0;
	static uint8_t last_cnos[GPS_NB_CHANNELS];
	if (i >= gps.nb_channels)
	{
		i = 0;
	}
	// send SVINFO for all satellites while no GPS fix,
	// after 3D fix, send avialable sats if they were updated
	if (gps.fix < GPS_FIX_3D)
	{
		send_svinfo_id(trans, dev, i);
	}
	else if (gps.svinfos[i].cno != last_cnos[i])
	{
		send_svinfo_id(trans, dev, i);
		last_cnos[i] = gps.svinfos[i].cno;
	}
	i++;
}

static void send_gps(struct transport_tx *trans, struct link_device *dev)
{
	uint8_t zero = 0;
	int16_t climb = -gps.ned_vel.z;
	int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_GPS(trans, dev, AC_ID, &gps.fix,
										&gps.utm_pos.east, &gps.utm_pos.north,
										&course, &gps.hmsl, &gps.gspeed, &climb,
										&gps.week, &gps.tow, &gps.utm_pos.zone, &zero);
	// send SVINFO for available satellites that have new data
	send_svinfo_available(trans, dev);
}

static void send_gps_int(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_GPS_INT(trans, dev, AC_ID,
												&gps.ecef_pos.x, &gps.ecef_pos.y, &gps.ecef_pos.z,
												&gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
												&gps.hmsl,
												&gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z,
												&gps.pacc, &gps.sacc,
												&gps.tow,
												&gps.pdop,
												&gps.pos_sv,
												&gps.fix,
												&gps.heading);
	// send SVINFO for available satellites that have new data
	send_svinfo_available(trans, dev);
}

static void send_gps_lla(struct transport_tx *trans, struct link_device *dev)
{
	uint8_t err = 0;
	int16_t climb = -gps.ned_vel.z;
	int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_GPS_LLA(trans, dev, AC_ID,
												&gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
												&gps.hmsl, &course, &gps.gspeed, &climb,
												&gps.week, &gps.tow,
												&gps.fix, &err);
}

static void send_gps_origin(struct transport_tx *trans, struct link_device *dev)
{
	float Px = gps_nmea.BESTXYZ.Px;
	float Py = gps_nmea.BESTXYZ.Py;
	float Pz = gps_nmea.BESTXYZ.Pz;
	float Vx = gps_nmea.BESTXYZ.Vx;
	float Vy = gps_nmea.BESTXYZ.Vy;
	float Vz = gps_nmea.BESTXYZ.Vz;

	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	pprz_msg_send_GPS_ORIGIN(trans, dev, AC_ID,
			&gps_nmea.BESTXYZ.error.PV_type_empty,
			&gps_nmea.BESTXYZ.error.PV_empty,
			&gps_nmea.BESTXYZ.P_sol,
			&gps_nmea.BESTXYZ.P_type,
			&Px,
			&Py,
			&Pz,
			&gps_nmea.BESTXYZ.Px_sd,
			&gps_nmea.BESTXYZ.Py_sd,
			&gps_nmea.BESTXYZ.Pz_sd,
			&gps_nmea.BESTXYZ.V_sol,
			&gps_nmea.BESTXYZ.V_type,
			&Vx,
			&Vy,
			&Vz,
			&gps_nmea.BESTXYZ.Vx_sd,
			&gps_nmea.BESTXYZ.Vy_sd,
			&gps_nmea.BESTXYZ.Vz_sd,
			&gps_nmea.BESTXYZ.SV_tracked,
			&gps_nmea.BESTXYZ.SV_used,
			&gps_nmea.GPTRA.error.sol_empty,
			&gps_nmea.GPTRA.error.heading_empty,
			&gps_nmea.GPTRA.heading,
			&gps_nmea.GPTRA.sol,
			&gps_nmea.GPTRA.SV_used);
}
#endif

void gps_init(void)
{
	gps.fix = GPS_FIX_NONE;
	gps.week = 0;
	gps.tow = 0;
	gps.cacc = 0;
	gps.last_3dfix_ticks = 0;
	gps.last_3dfix_time = 0;
	gps.last_msg_ticks = 0;
	gps.last_msg_time = 0;
	gps.alive = FALSE;

	gps.pos_timeout = TRUE;
	gps.head_timeout = TRUE;
#ifdef GPS_POWER_GPIO
	gpio_setup_output(GPS_POWER_GPIO);
	GPS_POWER_GPIO_ON(GPS_POWER_GPIO);
#endif
#ifdef GPS_LED
	LED_OFF(GPS_LED);
#endif
#ifdef GPS_TYPE_H
	gps_impl_init();
#endif

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS, send_gps);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_INT, send_gps_int);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_LLA, send_gps_lla);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_ORIGIN, send_gps_origin);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SVINFO, send_svinfo);
#endif
}

/*call by failsaft function, 20hz*/
void gps_periodic_check(void)
{
	if (sys_time.nb_sec - gps.last_msg_time > GPS_TIMEOUT)
	{
		gps.fix = GPS_FIX_NONE;
		gps.alive = FALSE;
	}
	else
	{
		gps.alive = TRUE;
	}

	/*ublox or nmea get gps.p_stable, use for monitoring*/
#ifndef USE_GPS_NMEA
	else if(gps.pacc < 200)
	{
		gps.p_stable = TRUE;
	}
	else if(gps.pacc > 500)
	{
		gps.p_stable = FALSE;
	}
#else
	gps_pos_state_update();
#ifdef USE_GPS_HEADING
	gps_head_state_update();
#endif
#endif
}

uint32_t gps_tow_from_sys_ticks(uint32_t sys_ticks)
{
	uint32_t clock_delta;
	uint32_t time_delta;
	uint32_t itow_now;

	if (sys_ticks < gps_time_sync.t0_ticks)
	{
		clock_delta = (0xFFFFFFFF - sys_ticks) + gps_time_sync.t0_ticks + 1;
	}
	else
	{
		clock_delta = sys_ticks - gps_time_sync.t0_ticks;
	}

	time_delta = msec_of_sys_time_ticks(clock_delta);

	itow_now = gps_time_sync.t0_tow + time_delta;
	if (itow_now > MSEC_PER_WEEK)
	{
		itow_now %= MSEC_PER_WEEK;
	}

	return itow_now;
}

/**
 * Default parser for GPS injected data
 */
void WEAK gps_inject_data(uint8_t packet_id __attribute__((unused)), uint8_t length __attribute__((unused)), uint8_t *data __attribute__((unused)))
{

}

#define MSG_TIME_OUT 1000  //unit:ms
/*run 20hz,use 2s time no fix pos set unstable*/

static void gps_pos_state_update(void)
{
	uint32_t now_time = get_sys_time_msec();

	gps.pos_timeout = ((now_time - gps_nmea.last_xyzmsg_time) > MSG_TIME_OUT);
}

static void gps_head_state_update(void)
{
	uint32_t now_time = get_sys_time_msec();

	gps.head_timeout = ((now_time - gps_nmea.last_tramsg_time) > MSG_TIME_OUT);
}

static bool_t gps_pos_valid(void)
{
	if( gps.pos_timeout )
	{
		return FALSE;
	}

	if( !BESTXYZ_data_valid() )
	{
		return FALSE;
	}

	return TRUE;
}

static bool_t gps_head_valid(void)
{
	if( gps.head_timeout )
	{
		return FALSE;
	}

	if( !GPTRA_heading_valid() )
	{
		return FALSE;
	}

	return TRUE;
}

bool_t rtk_pos_stable(void)
{
	return gps_pos_valid() && (gps.pos_sv > 4);
}

bool_t rtk_head_stable(void)
{
	return gps_head_valid() && (gps.head_sv > 4);
}

bool_t rtk_stable(void)
{
	return (rtk_pos_stable() && rtk_head_stable());
}

bool_t rtk_power_up_stable(void)
{
	return (rtk_stable() &&
			(gps_nmea.BESTXYZ.P_type == NARROW_INT) &&
			(gps_nmea.BESTXYZ.V_type == NARROW_INT) &&
			(gps_nmea.GPTRA.sol == 4) &&
			(gps.pos_sv >= RTK_MIN_POS_SV_NUM) &&
			(gps.head_sv >= RTK_MIN_HEADING_SV_NUM));
}
