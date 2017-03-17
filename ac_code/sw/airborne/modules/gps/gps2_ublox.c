/*
 * gps_ublox.c
 *
 *  Created on: Dec 19, 2016
 *      Author: jay
 */

#include "mcu_periph/link_device.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "gps2_ublox.h"

static void gps2_ublox_update(struct _s_ubx_parser *parser, struct GpsState *gps_s);

struct _s_gps2_ublox gps2_ublox;
struct GpsState gps2;

static bool_t update = FALSE;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gps2_ublox(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK, XBEE_ADDR_PC);
	pprz_msg_send_GPS2_UBLOX(trans, dev, AC_ID,
			&gps2.tow,
			&gps2.num_sv,
			&gps2.fix,
			&gps2.ecef_pos.x,
			&gps2.ecef_pos.y,
			&gps2.ecef_pos.z,
			&gps2.pacc,
			&gps2.pdop,
			&gps2.ecef_vel.x,
			&gps2.ecef_vel.y,
			&gps2.ecef_vel.z,
			&gps2.sacc,
			&gps2.ned_vel.x,
			&gps2.ned_vel.y,
			&gps2.ned_vel.z,
			&gps2.course,
			&gps2.cacc);
}
#endif

void gps2_ublox_init(void)
{
	gps2_ublox.dev = &((GPS2_UBLOX_LINK).device);
	UBX_parser_init(&gps2_ublox.parser);
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS2_UBLOX, send_gps2_ublox);
#endif
}

void gps2_ublox_event(void)
{
	while (gps2_ublox.dev->char_available(gps2_ublox.dev->periph))
	{
		UBX_message_parse(&gps2_ublox.parser ,
				gps2_ublox.dev->get_byte(gps2_ublox.dev->periph));
	}

	gps2_ublox_update(&gps2_ublox.parser, &gps2);
}

void gps2_ublox_periodic(void)
{

}

bool_t gps2_ublox_check_update(void)
{
	if (update)
	{
		update = FALSE;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

static void gps2_ublox_update(struct _s_ubx_parser *parser, struct GpsState *gps_s)
{
	if (parser->POSLLH_available)
	{
		parser->POSLLH_available = FALSE;
	}
	if (parser->SOL_available)
	{
		gps_time_sync.t0_ticks = sys_time.nb_tick;
		gps_time_sync.t0_tow =  parser->SOL.iTOW;
		gps_time_sync.t0_tow_frac = parser->SOL.fTOW;
		gps_s->tow = parser->SOL.iTOW;
		gps_s->week = parser->SOL.week;
		gps_s->fix = parser->SOL.gpsFix;
		gps_s->ecef_pos.x = parser->SOL.ecefX;
		gps_s->ecef_pos.y = parser->SOL.ecefY;
		gps_s->ecef_pos.z = parser->SOL.ecefZ;
		gps_s->pacc = parser->SOL.pAcc;
		gps_s->ecef_vel.x = parser->SOL.ecefVX;
		gps_s->ecef_vel.y = parser->SOL.ecefVY;
		gps_s->ecef_vel.z = parser->SOL.ecefVZ;
		gps_s->sacc = parser->SOL.sAcc;
		gps_s->pdop = parser->SOL.pDOP;
		gps_s->num_sv = parser->SOL.numSV;

		parser->SOL_available = FALSE;

		update = TRUE;
	}
	if (parser->VELNED_available)
	{
		gps_s->speed_3d = parser->VELNED.speed;
		gps_s->gspeed = parser->VELNED.gSpeed;
		gps_s->ned_vel.x = parser->VELNED.velN;
		gps_s->ned_vel.y = parser->VELNED.velE;
		gps_s->ned_vel.z = parser->VELNED.velD;
		// Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
		// I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
		// solution: First to radians, and then scale to 1e-7 radians
		// First x 10 for loosing less resolution, then to radians, then multiply x 10 again
		gps_s->course = (RadOfDeg(parser->VELNED.heading * 10)) * 10;
		gps_s->cacc = (RadOfDeg(parser->VELNED.cAcc * 10)) * 10;
		gps_s->tow = parser->VELNED.iTOW;
		gps2_ublox.have_velned = 1;

		parser->VELNED_available = FALSE;
	}
	if (parser->TIMEUTC_available)
	{
		parser->TIMEUTC_available = FALSE;
	}
}
