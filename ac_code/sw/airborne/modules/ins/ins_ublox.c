/*
 * ins_ublox.c
 *
 *  Created on: Dec 20, 2016
 *      Author: jay
 */

#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "modules/gps/gps2_ublox.h"
#include "ins_ublox.h"

struct _s_ins_ublox ins_ublox;
abi_event ublox_ev;

#ifdef PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins_ublox(struct transport_tx *trans, struct link_device *dev)
{
	xbee_tx_header(XBEE_NACK, XBEE_ADDR_PC);
	pprz_msg_send_INS_UBLOX(trans, dev, AC_ID,
			&ins_ublox.ublox_stable,
			&ins_ublox.ned_pos.x,
			&ins_ublox.ned_pos.y,
			&ins_ublox.ned_pos.z,
			&ins_ublox.ublox_ned_vel.x,
			&ins_ublox.ublox_ned_vel.y,
			&ins_ublox.ublox_ned_vel.z,
			&ins_ublox.ned_vel.x,
			&ins_ublox.ned_vel.y,
			&ins_ublox.ned_vel.z);
}
#endif

static void ublox_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
	static uint32_t last_stamp = 0;
	if (last_stamp > 0)
	{
		float dt = (float) (stamp - last_stamp) * 1e-6;

		ins_ublox.ublox_signal_stable = ((gps_s->fix >= GPS_FIX_3D) && (gps_s->num_sv > 5));
		ins_ublox.ublox_update = TRUE;

		// get ecef pos (cm) and scale to unit(m)
		VECT3_COPY(ins_ublox.ublox_ecef_pos, gps_s->ecef_pos);
		VECT3_SDIV(ins_ublox.ublox_ecef_pos, ins_ublox.ublox_ecef_pos, 100.0f);

		// get ned vel (cm/s) and scale to unit(m/s)
		VECT3_COPY(ins_ublox.ublox_ned_vel, gps_s->ned_vel);
		VECT3_SDIV(ins_ublox.ublox_ned_vel, ins_ublox.ublox_ned_vel, 100.0f);

		if(!ins_ublox.ltpDef_initialized)
		{
			if(ins_ublox.ublox_stable)
			{
				ltp_def_from_ecef_f(&ins_ublox.ltpDef, &ins_ublox.ublox_ecef_pos);
				ins_ublox.ltpDef_initialized = TRUE;
			}
		}
		else
		{
			ned_of_ecef_point_f(&ins_ublox.ned_pos, &ins_ublox.ltpDef, &ins_ublox.ublox_ecef_pos);
		}

		// estimate ned vel from ned pos diff
		if ((dt < 0.5f) && (dt > 0.05f))
		{
			update_sgdf_dT(&ins_ublox.vel_x_sgdf, dt, ins_ublox.ned_pos.x);
			update_sgdf_dT(&ins_ublox.vel_y_sgdf, dt, ins_ublox.ned_pos.y);
			update_sgdf_dT(&ins_ublox.vel_z_sgdf, dt, ins_ublox.ned_pos.z);

			//VECT3_DIFF(ins_ublox.ned_vel ,ins_ublox.ned_pos, ins_ublox.ned_pos_last);
			//VECT3_SDIV(ins_ublox.ned_vel, ins_ublox.ned_vel, dt);
		}
		ins_ublox.ned_pos_last = ins_ublox.ned_pos;
	}
	last_stamp = stamp;
}

void ins_ublox_init(void)
{
	AbiBindMsgGPS_UBX(ABI_BROADCAST, &ublox_ev, ublox_cb);
#ifdef PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_UBLOX, send_ins_ublox);
#endif

  INIT_SGDF(ins_ublox.vel_x_sgdf, NED_VEL_SGDF_WIN_SIZE, GPS2_UBLOX_UPDATE_FERQ);
  INIT_SGDF(ins_ublox.vel_y_sgdf, NED_VEL_SGDF_WIN_SIZE, GPS2_UBLOX_UPDATE_FERQ);
  INIT_SGDF(ins_ublox.vel_z_sgdf, NED_VEL_SGDF_WIN_SIZE, GPS2_UBLOX_UPDATE_FERQ);

  ins_ublox.ltpDef_initialized = FALSE;
  ins_ublox.ublox_stable = FALSE;
  ins_ublox.ublox_stable_first_time = TRUE;
}

void ins_ublox_event(void)
{

}

void ins_ublox_periodic(void)
{
	uint16_t stable_tick;

	if(!ins_ublox.ublox_update)
	{
		if(++ins_ublox.ublox_update_lost_count > INS_UBLOX_PERIODIC_FREQ)
		{
			ins_ublox.ublox_stable = FALSE;
			ins_ublox.ublox_update_lost_count = 0;
		}
	}
	else
	{
		ins_ublox.ublox_update_lost_count = 0;
		if(ins_ublox.ublox_signal_stable)
		{
			if(ins_ublox.ublox_stable_first_time)
			{
				stable_tick = 10 * 5;
			}
			else
			{
				stable_tick = 5;
			}

			if(++ins_ublox.ublox_stable_count > stable_tick)
			{
				ins_ublox.ublox_stable = TRUE;
				if (ins_ublox.ublox_stable_first_time)
				{
					ins_ublox.ublox_stable_first_time = FALSE;
				}
				ins_ublox.ublox_stable_count = 0;
			}
		}
		else
		{
			ins_ublox.ublox_stable = FALSE;
			ins_ublox.ublox_stable_count = 0;
		}
	}
	ins_ublox.ublox_update = FALSE;
}

