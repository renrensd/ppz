/*
 * Copyright (C) 2016  Lijie <lij@efficien.cn>
 */

/**
 * @file modules/mag_cali/mag_cali.h
 * @brief Calibration of magelarometer.
 * This module is based on sphere fitting).
 */

#include "mag_cali.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "math/my_math.h"
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif
#include "subsystems/datalink/downlink.h"

struct MagCali mag_cali;
static abi_event mag_ev;
static abi_event gps_ev;

static void sensors_mag_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);
static void sensors_mag_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *mag)
{
	uint8_t ok = 0;

	if(mag_cali.state == MAG_CALI_IDLE)
	{
		return;
	}

	if(mag_cali.state == MAG_CALI_GRAB)
	{
		if(mag_cali.grab_index_lock)
		{
			return;
		}

		if( mag_cali.grab_tick[mag_cali.grab_index] < MAG_CALI_GRAB_COUNT_MAX )
		{
			mag_cali.grab_tick[mag_cali.grab_index]++;
			mag_cali.grab_sum[mag_cali.grab_index][0] += imu.mag_unscaled.x;
			mag_cali.grab_sum[mag_cali.grab_index][1] += imu.mag_unscaled.y;
		}

		for(uint8_t i=0;i<MAG_CALI_GRAB_NUM;++i)
		{
			if( mag_cali.grab_tick[i] > 3 )
			{
				ok++;
			}
		}
		if(ok == MAG_CALI_GRAB_NUM)
		{
			mag_cali.state = MAG_CALI_CALC1;
		}
	}
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps)
{
	if(!gps->h_stable)
	{
		mag_cali.state = MAG_CALI_IDLE;
		return;
	}

	if(mag_cali.state == MAG_CALI_IDLE)
	{
		return;
	}

	if(mag_cali.state == MAG_CALI_INI)
	{
		for(uint8_t i=0;i<MAG_CALI_GRAB_NUM;++i)
		{
			mag_cali.grab_tick[i] = 0;
			mag_cali.grab_sum[i][0] = 0;
			mag_cali.grab_sum[i][1] = 0;
			mag_cali.grab_sum[i][2] = 0;
		}
		mag_cali.grab_index = 0;
		mag_cali.grab_index_lock = 0;
		mag_cali.convergence_tick = 0;

		mag_cali.state = MAG_CALI_GRAB;
	}
	else if(mag_cali.state == MAG_CALI_GRAB)
	{
		float gps_heading_rad = gps->heading * my_math_deg_to_rad;
		gps_heading_rad = LimitAngleTo_0_2pi(gps_heading_rad);
		uint8_t index = gps_heading_rad/(my_math_2pi/(float)MAG_CALI_CIRCLE_SPLITE_NUM);
		if(index > 9)
		{
			index = 0;
		}
		mag_cali.grab_index_lock = 1;
		mag_cali.grab_index = index;
		mag_cali.grab_index_lock = 0;
	}
}

void mag_cali_init(void)
{
	mag_cali.enable = FALSE;
	mag_cali.cali_ok = FALSE;

	AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
	AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}

void mag_cali_periodic(void)
{
	MATRIX_DEF(F, MAG_CALI_GRAB_NUM, 1);
	MATRIX_DEF(JT, 4, MAG_CALI_GRAB_NUM);
	MATRIX_DEF(p, 4, 1);
	MATRIX_DEF(ev, 4, 1);

	MATRIX_INI(F);
	MATRIX_INI(JT);
	MATRIX_INI(p);
	MATRIX_INI(ev);

	if(mag_cali.enable_prev != mag_cali.enable)
	{
		if(mag_cali.enable == TRUE)
		{
			sensors_mag_cali_start();
		}
		else
		{
			sensors_mag_cali_stop();
		}
	}
	mag_cali.enable_prev = mag_cali.enable;

	if(mag_cali.state == MAG_CALI_CALC1)
	{
		for (uint8_t i = 0; i < MAG_CALI_GRAB_NUM; ++i)
		{
			mag_cali.grab_sum[i][0] /= (float)mag_cali.grab_tick[i];
			mag_cali.grab_sum[i][1] /= (float)mag_cali.grab_tick[i];
			mag_cali.grab_sum[i][2] /= (float)mag_cali.grab_tick[i];
		}

		p.data[0] = 1.1f;
		p.data[1] = 1.1f;
		p.data[2] = 0.1f;
		p.data[3] = 0.1f;

		mag_cali.state = MAG_CALI_CALC2;
	}
	else if (mag_cali.state == MAG_CALI_CALC2)
	{
		sensors_mag_cali_calc_F(&F, &p, mag_cali.grab_sum);
		sensors_mag_cali_calc_JT(&JT, &p, mag_cali.grab_sum);
		MatMult(&ev, &JT, &F);
		MatScale(&ev, &ev, 0.1f);
		MatSub(&p, &p, &ev);

		if (mag_cali.convergence_tick++ > MAG_CALI_CONVERGENCE_COUNT)
		{
			mag_cali.state = MAG_CALI_CALC3;
		}
	}
	else if (mag_cali.state == MAG_CALI_CALC3)
	{
		mag_cali.gain[0] = fabsf(p.data[0]);
		mag_cali.gain[1] = fabsf(p.data[1]);
		mag_cali.offset[0] = p.data[2];
		mag_cali.offset[1] = p.data[3];
	}

#if PERIODIC_TELEMETRY
		RunOnceEvery(2,   {
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_MAG_CALI(DefaultChannel, DefaultDevice,
				&mag_cali.mag_norm,
				&mag_cali.mag_norm_filter,
				&mag_cali.mag_NEUTRAL[0],
				&mag_cali.mag_NEUTRAL[1],
				&mag_cali.mag_NEUTRAL[2],
				&mag_cali.mag_SENS[0],
				&mag_cali.mag_SENS[1],
				&mag_cali.mag_SENS[2],
				&mag_cali.is_body_static,
				&mag_cali.state);}   );
#endif
}

void mag_cali_event(void)
{

}

void sensors_mag_cali_start(void)
{
	if( mag_cali.state == MAG_CALI_IDLE )
	{
		mag_cali.state = MAG_CALI_INI;
	}
}
void sensors_mag_cali_stop(void)
{
	if( mag_cali.state != MAG_CALI_CALC1 )
	{
		mag_cali.state = MAG_CALI_IDLE;
	}
}

static void sensors_mag_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2])
{
	uint8_t i;
	float kx = p->data[0];
	float ky = p->data[1];
	float bx = p->data[2];
	float by = p->data[3];
	float kx2 = kx*kx;
	float ky2 = ky*ky;
	float bx2 = bx*bx;
	float by2 = by*by;
	float vx;
	float vy;
	float vz;
	float vx2;
	float vy2;
	float vz2;

	for(i=0;i<MAG_CALI_GRAB_NUM;++i)
	{
		vx = v[i][0];
		vy = v[i][1];
		vx2 = vx * vx;
		vy2 = vy * vy;

		F->data[i] =  kx2*vx2 - 2.0f*kx2*vx*bx + kx2*bx2 +
									ky2*vy2 - 2.0f*ky2*vy*by + ky2*by2 - 1.0f;
	}
}

static void sensors_mag_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2])
{
	uint8_t i;
	float kx = p->data[0];
	float ky = p->data[1];
	float bx = p->data[2];
	float by = p->data[3];
	float kx2 = kx*kx;
	float ky2 = ky*ky;
	float bx2 = bx*bx;
	float by2 = by*by;
	float vx;
	float vy;
	float vx2;
	float vy2;

	for(i=0;i<MAG_CALI_GRAB_NUM;++i)
	{
		vx = v[i][0];
		vy = v[i][1];
		vx2 = vx * vx;
		vy2 = vy * vy;

		JT->data[i*MAG_CALI_GRAB_NUM+0] = 2.0f*kx*vx2 - 4.0f*kx*vx*bx + 2.0f*kx*bx2;
		JT->data[i*MAG_CALI_GRAB_NUM+1] = 2.0f*ky*vy2 - 4.0f*ky*vy*by + 2.0f*ky*by2;
		JT->data[i*MAG_CALI_GRAB_NUM+2] = -2.0f*kx2*vx + 2.0f*kx2*bx;
		JT->data[i*MAG_CALI_GRAB_NUM+3] = -2.0f*ky2*vy + 2.0f*ky2*by;
	}
}
