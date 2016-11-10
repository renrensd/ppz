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
			//TODO : scale unscaled mag to gauss unit
			mag_cali.grab_sum[mag_cali.grab_index][0] += (float)imu.mag_unscaled.x/(float)5000.0f;
			mag_cali.grab_sum[mag_cali.grab_index][1] += (float)imu.mag_unscaled.y/(float)5000.0f;
		}

		for(uint8_t i=0;i<MAG_CALI_GRAB_NUM;++i)
		{
			if( mag_cali.grab_tick[i] > 50 )
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
		mag_cali.grab_index_lock = 1;
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

STATIC_MATRIX_DEF(F, MAG_CALI_GRAB_NUM, 1);
STATIC_MATRIX_DEF(JT, 4, MAG_CALI_GRAB_NUM);
STATIC_MATRIX_DEF(p, 4, 1);
STATIC_MATRIX_DEF(ev, 4, 1);

static float grab_sum[MAG_CALI_GRAB_NUM][2] =
{
{0.992084324, -0.118157975},
{0.693150163, -0.672581613},
{0.116317995, -0.972254038},
{-0.461079985, -0.80696398},
{-0.831259012, -0.418678433},
{-0.905969441, 0.140402019},
{-0.596551895, 0.802987754},
{-0.108388022, 1.01896238},
{0.534478009, 0.890410185},
{0.927089989, 0.464597851}
};

void mag_cali_init(void)
{
	MATRIX_INI(F);
	MATRIX_INI(JT);
	MATRIX_INI(p);
	MATRIX_INI(ev);

	mag_cali.enable = FALSE;
	mag_cali.cali_ok = FALSE;

	AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
	AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

	//TEST_CASE
	p.data[0] = 1.1f;
	p.data[1] = 1.1f;
	p.data[2] = 0.1f;
	p.data[3] = 0.1f;

	for (uint16_t i = 0; i < 10; ++i)
	{
		grab_sum[i][0] *= 10;
		grab_sum[i][1] *= 10;
	}
	for (uint16_t i = 0; i < 1000; ++i)
	{
		sensors_mag_cali_calc_F(&F, &p, grab_sum);
		sensors_mag_cali_calc_JT(&JT, &p, grab_sum);
		MatMult(&ev, &JT, &F);
		MatScale(&ev, &ev, 0.00001f);
		MatSub(&p, &p, &ev);
	}
}

void mag_cali_periodic(void)
{
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
			if(mag_cali.grab_tick[i] == 0)
			{
				mag_cali.state = MAG_CALI_IDLE;
				return;
			}
			mag_cali.grab_sum[i][0] /= (float)mag_cali.grab_tick[i];
			mag_cali.grab_sum[i][1] /= (float)mag_cali.grab_tick[i];
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
		MatScale(&ev, &ev, 0.01f);
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

		// range check
		mag_cali.cali_ok = TRUE;
		mag_cali.state = MAG_CALI_IDLE;
	}

#if PERIODIC_TELEMETRY
		RunOnceEvery(2,   {
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_MAG_CALI(DefaultChannel, DefaultDevice,
				&mag_cali.state,
				&mag_cali.cali_ok,
				&mag_cali.grab_index,
				MAG_CALI_GRAB_NUM, &mag_cali.grab_tick,
				&mag_cali.gain[0],
				&mag_cali.gain[1],
				&mag_cali.offset[0],
				&mag_cali.offset[1]);}   );
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

		JT->data[i*4+0] = 2.0f*kx*vx2 - 4.0f*kx*vx*bx + 2.0f*kx*bx2;
		JT->data[i*4+1] = 2.0f*ky*vy2 - 4.0f*ky*vy*by + 2.0f*ky*by2;
		JT->data[i*4+2] = -2.0f*kx2*vx + 2.0f*kx2*bx;
		JT->data[i*4+3] = -2.0f*ky2*vy + 2.0f*ky2*by;
	}
}
