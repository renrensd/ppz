/*
 * Copyright (C) 2016  Lijie <lij@efficien.cn>
 */

/**
 * @file modules/acc_cali/acc_cali.c
 * @brief Calibration of accelarometer.
 * This module is based on sphere fitting).
 */

#include "modules/acc_cali/acc_cali.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "my_math.h"
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif
#include "subsystems/datalink/downlink.h"

struct AccCali acc_cali;

static abi_event accel_ev;

static void sensors_acc_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[6][3]);
static void sensors_acc_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[6][3]);

static void acc_cali_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
#define ACC_CALI_CHECK_MIN	(0.6f)
#define ACC_CALI_CHECK_MAX	(1.4f)
#define ACC_NORM_FILTER_COEF	(2.0f * my_math_pi * 0.5f / (float)PERIODIC_FREQUENCY)
#define ACC_STATIC_TIME_THRESHOLD		((uint32_t)(0.1f*(float)PERIODIC_FREQUENCY))
#define ACC_STATIC_VALUE_THRESHOLD	(0.05f)
#define ACC_CALI_GRAB_TIME	(PERIODIC_FREQUENCY)
#define IS_WITHIN_OPEN_RANGE(value,min,max)	( (((value) < (max)) && ((value) > (min))) ? 1:0 )
#define IMU_ACCEL_SENS_SCALE_FACTOR (4096)

	uint16_t i,j;
	uint8_t grab_enable;
	MATRIX_DEF(F, 6, 1);
	MATRIX_DEF(JT, 6, 6);
	MATRIX_DEF(p, 6, 1);
	MATRIX_DEF(ev, 6, 1);

	MATRIX_INI(F);
	MATRIX_INI(JT);
	MATRIX_INI(p);
	MATRIX_INI(ev);

	if( acc_cali.state == ACC_CALI_IDLE )
	{
		return;
	}

	acc_cali.acc_phy_nc[0] = (float)imu.accel_unscaled.x/(float)IMU_ACCEL_SENS_SCALE_FACTOR;
	acc_cali.acc_phy_nc[1] = (float)imu.accel_unscaled.y/(float)IMU_ACCEL_SENS_SCALE_FACTOR;
	acc_cali.acc_phy_nc[2] = (float)imu.accel_unscaled.z/(float)IMU_ACCEL_SENS_SCALE_FACTOR;

	// acc static filter
	acc_cali.acc_norm = sqrtf(acc_cali.acc_phy_nc[0]*acc_cali.acc_phy_nc[0] +
														acc_cali.acc_phy_nc[1]*acc_cali.acc_phy_nc[1] +
														acc_cali.acc_phy_nc[2]*acc_cali.acc_phy_nc[2]);
	acc_cali.acc_norm_filter = acc_cali.acc_norm_filter +
														(acc_cali.acc_norm - acc_cali.acc_norm_filter) * ACC_NORM_FILTER_COEF;

	if( fabsf(acc_cali.acc_norm - acc_cali.acc_norm_filter) < ACC_STATIC_VALUE_THRESHOLD )
	{
		if( ++acc_cali.check_body_static_tick >= ACC_STATIC_TIME_THRESHOLD )
		{
			acc_cali.check_body_static_tick = 0;
			acc_cali.is_body_static = TRUE;
		}
	}
	else
	{
		acc_cali.check_body_static_tick = 0;
		acc_cali.is_body_static = FALSE;
	}

	if( acc_cali.state == ACC_CALI_INI )
		{
			for(i=0;i<6;++i)
			{
				for(j=0;j<3;++j)
				{
					acc_cali.acc_6point[i][j] = 0;
				}
			}

			acc_cali.acc_gain[0] = 1;
			acc_cali.acc_gain[1] = 1;
			acc_cali.acc_gain[2] = 1;
			acc_cali.acc_offset[0] = 0;
			acc_cali.acc_offset[1] = 0;
			acc_cali.acc_offset[2] = 0;

			acc_cali.acc_cali_tick = 0;
			acc_cali.state = ACC_CALI_GRAB_PX;
		}
		else if( acc_cali.state == ACC_CALI_CALC )
		{
			for(i=0;i<6;++i)
			{
				for(j=0;j<3;++j)
				{
					acc_cali.acc_6point[i][j] /= (float)ACC_CALI_GRAB_TIME;
				}
			}

			p.data[0] = 1.1f;
			p.data[1] = 1.1f;
			p.data[2] = 1.1f;
			p.data[3] = 0.2f;
			p.data[4] = 0.2f;
			p.data[5] = 0.2f;

			for(i=0;i<100;++i)
			{
				sensors_acc_cali_calc_F(&F, &p, acc_cali.acc_6point);
				sensors_acc_cali_calc_JT(&JT, &p, acc_cali.acc_6point);
				MatMult(&ev, &JT, &F);
				MatScale(&ev, &ev, 0.1f);
				MatSub(&p, &p, &ev);
			}

			acc_cali.acc_gain[0] = fabsf(p.data[0]);
			acc_cali.acc_gain[1] = fabsf(p.data[1]);
			acc_cali.acc_gain[2] = fabsf(p.data[2]);
			acc_cali.acc_offset[0] = p.data[3];
			acc_cali.acc_offset[1] = p.data[4];
			acc_cali.acc_offset[2] = p.data[5];

#define NEUTRAL_COEF ((float)IMU_ACCEL_SENS_SCALE_FACTOR)
			acc_cali.acc_NEUTRAL[0] = acc_cali.acc_offset[0] * NEUTRAL_COEF;
			acc_cali.acc_NEUTRAL[1] = acc_cali.acc_offset[1] * NEUTRAL_COEF;
			acc_cali.acc_NEUTRAL[2] = acc_cali.acc_offset[2] * NEUTRAL_COEF;
#define SENS_COEF (9.81f * (float)(1<<INT32_ACCEL_FRAC) / (float)IMU_ACCEL_SENS_SCALE_FACTOR)
			acc_cali.acc_SENS[0] = acc_cali.acc_gain[0] * SENS_COEF;
			acc_cali.acc_SENS[1] = acc_cali.acc_gain[1] * SENS_COEF;
			acc_cali.acc_SENS[2] = acc_cali.acc_gain[2] * SENS_COEF;

			acc_cali.state = ACC_CALI_IDLE;
		}
		else
		{
			grab_enable = 0;
			if( acc_cali.is_body_static )
			{
				if(acc_cali.state == ACC_CALI_GRAB_PX)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[0], ACC_CALI_CHECK_MIN, ACC_CALI_CHECK_MAX ) )
					{
						grab_enable = 1;
					}
				}
				else if(acc_cali.state == ACC_CALI_GRAB_NX)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[0], -ACC_CALI_CHECK_MAX, -ACC_CALI_CHECK_MIN ) )
					{
						grab_enable = 1;
					}
				}
				else if(acc_cali.state == ACC_CALI_GRAB_PY)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[1], ACC_CALI_CHECK_MIN, ACC_CALI_CHECK_MAX ) )
					{
						grab_enable = 1;
					}
				}
				else if(acc_cali.state == ACC_CALI_GRAB_NY)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[1], -ACC_CALI_CHECK_MAX, -ACC_CALI_CHECK_MIN ) )
					{
						grab_enable = 1;
					}
				}
				else if(acc_cali.state == ACC_CALI_GRAB_NZ)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[2], ACC_CALI_CHECK_MIN, ACC_CALI_CHECK_MAX ) )
					{
						grab_enable = 1;
					}
				}
				else if(acc_cali.state == ACC_CALI_GRAB_PZ)
				{
					if( IS_WITHIN_OPEN_RANGE(acc_cali.acc_phy_nc[2], -ACC_CALI_CHECK_MAX, -ACC_CALI_CHECK_MIN ) )
					{
						grab_enable = 1;
					}
				}

				if(grab_enable)
				{
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][0] += acc_cali.acc_phy_nc[0];
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][1] += acc_cali.acc_phy_nc[1];
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][2] += acc_cali.acc_phy_nc[2];

					if(++acc_cali.acc_cali_tick == ACC_CALI_GRAB_TIME)
					{
						acc_cali.acc_cali_tick = 0;
						acc_cali.state++;
					}
				}
				else
				{
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][0] = 0;
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][1] = 0;
					acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][2] = 0;
				}
			}
			else
			{
				acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][0] = 0;
				acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][1] = 0;
				acc_cali.acc_6point[acc_cali.state - ACC_CALI_GRAB_PX][2] = 0;
				acc_cali.acc_cali_tick = 0;
			}
		}
}

void acc_cali_init(void)
{
	acc_cali.acc_gain[0] = 1.002438f;
	acc_cali.acc_gain[1] = 1.002052f;
	acc_cali.acc_gain[2] = 0.990015f;
	acc_cali.acc_offset[0] = 0.038478f;
	acc_cali.acc_offset[1] = 0.095785f;
	acc_cali.acc_offset[2] = 0.12287f;

	AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, acc_cali_cb);
}

void acc_cali_periodic(void)
{
	if(acc_cali.enable_prev != acc_cali.enable)
	{
		if(acc_cali.enable == TRUE)
		{
			sensors_acc_cali_start();
		}
		else
		{
			sensors_acc_cali_stop();
		}
	}
	acc_cali.enable_prev = acc_cali.enable;

#if PERIODIC_TELEMETRY
		RunOnceEvery(2,   {
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_ACC_CALI(DefaultChannel, DefaultDevice, &acc_cali.acc_norm,
																													&acc_cali.acc_norm_filter,
																													&acc_cali.acc_NEUTRAL[0],
																													&acc_cali.acc_NEUTRAL[1],
																													&acc_cali.acc_NEUTRAL[2],
																													&acc_cali.acc_SENS[0],
																													&acc_cali.acc_SENS[1],
																													&acc_cali.acc_SENS[2],
																													&acc_cali.is_body_static,
																													&acc_cali.state);}   );
#endif
}

void acc_cali_event(void)
{

}

void sensors_acc_cali_start(void)
{
	if( acc_cali.state == ACC_CALI_IDLE )
	{
		acc_cali.state = ACC_CALI_INI;
	}
}
void sensors_acc_cali_stop(void)
{
	if( acc_cali.state != ACC_CALI_CALC )
	{
		acc_cali.state = ACC_CALI_IDLE;
	}
}

static void sensors_acc_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[6][3])
{
	uint8_t i;
	float kx = p->data[0];
	float ky = p->data[1];
	float kz = p->data[2];
	float bx = p->data[3];
	float by = p->data[4];
	float bz = p->data[5];
	float kx2 = kx*kx;
	float ky2 = ky*ky;
	float kz2 = kz*kz;
	float bx2 = bx*bx;
	float by2 = by*by;
	float bz2 = bz*bz;
	float vx;
	float vy;
	float vz;
	float vx2;
	float vy2;
	float vz2;

	for(i=0;i<6;++i)
	{
		vx = v[i][0];
		vy = v[i][1];
		vz = v[i][2];
		vx2 = vx * vx;
		vy2 = vy * vy;
		vz2 = vz * vz;

		F->data[i] =  kx2*vx2 - 2.0f*kx2*vx*bx + kx2*bx2 +
									ky2*vy2 - 2.0f*ky2*vy*by + ky2*by2 +
									kz2*vz2 - 2.0f*kz2*vz*bz + kz2*bz2 - 1.0f;
	}
}

static void sensors_acc_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[6][3])
{
	uint8_t i;
	float kx = p->data[0];
	float ky = p->data[1];
	float kz = p->data[2];
	float bx = p->data[3];
	float by = p->data[4];
	float bz = p->data[5];
	float kx2 = kx*kx;
	float ky2 = ky*ky;
	float kz2 = kz*kz;
	float bx2 = bx*bx;
	float by2 = by*by;
	float bz2 = bz*bz;
	float vx;
	float vy;
	float vz;
	float vx2;
	float vy2;
	float vz2;

	for(i=0;i<6;++i)
	{
		vx = v[i][0];
		vy = v[i][1];
		vz = v[i][2];
		vx2 = vx * vx;
		vy2 = vy * vy;
		vz2 = vz * vz;

		JT->data[i*6+0] = 2.0f*kx*vx2 - 4.0f*kx*vx*bx + 2.0f*kx*bx2;
		JT->data[i*6+1] = 2.0f*ky*vy2 - 4.0f*ky*vy*by + 2.0f*ky*by2;
		JT->data[i*6+2] = 2.0f*kz*vz2 - 4.0f*kz*vz*bz + 2.0f*kz*bz2;
		JT->data[i*6+3] = -2.0f*kx2*vx + 2.0f*kx2*bx;
		JT->data[i*6+4] = -2.0f*ky2*vy + 2.0f*ky2*by;
		JT->data[i*6+5] = -2.0f*kz2*vz + 2.0f*kz2*bz;
	}
}

