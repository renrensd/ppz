/*
 * Copyright (C) 2016  Lijie <lij@efficien.cn>
 */

/**
 * @file modules/mag_cali/mag_cali.h
 * @brief Calibration of magelarometer.
 * This module is based on sphere fitting).
 */

#include "mag_cali.h"
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

static void mag_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);
static void mag_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);

static void mag_cali_start(bool_t auto_cali);
static void mag_cali_stop(void);
static bool_t is_mag_cali_done(void);

#define MAG_SENSITIVITY		(12000) // LSB/G
#define NORM_MAG	(0.6f) // Gauss

extern int32_t nav_heading; // from navigation.c

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
			mag_cali.grab_sum[mag_cali.grab_index][0] += (float)imu.mag_unscaled.x/(float)MAG_SENSITIVITY;
			mag_cali.grab_sum[mag_cali.grab_index][1] += (float)imu.mag_unscaled.y/(float)MAG_SENSITIVITY;
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
			mag_cali.state = MAG_CALI_HEADING_ALIGN;
		}
	}
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
	if(!gps_s->h_stable)
	{
		mag_cali_stop();
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
		}
		mag_cali.grab_index = 0;
		mag_cali.grab_index_lock = 1;
		mag_cali.convergence_tick = 0;

		mag_cali.state = MAG_CALI_GRAB;
	}
	else if(mag_cali.state == MAG_CALI_GRAB)
	{
		float gps_heading_rad = gps_s->heading * my_math_deg_to_rad;
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
STATIC_MATRIX_DEF(step, 4, 1);

static float grab_sum[MAG_CALI_GRAB_NUM][2] =
{
//{0.81444, -0.169973},
//{0.734748, -0.421992},
//{0.518295, -0.576011},
//{0.320252, -0.483407},
//{0.163265, -0.22022},
//{0.114414, 0.145734},
//{0.197053, 0.4374},
//{0.365831, 0.572692},
//{0.571578, 0.515232},
//{0.760845, 0.216879}
	{0.787996709, -0.280681372},
	{0.669165194, -0.50187403},
	{0.445261717, -0.559789181},
	{0.248248234, -0.408304095},
	{0.124468334, -0.103591673},
	{0.123394191, 0.268348336},
	{0.259845763, 0.524962544},
	{0.49145335, 0.572828531},
	{0.683733344, 0.379964918},
	{0.766995072, 0.232583329}
};

static float grab_sum2[MAG_CALI_GRAB_NUM][2];


void mag_cali_init(void)
{
	MATRIX_INI(F);
	MATRIX_INI(JT);
	MATRIX_INI(p);
	MATRIX_INI(ev);
	MATRIX_INI(step);

	mag_cali.manual_enable = FALSE;
	mag_cali.manual_enable_prev = FALSE;
	mag_cali.state = MAG_CALI_IDLE;
	mag_cali.need_cali = FALSE; // TODO: according to fram mag cali block
	mag_cali.auto_cali = FALSE;
	mag_cali.cali_ok = FALSE;

	AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
	AbiBindMsgGPS_HEADING(ABI_BROADCAST, &gps_ev, gps_cb);

	MatSetValue(&step, 0.001f);

	//TEST_CASE

	// test data
	float theta,max,min,offset[2],gain[2];
//	for (uint16_t i = 0; i < 10; ++i)
//	{
//		theta = my_math_2pi/(float)MAG_CALI_GRAB_NUM*(float)i;
//		grab_sum[i][0] = cosf(theta) * 1.0f;
//		grab_sum[i][1] = sinf(theta) * 1.0f;
//
//		grab_sum[i][0] *= 1.0f/0.1f;
//		grab_sum[i][1] *= 1.0f/0.2f;
//		grab_sum[i][0] += 2;
//		grab_sum[i][1] += 3;
//	}

	// ini guess of offset and gain
	for (uint8_t i = 0; i < 2; ++i)
	{
		max = grab_sum[0][i];
		min = grab_sum[0][i];
		for (uint8_t j = 0; j < MAG_CALI_GRAB_NUM; ++j)
		{
			if(grab_sum[j][i] > max )
			{
				max = grab_sum[j][i];
			}
			else if(grab_sum[j][i] < min)
			{
				min = grab_sum[j][i];
			}
		}
		offset[i] = (float)(max + min)/2.0f;
	}

	for (uint8_t i = 0; i < 2; ++i)
	{
		min = 0;
		for (uint8_t j = 0; j < MAG_CALI_GRAB_NUM; ++j)
		{
			grab_sum2[j][i] = grab_sum[j][i] - offset[i];
			min += fabsf(grab_sum2[j][i]);
		}
		gain[i] = min/(float)MAG_CALI_GRAB_NUM;
	}

	for (uint8_t i = 0; i < 2; ++i)
	{
		max = fabsf(grab_sum2[0][i]);
		for (uint8_t j = 0; j < MAG_CALI_GRAB_NUM; ++j)
		{
			if(fabsf(grab_sum2[j][i]) > max )
			{
				max = fabsf(grab_sum2[j][i]);
			}
		}
		gain[i] = 1.0f/max;
	}

	//sensors.mag_gain[0] = 1.0f;
	//sensors.mag_gain[1] = (float)(sensors.mag_max[0] - sensors.mag_min[0])/(float)(sensors.mag_max[1] - sensors.mag_min[1]);

	p.data[0] = gain[0];
	p.data[1] = gain[1];
	p.data[2] = offset[0];
	p.data[3] = offset[1];

	for (uint16_t i = 0; i < 200; ++i)
	{
		mag_cali_calc_F(&F, &p, grab_sum);
		mag_cali_calc_JT(&JT, &p, grab_sum);
		MatMult(&ev, &JT, &F);
		VectMult(&ev, 0, &ev, 0, &step, 0);
		MatSub(&p, &p, &ev);
	}

	mag_cali.gain[0] = fabsf(p.data[0]);
	mag_cali.gain[1] = fabsf(p.data[1]);
	mag_cali.offset[0] = p.data[2];
	mag_cali.offset[1] = p.data[3];

	imu.mag_neutral.x = mag_cali.offset[0]*(float)MAG_SENSITIVITY;
	imu.mag_neutral.y = mag_cali.offset[1]*(float)MAG_SENSITIVITY;
	imu.mag_neutral.z = 0;
	imu.mag_sens.x = mag_cali.gain[0];
	imu.mag_sens.y = mag_cali.gain[1];
	imu.mag_sens.z = 0;
	mag_cali.cali_ok = TRUE;
}

#define HEADING_ROTATE_DELTA	((ANGLE_BFP_OF_REAL(RadOfDeg(30.)) / MAG_CALI_PERIODIC_FREQ))

static void mag_cali_heading_rotate(bool_t dir)
{
	if(dir)
	{
		nav_heading += HEADING_ROTATE_DELTA;
	}
	else
	{
		nav_heading -= HEADING_ROTATE_DELTA;
	}
	INT32_COURSE_NORMALIZE(nav_heading);
}

void mag_cali_imu_scale(struct Imu *_imu)
{
	_imu->mag_real.x = (float)(_imu->mag_unscaled.x - _imu->mag_neutral.x) * _imu->mag_sens.x / (float)MAG_SENSITIVITY;
	_imu->mag_real.y = (float)(_imu->mag_unscaled.y - _imu->mag_neutral.y) * _imu->mag_sens.y / (float)MAG_SENSITIVITY;
	_imu->mag_real.z = (float)(_imu->mag_unscaled.z - _imu->mag_neutral.z) * _imu->mag_sens.z / (float)MAG_SENSITIVITY;
	MAGS_BFP_OF_REAL(_imu->mag, _imu->mag_real);
	VECT3_COPY(_imu->mag_scaled, _imu->mag);
}

void mag_cali_periodic(void)
{
	if(mag_cali.manual_enable_prev != mag_cali.manual_enable)
	{
		if(mag_cali.manual_enable == TRUE)
		{
			mag_cali_start(FALSE);
		}
		else
		{
			mag_cali_stop();
		}
	}
	mag_cali.manual_enable_prev = mag_cali.manual_enable;

	if(mag_cali.state == MAG_CALI_GRAB)
	{
		if(mag_cali.auto_cali)
		{
			mag_cali_heading_rotate(TRUE);
		}
	}
	else if(mag_cali.state == MAG_CALI_HEADING_ALIGN)
	{
		if(mag_cali.auto_cali)
		{
			int32_t heading_err = mag_cali.nav_heading_ini - nav_heading;
			INT32_COURSE_NORMALIZE(heading_err);
			if(heading_err > 2*HEADING_ROTATE_DELTA)
			{
				mag_cali_heading_rotate((heading_err <= (ANGLE_BFP_OF_REAL(RadOfDeg(180.)))));
			}
			else
			{
				nav_heading = mag_cali.nav_heading_ini;
				mag_cali.state = MAG_CALI_CALC1;
			}
		}
		else
		{
			mag_cali.state = MAG_CALI_CALC1;
		}
	}
	else if(mag_cali.state == MAG_CALI_CALC1)
	{
		for (uint8_t i = 0; i < MAG_CALI_GRAB_NUM; ++i)
		{
			if(mag_cali.grab_tick[i] == 0)
			{
				mag_cali_stop();
				return;
			}
			mag_cali.grab_sum[i][0] /= (float)mag_cali.grab_tick[i];
			mag_cali.grab_sum[i][1] /= (float)mag_cali.grab_tick[i];
		}

		// ini guess of offset and gain
		float min,max,gain[2],offset[2];

		for (uint8_t i = 0; i < 2; ++i)
		{
			max = mag_cali.grab_sum[0][i];
			min = mag_cali.grab_sum[0][i];
			for (uint8_t j = 0; j < MAG_CALI_GRAB_NUM; ++j)
			{
				if (mag_cali.grab_sum[j][i] > max)
				{
					max = mag_cali.grab_sum[j][i];
				}
				else if (mag_cali.grab_sum[j][i] < min)
				{
					min = mag_cali.grab_sum[j][i];
				}
			}
			offset[i] = (float) (max + min) / 2.0f;
		}

		for (uint8_t i = 0; i < 2; ++i)
		{
			max = fabsf(mag_cali.grab_sum[0][i] - offset[i]);
			for (uint8_t j = 0; j < MAG_CALI_GRAB_NUM; ++j)
			{
				float abs_value = fabsf(mag_cali.grab_sum[j][i] - offset[i]);
				if (abs_value > max)
				{
					max = abs_value;
				}
			}
			if(max < 0.1f)
			{
				mag_cali_stop();
				return;
			}
			gain[i] = 1.0f / max;
		}

		p.data[0] = gain[0];
		p.data[1] = gain[1];
		p.data[2] = offset[0];
		p.data[3] = offset[1];

		mag_cali.state = MAG_CALI_CALC2;
	}
	else if (mag_cali.state == MAG_CALI_CALC2)
	{
		mag_cali_calc_F(&F, &p, mag_cali.grab_sum);
		mag_cali_calc_JT(&JT, &p, mag_cali.grab_sum);
		MatMult(&ev, &JT, &F);
		VectMult(&ev, 0, &ev, 0, &step, 0);
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
		float err = 0;
		if((mag_cali.gain[0] < 0.1f) || (mag_cali.gain[0] > 10.0f))
		{
			err++;
		}
		if ((mag_cali.gain[1] < 0.1f) || (mag_cali.gain[1] > 10.0f))
		{
			err++;
		}
		if ((mag_cali.offset[0] < -0.5f) || (mag_cali.offset[0] > 0.5f))
		{
			err++;
		}
		if ((mag_cali.offset[1] < -0.5f) || (mag_cali.offset[1] > 0.5f))
		{
			err++;
		}
		if(err)
		{
			mag_cali_stop();
			return;
		}

		// ok
		imu.mag_neutral.x = mag_cali.offset[0]*(float)MAG_SENSITIVITY;
		imu.mag_neutral.y = mag_cali.offset[1]*(float)MAG_SENSITIVITY;
		imu.mag_neutral.z = 0;
		imu.mag_sens.x = mag_cali.gain[0];
		imu.mag_sens.y = mag_cali.gain[1];
		imu.mag_sens.z = 0;
		mag_cali.cali_ok = TRUE;

		mag_cali_stop();
	}

float grab_x[MAG_CALI_GRAB_NUM];
float grab_y[MAG_CALI_GRAB_NUM];

for(int i=0;i<MAG_CALI_GRAB_NUM;++i)
{
	grab_x[i] = mag_cali.grab_sum[i][0];
	grab_y[i] = mag_cali.grab_sum[i][1];
}

#if PERIODIC_TELEMETRY
		RunOnceEvery(25,   {
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_MAG_CALI(DefaultChannel, DefaultDevice,
				&mag_cali.state,
				&mag_cali.cali_ok,
				&mag_cali.grab_index,
				MAG_CALI_GRAB_NUM,
				mag_cali.grab_tick,
				&mag_cali.gain[0],
				&mag_cali.gain[1],
				&mag_cali.offset[0],
				&mag_cali.offset[1],
				MAG_CALI_GRAB_NUM,
				grab_x,
				MAG_CALI_GRAB_NUM,
				grab_y);}   );
#endif
}

void mag_cali_event(void)
{

}

static void mag_cali_start(bool_t auto_cali)
{
	if( mag_cali.state == MAG_CALI_IDLE )
	{
		mag_cali.auto_cali = auto_cali;
		mag_cali.state = MAG_CALI_INI;
	}
}
static void mag_cali_stop(void)
{
	mag_cali.state = MAG_CALI_IDLE;
	mag_cali.auto_cali = FALSE;
}
static bool_t is_mag_cali_done(void)
{
	return (mag_cali.state == MAG_CALI_IDLE);
}

bool_t mag_cali_nav_loop(bool_t run)
{
	bool_t to_next_step = FALSE;
	static bool_t run_prev = FALSE;

	if(!run)
	{
		mag_cali_stop();
		return TRUE;
	}

	if(mag_cali.need_cali)
	{
		if ((!run_prev) && run)
		{
			mag_cali_start(TRUE);
			mag_cali.nav_heading_ini = nav_heading;
		}
		run_prev = run;

		if(is_mag_cali_done())
		{
			to_next_step = TRUE;
			run_prev = FALSE;
		}
		else
		{
			to_next_step = FALSE;
		}
	}
	else
	{
		to_next_step = TRUE;
		run_prev = FALSE;
	}

	return to_next_step;
}

static void mag_cali_calc_F(struct _s_matrix *_F, struct _s_matrix *_p, float _v[MAG_CALI_GRAB_NUM][2])
{
	uint8_t i;
	float kx = _p->data[0];
	float ky = _p->data[1];
	float bx = _p->data[2];
	float by = _p->data[3];
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
		vx = _v[i][0];
		vy = _v[i][1];
		vx2 = vx * vx;
		vy2 = vy * vy;

		_F->data[i] =  kx2*vx2 - 2.0f*kx2*vx*bx + kx2*bx2 +
									ky2*vy2 - 2.0f*ky2*vy*by + ky2*by2 - 1.0f;
	}
}

static void mag_cali_calc_JT(struct _s_matrix *_JT, struct _s_matrix *_p, float _v[MAG_CALI_GRAB_NUM][2])
{
	uint8_t i;
	float kx = _p->data[0];
	float ky = _p->data[1];
	float bx = _p->data[2];
	float by = _p->data[3];
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
		vx = _v[i][0];
		vy = _v[i][1];
		vx2 = vx * vx;
		vy2 = vy * vy;

		_JT->data[i*4+0] = 2.0f*kx*vx2 - 4.0f*kx*vx*bx + 2.0f*kx*bx2;
		_JT->data[i*4+1] = 2.0f*ky*vy2 - 4.0f*ky*vy*by + 2.0f*ky*by2;
		_JT->data[i*4+2] = -2.0f*kx2*vx + 2.0f*kx2*bx;
		_JT->data[i*4+3] = -2.0f*ky2*vy + 2.0f*ky2*by;
	}
}
