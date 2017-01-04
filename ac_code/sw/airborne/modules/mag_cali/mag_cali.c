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
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/fram/fram_if.h"
#include "data_check/crc16.h"
#include "subsystems/ahrs/ahrs_float_mlkf.h"
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif
#include "subsystems/datalink/downlink.h"

struct MagCali mag_cali;
static abi_event mag_ev;
static abi_event gps_ev;

STATIC_MATRIX_DEF(F, MAG_CALI_GRAB_NUM, 1);
STATIC_MATRIX_DEF(JT, 4, MAG_CALI_GRAB_NUM);
STATIC_MATRIX_DEF(p, 4, 1);
STATIC_MATRIX_DEF(ev, 4, 1);
STATIC_MATRIX_DEF(step, 4, 1);

static void mag_cali_calc_F(struct _s_matrix *F, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);
static void mag_cali_calc_JT(struct _s_matrix *JT, struct _s_matrix *p, float v[MAG_CALI_GRAB_NUM][2]);

static void mag_cali_start(bool_t auto_cali);
static void mag_cali_stop(bool_t ok);
static bool_t is_mag_cali_done(void);
static bool_t mag_cali_load_to_imu(void);

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

		if( mag_cali.grab_tick[mag_cali.grab_index] == 0 )
		{
			mag_cali.grab_heading_pair[mag_cali.grab_index][0] = mag_cali.gps_heading;
			mag_cali.grab_heading_pair[mag_cali.grab_index][1] = (float)imu.mag_unscaled.x/(float)MAG_SENSITIVITY;
			mag_cali.grab_heading_pair[mag_cali.grab_index][2] = (float)imu.mag_unscaled.y/(float)MAG_SENSITIVITY;
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
	mag_cali.gps_heading = gps_s->heading * my_math_deg_to_rad;
	mag_cali.gps_heading = LimitAngleTo_0_2pi(mag_cali.gps_heading);

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
		uint8_t index = mag_cali.gps_heading/(my_math_2pi/(float)MAG_CALI_CIRCLE_SPLITE_NUM);
		if(index > 9)
		{
			index = 0;
		}
		mag_cali.grab_index_lock = 1;
		mag_cali.grab_index = index;
		mag_cali.grab_index_lock = 0;
	}
}

static void mag_cali_persistent_read(void)
{
	struct MagCaliPersData temp_MagCaliFramData;
	uint8_t *ptemp = (uint8_t *) (&temp_MagCaliFramData);

	uint8_t err = fram_mag_cali_data_read(ptemp);

	if (err)
	{
		return;
	}

	uint32_t temp_crc16 = Crc16_normal(ptemp, 0, MAG_CALI_PERS_DATA_STRUCT_LENGTH - 4);
	if(temp_crc16 != temp_MagCaliFramData.crc16)
	{
		return;
	}

	mag_cali.gain[0] = temp_MagCaliFramData.gain[0];
	mag_cali.gain[1] = temp_MagCaliFramData.gain[1];
	mag_cali.offset[0] = temp_MagCaliFramData.offset[0];
	mag_cali.offset[1] = temp_MagCaliFramData.offset[1];
	mag_cali.mag_declination = temp_MagCaliFramData.mag_declination;
	mag_cali.cali_ok = temp_MagCaliFramData.cali_ok;


	struct NedCoor_i ned_last_cali_pos_i;
	ned_of_ecef_point_i(&ned_last_cali_pos_i, &state.ned_origin_i, &temp_MagCaliFramData.cali_ecef_pos_i);
	struct FloatVect3 dist_v_f;

	struct NedCoor_i curr_ned_pos_i = *stateGetPositionNed_i();

	dist_v_f.x = POS_FLOAT_OF_BFP((curr_ned_pos_i.x - ned_last_cali_pos_i.x));
	dist_v_f.y = POS_FLOAT_OF_BFP((curr_ned_pos_i.y - ned_last_cali_pos_i.y));
	dist_v_f.z = POS_FLOAT_OF_BFP((curr_ned_pos_i.z - ned_last_cali_pos_i.z));
	float dist_f = float_vect3_norm(&dist_v_f);

	if(dist_f > 5000)
	{
		mag_cali.cali_ok = FALSE;
	}
}

static void mag_cali_persistent_write(void)
{
	struct MagCaliPersData temp_MagCaliFramData;
	uint8_t *ptemp = (uint8_t *) (&temp_MagCaliFramData);

	temp_MagCaliFramData.cali_ecef_pos_i = *stateGetPositionEcef_i();
	temp_MagCaliFramData.gain[0] = mag_cali.gain[0];
	temp_MagCaliFramData.gain[1] = mag_cali.gain[1];
	temp_MagCaliFramData.offset[0] = mag_cali.offset[0];
	temp_MagCaliFramData.offset[1] = mag_cali.offset[1];
	temp_MagCaliFramData.mag_declination = mag_cali.mag_declination;
	temp_MagCaliFramData.cali_ok = mag_cali.cali_ok;
	temp_MagCaliFramData.crc16 = Crc16_normal(ptemp, 0, MAG_CALI_PERS_DATA_STRUCT_LENGTH - 4);

	fram_mag_cali_data_write(ptemp);
}

static void mag_cali_manual_fram_erase(void)
{
	mag_cali.gain[0] = 1;
	mag_cali.gain[1] = 1;
	mag_cali.offset[0] = 0;
	mag_cali.offset[1] = 0;
	mag_cali.cali_ok = FALSE;

	mag_cali_persistent_write();
}

void mag_cali_init(void)
{
	MATRIX_INI(F);
	MATRIX_INI(JT);
	MATRIX_INI(p);
	MATRIX_INI(ev);
	MATRIX_INI(step);

	mag_cali.manual_enable = FALSE;
	mag_cali.manual_enable_prev = FALSE;
	mag_cali.manual_fram_erase = FALSE;
	mag_cali.manual_fram_erase_prev = FALSE;
	mag_cali.state = MAG_CALI_IDLE;

	mag_cali.gain[0] = 1;
	mag_cali.gain[1] = 1;
	mag_cali.offset[0] = 0;
	mag_cali.offset[1] = 0;
	mag_cali.cali_ok = FALSE;
	mag_cali.auto_cali = FALSE;
	mag_cali.persistent_store = FALSE;
	mag_cali.persistent_read = TRUE;

	AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
	AbiBindMsgGPS_HEADING(ABI_BROADCAST, &gps_ev, gps_cb);

	MatSetValue(&step, 0.001f);
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

static bool_t mag_cali_load_to_imu(void)
{
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
	if ((mag_cali.offset[0] < -1.0f) || (mag_cali.offset[0] > 1.0f))
	{
		err++;
	}
	if ((mag_cali.offset[1] < -1.0f) || (mag_cali.offset[1] > 1.0f))
	{
		err++;
	}
	if( fabsf(mag_cali.mag_declination) > 20.0f )
	{
		err++;
	}

	if(err)
	{
		return FALSE;
	}
	else
	{
		imu.mag_neutral.x = mag_cali.offset[0]*(float)MAG_SENSITIVITY;
		imu.mag_neutral.y = mag_cali.offset[1]*(float)MAG_SENSITIVITY;
		imu.mag_neutral.z = 0;
		imu.mag_sens.x = mag_cali.gain[0];
		imu.mag_sens.y = mag_cali.gain[1];
		imu.mag_sens.z = mag_cali.gain[1];

		// mag declination
		ahrs_mlkf.mag_h_cali.x = cosf(mag_cali.mag_declination);
		ahrs_mlkf.mag_h_cali.y = sinf(mag_cali.mag_declination);
		ahrs_mlkf.mag_h_cali.z = 0;

		return TRUE;
	}
}

void mag_cali_imu_scale(struct Imu *_imu)
{
	_imu->mag_real.x = (float)(_imu->mag_unscaled.x - _imu->mag_neutral.x) * _imu->mag_sens.x / (float)MAG_SENSITIVITY;
	_imu->mag_real.y = (float)(_imu->mag_unscaled.y - _imu->mag_neutral.y) * _imu->mag_sens.y / (float)MAG_SENSITIVITY;
	_imu->mag_real.z = (float)(_imu->mag_unscaled.z - _imu->mag_neutral.z) * _imu->mag_sens.z / (float)MAG_SENSITIVITY;
	MAGS_BFP_OF_REAL(_imu->mag, _imu->mag_real);
	VECT3_COPY(_imu->mag_scaled, _imu->mag);
	_imu->mag.z = 0;
}

void mag_cali_periodic(void)
{
	if(mag_cali.persistent_store)	// storge mag cali data when throttle killed
	{
		if(kill_throttle)
		{
			mag_cali_persistent_write();
			mag_cali.persistent_store = FALSE;
		}
	}

	if (!is_mag_cali_done())
	{
		if (!gps.h_stable)	// stop calibration if gps heading unstable
		{
			mag_cali_stop(FALSE);
		}
	}

	// read persistent cali data after ltp_initialized to decide need cali or not

	if (mag_cali.persistent_read)
	{
		if (ins_int.ltp_initialized)
		{
			mag_cali_persistent_read();

			if (mag_cali.cali_ok)
			{
				mag_cali.need_cali = !mag_cali_load_to_imu();
			}
			else
			{
				mag_cali.need_cali = TRUE;
			}
			mag_cali.persistent_read = FALSE;
		}
	}

	if(mag_cali.manual_fram_erase_prev != mag_cali.manual_fram_erase)
	{
		if (mag_cali.manual_fram_erase == TRUE)
		{
			mag_cali_manual_fram_erase();
		}
	}
	mag_cali.manual_fram_erase_prev = mag_cali.manual_fram_erase;

	if(mag_cali.manual_enable_prev != mag_cali.manual_enable)
	{
		if(mag_cali.manual_enable == TRUE)
		{
			mag_cali_start(FALSE);
		}
		else
		{
			mag_cali_stop(FALSE);
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
				mag_cali_stop(FALSE);
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
				mag_cali_stop(FALSE);
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

		float x,y,mag_heading,declination = 0;
		for(int i=0;i<MAG_CALI_GRAB_NUM;++i)
		{
			x = (mag_cali.grab_heading_pair[i][1] - mag_cali.offset[0]) * mag_cali.gain[0];
			y = (mag_cali.grab_heading_pair[i][2] - mag_cali.offset[1]) * mag_cali.gain[1];
			mag_heading = LimitAngleTo_0_2pi(atan2f(-y, x));
			declination += mag_heading - mag_cali.grab_heading_pair[i][0];
		}
		mag_cali.mag_declination = declination / (float)MAG_CALI_GRAB_NUM;

		// check and load
		mag_cali_stop(mag_cali_load_to_imu());
	}

#ifdef PERIODIC_TELEMETRY
		RunOnceEvery(25,   {

			float grab_x[MAG_CALI_GRAB_NUM];
			float grab_y[MAG_CALI_GRAB_NUM];

			for(int i=0;i<MAG_CALI_GRAB_NUM;++i)
			{
				grab_x[i] = mag_cali.grab_sum[i][0];
				grab_y[i] = mag_cali.grab_sum[i][1];
			}

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
				&mag_cali.mag_declination,
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
		mag_cali.cali_ok_last = mag_cali.cali_ok;
	}
}
static void mag_cali_stop(bool_t ok)
{
	mag_cali.state = MAG_CALI_IDLE;
	mag_cali.auto_cali = FALSE;
	if(ok)
	{
		mag_cali.cali_ok = TRUE;
		mag_cali.persistent_store = TRUE;
	}
	else
	{
		mag_cali.cali_ok = mag_cali.cali_ok_last;
	}
	mag_cali.need_cali = !mag_cali.cali_ok;
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
		mag_cali_stop(FALSE);
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
