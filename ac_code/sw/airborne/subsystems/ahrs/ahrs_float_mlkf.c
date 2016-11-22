/*
 * Copyright (C) 2011-2012  Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2013       Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file subsystems/ahrs/ahrs_float_mlkf.c
 *
 * Multiplicative linearized Kalman Filter in quaternion formulation.
 *
 * Estimate the attitude, heading and gyro bias.
 */

#include "subsystems/ahrs/ahrs_float_mlkf.h"
#include "subsystems/ahrs/ahrs_float_utils.h"

#include <string.h>  /* for memcpy      */

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "generated/airframe.h"

#include <math.h>
#include "math/my_math.h"

//#include <stdio.h>

//#define AHRS_PROPAGATE_LOW_PASS_RATES
//#define AHRS_GYRO_LEADLAG_FILTER
//#define AHRS_GYRO_BW_FILTER
//#define AHRS_GYRO_BW_FLOAT_FILTER
#define AHRS_GYRO_PASSTHROUGH

#if defined AHRS_GYRO_BW_FILTER || defined AHRS_GYRO_BW_FLOAT_FILTER
 #include "filters/low_pass_filter.h"
#endif

static inline void propagate_ref(struct Int32Rates *gyro, float dt);
static inline void propagate_state(float dt);
static inline void update_state(const struct FloatVect3 *i_expected,
                                struct FloatVect3 *b_measured,
                                struct FloatVect3 *noise);
static inline void update_state_heading(const struct FloatVect3 *i_expected,
                                        struct FloatVect3 *b_measured,
                                        struct FloatVect3 *noise);
static inline void reset_state(void);

#ifdef AHRS_GYRO_LEADLAG_FILTER
static inline float leadlag_filter(float in_now, float in_last, float out_last, float dt);
#endif

struct AhrsMlkf ahrs_mlkf;

#ifdef AHRS_GYRO_BW_FILTER
	Butterworth2LowPass_int filter_p;
	Butterworth2LowPass_int filter_q;
	Butterworth2LowPass_int filter_r;
#endif

#ifdef AHRS_GYRO_BW_FLOAT_FILTER
	Butterworth2LowPass filter_p;
	Butterworth2LowPass filter_q;
	Butterworth2LowPass filter_r;
#endif

void ahrs_mlkf_init(void)
{
  ahrs_mlkf.is_aligned = FALSE;

  /* init ltp_to_imu quaternion as zero/identity rotation */
  float_quat_identity(&ahrs_mlkf.ltp_to_imu_quat);
  FLOAT_RATES_ZERO(ahrs_mlkf.imu_rate);

  VECT3_ASSIGN(ahrs_mlkf.mag_h, AHRS_H_X, AHRS_H_Y, AHRS_H_Z);

  /*
   * Initialises our state
   */
  FLOAT_RATES_ZERO(ahrs_mlkf.gyro_bias);
  const float P0_a = 1.;
  const float P0_b = 1e-4;
  float P0[6][6] = { { P0_a, 0.,   0.,   0.,   0.,   0.  },
					    { 0.,   P0_a, 0.,   0.,   0.,   0.  },
					    { 0.,   0.,   P0_a, 0.,   0.,   0.  },
					    { 0.,   0.,   0.,   P0_b, 0.,   0.  },
					    { 0.,   0.,   0.,   0.,   P0_b, 0.  },
					    { 0.,   0.,   0.,   0.,   0.,   P0_b}
  };
  memcpy(ahrs_mlkf.P, P0, sizeof(P0));

  VECT3_ASSIGN(ahrs_mlkf.mag_noise, 10, 10, 10);
  VECT3_ASSIGN(ahrs_mlkf.gps_heading_noise, 0.1f, 0.1f, 0.1f);

#ifdef AHRS_GYRO_BW_FILTER
  init_butterworth_2_low_pass_int(&filter_p, 18, (1. / 512), 0);
  init_butterworth_2_low_pass_int(&filter_q, 18, (1. / 512), 0);
  init_butterworth_2_low_pass_int(&filter_r, 18, (1. / 512), 0);
#endif

#ifdef AHRS_GYRO_BW_FLOAT_FILTER   //tau = 0.1592/cuf_off_fre   18hz:0.00885   10hz:0.01592  6hz:0.02653
  init_butterworth_2_low_pass(&filter_p, 0.00885, (1. / 512), 0.);
  init_butterworth_2_low_pass(&filter_q, 0.00885, (1. / 512), 0.);
  init_butterworth_2_low_pass(&filter_r, 0.00885, (1. / 512), 0.);
#endif

  ahrs_mlkf.virtual_h_stable = 1;
  ahrs_mlkf.heading_state = AMHS_MAG;
}

void ahrs_mlkf_set_body_to_imu(struct OrientationReps *body_to_imu)
{
  ahrs_mlkf_set_body_to_imu_quat(orientationGetQuat_f(body_to_imu));
}

void ahrs_mlkf_set_body_to_imu_quat(struct FloatQuat *q_b2i)
{
  orientationSetQuat_f(&ahrs_mlkf.body_to_imu, q_b2i);

  if (!ahrs_mlkf.is_aligned) {
    /* Set ltp_to_imu so that body is zero */
    ahrs_mlkf.ltp_to_imu_quat = *orientationGetQuat_f(&ahrs_mlkf.body_to_imu);
  }
}


bool_t ahrs_mlkf_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{

  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_mlkf.ltp_to_imu_quat, lp_accel, lp_mag);

  /* used averaged gyro as initial value for bias */
  struct Int32Rates bias0;
  RATES_COPY(bias0, *lp_gyro);
  VECT3_COPY(ahrs_mlkf.lp_accel_ini, *lp_accel);
  RATES_ASSIGN(ahrs_mlkf.gyro_bias,0,0,0);
  RATES_FLOAT_OF_BFP(ahrs_mlkf.gyro_bias_ini, bias0);

  ahrs_mlkf.is_aligned = TRUE;

  return TRUE;
}




/*gyro update function:updateahrs_mlkf.ltp_to_imu_quat, caculate P(k+1|k)*/
void ahrs_mlkf_propagate(struct Int32Rates *gyro, float dt)
{
  propagate_ref(gyro, dt);
  propagate_state(dt);
}

float acc_noise_ratio = 10000.0;
/*accel update function: caculate K, update P(k+1|k+1), get X(k+1|k+1)*/
void ahrs_mlkf_update_accel(struct Int32Vect3 *accel)
{
  struct FloatVect3 imu_g;
  ACCELS_FLOAT_OF_BFP(imu_g, *accel);
  const float alpha = 0.92;
  ahrs_mlkf.lp_accel = alpha * ahrs_mlkf.lp_accel +
                       (1. - alpha) * (float_vect3_norm(&imu_g) - 9.81);
  const struct FloatVect3 earth_g = {0.,  0., -9.81 };
  const float dn = acc_noise_ratio * fabs(ahrs_mlkf.lp_accel);//250 * fabs(ahrs_mlkf.lp_accel);
  struct FloatVect3 g_noise = {1. + dn, 1. + dn, 1. + dn};
  update_state(&earth_g, &imu_g, &g_noise);
  reset_state();
}

/*mag update function: caculate K, update P(k+1|k+1), get X(k+1|k+1)*/
void ahrs_mlkf_update_mag(struct Int32Vect3 *mag)
{
	if(ahrs_mlkf.heading_state == AMHS_MAG)
	{
#if AHRS_MAG_UPDATE_ALL_AXES
		ahrs_mlkf_update_mag_full(mag);
#else
		//ahrs_mlkf_update_mag_2d(mag);
		ahrs_mlkf_update_mag_2d_new(mag);
#endif
	}
}

void ahrs_mlkf_update_mag_2d_new(struct Int32Vect3 *mag)
{
  struct FloatVect3 mag_bm;
  struct FloatVect3 mag_bmv;
  struct FloatVect3 mag_bm_i;
  struct FloatVect3 mag_ic;

  MAGS_FLOAT_OF_BFP(mag_bm, *mag);

  // generate a virtual mag_bm that has no z-value in ltp coordinate
  float_vect3_normalize(&mag_bm);
  float_quat_vmult_inv(&mag_bm_i, &ahrs_mlkf.ltp_to_imu_quat, &mag_bm);
  mag_bm_i.z = 0;
  float_quat_vmult(&mag_bmv, &ahrs_mlkf.ltp_to_imu_quat, &mag_bm_i);
  //float_vect3_normalize(&mag_bm);


  // generate a virtual mag_ic that has the same length with mag_bm generated in last step
  mag_ic = ahrs_mlkf.mag_h;
  mag_ic.z = 0;
  float_vect3_normalize(&mag_ic);

  float norm = float_vect3_norm(&mag_bm);
  //norm = float_vect3_norm(&mag_bm_i);
  mag_ic.x *= norm;
  mag_ic.y *= norm;

  // update mlkf
  update_state(&mag_ic, &mag_bmv, &ahrs_mlkf.mag_noise);
  reset_state();
}

void ahrs_mlkf_update_mag_2d(struct Int32Vect3 *mag)
{
  struct FloatVect3 imu_h;
  MAGS_FLOAT_OF_BFP(imu_h, *mag);
  update_state_heading(&ahrs_mlkf.mag_h, &imu_h, &ahrs_mlkf.mag_noise);
  reset_state();
}

void ahrs_mlkf_update_mag_full(struct Int32Vect3 *mag)
{
  struct FloatVect3 imu_h;
  MAGS_FLOAT_OF_BFP(imu_h, *mag);
  update_state(&ahrs_mlkf.mag_h, &imu_h, &ahrs_mlkf.mag_noise);
  reset_state();
}

/*leadlag function*/
#ifdef AHRS_GYRO_LEADLAG_FILTER
static inline float leadlag_filter(float in_now, float in_last, float out_last, float dt)
{
	const float lead = 0.0;
	const float lag = 5.0*dt;
	const float gain = 1.0;
	float k1 = gain * (dt+2.0*lead)/(dt+2.0*lag);
	float k2 = gain * (dt-2.0*lead)/(dt+2.0*lag);
	float k3 = gain * (2.0*lag-dt)/(2.0*lag+dt);

	return k1*in_now + k2*in_last + k3*out_last;	
}
#endif


static inline void propagate_ref(struct Int32Rates *gyro, float dt)
{
  /* converts gyro to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, *gyro);
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias_ini);
  
 #ifdef AHRS_GYRO_BW_FILTER
  struct Int32Rates gyro_filtered;
  gyro_filtered.p = update_butterworth_2_low_pass_int(&filter_p, gyro->p);
  gyro_filtered.q = update_butterworth_2_low_pass_int(&filter_q, gyro->q);
  gyro_filtered.r = update_butterworth_2_low_pass_int(&filter_r, gyro->r);
  
  RATES_FLOAT_OF_BFP(gyro_float, gyro_filtered);
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias);
  RATES_COPY(ahrs_mlkf.imu_rate, gyro_float);
 #endif  

 #ifdef AHRS_GYRO_BW_FLOAT_FILTER
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias);
 
  ahrs_mlkf.imu_rate.p = update_butterworth_2_low_pass(&filter_p, gyro_float.p);
  ahrs_mlkf.imu_rate.q = update_butterworth_2_low_pass(&filter_q, gyro_float.q);
  ahrs_mlkf.imu_rate.r = update_butterworth_2_low_pass(&filter_r, gyro_float.r);  
 #endif  

 #ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias);
  /* lowpass angular rates */
  const float alpha = 0.1;
  FLOAT_RATES_LIN_CMB(ahrs_mlkf.imu_rate, ahrs_mlkf.imu_rate,
                      (1. - alpha), gyro_float, alpha);
 #endif

 #ifdef AHRS_GYRO_LEADLAG_FILTER
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias);
  static struct FloatRates gyro_float_last;
  ahrs_mlkf.imu_rate.p = leadlag_filter(gyro_float.p, gyro_float_last.p, ahrs_mlkf.imu_rate.p, dt);
  ahrs_mlkf.imu_rate.q = leadlag_filter(gyro_float.q, gyro_float_last.q, ahrs_mlkf.imu_rate.q, dt);
  ahrs_mlkf.imu_rate.r = leadlag_filter(gyro_float.r, gyro_float_last.r, ahrs_mlkf.imu_rate.r, dt);
  RATES_COPY(gyro_float_last, gyro_float);
 #endif

 #ifdef AHRS_GYRO_PASSTHROUGH
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_mlkf.gyro_bias);
  RATES_COPY(ahrs_mlkf.imu_rate, gyro_float);
 #endif

  /* propagate reference quaternion */
  float_quat_integrate(&ahrs_mlkf.ltp_to_imu_quat, &ahrs_mlkf.imu_rate, dt);

}

/**
 * Progagate filter's covariance
 * We don't propagate state as we assume to have reseted
 */
static inline void propagate_state(float dt)
{

  /* predict covariance */
  const float dp = ahrs_mlkf.imu_rate.p * dt;
  const float dq = ahrs_mlkf.imu_rate.q * dt;
  const float dr = ahrs_mlkf.imu_rate.r * dt;

  float F[6][6] = {{  1.,   dr,  -dq,  -dt,   0.,   0.  },
				    { -dr,   1.,   dp,   0.,  -dt,   0.  },
				    {  dq,  -dp,   1.,   0.,   0.,  -dt  },
				    {  0.,   0.,   0.,   1.,   0.,   0.  },
				    {  0.,   0.,   0.,   0.,   1.,   0.  },
				    {  0.,   0.,   0.,   0.,   0.,   1.  }
  };
  // P = FPF' + GQG
  float tmp[6][6];
  MAT_MUL(6, 6, 6, tmp, F, ahrs_mlkf.P);
  MAT_MUL_T(6, 6, 6,  ahrs_mlkf.P, tmp, F);
  const float dt2 = dt * dt;
  const float GQG[6] = {dt2 * 10e-3, dt2 * 10e-3, dt2 * 10e-3, dt2 * 9e-6, dt2 * 9e-6, dt2 * 9e-6 };
  for (int i = 0; i < 6; i++) {
    ahrs_mlkf.P[i][i] += GQG[i];
  }

}


/**
 * Incorporate one 3D vector measurement.
 * @param i_expected expected 3d vector in inertial frame
 * @param b_measured measured 3d vector in body/imu frame
 * @param noise measurement noise vector (diagonal of covariance)
 */
static inline void update_state(const struct FloatVect3 *i_expected, struct FloatVect3 *b_measured,
                                struct FloatVect3 *noise)
{

  /* converted expected measurement from inertial to body frame */
  struct FloatVect3 b_expected;
  float_quat_vmult(&b_expected, &ahrs_mlkf.ltp_to_imu_quat, i_expected);

  // S = HPH' + JRJ
  float H[3][6] = {{           0., -b_expected.z,  b_expected.y, 0., 0., 0.},
                   { b_expected.z,            0., -b_expected.x, 0., 0., 0.},
                   { -b_expected.y, b_expected.x,            0., 0., 0., 0.}
  };
  float tmp[3][6];
  MAT_MUL(3, 6, 6, tmp, H, ahrs_mlkf.P);
  float S[3][3];
  MAT_MUL_T(3, 6, 3, S, tmp, H);

  /* add the measurement noise */
  S[0][0] += noise->x;
  S[1][1] += noise->y;
  S[2][2] += noise->z;

  float invS[3][3];
  MAT_INV33(invS, S);

  // K = PH'invS
  float tmp2[6][3];
  MAT_MUL_T(6, 6, 3, tmp2, ahrs_mlkf.P, H);
  float K[6][3];
  MAT_MUL(6, 3, 3, K, tmp2, invS);

  // P = (I-KH)P
  float tmp3[6][6];
  MAT_MUL(6, 3, 6, tmp3, K, H);
  float I6[6][6] = {{ 1., 0., 0., 0., 0., 0. },
    {  0., 1., 0., 0., 0., 0. },
    {  0., 0., 1., 0., 0., 0. },
    {  0., 0., 0., 1., 0., 0. },
    {  0., 0., 0., 0., 1., 0. },
    {  0., 0., 0., 0., 0., 1. }
  };
  float tmp4[6][6];
  MAT_SUB(6, 6, tmp4, I6, tmp3);
  float tmp5[6][6];
  MAT_MUL(6, 6, 6, tmp5, tmp4, ahrs_mlkf.P);
  memcpy(ahrs_mlkf.P, tmp5, sizeof(ahrs_mlkf.P));

  // X = X + Ke
  struct FloatVect3 e;
  VECT3_DIFF(e, *b_measured, b_expected);
  ahrs_mlkf.gibbs_cor.qx += K[0][0] * e.x + K[0][1] * e.y + K[0][2] * e.z;
  ahrs_mlkf.gibbs_cor.qy += K[1][0] * e.x + K[1][1] * e.y + K[1][2] * e.z;
  ahrs_mlkf.gibbs_cor.qz += K[2][0] * e.x + K[2][1] * e.y + K[2][2] * e.z;
  ahrs_mlkf.gyro_bias.p  += K[3][0] * e.x + K[3][1] * e.y + K[3][2] * e.z;
  ahrs_mlkf.gyro_bias.q  += K[4][0] * e.x + K[4][1] * e.y + K[4][2] * e.z;
  ahrs_mlkf.gyro_bias.r  += K[5][0] * e.x + K[5][1] * e.y + K[5][2] * e.z;

}


/**
 * Incorporate one 3D vector measurement, only correcting heading.
 * @param i_expected expected 3d vector in inertial frame
 * @param b_measured measured 3d vector in body/imu frame
 * @param noise measurement noise vector (diagonal of covariance)
 * TODO: optimize
 */
static inline void update_state_heading(const struct FloatVect3 *i_expected,
                                        struct FloatVect3 *b_measured,
                                        struct FloatVect3 *noise)
{

  /* converted expected measurement from inertial to body frame */

  struct FloatVect3 b_expected;
  float_quat_vmult(&b_expected, &ahrs_mlkf.ltp_to_imu_quat, i_expected);
//  struct FloatVect3 i_b_measured;
//  float_vect3_normalize(b_measured);
//  float_quat_vmult_inv(&i_b_measured, &ahrs_mlkf.ltp_to_imu_quat, b_measured);
//  i_b_measured.z = 0;
//  float_quat_vmult(b_measured, &ahrs_mlkf.ltp_to_imu_quat, &i_b_measured);
//  float_vect3_normalize(b_measured);

  /* set roll/pitch errors to zero to only correct heading */
  // S = HPH' + JRJ
  struct FloatVect3 i_h_2d = {i_expected->y, -i_expected->x, 0.f};
  struct FloatVect3 b_yaw;
  float_quat_vmult(&b_yaw, &ahrs_mlkf.ltp_to_imu_quat, &i_h_2d);
  // S = HPH' + JRJ
  float H[3][6] = {{ 0., 0., b_yaw.x, 0., 0., 0.},
                   { 0., 0., b_yaw.y, 0., 0., 0.},
                   { 0., 0., b_yaw.z, 0., 0., 0.}
  };
  float tmp[3][6];
  MAT_MUL(3, 6, 6, tmp, H, ahrs_mlkf.P);
  float S[3][3];
  MAT_MUL_T(3, 6, 3, S, tmp, H);

  /* add the measurement noise */
  S[0][0] += noise->x;
  S[1][1] += noise->y;
  S[2][2] += noise->z;

  float invS[3][3];
  MAT_INV33(invS, S);

  // K = PH'invS
  float tmp2[6][3];
  MAT_MUL_T(6, 6, 3, tmp2, ahrs_mlkf.P, H);
  float K[6][3];
  MAT_MUL(6, 3, 3, K, tmp2, invS);

  // P = (I-KH)P
  float tmp3[6][6];
  MAT_MUL(6, 3, 6, tmp3, K, H);
  float I6[6][6] = {{ 1., 0., 0., 0., 0., 0. },
    {  0., 1., 0., 0., 0., 0. },
    {  0., 0., 1., 0., 0., 0. },
    {  0., 0., 0., 1., 0., 0. },
    {  0., 0., 0., 0., 1., 0. },
    {  0., 0., 0., 0., 0., 1. }
  };
  float tmp4[6][6];
  MAT_SUB(6, 6, tmp4, I6, tmp3);
  float tmp5[6][6];
  MAT_MUL(6, 6, 6, tmp5, tmp4, ahrs_mlkf.P);
  memcpy(ahrs_mlkf.P, tmp5, sizeof(ahrs_mlkf.P));

  // X = X + Ke
  struct FloatVect3 e;
  VECT3_DIFF(e, *b_measured, b_expected);
  ahrs_mlkf.gibbs_cor.qx += K[0][0] * e.x + K[0][1] * e.y + K[0][2] * e.z;
  ahrs_mlkf.gibbs_cor.qy += K[1][0] * e.x + K[1][1] * e.y + K[1][2] * e.z;
  ahrs_mlkf.gibbs_cor.qz += K[2][0] * e.x + K[2][1] * e.y + K[2][2] * e.z;
  ahrs_mlkf.gyro_bias.p  += K[3][0] * e.x + K[3][1] * e.y + K[3][2] * e.z;
  ahrs_mlkf.gyro_bias.q  += K[4][0] * e.x + K[4][1] * e.y + K[4][2] * e.z;
  ahrs_mlkf.gyro_bias.r  += K[5][0] * e.x + K[5][1] * e.y + K[5][2] * e.z;

}

#ifdef USE_GPS_HEADING
void ahrs_mlkf_update_gps(struct GpsState *gps_s)
{
	static bool_t gps_heading_aligned = FALSE;

	if (gps_s->h_stable && ahrs_mlkf.virtual_h_stable)
	{
		if(ahrs_mlkf.heading_state == AMHS_MAG)
		{
			ahrs_mlkf.heading_state = AMHS_GPS;
		}
		else if(ahrs_mlkf.heading_state == AMHS_GPS)
		{
			if(!gps_heading_aligned)
			{
				gps_heading_aligned = TRUE;
				ahrs_float_get_quat_from_gps_heading(&ahrs_mlkf.ltp_to_imu_quat, gps_s);
			}
			ahrs_mlkf_update_gps_heading(gps_s);
		}
		else if(ahrs_mlkf.heading_state == AMHS_SWITCHING)
		{
		}
	}
	else
	{
		if(ahrs_mlkf.heading_state != AMHS_MAG)
		{
			ahrs_mlkf.heading_state = AMHS_MAG;
		}
	}
}

#define MAG_OFFSET_ANGLE 2.7
void ahrs_mlkf_update_gps_heading(struct GpsState *gps_s)
{
//	struct FloatVect3 imu_h;
//	float gps_psi_rad;
//	gps_psi_rad = (gps_s->heading - MAG_OFFSET_ANGLE)*my_math_deg_to_rad;
//	imu_h.x = -cosf(gps_psi_rad);
//	imu_h.y = sinf(gps_psi_rad);
//	imu_h.z = 0.0;
//
//	struct FloatVect3 i_meas;
//	float_quat_vmult(&i_meas, &ahrs_mlkf.ltp_to_body_quat, &imu_h);
//
//	update_state_heading(&ahrs_mlkf.mag_h, &i_meas, &ahrs_mlkf.mag_noise);
//	reset_state();

	float heading;
	struct FloatVect3 heading_bm;
	struct FloatVect3 heading_bmv;
	struct FloatVect3 heading_bm_i;
	struct FloatVect3 heading_ic;

	heading = gps_s->heading * my_math_deg_to_rad;
	heading_bm.x = + cosf(heading);
	heading_bm.y = - sinf(heading);
	heading_bm.z = 0;
	float_quat_vmult_inv(&heading_bm_i, &ahrs_mlkf.ltp_to_imu_quat, &heading_bm);
	heading_bm_i.z = 0;
	float_quat_vmult(&heading_bmv, &ahrs_mlkf.ltp_to_imu_quat, &heading_bm_i);
	float_vect3_normalize(&heading_bmv);

	heading_ic.x = 1;
	heading_ic.y = 0;
	heading_ic.z = 0;

	// update mlkf
	update_state(&heading_ic, &heading_bmv, &ahrs_mlkf.gps_heading_noise);
	reset_state();
}
#endif

void ahrs_float_mlkf_SetMagNoise(float noise)
{
	VECT3_ASSIGN(ahrs_mlkf.mag_noise, noise, noise, noise);
}

void ahrs_float_mlkf_SetGpsHeadingNoise(float noise)
{
	VECT3_ASSIGN(ahrs_mlkf.gps_heading_noise, noise, noise, noise);
}

/**
 * Incorporate errors to reference and zeros state
 */
static inline void reset_state(void)
{

  ahrs_mlkf.gibbs_cor.qi = 2.;
  struct FloatQuat q_tmp;
  float_quat_comp(&q_tmp, &ahrs_mlkf.ltp_to_imu_quat, &ahrs_mlkf.gibbs_cor);
  float_quat_normalize(&q_tmp);
  ahrs_mlkf.ltp_to_imu_quat = q_tmp;
  float_quat_identity(&ahrs_mlkf.gibbs_cor);

}
