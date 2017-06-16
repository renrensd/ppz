/*
 * sgdf_filter.h
 *
 *  Created on: Dec 10, 2016
 *      Author: lijie
 */

#ifndef SW_AIRBORNE_FILTERS_SGDF_FILTER_H_
#define SW_AIRBORNE_FILTERS_SGDF_FILTER_H_

static const float SGDF2_filter_coefs[2] = {1.0f,-1.0f};
static const float SGDF3_filter_coefs[3] = {0.5f,0,-0.5f};
static const float SGDF4_filter_coefs[4] = {0.3f,0.1f,-0.1f,-0.3f};
static const float SGDF5_filter_coefs[5] = {0.2f,0.1f,0,-0.1f,-0.2f};
static const float SGDF6_filter_coefs[6] = {0.14286f,0.08571f,0.02857f,-0.02857f,-0.08571f,-0.14286f};
static const float SGDF7_filter_coefs[7] = {0.10714f, 0.07143f, 0.03571f, 0, -0.03571f, -0.07143f, -0.10714f};
static const float SGDF13_filter_coefs[13] = {0.03297f, 0.02747f, 0.02198f, 0.01648f, 0.01099f, 0.00549f , 0,
																							-0.00549f, -0.01099f, -0.01648f, -0.02198f, -0.02747f, -0.03297f
																						 };

struct _s_sgdf_filter
{
	unsigned short win_size;
	unsigned short data_index;
	float *data;
	float *dT;
	unsigned short dT_index;
	float *coefs; // first coef is corresponding to newest data
	float Fs;
	float out;
};

#define DECLARE_SGDF(name, size)		struct _s_sgdf_filter name;\
																		float name##_##data[size];\
																		float name##_##dT[size];
#define INIT_SGDF(name, size, Fs)		init_sgdf(&name, name##_##data, name##_##dT, size, Fs);
#define UPDATE_SGDF(name, in)		update_sgdf(&name, in);

static inline void init_sgdf(struct _s_sgdf_filter *sgdf, float *data, float *dT_data, unsigned char win_size, float Fs)
{
	sgdf->coefs = 0;
	sgdf->data = data;
	sgdf->dT = dT_data;
	sgdf->win_size = win_size;
	sgdf->data_index = 0;
	sgdf->Fs = Fs;

	switch (win_size)
	{
	case 2:
		sgdf->coefs = (float *)SGDF2_filter_coefs;
		break;
	case 3:
		sgdf->coefs = (float *)SGDF3_filter_coefs;
		break;
	case 4:
		sgdf->coefs = (float *)SGDF4_filter_coefs;
		break;
	case 5:
		sgdf->coefs = (float *)SGDF5_filter_coefs;
		break;
	case 6:
		sgdf->coefs = (float *)SGDF6_filter_coefs;
		break;
	case 7:
		sgdf->coefs = (float *)SGDF7_filter_coefs;
		break;
	case 13:
		sgdf->coefs = (float *)SGDF13_filter_coefs;
		break;
	default:
		break;
	}
}

static inline float update_sgdf(struct _s_sgdf_filter *sgdf, float in)
{
	unsigned short i;
	unsigned short j;

	if(sgdf == 0)
	{
		return 0;
	}
	if( sgdf->coefs == 0 )
	{
		return 0;
	}
	if( sgdf->data == 0 )
	{
		return 0;
	}

	++sgdf->data_index;
	if (sgdf->data_index >= sgdf->win_size)
	{
		sgdf->data_index = 0;
	}
	sgdf->data[sgdf->data_index] = in;

	sgdf->out = 0;
	for (i = 0; i < sgdf->win_size; ++i)
	{
		if (i > sgdf->data_index)
		{
			j = sgdf->data_index + sgdf->win_size - i;
		}
		else
		{
			j = sgdf->data_index - i;
		}
		sgdf->out += sgdf->data[j] * sgdf->coefs[i];
	}

	sgdf->out *= sgdf->Fs;

	return sgdf->out;
}

static inline float update_sgdf_dT(struct _s_sgdf_filter *sgdf, float dT, float in)
{
	float dT_sum = 0;
	if(dT > 0)
	{
		sgdf->dT[sgdf->dT_index] = dT;
		if(++sgdf->dT_index >= sgdf->win_size)
		{
			sgdf->dT_index = 0;
		}
		for (unsigned short i = 0; i < sgdf->win_size; ++i)
		{
			dT_sum += sgdf->dT[i];
		}
		sgdf->Fs = 1.0f/(dT_sum/(float)sgdf->win_size);
	}
	return update_sgdf(sgdf, in);
}

#endif /* SW_AIRBORNE_FILTERS_SGDF_FILTER_H_ */
