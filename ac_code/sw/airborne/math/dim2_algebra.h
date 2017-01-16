/*
 * dim2_algebra.h
 *
 *  Created on: Jan 16, 2017
 *      Author: jay
 */

#ifndef SW_AIRBORNE_MATH_DIM2_ALGEBRA_H_
#define SW_AIRBORNE_MATH_DIM2_ALGEBRA_H_

#include "math/pprz_algebra_float.h"

struct _s_matrix22
{
	float m11;float m12;
	float m21;float m22;
};

static inline void Rotate_vect2(struct FloatVect2 *vo, struct _s_matrix22 *R, struct FloatVect2 *vi)
{
	struct FloatVect2 vt;

	vt.x = vi->x * R->m11 + vi->y * R->m12;
	vt.y = vi->x * R->m21 + vi->y * R->m22;

	VECT2_COPY(*vo, vt);
}

static inline void Matrix22_set_i(struct _s_matrix22 *mo)
{
	mo->m11 = 1; mo->m12 = 0;
	mo->m21 = 0; mo->m22 = 1;
}

static inline void Matrix22_copy(struct _s_matrix22 *mo, struct _s_matrix22 *ma)
{
	mo->m11 = ma->m11;
	mo->m12 = ma->m12;
	mo->m21 = ma->m21;
	mo->m22 = ma->m22;
}

static inline void Matrix22_mult(struct _s_matrix22 *mo, struct _s_matrix22 *ma, struct _s_matrix22 *mb)
{
	struct _s_matrix22 mt;

	mt.m11 = ma->m11 * mb->m11 + ma->m12 * mb->m21;
	mt.m21 = ma->m21 * mb->m11 + ma->m22 * mb->m21;
	mt.m12 = ma->m11 * mb->m12 + ma->m12 * mb->m22;
	mt.m22 = ma->m21 * mb->m12 + ma->m22 * mb->m22;

	Matrix22_copy(mo, &mt);
}

static inline void Matrix22_trans(struct _s_matrix22 *mo, struct _s_matrix22 *ma)
{
	struct _s_matrix22 mt;

	mt.m11 = ma->m11; mt.m12 = ma->m21;
	mt.m21 = ma->m12; mt.m22 = ma->m22;

	Matrix22_copy(mo, &mt);
}


#endif /* SW_AIRBORNE_MATH_DIM2_ALGEBRA_H_ */
