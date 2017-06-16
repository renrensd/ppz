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
	float m11;
	float m12;
	float m21;
	float m22;
};

extern float determinant(float a, float b, float c, float d);
extern float point2_distance(struct FloatVect2 *p1, struct FloatVect2 *p2);
extern void Rotate_vect2(struct FloatVect2 *vo, struct _s_matrix22 *R, struct FloatVect2 *vi);
extern void Matrix22_set_i(struct _s_matrix22 *mo);
extern void Matrix22_copy(struct _s_matrix22 *mo, struct _s_matrix22 *ma);
extern void Matrix22_mult(struct _s_matrix22 *mo, struct _s_matrix22 *ma, struct _s_matrix22 *mb);
extern void Matrix22_trans(struct _s_matrix22 *mo, struct _s_matrix22 *ma);

#endif /* SW_AIRBORNE_MATH_DIM2_ALGEBRA_H_ */
