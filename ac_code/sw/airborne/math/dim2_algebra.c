/*
 * dim2_algebra.c
 *
 *  Created on: Apr 24, 2017
 *      Author: jay
 */

#include "dim2_algebra.h"

float determinant(float a, float b, float c, float d)
{
	return a * d - b * c;
}

float point2_distance(struct FloatVect2 *p1, struct FloatVect2 *p2)
{
	struct FloatVect2 s;
	VECT2_DIFF(s, *p1, *p2);
	return float_vect2_norm(&s);
}

void Rotate_vect2(struct FloatVect2 *vo, struct _s_matrix22 *R, struct FloatVect2 *vi)
{
	struct FloatVect2 vt;

	vt.x = vi->x * R->m11 + vi->y * R->m12;
	vt.y = vi->x * R->m21 + vi->y * R->m22;

	VECT2_COPY(*vo, vt);
}

void Matrix22_set_i(struct _s_matrix22 *mo)
{
	mo->m11 = 1; mo->m12 = 0;
	mo->m21 = 0; mo->m22 = 1;
}

void Matrix22_copy(struct _s_matrix22 *mo, struct _s_matrix22 *ma)
{
	mo->m11 = ma->m11;
	mo->m12 = ma->m12;
	mo->m21 = ma->m21;
	mo->m22 = ma->m22;
}

void Matrix22_mult(struct _s_matrix22 *mo, struct _s_matrix22 *ma, struct _s_matrix22 *mb)
{
	struct _s_matrix22 mt;

	mt.m11 = ma->m11 * mb->m11 + ma->m12 * mb->m21;
	mt.m21 = ma->m21 * mb->m11 + ma->m22 * mb->m21;
	mt.m12 = ma->m11 * mb->m12 + ma->m12 * mb->m22;
	mt.m22 = ma->m21 * mb->m12 + ma->m22 * mb->m22;

	Matrix22_copy(mo, &mt);
}

void Matrix22_trans(struct _s_matrix22 *mo, struct _s_matrix22 *ma)
{
	struct _s_matrix22 mt;

	mt.m11 = ma->m11; mt.m12 = ma->m21;
	mt.m21 = ma->m12; mt.m22 = ma->m22;

	Matrix22_copy(mo, &mt);
}
