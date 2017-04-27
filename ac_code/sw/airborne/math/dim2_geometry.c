/*
 * dim2_geometry.c
 *
 *  Created on: Apr 24, 2017
 *      Author: jay
 */

#include "math/dim2_geometry.h"
#include "math/dim2_algebra.h"
#include "std.h"
#include <math.h>
#include "pprz_algebra.h"

#define TOLERANCE	(1e-6)

/*
 * TEST_FUNCTION
 */
struct FloatVect2 a={0,0};
struct FloatVect2 b={1,4};
struct FloatVect2 c={1,-3};
struct FloatVect2 d={2.4,0};

struct FloatVect2 e={1.234,3.456};
struct FloatVect2 f={0,0};

/*
<waypoint name="HOME" x="-4.7" y="3.8"/>
<waypoint name="CLIMB" x="-9.5" y="1.9"/>
<waypoint name="STDBY" x="19.2" y="-27.0"/>
<waypoint name="v0" x="11.5" y="2.0"/>
<waypoint name="v1" x="17.8" y="8.2"/>
<waypoint name="v2" x="27.2" y="5.6"/>
<waypoint name="v3" x="25.7" y="1.9"/>
<waypoint name="v4" x="27.7" y="-4.7"/>
<waypoint name="v5" x="35.5" y="-4.4"/>
<waypoint name="v6" x="35.4" y="-10.6"/>
<waypoint name="v7" x="34.0" y="-16.7"/>
<waypoint name="v8" x="18.1" y="-20.1"/>
<waypoint name="v9" x="7.4" y="-17.9"/>
<waypoint name="v10" x="9.1" y="-12.0"/>
<waypoint name="v11" x="3.8" y="-7.9"/>
<waypoint name="v12" x="4.9" y="-1.1"/>
*/
struct FloatVect2 Point = {12.5,5.3};
struct FloatVect2 land_p = {19.2,-27};
struct FloatVect2 test_vertices[20] =
{
		{11.5,2.0},
		{17.8,8.2},
		{27.2,5.6},
		{25.7,1.9},
		{27.7,-4.7},
		{35.5,-4.4},
		{35.4,-10.6},
		{34.0,-16.7},
		{18.1,-20.1},
		{7.4,-17.9},
		{9.1,-12},
		{3.8,-7.9},
		{4.9,-1.1}
};
struct _s_polygon test_poly;

void dim2_geometry_test(void)
{
	polygon_init(&test_poly, test_vertices, 13);

	while(1)
	{
		get_2_segments_relation(&a,&b,&c,&d);
		is_hray_intersect_with_edge(&a, &b, &c);
		is_point_in_polygon(&Point, &test_poly);
	}
	f = e;
}

int polygon_init(struct _s_polygon *polygon, struct FloatVect2 *vertices, uint8_t num)
{
	if ((polygon == NULL) || (vertices == NULL) || (num < 2))
	{
		return -1;
	}

	polygon->n = num;
	polygon->v = vertices;

	return 0;
}

/*
 * check if v1 on segment v0-v2
 * WARNNING: v0 v1 v2 must collinear
 */
static bool_t is_on_segment(struct FloatVect2 *v0, struct FloatVect2 *v1, struct FloatVect2 *v2)
{
	if ((v1->x <= Max(v0->x, v2->x)) && (v1->x >= Min(v0->x, v2->x)) &&
			(v1->y <= Max(v0->y, v2->y)) && (v1->y >= Min(v0->y, v2->y)))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool_t is_relation_collinetion(enum _e_segment_relation relation)
{
	if ((relation == SR_COLLINEATION_SEPARATION) ||
			(relation == SR_COLLINEATION_INSIDE) ||
			(relation == SR_COLLINEATION_CONTAIN) ||
			(relation == SR_COLLINEATION_OVERLAP))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*
 * check 2 line relation
 * v0-v1 : line 1
 * v2-v3 : line 2
 */
enum _e_segment_relation get_2_segments_relation(struct FloatVect2 *v0, struct FloatVect2 *v1,
		struct FloatVect2 *v2, struct FloatVect2 *v3)
{
	float detL1 = determinant(v0->x, v0->y, v1->x, v1->y);
	float detL2 = determinant(v2->x, v2->y, v3->x, v3->y);
	float x0_x1 = v0->x - v1->x;
	float x2_x3 = v2->x - v3->x;
	float y0_y1 = v0->y - v1->y;
	float y2_y3 = v2->y - v3->y;

	float xnum = determinant(detL1, x0_x1, detL2, x2_x3);
	float ynum = determinant(detL1, y0_y1, detL2, y2_y3);
	float denom = determinant(x0_x1, y0_y1, x2_x3, y2_y3);

	if (fabsf(denom) < TOLERANCE)	// parallel
	{
		float area = v0->x * (v1->y - v2->y) + v1->x * (v2->y - v0->y) + v2->x * (v0->y - v1->y);
		if (fabsf(area) < TOLERANCE)
		{
			if (is_on_segment(v2, v0, v3))
			{
				if (is_on_segment(v2, v1, v3))
				{
					return SR_COLLINEATION_INSIDE;
				}
				else
				{
					return SR_COLLINEATION_OVERLAP;
				}
			}
			else
			{
				if (is_on_segment(v2, v1, v3))
				{
					return SR_COLLINEATION_OVERLAP;
				}
				else
				{
					if(is_on_segment(v0, v2, v1))
					{
						return SR_COLLINEATION_CONTAIN;
					}
					else
					{
						return SR_COLLINEATION_SEPARATION;
					}
				}
			}
		}
		else
		{
			return SR_PARALLEL;
		}
	}
	else
	{
		struct FloatVect2 P;
		P.x = xnum / denom;
		P.y = ynum / denom;
		if (is_on_segment(v0, &P, v1) && is_on_segment(v2, &P, v3))
		{
			return SR_INTERSECTION_INSIDE;
		}
		else
		{
			return SR_INTERSECTION_OUTSIDE;
		}
	}
}

/*
 * check if horizon ray start from p intersect with edge
 * P : through point
 * v0-v1 : edge
 */
bool_t is_hray_intersect_with_edge(struct FloatVect2 *P, struct FloatVect2 *v0, struct FloatVect2 *v1)
{
	struct FloatVect2 *A, *B;
	struct FloatVect2 AB, AP;
	if (v0->y < v1->y)
	{
		A = v0;
		B = v1;
	}
	else
	{
		A = v1;
		B = v0;
	}

	if ((P->y < A->y) || (P->y > B->y))
	{
		return FALSE;
	}

	if (P->x > Max(A->x, B->x))
	{
		return FALSE;
	}

	if (P->x < Min(A->x, B->x))
	{
		return TRUE;
	}

	VECT2_DIFF(AB, *B, *A);
	VECT2_DIFF(AP, *P, *A);
	float_vect2_normalize(&AB);
	float_vect2_normalize(&AP);
	float cross = VECT2_CROSS_PRODUCT(AB, AP);
	float theta = asinf(cross);
	if(theta > 0)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*
 * check if point in polygon
 */
bool_t is_point_in_polygon(struct FloatVect2 *P, struct _s_polygon *polygon)
{
	float min_x, min_y, max_x, max_y;
	uint8_t i, j;
	uint8_t intersect = 0;

	if ((P == NULL) || (polygon == NULL) || (polygon->n < 2))
	{
		return FALSE;
	}

	min_x = max_x = polygon->v[0].x;
	min_y = max_y = polygon->v[1].y;

	for (i = 0; i < polygon->n; ++i)
	{
		if (polygon->v[i].x > max_x)
		{
			max_x = polygon->v[i].x;
		}
		if (polygon->v[i].x < min_x)
		{
			min_x = polygon->v[i].x;
		}
		if (polygon->v[i].y > max_y)
		{
			max_y = polygon->v[i].y;
		}
		if (polygon->v[i].y < min_y)
		{
			min_y = polygon->v[i].y;
		}
	}

	// point outside bound box
	if ((P->x < min_x) || (P->x > max_x) || (P->y < min_y) || (P->y > max_y))
	{
		return FALSE;
	}

	for (i = 0; i < polygon->n; ++i)
	{
		j = (i + 1) % polygon->n;

		// point on edge
		enum _e_segment_relation rel = get_2_segments_relation(P, &(polygon->v[i]), P, &(polygon->v[j]));
		if (is_relation_collinetion(rel))
		{
			if (is_on_segment(&(polygon->v[i]), P, &(polygon->v[j])))
			{
				return TRUE;
			}
		}

		if (is_hray_intersect_with_edge(P, &(polygon->v[i]), &(polygon->v[j])))
		{
			++intersect;
		}
	}
	if (intersect % 2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*
 * check if line in polygon
 */
bool_t is_line_in_polygon(struct FloatVect2 *v0, struct FloatVect2 *v1, struct _s_polygon *polygon)
{
	uint8_t i, j;

	if (!is_point_in_polygon(v0, polygon))
	{
		return FALSE;
	}
	if (!is_point_in_polygon(v1, polygon))
	{
		return FALSE;
	}
	for (i = 0; i < polygon->n; ++i)
	{
		j = (i + 1) % polygon->n;

		enum _e_segment_relation rel = get_2_segments_relation(v0, v1, &(polygon->v[i]), &(polygon->v[j]));

		if((rel == SR_INTERSECTION_INSIDE) ||
				(rel == SR_COLLINEATION_CONTAIN) ||
				(rel == SR_COLLINEATION_OVERLAP))
		{
			return FALSE;
		}
	}

	return TRUE;
}

static uint8_t find_index(int16_t base, int16_t delta, int16_t max)
{
	int16_t index;

	index = base + delta;

	while (index > max)
		index -= max;
	while (index < 0)
		index += max;

	Bound(index, 0, max);
	return index;
}

/*
 * TODO: this function need to move to appication layer
 *
 *	WARNNING!!!!: make sure spray_boundary array have enough space for insert 1 point
 *	WARNNING!!!!: make sure spray_boundary array and land_point are global variable
 *	WARNNING!!!!: make sure vaid_area->v was correctly initialized
 */
int generate_valid_area(struct _s_polygon *vaid_area, struct FloatVect2 *spray_boundary, uint8_t num,
		struct FloatVect2 *land_point)
{
	uint8_t i;
	uint8_t min_v0;
	uint8_t min_v1;
	uint8_t remain;
	float min_dist;
	float dist;

	if ((vaid_area == NULL) || (spray_boundary == NULL) || (land_point == NULL) || (num < 2))
	{
		return -1;
	}

	// get min distance vertex between land_point
	min_dist = point2_distance(land_point, &(spray_boundary[0]));
	min_v0 = 0;

	for (i = 0; i < num; ++i)
	{
		dist = point2_distance(land_point, &(spray_boundary[i]));
		if (dist < min_dist)
		{
			min_v0 = i;
		}
	}

	// get second min distance vertex between land_point
	for (i = 0; i < num; ++i)
	{
		if (i != min_v0)
		{
			min_dist = point2_distance(land_point, &(spray_boundary[i]));
			min_v1 = i;
			break;
		}
	}

	for (i = 0; i < num; ++i)
	{
		if (i == min_v0)
		{
			continue;
		}
		dist = point2_distance(land_point, &(spray_boundary[i]));
		if (dist < min_dist)
		{
			min_v1 = i;
		}
	}

	if (min_v0 == min_v1)
	{
		return -2;
	}
	else if (min_v0 > min_v1)
	{
		i = min_v0;
		min_v0 = min_v1;
		min_v1 = i;
	}

	// use min_v0 as first vertex
	vaid_area->v[0] = spray_boundary[min_v0];
	// use land_point as second vertex
	vaid_area->v[1] = *(land_point);
	// use min_v1 as third vertex
	vaid_area->v[2] = spray_boundary[min_v1];
	// add remain vertex in spray_boundary
	remain = num - ((min_v1 - min_v0 - 1) + 2);
	for (i = 0; i <= remain; ++i)
	{
		vaid_area->v[i + 3] = spray_boundary[find_index(min_v1, i, num)];
	}
	vaid_area->n = remain + 3;

	return 0;
}
