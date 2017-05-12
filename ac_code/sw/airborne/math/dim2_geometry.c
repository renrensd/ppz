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

#include "mcu_periph/sys_time.h"
#include "generated/flight_plan.h"
#include "subsystems/navigation/waypoints.h"

#define TOLERANCE	(1e-6)

#define MAX_BOUNDARY_VERTICES_NUM	(20)
#define MAX_OBSTACLES_NUM					(5)

/*
 * TEST_FUNCTION
 */

#define P_NUM	(8)
#define O_NUM	(5)

struct FloatVect2 home = {0,0};
struct FloatVect2 P_array[P_NUM];
struct FloatVect2 O_array[O_NUM];
struct FloatVect2 valid_area_boundary_vertices_array[P_NUM + 1];
struct _s_polygon P_polygon;
struct _s_polygon test_valid_area;
float time_elapse;

void dim2_geometry_test(void)
{
	for(int i=0;i<P_NUM;++i)
	{
		VECT2_COPY(P_array[i], waypoints[WP_P1+i].enu_f);
	}
	for (int i = 0; i < O_NUM; ++i)
	{
		VECT2_COPY(O_array[i], waypoints[WP_O1+i].enu_f);
	}

	VECT2_COPY(home, waypoints[WP_HOME].enu_f);

	polygon_init(&P_polygon, P_array, P_NUM);
	polygon_init(&test_valid_area, valid_area_boundary_vertices_array, 0);
	generate_valid_area(&test_valid_area, &P_polygon, &home);

	while(1)
	{
		/*
		time_elapse = get_sys_time_float();
		struct FloatVect2 a;
		struct FloatVect2 b;
		float angle,ccw,cw;
		VECT2_DIFF(a, P_array[1], P_array[0]);
		VECT2_DIFF(b, P_array[2], P_array[0]);
		angle = vector_angle(&a, &b, FALSE);
		float_vect2_normalize(&a);
		float_vect2_normalize(&b);
		angle = vector_angle(&a, &b, TRUE);
		ccw = CCW_angle(angle);
		cw = CW_angle(angle);
*/
		//polygon_area(&P_polygon);
		/*
		for(int i=0;i<1000;++i)
		{
			is_line_in_polygon(&(O_array[0]), &(O_array[1]), &test_valid_area);
		}*/
		time_elapse = get_sys_time_float() - time_elapse;
	}
}

int polygon_init(struct _s_polygon *polygon, struct FloatVect2 *vertices, uint8_t num)
{
	if ((polygon == NULL) || (vertices == NULL) )
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

bool_t is_relation_collineation(enum _e_segment_relation relation)
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
		if (is_relation_collineation(rel))
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

float vector_angle(struct FloatVect2 *v0, struct FloatVect2 *v1, bool_t normalized)
{
	float dot, cross;
	struct FloatVect2 a = *v0;
	struct FloatVect2 b = *v1;

	if (!normalized)
	{
		float_vect2_normalize(&a);
		float_vect2_normalize(&b);
	}
	if (VECT2_IS_EQUAL(a, b))
	{
		return 0;
	}

	dot = VECT2_DOT_PRODUCT(a, b);
	cross = VECT2_CROSS_PRODUCT(a, b);
	Bound(dot, -1, 1);
	if (cross < 0)
	{
		return -acosf(dot);
	}
	else
	{
		return acosf(dot);
	}
}

float polygon_area(struct _s_polygon *polygon)
{
	uint8_t i, j;
	float area = 0;

	if ((polygon == NULL) || (polygon->v == NULL) || (polygon->n < 3))
	{
		return 0;
	}

	for (i = 0; i < polygon->n; ++i)
	{
		j = (i + 1) % polygon->n;

		area += (polygon->v[i].x - polygon->v[j].x) * (polygon->v[i].y + polygon->v[j].y) * 0.5f;
	}

	return area;
}

float CCW_angle(float angle)
{
	if (angle < 0)
	{
		return angle + 2.0f * M_PI;
	}
	else
	{
		return angle;
	}
}

float CW_angle(float angle)
{
	if (angle > 0)
	{
		return 2.0f * M_PI - angle;
	}
	else
	{
		return -angle;
	}
}

bool_t is_corner_concave(struct _s_polygon *polygon, uint8_t corner)
{
	return FALSE;
}

/*
 * TODO: this function need to move to appication layer
 *
 *	WARNNING!!!!: make sure valid_area array have enough space for insert 1 point
 *	WARNNING!!!!: make sure spray_area->n array and land_point are global variable
 *	WARNNING!!!!: make sure valid_area->v was correctly initialized
 */

static uint8_t get_index(int8_t base, int8_t delta, uint8_t max)
{
	uint8_t index = (base + delta) % max;
	Bound(index, 0, max - 1);
	return index;
}

static uint8_t get_delta(uint8_t st, uint8_t end, uint8_t max)
{
	if(end >= st)
	{
		return end - st + 1;
	}
	else
	{
		return max - end + st + 1;
	}
}


int generate_valid_area(struct _s_polygon *valid_area, struct _s_polygon *spray_area, struct FloatVect2 *land_point)
{
	struct FloatVect2 v_st;
	struct FloatVect2 v_end;
	struct FloatVect2 center;
	uint8_t i, j, k;
	float angle;
	float max_angle;
	uint8_t index_st;
	uint8_t index_end;
	bool_t spray_vertex_dir;

	struct _s_polygon concave;
	bool_t concave_start = FALSE;
	bool_t concave_corner_flag[MAX_BOUNDARY_VERTICES_NUM];
	uint8_t concave_corner_edge[MAX_BOUNDARY_VERTICES_NUM];	// rising: 1    falling: 2
	uint8_t concave_corner_num = 0;
	struct FloatVect2 concave_v[MAX_BOUNDARY_VERTICES_NUM];
	bool_t in_concave = FALSE;

	if ((valid_area == NULL) || (spray_area == NULL) || (spray_area->v == NULL) || (land_point == NULL)
			|| (spray_area->n < 2))
	{
		return -1;
	}

	// land_point in spray_area
	if (is_point_in_polygon(land_point, spray_area))
	{
		for (i = 0; i < spray_area->n; ++i)
		{
			valid_area->v[i] = spray_area->v[i];
		}
		valid_area->n = spray_area->n;

		goto enlarge;
	}

	// polygon vertices direction
	spray_vertex_dir = (polygon_area(spray_area) > 0);

	// check concave
	// TODO: multiple concave corner not in consideration
	for (i = 0; i < spray_area->n; ++i)
	{
		concave_corner_flag[i] = FALSE;
		concave_corner_edge[i] = 0;
	}

	for (j = 0; j < spray_area->n; ++j)
	{
		i = get_index(j,-1,spray_area->n);
		k = get_index(j,1,spray_area->n);
		VECT2_DIFF(v_st, spray_area->v[i], spray_area->v[j]);
		VECT2_DIFF(v_end, spray_area->v[k], spray_area->v[j]);

		if (spray_vertex_dir)
		{
			angle = CW_angle(vector_angle(&v_st, &v_end, FALSE));
		}
		else
		{
			angle = CCW_angle(vector_angle(&v_st, &v_end, FALSE));
		}
		if (angle > M_PI)
		{
			concave_corner_flag[j] = TRUE;
			++concave_corner_num;

			//check point in concave
			/*
			struct _s_polygon concave;
			struct FloatVect2 concave_v[3];
			concave_v[0] = spray_area->v[i];
			concave_v[1] = spray_area->v[j];
			concave_v[2] = spray_area->v[k];
			polygon_init(&concave, concave_v, 3);
			if (is_point_in_polygon(land_point, &concave))
			{
				in_concave = TRUE;
				index_st = k;
				index_end = i;
				break;
			}
			*/
		}
	}

	for (j = 0; j < spray_area->n; ++j)
	{
		i = get_index(j, -1, spray_area->n);
		k = get_index(j, 1, spray_area->n);
		if ((concave_corner_flag[i] == FALSE) && (concave_corner_flag[j] == TRUE))
		{
			concave_corner_edge[j] = 1;
		}
		else if ((concave_corner_flag[i] == TRUE) && (concave_corner_flag[j] == FALSE))
		{
			concave_corner_edge[j] = 2;
		}
		else
		{
			concave_corner_edge[j] = 0;
		}
	}

	for (j = 0; j < spray_area->n; ++j)
	{
		if (concave_start)
		{
			if (concave_corner_edge[j] == 2)
			{
				index_end = j;

				get_delta

				concave_v[0] = spray_area->v[i];
				concave_v[1] = spray_area->v[j];
				concave_v[2] = spray_area->v[k];
				polygon_init(&concave, concave_v, 3);
				if (is_point_in_polygon(land_point, &concave))
				{
					in_concave = TRUE;
				}
			}
			else
			{
				if (concave_corner_edge[j] == 1)
				{
					index_st = get_index(j, -1, spray_area->n);
					concave_start = TRUE;
				}
			}
	}

	if (!in_concave)
	{
		// first search (CCW)
		VECT2_DIFF(v_st, spray_area->v[0], *land_point);
		float_vect2_normalize(&v_st);
		max_angle = 0;
		index_st = 0;

		for (i = 1; i < spray_area->n; ++i)
		{
			VECT2_DIFF(v_end, spray_area->v[i], *land_point);
			float_vect2_normalize(&v_end);
			angle = CCW_angle(vector_angle(&v_st, &v_end, TRUE));
			if ((angle > max_angle) && (angle < M_PI))
			{
				max_angle = angle;
				index_st = i;
			}
		}

		// second search (CW)
		VECT2_DIFF(v_st, spray_area->v[index_st], *land_point);
		float_vect2_normalize(&v_st);
		max_angle = 0;
		index_end = 0;

		for (i = 0; i < spray_area->n; ++i)
		{
			if (i == index_st)
			{
				continue;
			}
			VECT2_DIFF(v_end, spray_area->v[i], *land_point);
			float_vect2_normalize(&v_end);
			angle = CW_angle(vector_angle(&v_st, &v_end, TRUE));
			if ((angle > max_angle) && (angle < M_PI))
			{
				max_angle = angle;
				index_end = i;
			}
		}

		// check include
		for (i = 0; i < spray_area->n; ++i)
		{
			if ((i == index_st) || (i == index_end))
			{
				continue;
			}
			VECT2_DIFF(v_end, spray_area->v[i], *land_point);
			float_vect2_normalize(&v_end);
			angle = CW_angle(vector_angle(&v_st, &v_end, TRUE));
			if (angle > max_angle)
			{
				return -2;
			}
		}

		if (spray_vertex_dir)
		{
			i = index_end;
			index_end = index_st;
			index_st = i;
		}
	}
	else
	{
		//return -3;
	}

	// add vertex to valid_area
	valid_area->n = 0;
	for (i = 0; i < spray_area->n; ++i)
	{
		j = (index_st + i) % spray_area->n;
		valid_area->v[i] = spray_area->v[j];
		++valid_area->n;
		if (j == index_end)
		{
			valid_area->v[i + 1] = *land_point;
			++valid_area->n;
			break;
		}
	}

	enlarge:
	// find a center point
	VECT2_ASSIGN(center, 0, 0);
	for (i = 0; i < valid_area->n; ++i)
	{
		VECT2_SUM(center, center, valid_area->v[i]);
	}
	VECT2_SDIV(center, center, (float )valid_area->n);

	// enlarge each vertex
	struct FloatVect2 ray;
	for (i = 0; i < valid_area->n; ++i)
	{
		VECT2_DIFF(ray, valid_area->v[i], center);
		float_vect2_normalize(&ray);
		VECT2_SMUL(ray, ray, 2);
		VECT2_SUM(valid_area->v[i], valid_area->v[i], ray);
	}

	return 0;
}

