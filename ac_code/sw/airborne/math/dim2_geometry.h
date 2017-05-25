/*
 * dim2_geometry.h
 *
 *  Created on: Apr 24, 2017
 *      Author: jay
 */

#ifndef SW_AIRBORNE_MATH_DIM2_GEOMETRY_H_
#define SW_AIRBORNE_MATH_DIM2_GEOMETRY_H_

#include "math/pprz_algebra_float.h"

enum _e_segment_relation
{
	SR_INTERSECTION_INSIDE = 0,
	SR_INTERSECTION_OUTSIDE,
	SR_PARALLEL,
	SR_COLLINEATION_SEPARATION,
	SR_COLLINEATION_INSIDE,
	SR_COLLINEATION_CONTAIN,
	SR_COLLINEATION_OVERLAP
};

/*
 * n: vertices number
 * v: vertices array pointer
 */
struct _s_polygon
{
	uint8_t n;
	bool_t dir;
	bool_t concave;
	struct FloatVect2 *v;
};

/*
 * circle with center P and radius r
 */
struct _s_circle
{
	float r;
	struct FloatVect2 P;
};

extern bool_t is_corner_concave(struct _s_polygon *polygon, uint8_t corner);
extern float point_close_2_segment(struct FloatVect2 *P, struct FloatVect2 *v0, struct FloatVect2 *v1);
extern float vector_angle(struct FloatVect2 *v0, struct FloatVect2 *v1, bool_t normalized);
extern float CCW_angle(float angle);
extern float CW_angle(float angle);
extern void dim2_geometry_test(void);
extern void dim2_geometry_update_flightplan(void);
extern int polygon_init(struct _s_polygon *polygon, struct FloatVect2 *vertices, uint8_t num);
extern float polygon_area(struct _s_polygon *polygon);
extern bool_t is_relation_collineation(enum _e_segment_relation relation);
extern enum _e_segment_relation get_2_segments_relation(struct FloatVect2 *v0, struct FloatVect2 *v1,
		struct FloatVect2 *v2, struct FloatVect2 *v3);
extern bool_t is_hray_intersect_with_edge(struct FloatVect2 *P, struct FloatVect2 *v0, struct FloatVect2 *v1);
extern bool_t is_point_in_polygon(struct FloatVect2 *P, struct _s_polygon *polygon);
extern bool_t is_line_in_polygon(struct FloatVect2 *v0, struct FloatVect2 *v1, struct _s_polygon *polygon);
extern int generate_valid_area(struct _s_polygon *valid_area, struct _s_polygon *spray_area, struct FloatVect2 *land_point);

#endif /* SW_AIRBORNE_MATH_DIM2_GEOMETRY_H_ */
