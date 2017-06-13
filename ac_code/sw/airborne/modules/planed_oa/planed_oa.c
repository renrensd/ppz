#include "std.h"
#include <math.h>
#include <string.h>
#include "math/pprz_geodetic_float.h"
#include "mcu_periph/sys_time.h"
#include "math/dim2_geometry.h"
#include "math/pprz_algebra.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/ins/ins_int.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "modules/planed_oa/planed_oa.h"
#include "generated/flight_plan.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/mission/task_manage.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/eng/eng_app.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#define Sign(_x) ((_x) > 0 ? 1 : (-1))
#define Bound_max(_x, _max) {if(_x >= _max) _x = _max;}
#define Bound_min(_x, _min) {if(_x <= _min) _x = _min;}

#define ANGLE_RAD     (M_PI/180.0f)   //PI/180.0
#define RAD_ANGLE     (180.0f/M_PI)   //180.0/PI

#define OA_SAFE_DIST    2.0
#define V_MAX_DIST_F    OA_SAFE_DIST * 2

#define OA_MAX_SPEED 2.0
#define OA_MAX_ACCEL 1.5

#define MIN_LENGTH 0.000001

#define MAX_SELECT_LENGTH 10000.0f
#define MIN_SELECT_LENGTH -1.0f
#define VERTICAL_STEP_LENGTH 1.0f
#define VERTICAL_MAX_SEARCH_NUM 50

#define HORIZONTAL_STEP_LENGTH 1.0f
#define HORIZONTAL_MAX_SEARCH_NUM 20

#define MAX_SEARCH_TIME 10000000   //us  =20s

#define PROJ_NUM   (OA_MAX_OBSTACLES_NUM * OA_OBSTACLE_CORNER_NUM)

//use oa debug by set 1
#define USE_OA_DEBUG 0

enum Obstacle_Avoidance_Wp_Search_State oa_wp_search_state;
enum Obstacle_Avoidance_Wp_Search_Direction oa_wp_search_direction;
enum Obstacle_Avoidance_State planed_oa_state;
enum Insert_State insert_state;

struct _s_planed_oa planed_oa;

float start_backward_search_dist;
float end_backward_search_dist;
float vertical_search_dist;
float horizontal_search_dist;
float vert_search_direction;
int8_t horizontal_step_num;
int8_t vertical_step_num;
float start_max_dist;
float end_max_dist;
float start_horizontal_step_length;
float end_horizontal_step_length;

float v_dist[OA_MAX_OBSTACLES_NUM];
bool_t o_flag[OA_MAX_OBSTACLES_NUM];
bool_t spray_boundary_vaild_flag[OA_MAX_BOUNDARY_VERTICES_NUM];
bool_t error_spray[OA_MAX_BOUNDARY_VERTICES_NUM * 2];
float error_spray_cord[OA_MAX_BOUNDARY_VERTICES_NUM * 2];
float insert_start_end_cord[4];
bool_t direction_flag;

static int direction_i;
static int horizontal_i;
static int vertical_i;
static int oa_line_i;

static uint8_t spray_i;
static uint8_t insert_i;

uint32_t start_search_time;
uint32_t run_time[10];

static void update_obstacles_info(void);
static void planed_oa_run(void);
static void planed_oa_geometry_prepare(void);
static void creat_a_fake_oa_line(void);
static void update_vaild_spray_edge(void);

static float vector_angle_calc(struct FloatVect2 *vect_a, struct FloatVect2 *vect_b);
static float points_length_calc(struct FloatVect2 *point1, struct FloatVect2 *point2);
static float vector_length_calc(struct FloatVect2 *vect);
static void get_project_point(struct FloatVect2 *proj_point, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp,
		struct FloatVect2 *point);
static uint8_t min_dist_select(float *dist, uint8_t n);
static void get_oa_search_direction(void);
static void oa_wp_generation(void);
static void guidance_h_trajectory_tracking_brake_setting_release(void);
static bool_t achieve_next_oa_wp(void);
static void get_task_wp(void);
static void get_oa_from_next_wp(void);
static void send_point_to_pprz(void);
static bool_t get_initial_start_end_oa_wp(struct FloatVect2 *init_start_oa_wp, struct FloatVect2 *init_end_oa_wp,
		struct _s_planed_obstacle *obstacles, uint8_t o_num, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp);
static void get_s1_s4_wp(struct FloatVect2 *start_oa_wp, struct FloatVect2 *end_oa_wp, struct FloatVect2 *init_start_oa_wp,
		struct FloatVect2 *init_end_oa_wp, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp);
static bool_t rectangle_obstacle_on_oa_route(struct _s_planed_obstacle *obstacles, uint8_t o_num,
		struct FloatVect2 *start_wp, struct FloatVect2 *end_wp);
static void planed_oa_test(void);

static void planed_oa_debug_prepare(void)
{
	VECT2_ASSIGN(oa_data.spray_boundary_vertices_array[0], -19.566406, -5.726562);
	VECT2_ASSIGN(oa_data.spray_boundary_vertices_array[1], -34.109375, 19.839844);
	VECT2_ASSIGN(oa_data.spray_boundary_vertices_array[2], -13.4375, 32.527344);
	VECT2_ASSIGN(oa_data.spray_boundary_vertices_array[3], 1.9375, 7.207031);
	oa_data.spray_boundary_vertices_num = 4;

	VECT2_ASSIGN(oa_data.home, 0.128906, 0.316406);

	VECT2_ASSIGN(planed_oa.from_wp, 0.128906, 0.316406);
	VECT2_ASSIGN(planed_oa.next_wp, -1.25, 7.207031);
}

static void planed_oa_test(void)
{
	static bool_t ini = FALSE;

	if (!ini)
	{
		ini = TRUE;

		planed_oa_debug_prepare();
		planed_oa_geometry_prepare();
		send_point_to_pprz();
		waypoint_set_vect2(WP_From, &planed_oa.from_wp);
		waypoint_set_vect2(WP_Next, &planed_oa.next_wp);
		waypoint_set_vect2(WP_HOME, &oa_data.home);

		/*
		for (uint8_t i = 0; i < OA_MAX_BOUNDARY_VERTICES_NUM; ++i)
		{
			VECT2_COPY(oa_data.spray_boundary_vertices_array[i], waypoints[WP_V0 + i].enu_f);
		}
		oa_data.spray_boundary_vertices_num = OA_MAX_BOUNDARY_VERTICES_NUM;

		for (uint8_t i = 0; i < OA_MAX_OBSTACLES_NUM * OA_OBSTACLE_CORNER_NUM; ++i)
		{
			VECT2_COPY(oa_data.obstacles_vertices_array[i / 4][i % 4], waypoints[WP_O11 + i].enu_f);
		}
		oa_data.obstacles_num = OA_MAX_OBSTACLES_NUM;

		VECT2_COPY(oa_data.home, waypoints[WP_HOME].enu_f);

		planed_oa_geometry_prepare();

		send_point_to_pprz();
		*/
	}

	VECT2_COPY(planed_oa.from_wp, waypoints[WP_From].enu_f);
	VECT2_COPY(planed_oa.next_wp, waypoints[WP_Next].enu_f);

	struct FloatVect2 P;
	if (is_line_in_polygon(&planed_oa.from_wp, &planed_oa.next_wp, &(planed_oa.flight_area)))
	{
		VECT2_ASSIGN(P, 0, 0);
	}
	else
	{
		VECT2_ASSIGN(P, 10, 10);
	}
	waypoint_set_vect2(WP_ERROR_N, &P);
}

/*
 * Initialize oa data when power on
 */
void planed_oa_data_init(void)
{
	planed_oa.test_on = FALSE;
	planed_oa.sp_state = FALSE;

	oa_wp_search_state = not_search;
	oa_wp_search_direction = left_side_first;
	planed_oa_state = release_sp;
	insert_state = no_insert_wp;
	planed_oa.oa_necessity_flag = FALSE;
	planed_oa.wp_move_done_flag = FALSE;
	planed_oa.manual_prepare_flag = FALSE;
	planed_oa.manual_callback_flag = FALSE;

	//new for obstacles_boundary_insert:
	planed_oa.obstacles_boundary_insert_flag = FALSE;
	planed_oa.spray_boundary_insert_flag = FALSE;
	planed_oa.obstacles_boundary_insert_run_flag = FALSE;
	direction_flag = TRUE;

	vertical_step_num = VERTICAL_MAX_SEARCH_NUM;
	horizontal_step_num = HORIZONTAL_MAX_SEARCH_NUM;

	planed_oa.vect_to_north.x = 0.0f;
	planed_oa.vect_to_north.y = 100.0f;

	planed_oa.back_home_ready = FALSE;
	planed_oa.oa_home_flag = TRUE;
	planed_oa.heading_ready = FALSE;

	planed_oa.total_search_time = 0;
	start_search_time = 0;

	direction_i = 0;
	horizontal_i = 0;
	vertical_i = 1;
	oa_line_i = 0;

	spray_i = 0;
	insert_i = 0;

	for (int i = 0; i < OA_MAX_OBSTACLES_NUM; i++)
	{
		planed_oa.obstacles[i].obstacle_flag = FALSE;
		planed_oa.obstacles[i].pos.x = 0.0f;
		planed_oa.obstacles[i].pos.y = 0.0f;
		planed_oa.obstacles[i].raduis = 0.0f;
	}

	memset(spray_boundary_vaild_flag, 0, sizeof(spray_boundary_vaild_flag));
	memset(o_flag, 0, sizeof(o_flag));

	for(int i = 0; i < (OA_MAX_BOUNDARY_VERTICES_NUM * 2); i++)
	{
		error_spray[i] = TRUE;
		error_spray_cord[i] = 0.0f;
	}

	for(int i = 0; i < 4; i++)
	{
		insert_start_end_cord[i] = 0.0f;
	}

	for(int i = 0; i < 10; i++)
	{
		run_time[i] = 0;
	}
}

/*
 * Reset oa data when flight state is not crusising
 */
void planed_oa_data_reset(void)
{
	planed_oa.sp_state = FALSE;

	oa_wp_search_state = not_search;
	oa_wp_search_direction = left_side_first;
	planed_oa_state = release_sp;
	insert_state = no_insert_wp;
	planed_oa.oa_necessity_flag = FALSE;
	planed_oa.wp_move_done_flag = FALSE;
	planed_oa.manual_prepare_flag = FALSE;
	planed_oa.manual_callback_flag = FALSE;

	//new for obstacles_boundary_insert:
	planed_oa.obstacles_boundary_insert_flag = FALSE;
	planed_oa.spray_boundary_insert_flag = FALSE;
	planed_oa.obstacles_boundary_insert_run_flag = FALSE;
	direction_flag = TRUE;

	planed_oa.back_home_ready = FALSE;
	planed_oa.oa_home_flag = TRUE;
	planed_oa.heading_ready = FALSE;

	planed_oa.total_search_time = 0;
	start_search_time = 0;

	direction_i = 0;
	horizontal_i = 0;
	vertical_i = 1;
	oa_line_i = 0;

	spray_i = 0;
	insert_i = 0;

	memset(o_flag, 0, sizeof(o_flag));

	for(int i = 0; i < (OA_MAX_BOUNDARY_VERTICES_NUM * 2); i++)
	{
		error_spray[i] = TRUE;
		error_spray_cord[i] = 0.0f;
	}

	for(int i = 0; i < 4; i++)
	{
		insert_start_end_cord[i] = 0.0f;
	}

	for(int i = 0; i < 10; i++)
	{
		run_time[i] = 0;
	}

	for (int i = 0; i < OA_MAX_OBSTACLES_NUM; i++)
	{
		planed_oa.obstacles[i].obstacle_flag = FALSE;
	}
}

static void planed_oa_geometry_prepare(void)
{
	polygon_init(&planed_oa.spray_area, oa_data.spray_boundary_vertices_array, oa_data.spray_boundary_vertices_num);
	polygon_init(&planed_oa.flight_area, planed_oa.flight_boundary_vertices_array, 0);
	planed_oa.obstacles_num = oa_data.obstacles_num;

	for (int i = 0; i < planed_oa.obstacles_num; ++i)
	{
		polygon_init(&(planed_oa.obstacles[i].polygon), oa_data.obstacles_vertices_array[i], OA_OBSTACLE_CORNER_NUM);
	}

	int area_generate_error = generate_valid_area(&planed_oa.flight_area, &planed_oa.spray_area, &oa_data.home);

	if (area_generate_error != 0)
	{
		if (area_generate_error == -1)
		{
			oa_wp_search_state = area_generate_error_parameter_invild;
		}
		else
		{
			oa_wp_search_state = area_generate_error_cant_gen_area;
		}
	}
	else
	{
		oa_wp_search_state = not_search;
	}
}

/*
 * Calc length of two input points
 */
float points_length_calc(struct FloatVect2 *point1, struct FloatVect2 *point2)
{
	float res;
	struct FloatVect2 vect;
	VECT2_DIFF(vect, *point1, *point2);
	res = sqrtf(VECT2_NORM2(vect));
	Bound_min(res, MIN_LENGTH);

	return res;
}

/*
 * Calc length of input vecter
 */
static float vector_length_calc(struct FloatVect2 *vect)
{
	float res;
	res = sqrtf(VECT2_NORM2(*vect));
	Bound_min(res, MIN_LENGTH);

	return res;
}

/*
 * calc angle(degree) rotate from vect_a to vect_b
 * (retrun postive result if the rotate direction is lockwise)
 */
static float vector_angle_calc(struct FloatVect2 *vect_a, struct FloatVect2 *vect_b)
{
	if ((vector_length_calc(vect_a) == MIN_LENGTH) || (vector_length_calc(vect_b) == MIN_LENGTH))
	{
		return 0.0f;
	}

	float cross = VECT2_CROSS_PRODUCT(*vect_a, *vect_b);
	float res = VECT2_DOT_PRODUCT(*vect_a, *vect_b) / (vector_length_calc(vect_a) * vector_length_calc(vect_b));
	Bound(res, -1.0f, 1.0f);

	if (cross < 0.0f)
	{
		return RAD_ANGLE * acosf(res);
	}
	else if (cross > 0.0f)
	{
		return - RAD_ANGLE * acosf(res);
	}
	else if (res > 0.0f)
	{
		return 0.0f;
	}
	else
	{
		return 180.0f;
	}
}

/*
 * Get project point by two input terminal point on project line and on project point
 */
static void get_project_point(struct FloatVect2 *proj_point, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp,
		struct FloatVect2 *point)
{
	float angle, slope;

	struct FloatVect2 vect_start_to_end_wp;
	VECT2_DIFF(vect_start_to_end_wp, *end_wp, *start_wp);

	angle = vector_angle_calc(&planed_oa.vect_to_north, &vect_start_to_end_wp);

	if (angle == 0.0 || fabs(angle) == 180.0)
	{
		proj_point->x = start_wp->x;
		proj_point->y = point->y;
	}
	else if (fabs(angle) == 90.0)
	{
		proj_point->x = point->x;
		proj_point->y = start_wp->y;
	}
	else
	{
		if (angle > (90.0 - MIN_LENGTH) && angle < (90.0 + MIN_LENGTH))
		{
			angle = 90.0 - MIN_LENGTH;
		}

		if (angle > (-90.0 - MIN_LENGTH) && angle < (-90.0 + MIN_LENGTH))
		{
			angle = -90.0 + MIN_LENGTH;
		}

		slope = tanf(ANGLE_RAD * angle);

		if (slope > (-MIN_LENGTH) && slope < 0.0)
		{
			slope = -MIN_LENGTH;
		}
		else if (slope > 0.0 && slope < MIN_LENGTH)
		{
			slope = MIN_LENGTH;
		}

		proj_point->y = (slope * start_wp->y + point->y / slope + point->x - start_wp->x) / (1.0 / slope + slope);
		proj_point->x = (-1.0 / slope * (proj_point->y - point->y)) + point->x;
	}
}

/*
 * Get min value of dist[]
 */
static uint8_t min_dist_select(float *dist, uint8_t n)
{
	uint8_t min = 0;

	for (uint8_t i = 0; i < n; i++)
	{
		if (dist[min] > dist[i])
		{
			min = i;
		}
	}

	return min;
}

/*
 * Get initial wp (s1 and s4) by inputed start wp, end wp
 */
static bool_t get_initial_start_end_oa_wp(struct FloatVect2 *init_start_oa_wp, struct FloatVect2 *init_end_oa_wp,
		struct _s_planed_obstacle *obstacles, uint8_t o_num, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp)
{
	if (o_num <= 0)
	{
		oa_wp_search_state = search_error_obstacle_flag_wrong;
		return FALSE;
	}

	uint8_t proj_num = o_num * OA_OBSTACLE_CORNER_NUM;
	float proj_start_dist[PROJ_NUM];
	float proj_end_dist[PROJ_NUM];
	struct FloatVect2 project_point[PROJ_NUM];

	bool_t start_flag = FALSE;
	bool_t end_flag = FALSE;

	float start_end_dist = points_length_calc(start_wp, end_wp);

	for (int i = 0; i < o_num; i++)
	{
		if (obstacles[i].obstacle_flag)
		{
			for (int j = 0; j < OA_OBSTACLE_CORNER_NUM; j++)
			{
				get_project_point(&(project_point[i * OA_OBSTACLE_CORNER_NUM + j]), start_wp, end_wp, &(obstacles[i].polygon.v[j]));
				proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] = points_length_calc(&(project_point[i * OA_OBSTACLE_CORNER_NUM + j]), start_wp);
				proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j] = points_length_calc(&(project_point[i * OA_OBSTACLE_CORNER_NUM + j]), end_wp);

				if ((proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] > start_end_dist)
						|| (proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j] > start_end_dist))
				{
					if (proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] < proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j])
					{
						start_flag = TRUE;
					}

					if (proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] > proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j])
					{
						end_flag = TRUE;
					}

					proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] = MAX_SELECT_LENGTH;
					proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j] = MAX_SELECT_LENGTH;
				}
			}
		}
		else
		{
			for (int j = 0; j < OA_OBSTACLE_CORNER_NUM; j++)
			{
				proj_start_dist[i * OA_OBSTACLE_CORNER_NUM + j] = MAX_SELECT_LENGTH;
				proj_end_dist[i * OA_OBSTACLE_CORNER_NUM + j] = MAX_SELECT_LENGTH;
			}
		}
	}

	uint8_t k1 = min_dist_select(proj_start_dist, proj_num);
	uint8_t k4 = min_dist_select(proj_end_dist, proj_num);

	if ((proj_start_dist[k1] == MAX_SELECT_LENGTH) || (proj_end_dist[k4] == MAX_SELECT_LENGTH))
	{
		oa_wp_search_state = search_error_obstacle_flag_wrong;
		return FALSE;
	}

	if (start_flag)
	{
		VECT2_COPY(*init_start_oa_wp, *start_wp);
		start_max_dist = 0.0f;
	}
	else
	{
		VECT2_COPY(*init_start_oa_wp, project_point[k1]);
		start_max_dist = proj_start_dist[k1];
	}

	if (end_flag)
	{
		VECT2_COPY(*init_end_oa_wp, *end_wp);
		end_max_dist = 0.0f;
	}
	else
	{
		VECT2_COPY(*init_end_oa_wp, project_point[k4]);
		end_max_dist = proj_end_dist[k4];
	}

	return TRUE;
}

/*
 * Get nearest wp (s1 and s4) by inputed start wp, end wp
 */
static void get_s1_s4_wp(struct FloatVect2 *start_oa_wp, struct FloatVect2 *end_oa_wp, struct FloatVect2 *init_start_oa_wp,
		struct FloatVect2 *init_end_oa_wp, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp)
{
	float h_step = (float) (1.0f / HORIZONTAL_MAX_SEARCH_NUM);

	start_oa_wp->x = h_step * horizontal_i * (start_wp->x) + (1.0f - h_step * horizontal_i) * (init_start_oa_wp->x);
	start_oa_wp->y = h_step * horizontal_i * (start_wp->y) + (1.0f - h_step * horizontal_i) * (init_start_oa_wp->y);

	end_oa_wp->x = h_step * horizontal_i * (end_wp->x) + (1.0f - h_step * horizontal_i) * (init_end_oa_wp->x);
	end_oa_wp->y = h_step * horizontal_i * (end_wp->y) + (1.0f - h_step * horizontal_i) * (init_end_oa_wp->y);
}

/*
 * Judge the necessity of oa by konwn obstacle[]. 
 * Return TRUE: need oa; FALSE: not need oa; 
 * Get: obstacle[i].obstacle_flag
 */
static bool_t rectangle_obstacle_on_nav_route(struct _s_planed_obstacle *obstacles, uint8_t o_num,
		struct FloatVect2 *start_wp, struct FloatVect2 *end_wp)
{
	for (int i = 0; i < o_num; i++)
	{
		/*if(    get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[0]), &(obstacle[i].v[1]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[1]), &(obstacle[i].v[2]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[2]), &(obstacle[i].v[3]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[3]), &(obstacle[i].v[0]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[0]), &(obstacle[i].v[2]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[1]), &(obstacle[i].v[3]) ) == SR_INTERSECTION_INSIDE )*/
		enum _e_segment_relation rel_12 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[0]), &(obstacles[i].polygon.v[1]));
		enum _e_segment_relation rel_23 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[1]), &(obstacles[i].polygon.v[2]));
		enum _e_segment_relation rel_34 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[2]), &(obstacles[i].polygon.v[3]));
		enum _e_segment_relation rel_41 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[3]), &(obstacles[i].polygon.v[0]));
		enum _e_segment_relation rel_13 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[0]), &(obstacles[i].polygon.v[2]));
		enum _e_segment_relation rel_24 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[1]), &(obstacles[i].polygon.v[3]));

		if (rel_13 == SR_INTERSECTION_INSIDE
				|| rel_24 == SR_INTERSECTION_INSIDE
				|| rel_12 == SR_INTERSECTION_INSIDE
				|| rel_23 == SR_INTERSECTION_INSIDE
				|| rel_34 == SR_INTERSECTION_INSIDE
				|| rel_41 == SR_INTERSECTION_INSIDE)
		{
			obstacles[i].obstacle_flag = TRUE;
		}
		else
		{
			obstacles[i].obstacle_flag = FALSE;
		}
	}

	for (int i = 0; i < o_num; i++)
	{
		if (obstacles[i].obstacle_flag)
		{
			return TRUE;
		}
	}

	return FALSE;
}

/*
 * return TRUE ;
 */
static bool_t rectangle_obstacle_on_oa_route(struct _s_planed_obstacle *obstacles, uint8_t o_num,
		struct FloatVect2 *start_wp, struct FloatVect2 *end_wp)
{
	bool_t obstacle_flag[OA_MAX_OBSTACLES_NUM];

	for (int i = 0; i < o_num; i++)
	{
		/*for(int j=0; j<OA_OBSTACLE_CORNER_NUM; j++)
		 {
		 for(int k=0; k<OA_OBSTACLE_CORNER_NUM; k++)
		 {
		 if(k != j)
		 {
		 enum _e_segment_relation rel = get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[j]), &(obstacle[i].v[k]) );

		 if( rel == SR_INTERSECTION_INSIDE )
		 {
		 return TRUE;
		 }
		 }
		 }
		 }*/

		enum _e_segment_relation rel_12 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[0]), &(obstacles[i].polygon.v[1]));
		enum _e_segment_relation rel_23 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[1]), &(obstacles[i].polygon.v[2]));
		enum _e_segment_relation rel_34 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[2]), &(obstacles[i].polygon.v[3]));
		enum _e_segment_relation rel_41 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[3]), &(obstacles[i].polygon.v[0]));
		enum _e_segment_relation rel_13 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[0]), &(obstacles[i].polygon.v[2]));
		enum _e_segment_relation rel_24 = get_2_segments_strict_relation(start_wp, end_wp,
				&(obstacles[i].polygon.v[1]), &(obstacles[i].polygon.v[3]));

		if (rel_13 == SR_INTERSECTION_INSIDE
				|| rel_24 == SR_INTERSECTION_INSIDE
				|| rel_12 == SR_INTERSECTION_INSIDE
				|| rel_23 == SR_INTERSECTION_INSIDE
				|| rel_34 == SR_INTERSECTION_INSIDE
				|| rel_41 == SR_INTERSECTION_INSIDE)
		{
			return TRUE;
		}

		/*if(    get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[0]), &(obstacle[i].v[1]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[1]), &(obstacle[i].v[2]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[2]), &(obstacle[i].v[3]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[3]), &(obstacle[i].v[0]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[0]), &(obstacle[i].v[2]) ) == SR_INTERSECTION_INSIDE
		 || get_2_segments_strict_relation( start_wp, end_wp, &(obstacle[i].v[1]), &(obstacle[i].v[3]) ) == SR_INTERSECTION_INSIDE )
		 {
		 return TRUE;
		 }*/
	}

	return FALSE;
}

/*
 * Get oa search direction by konwn obstacle[] and nav line;
 */
static void get_oa_search_direction(void)
{
	float left_vert_dist_sum = 0.0f;
	float right_vert_dist_sum = 0.0f;

	for (int i = 0; i < planed_oa.obstacles_num; i++)
	{
		VECT2_DIFF(planed_oa.vect_from_to_o, planed_oa.obstacles[i].pos, planed_oa.from_wp);
		float obstacle_angle = vector_angle_calc(&planed_oa.vect_from_to_next_wp, &planed_oa.vect_from_to_o);

		struct FloatVect2 project_point;
		get_project_point(&project_point, &planed_oa.from_wp, &planed_oa.next_wp, &(planed_oa.obstacles[i].pos));
		v_dist[i] = points_length_calc(&project_point, &(planed_oa.obstacles[i].pos));

		if (obstacle_angle < 0.0f)
		{
			if (v_dist[i] >= planed_oa.obstacles[i].raduis)
			{
				left_vert_dist_sum = left_vert_dist_sum + 2.0f * planed_oa.obstacles[i].raduis;
			}
			else
			{
				left_vert_dist_sum = left_vert_dist_sum + planed_oa.obstacles[i].raduis + v_dist[i];
				right_vert_dist_sum = right_vert_dist_sum + (planed_oa.obstacles[i].raduis - v_dist[i]);
			}
		}
		else
		{

			if (v_dist[i] >= planed_oa.obstacles[i].raduis)
			{
				right_vert_dist_sum = right_vert_dist_sum + 2.0f * planed_oa.obstacles[i].raduis;
			}
			else
			{
				right_vert_dist_sum = right_vert_dist_sum + planed_oa.obstacles[i].raduis + v_dist[i];
				left_vert_dist_sum = left_vert_dist_sum + (planed_oa.obstacles[i].raduis - v_dist[i]);
			}
		}
	}

	if (left_vert_dist_sum < right_vert_dist_sum)
	{
		oa_wp_search_direction = left_side_first;
	}
	else
	{
		oa_wp_search_direction = right_side_first;
	}
}

/*
 * Search, generate and check oa wp:
 */
static void oa_wp_generation(void)
{
	if ( direction_flag )
	{
		planed_oa.search_times = 0;
		get_oa_search_direction();
		direction_flag = FALSE;
	}

	if (oa_wp_search_direction == left_side_first)
	{
		vert_search_direction = 1.0f;
	}
	else
	{
		vert_search_direction = -1.0f;
	}

	if (direction_i < 2)
	{
		if (horizontal_i < HORIZONTAL_MAX_SEARCH_NUM)
		{
			get_s1_s4_wp(&planed_oa.oa_wp.s1, &planed_oa.oa_wp.s4, &planed_oa.init_start_oa_wp, &planed_oa.init_end_oa_wp,
					&planed_oa.from_wp, &planed_oa.next_wp);

			if (vertical_i <= vertical_step_num)
			{
				vertical_search_dist = vertical_i * VERTICAL_STEP_LENGTH;

				planed_oa.oa_wp.s2.x = planed_oa.oa_wp.s1.x
						- vertical_search_dist * Sign(vert_search_direction) * cosf(planed_oa.nav_north_angle * ANGLE_RAD);
				planed_oa.oa_wp.s2.y = planed_oa.oa_wp.s1.y
						+ vertical_search_dist * Sign(vert_search_direction) * sinf(planed_oa.nav_north_angle * ANGLE_RAD);

				planed_oa.oa_wp.s3.x = planed_oa.oa_wp.s4.x
						- vertical_search_dist * Sign(vert_search_direction) * cosf(planed_oa.nav_north_angle * ANGLE_RAD);
				planed_oa.oa_wp.s3.y = planed_oa.oa_wp.s4.y
						+ vertical_search_dist * Sign(vert_search_direction) * sinf(planed_oa.nav_north_angle * ANGLE_RAD);

				oa_wp_search_state = on_search;

				planed_oa.search_times++;

				bool_t oa_line_in_polygon_flag;

				if (oa_line_i < 3)
				{
					if (oa_line_i == 0)
					{
						oa_line_in_polygon_flag = is_line_in_polygon(&planed_oa.oa_wp.s1, &planed_oa.oa_wp.s2, &planed_oa.flight_area);

						if (!rectangle_obstacle_on_oa_route(planed_oa.obstacles, planed_oa.obstacles_num, &planed_oa.oa_wp.s1, &planed_oa.oa_wp.s2)
								&& oa_line_in_polygon_flag)
						{
							oa_line_i++;
						}
						else
						{
							vertical_i++;
							oa_line_i = 0;

							if (!oa_line_in_polygon_flag)
							{
								horizontal_i++;
								vertical_i = 1;
							}
						}
					}
					else if (oa_line_i == 1)
					{
						oa_line_in_polygon_flag = is_line_in_polygon(&planed_oa.oa_wp.s2, &planed_oa.oa_wp.s3, &planed_oa.flight_area);

						bool_t s2_s3_in_poly_flag = TRUE;

						for (int i = 0; i < planed_oa.obstacles_num; i++)
						{
							if (is_point_in_polygon(&planed_oa.oa_wp.s2, &(planed_oa.obstacles[i].polygon))
									|| is_point_in_polygon(&planed_oa.oa_wp.s3, &(planed_oa.obstacles[i].polygon)))
							{
								s2_s3_in_poly_flag = FALSE;
								break;
							}
						}

						if (!rectangle_obstacle_on_oa_route(planed_oa.obstacles, planed_oa.obstacles_num, &planed_oa.oa_wp.s2, &planed_oa.oa_wp.s3)
								&& oa_line_in_polygon_flag && s2_s3_in_poly_flag)
						{
							oa_line_i++;
						}
						else
						{
							vertical_i++;
							oa_line_i = 0;

							if (!oa_line_in_polygon_flag)
							{
								horizontal_i++;
								vertical_i = 1;
								oa_line_i = 0;
							}
						}
					}
					else
					{
						oa_line_in_polygon_flag = is_line_in_polygon(&planed_oa.oa_wp.s3, &planed_oa.oa_wp.s4, &planed_oa.flight_area);

						if (!rectangle_obstacle_on_oa_route(planed_oa.obstacles, planed_oa.obstacles_num, &planed_oa.oa_wp.s3, &planed_oa.oa_wp.s4)
								&& oa_line_in_polygon_flag)
						{
							waypoint_set_vect2(WP_S1, &planed_oa.oa_wp.s1);
							waypoint_set_vect2(WP_S2, &planed_oa.oa_wp.s2);
							waypoint_set_vect2(WP_S3, &planed_oa.oa_wp.s3);
							waypoint_set_vect2(WP_S4, &planed_oa.oa_wp.s4);

							planed_oa.sp_state = TRUE;
							oa_wp_search_state = search_done;
							planed_oa_state = go_to_start_wp;

							planed_oa.total_search_time = usec_of_sys_time_ticks(sys_time.nb_tick) - start_search_time;

							direction_i = 0;
							horizontal_i = 0;
							vertical_i = 1;
							oa_line_i = 0;
							return;
						}
						else
						{
							vertical_i++;
							oa_line_i = 0;

							if (!oa_line_in_polygon_flag)
							{
								horizontal_i++;
								vertical_i = 1;
								oa_line_i = 0;
							}
						}

						oa_line_i = 0;
					}
				}
			}
			else
			{
				horizontal_i++;
				vertical_i = 1;
				oa_line_i = 0;
			}
		}
		else
		{
			direction_i++;

			if (oa_wp_search_direction == left_side_first)
			{
				oa_wp_search_direction = right_side_first;
			}
			else
			{
				oa_wp_search_direction = left_side_first;
			}
			planed_oa.search_times = 0;
			horizontal_i = 0;
			vertical_i = 1;
			oa_line_i = 0;
		}
	}
	else
	{
		//open or close obstacles boundary insert:
		oa_wp_search_state = search_error_no_path;
		/*if( planed_oa.spray_boundary_insert_flag || (planed_oa.obstacles_boundary_insert_run_flag && (insert_state == insert_second_wp)) )
		 {
		 oa_wp_search_state = search_error_no_path;
		 }
		 else
		 {
		 planed_oa.obstacles_boundary_insert_flag = TRUE;
		 }*/

		planed_oa.total_search_time = usec_of_sys_time_ticks(sys_time.nb_tick) - start_search_time;
		direction_i = 0;
		horizontal_i = 0;
		vertical_i = 1;
		oa_line_i = 0;
	}
}

/*
 * Ge obstacle info:
 */
static void update_obstacles_info(void)
{
	for (int i = 0; i < planed_oa.obstacles_num; i++)
	{
		planed_oa.obstacles[i].pos.x = (planed_oa.obstacles[i].polygon.v[0].x + planed_oa.obstacles[i].polygon.v[2].x) * 0.5f;
		planed_oa.obstacles[i].pos.y = (planed_oa.obstacles[i].polygon.v[0].y + planed_oa.obstacles[i].polygon.v[2].y) * 0.5f;

		planed_oa.obstacles[i].raduis = points_length_calc(&(planed_oa.obstacles[i].pos), &(planed_oa.obstacles[i].polygon.v[0]));
	}
}

/*
 * release oa speed and acc:
 */
static void guidance_h_trajectory_tracking_brake_setting_release(void)
{
	if (planed_oa.back_home_ready)
	{
		nav_set_flight_speed(ac_config_info.max_flight_speed);
	}
	else
	{
		switch (from_wp.action)
		{
		case FLIGHT_LINE:
			nav_set_flight_speed(ac_config_info.max_flight_speed);
			break;

		case SPRAY_LINE:
			nav_set_flight_speed(ac_config_info.spray_speed);
			break;

		case SPRAY_CONVERT:
			nav_set_flight_speed(ac_config_info.spray_speed);
			break;

		case TERMINATION:
			nav_set_flight_speed(ac_config_info.max_flight_speed);
			break;

		default:
			break;
		}
	}

	guidance_h_trajectory_tracking_set_max_acc(traj.max_acc_backup);
}

/*
 * achieve next oa wp after reach target oa wp
 */
static bool_t achieve_next_oa_wp(void)
{
	if (planed_oa_state < reach_end_wp)
	{
		switch (planed_oa_state)
		{
		case release_sp:
			{
			break;
		}
		case go_to_start_wp:
			{
			ENU_BFP_VECT2_OF_REAL(oa_from_wp.wp_en, planed_oa.from_wp);
			ENU_BFP_VECT2_OF_REAL(oa_next_wp.wp_en, planed_oa.oa_wp.s1);
			break;
		}
		case reach_start_wp:
			{
			ENU_BFP_VECT2_OF_REAL(oa_from_wp.wp_en, planed_oa.oa_wp.s1);
			ENU_BFP_VECT2_OF_REAL(oa_next_wp.wp_en, planed_oa.oa_wp.s2);

			guidance_h_trajectory_tracking_set_ref_speed(OA_MAX_SPEED);
			guidance_h_trajectory_tracking_set_max_acc(OA_MAX_ACCEL);

			break;
		}
		case reach_frist_corner_wp:
			{
			ENU_BFP_VECT2_OF_REAL(oa_from_wp.wp_en, planed_oa.oa_wp.s2);
			ENU_BFP_VECT2_OF_REAL(oa_next_wp.wp_en, planed_oa.oa_wp.s3);

			guidance_h_trajectory_tracking_set_ref_speed(OA_MAX_SPEED);
			guidance_h_trajectory_tracking_set_max_acc(OA_MAX_ACCEL);

			break;
		}
		case reach_last_corner_wp:
			{
			ENU_BFP_VECT2_OF_REAL(oa_from_wp.wp_en, planed_oa.oa_wp.s3);
			ENU_BFP_VECT2_OF_REAL(oa_next_wp.wp_en, planed_oa.oa_wp.s4);

			guidance_h_trajectory_tracking_set_ref_speed(OA_MAX_SPEED);
			guidance_h_trajectory_tracking_set_max_acc(OA_MAX_ACCEL);

			break;
		}
		case reach_last_wp:
			{
			ENU_BFP_VECT2_OF_REAL(oa_from_wp.wp_en, planed_oa.oa_wp.s4);
			ENU_BFP_VECT2_OF_REAL(oa_next_wp.wp_en, planed_oa.next_wp);

			guidance_h_trajectory_tracking_brake_setting_release();

			break;
		}
		default:
			break;

		}
		return TRUE;
	}
	else
	{
		planed_oa_state = reach_end_wp;
		return FALSE;
	}
}

/*
 * get from wp and next wp
 */
static void get_task_wp(void)
{
	/* define from, next wp */
	if (FLIGHT_LINE == from_wp.action)
	{
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_from_wp, from_wp.wp_en);
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_next_wp, next_wp.wp_en);
	}

	if (planed_oa.back_home_ready)
	{
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_from_wp, interrupt_wp_scene);
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_next_wp, home_wp_enu);
	}
	else if (TERMINATION == from_wp.action)
	{
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_from_wp, from_wp.wp_en);
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_next_wp, home_wp_enu);
	}
	else
	{
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_from_wp, from_wp.wp_en);
		ENU_FLOAT_VECT2_OF_BFP(planed_oa.pre_next_wp, next_wp.wp_en);
	}

	//close for debug:
	/* define from, next wp */
	waypoint_set_vect2(WP_Pre_From, &planed_oa.pre_from_wp);
	waypoint_set_vect2(WP_Pre_Next, &planed_oa.pre_next_wp);
}

/*
 * get present from_wp and next_wp to generate oa route
 */
static void get_oa_from_next_wp(void)
{
	if (insert_state == insert_frist_wp)
	{
		VECT2_COPY(planed_oa.from_wp, planed_oa.pre_from_wp);
		VECT2_COPY(planed_oa.next_wp, planed_oa.insert_oa_wp);
		//planed_oa.from_wp = planed_oa.pre_from_wp;
		//planed_oa.next_wp = planed_oa.insert_oa_wp;
	}
	else if (insert_state == insert_second_wp)
	{
		VECT2_COPY(planed_oa.from_wp, planed_oa.insert_oa_wp);
		VECT2_COPY(planed_oa.next_wp, planed_oa.pre_next_wp);
		//planed_oa.from_wp = planed_oa.insert_oa_wp;
		//planed_oa.next_wp = planed_oa.pre_next_wp;
	}
	else
	{
		VECT2_COPY(planed_oa.from_wp, planed_oa.pre_from_wp);
		VECT2_COPY(planed_oa.next_wp, planed_oa.pre_next_wp);
		//planed_oa.from_wp = planed_oa.pre_from_wp;
		//planed_oa.next_wp = planed_oa.pre_next_wp;
	}

	ENU_BFP_VECT2_OF_REAL(planed_oa.from_wp_i, planed_oa.from_wp);
	ENU_BFP_VECT2_OF_REAL(planed_oa.next_wp_i, planed_oa.next_wp);
}

/*
 * find nearest avaliable spray edge point to define insert wp
 */
void find_shortest_avaliable_spray_boundary(struct FloatVect2 *spray_boundary_array,
		uint8_t spray_boundary_array_num, struct FloatVect2 *insert_oa_wp, struct FloatVect2 *start_wp,
		struct FloatVect2 *end_wp)
{
	int8_t k = 0;
	static float p2p_dist[OA_MAX_BOUNDARY_VERTICES_NUM];

	if( spray_i == 0 )
	{
		insert_start_end_cord[0] = start_wp->x;
		insert_start_end_cord[1] = start_wp->y;
		insert_start_end_cord[2] = end_wp->x;
		insert_start_end_cord[3] = end_wp->y;

		error_spray_cord[36] = start_wp->x;
		error_spray_cord[37] = start_wp->y;
		error_spray_cord[38] = end_wp->x;
		error_spray_cord[39] = end_wp->y;
	}

	//error_spray_cord[2*spray_i] = spray_boundary_array[spray_i].x;
	//error_spray_cord[2*spray_i + 1] = spray_boundary_array[spray_i].y;

	if(spray_i < 18)
	{
		error_spray_cord[2*spray_i] = planed_oa.flight_area.v[spray_i].x;
		error_spray_cord[2*spray_i + 1] = planed_oa.flight_area.v[spray_i].y;
	}

	if( spray_i < spray_boundary_array_num )
	{
		if( spray_boundary_vaild_flag[spray_i] )
		{
			if( insert_i == 0 )
			{
				if( is_line_in_polygon(start_wp, &(spray_boundary_array[spray_i]), &planed_oa.flight_area) )
				{
					error_spray[2*spray_i] = TRUE;
				}
				else
				{
					error_spray[2*spray_i] = FALSE;
				}

				insert_i++;
				return;
			}

			if( insert_i == 1 )
			{
				if( is_line_in_polygon(&(spray_boundary_array[spray_i]), end_wp, &planed_oa.flight_area) )
				{
					error_spray[2*spray_i+1] = TRUE;

					if( error_spray[2*spray_i] )
					{
						float start_wp_to_spray_boundary_dist = points_length_calc(&spray_boundary_array[spray_i], start_wp);
						float end_wp_to_spray_boundary_dist = points_length_calc(&spray_boundary_array[spray_i], end_wp);
						p2p_dist[spray_i] = start_wp_to_spray_boundary_dist + end_wp_to_spray_boundary_dist;
					}
					else
					{
						p2p_dist[spray_i] = MAX_SELECT_LENGTH;
					}
				}
				else
				{
					p2p_dist[spray_i] = MAX_SELECT_LENGTH;
					error_spray[2*spray_i+1] = FALSE;
				}

				insert_i = 0;
				spray_i++;
				return;
			}
		}
		else
		{
			error_spray[2*spray_i] = FALSE;
			error_spray[2*spray_i+1] = FALSE;
			p2p_dist[spray_i] = MAX_SELECT_LENGTH;
			insert_i = 0;
			spray_i++;
			return;
		}
	}
	else
	{
		spray_i = 0;
		insert_i = 0;

	}

	k = min_dist_select(p2p_dist, spray_boundary_array_num);

	if (p2p_dist[k] == MAX_SELECT_LENGTH)
	{
		oa_wp_search_state = search_error_no_vaild_insert_wp;
		planed_oa.spray_boundary_insert_flag = FALSE;
		return;
	}

	insert_oa_wp->x = spray_boundary_array[k].x;
	insert_oa_wp->y = spray_boundary_array[k].y;

	insert_state = insert_frist_wp;
	oa_wp_search_state = on_search;
}

/*
 * call by monitoring_misc.c to detect error
 */
bool_t planed_oa_search_valid(void)
{
	/* report an error and hover if search error */
	if (oa_wp_search_state > 2)
	{
		return FALSE;
	}

	return TRUE;
}

/*
 * call by monitoring_misc.c to detect error
 */
static void creat_a_fake_oa_line(void)
{
	VECT2_COPY(planed_oa.oa_wp.s1, planed_oa.from_wp);
	VECT2_COPY(planed_oa.oa_wp.s2, planed_oa.from_wp);
	VECT2_COPY(planed_oa.oa_wp.s3, planed_oa.from_wp);
	VECT2_COPY(planed_oa.oa_wp.s4, planed_oa.from_wp);

	waypoint_set_vect2(WP_S1, &planed_oa.oa_wp.s1);
	waypoint_set_vect2(WP_S2, &planed_oa.oa_wp.s2);
	waypoint_set_vect2(WP_S3, &planed_oa.oa_wp.s3);
	waypoint_set_vect2(WP_S4, &planed_oa.oa_wp.s4);

	planed_oa.sp_state = TRUE;
	oa_wp_search_state = search_done;
	planed_oa_state = go_to_start_wp;
}

/*
 * main func
 */
static void planed_oa_run(void)
{
	//if(oa_wp_search_state == not_search && !planed_oa.obstacles_boundary_insert_run_flag )
	if (oa_wp_search_state == not_search)
	{
		direction_flag = TRUE;

		/* define from, next wp */

		start_search_time = usec_of_sys_time_ticks(sys_time.nb_tick);

		/* see if the nav route out of area, if yes, insert one wp */
		if( !planed_oa.spray_boundary_insert_flag )
		{
			if ( (!is_line_in_polygon(&planed_oa.pre_from_wp, &planed_oa.pre_next_wp, &planed_oa.flight_area)) && (insert_state == no_insert_wp))
			{
				planed_oa.spray_boundary_insert_flag = TRUE;
				waypoint_set_vect2(WP_ERROR_F, &planed_oa.pre_from_wp);
				waypoint_set_vect2(WP_ERROR_N, &planed_oa.pre_next_wp);
				return;
			}
		}

		if( planed_oa.spray_boundary_insert_flag )
		{
			find_shortest_avaliable_spray_boundary(planed_oa.spray_area.v, planed_oa.spray_area.n,
					&planed_oa.insert_oa_wp, &planed_oa.pre_from_wp, &planed_oa.pre_next_wp);

			if(oa_wp_search_state != on_search)
			{
				return;
			}

			planed_oa.spray_boundary_insert_flag = FALSE;

			/*if( planed_oa.spray_boundary_insert_flag || oa_wp_search_state == search_error_no_vaild_insert_wp)
			{
				return;
			}*/
		}

		/* define and display from, next oa wp */
		get_oa_from_next_wp();

		/* calc nav_north_angle by from, next wp */
		VECT2_DIFF(planed_oa.vect_from_to_next_wp, planed_oa.next_wp, planed_oa.from_wp);
		planed_oa.nav_north_angle = vector_angle_calc(&planed_oa.vect_to_north, &planed_oa.vect_from_to_next_wp);

		/* necessity of oa */
		planed_oa.oa_necessity_flag = rectangle_obstacle_on_nav_route(planed_oa.obstacles, planed_oa.obstacles_num, &planed_oa.from_wp,
				&planed_oa.next_wp);

		if (planed_oa.oa_necessity_flag)
		{
			/* get initial start end oa wp to calc s1 and s4 */
			get_initial_start_end_oa_wp(&planed_oa.init_start_oa_wp, &planed_oa.init_end_oa_wp, planed_oa.obstacles, planed_oa.obstacles_num,
					&planed_oa.from_wp, &planed_oa.next_wp);
		}

		waypoint_set_vect2(WP_T1, &planed_oa.init_start_oa_wp);
		waypoint_set_vect2(WP_T4, &planed_oa.init_end_oa_wp);

		waypoint_set_vect2(WP_T0, &planed_oa.from_wp);
		waypoint_set_vect2(WP_T5, &planed_oa.next_wp);
	}

	if (planed_oa.total_search_time > MAX_SEARCH_TIME)
	{
		oa_wp_search_state = search_error_no_path;
		return;
	}

	/* if oa_necessity_flag is TRUE, generate oa wp*/
	if (planed_oa.oa_necessity_flag)
	{
		/* search and generate oa wp*/
		oa_wp_generation();
	}
	else
	{
		creat_a_fake_oa_line();
	}
}

/*
 * calc vail spray area edge point, if the point is not affected by obstacles, the responding spray_boundary_vaild_flag[i] will be TRUE
 */
static void update_vaild_spray_edge(void)
{
	memset(spray_boundary_vaild_flag, 0, sizeof(spray_boundary_vaild_flag));

	for (uint8_t i = 0; i < planed_oa.spray_area.n; i++)
	{
		for (uint8_t j = 0; j < planed_oa.obstacles_num; j++)
		{
			if (is_point_in_polygon(&(planed_oa.spray_area.v[i]), &(planed_oa.obstacles[j].polygon)))
			{
				spray_boundary_vaild_flag[i] = FALSE;
				break;
			}
			else
			{
				spray_boundary_vaild_flag[i] = TRUE;
			}
		}
	}
}

bool_t oa_task_nav_path(struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp)
{
	if (planed_oa.test_on
			&& (FLIGHT_LINE == from_wp.action || planed_oa.back_home_ready || TERMINATION == from_wp.action))
	{
		if (planed_oa.sp_state)
		{
			if (!achieve_next_oa_wp())
			{
				/*no more task_wp to run*/
				planed_oa_state = release_sp;
				oa_wp_search_state = not_search;
				planed_oa.sp_state = FALSE;
				planed_oa.wp_move_done_flag = FALSE;

				if (insert_state == no_insert_wp)
				{
					return FALSE;
				}
				else if (insert_state == insert_frist_wp)
				{
					insert_state = insert_second_wp;
					planed_oa.wp_move_done_flag = TRUE;
				}
				else
				{
					insert_state = no_insert_wp;
					return FALSE;
				}
			}

			VECT3_COPY(p_start_wp, oa_from_wp.wp_en);
			VECT3_COPY(p_end_wp, oa_next_wp.wp_en);

			if (nav_approaching_from(&p_end_wp, &p_start_wp, 0) && planed_oa.sp_state)
			{
				planed_oa_state++;

				if (planed_oa_state >= reach_end_wp)
				{
					planed_oa_state = reach_end_wp;
				}
			}
		}
		else
		{
			if (insert_state > no_insert_wp && oa_wp_search_state > 0)
			{
				VECT3_COPY(p_start_wp, planed_oa.from_wp_i);
				VECT3_COPY(p_end_wp, planed_oa.from_wp_i);
			}
			else if (insert_state == no_insert_wp && ( oa_wp_search_state > 0 || planed_oa.spray_boundary_insert_flag ))
			{
				VECT3_COPY(p_end_wp, p_start_wp);
			}
			else
			{
				if (nav_approaching_from(&p_end_wp, &p_start_wp, 0))
				{
					return FALSE;
				}
			}
		}
	}
	else
	{
		if (nav_approaching_from(&p_end_wp, &p_start_wp, 0))
		{
			return FALSE;
		}
	}

	horizontal_mode = HORIZONTAL_MODE_ROUTE;
	nav_route(&p_start_wp, &p_end_wp);
	return TRUE;
}

/*
 * force to recover the current route, only for test;
 */
void oa_error_force_recover(void)
{
	if (planed_oa.test_on)
	{
		if (oa_wp_search_state > 2)
		{
			get_oa_from_next_wp();
			creat_a_fake_oa_line();
		}
	}
}

static void send_point_to_pprz(void)
{
	uint8_t i, j;

	struct FloatVect2 temp; 
	if(p_transfer_useful == TRUE)
	{
		temp.x = POS_FLOAT_OF_BFP(vertipad.x);
		temp.y = POS_FLOAT_OF_BFP(vertipad.y);
		waypoint_set_vect2(WP_TP, &temp);
	}

	for (i = 0; i < OA_MAX_BOUNDARY_VERTICES_NUM; ++i)
	{
		if (i < planed_oa.spray_area.n)
		{
			waypoint_set_vect2(WP_V0 + i, &(planed_oa.spray_area.v[i]));
		}
		else
		{
			waypoint_set_vect2(WP_V0 + i, &(planed_oa.spray_area.v[planed_oa.spray_area.n - 1]));
		}
	}

	for (i = 0; i < OA_MAX_OBSTACLES_NUM; ++i)
	{
		if (i < planed_oa.obstacles_num)
		{
			for (j = 0; j < OA_OBSTACLE_CORNER_NUM; j++)
			{
				waypoint_set_vect2(WP_O11 + (OA_OBSTACLE_CORNER_NUM * i + j), &(planed_oa.obstacles[i].polygon.v[j]));
			}
		}
		else
		{
			for (j = 0; j < OA_OBSTACLE_CORNER_NUM; j++)
			{
				struct FloatVect2 v = {0, 0 };
				waypoint_set_vect2(WP_O11 + (OA_OBSTACLE_CORNER_NUM * i + j), &v);
			}
		}
	}

	for (i = 0; i <= OA_MAX_BOUNDARY_VERTICES_NUM; ++i)
	{
		if (i < planed_oa.flight_area.n)
		{
			waypoint_set_vect2(WP_A0 + i, &(planed_oa.flight_area.v[i]));
		}
		else
		{
			waypoint_set_vect2(WP_A0 + i, &(planed_oa.flight_area.v[planed_oa.flight_area.n - 1]));
		}
	}

}

/*
 * called by LiGang when received spray boundary coordination:
 */
void planed_oa_prepare(void)
{
	if(eng_app_check_debug_sn())
	{
		planed_oa_debug_prepare();
	}
	planed_oa_data_reset();
	planed_oa_geometry_prepare();
	update_obstacles_info();
	update_vaild_spray_edge();
	send_point_to_pprz();
	planed_oa.planed_oa_ready = TRUE;
}

/*
 * periodic func
 */
void planed_oa_periodic_run(void)
{
	//run_time[0] = usec_of_sys_time_ticks(sys_time.nb_tick);

	if( eng_app_check_debug_sn() )
	{
		planed_oa_test();
	}

	if (planed_oa.test_on && from_wp_useful)
	{
		get_task_wp();

		if (planed_oa.manual_prepare_flag)
		{
			planed_oa_prepare();
			planed_oa.manual_prepare_flag = FALSE;
		}

		/*call back UAV if error occurs*/
		if (planed_oa.manual_callback_flag)
		{
			//close for debug:
			oa_error_force_recover();
			planed_oa.manual_callback_flag = FALSE;
		}

		if ((flight_state == cruising) && (guidance_h.mode == GUIDANCE_H_MODE_NAV) && (planed_oa.planed_oa_ready))
		{
			if ((FLIGHT_LINE == from_wp.action) || (TERMINATION == from_wp.action) || (planed_oa.back_home_ready))
			{
				if ((!planed_oa.sp_state) && (planed_oa.wp_move_done_flag)
						&& (oa_wp_search_state == not_search || oa_wp_search_state == on_search))
				{
					//run_time[0] = usec_of_sys_time_ticks(sys_time.nb_tick);
					planed_oa_run();
					//run_time[1] = usec_of_sys_time_ticks(sys_time.nb_tick) - run_time[0];

					for (int i = 0; i < OA_MAX_OBSTACLES_NUM; i++)
					{
						o_flag[i] = planed_oa.obstacles[i].obstacle_flag;
					}
				}
			}
			else
			{
				oa_wp_search_state = not_search;
			}

			//close for debug:
			waypoint_set_vect2(WP_From, &planed_oa.from_wp);
			waypoint_set_vect2(WP_Next, &planed_oa.next_wp);
			waypoint_set_vect2(WP_HOME, &oa_data.home);
		}

		if (flight_state == landing)
		{
			planed_oa.planed_oa_ready = FALSE;
			planed_oa.test_on = FALSE;
		}
	}

	//run_time[1] = usec_of_sys_time_ticks(sys_time.nb_tick) - run_time[0];

	/* send message for tuning */
#if PERIODIC_TELEMETRY
	RunOnceEvery(20,
			{
				xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
				DOWNLINK_SEND_PLANED_OA(DefaultChannel, DefaultDevice,
						&spray_boundary_vaild_flag[0],
						&planed_oa.test_on,
						&planed_oa.planed_oa_ready,
						&o_flag[0],
						&planed_oa.obstacles_num,
						&oa_wp_search_state,
						&planed_oa.wp_move_done_flag,
						&planed_oa.obstacles_boundary_insert_flag,
						&vert_search_direction,
						&planed_oa.search_times,
						&planed_oa_state,
						&insert_state,
						&planed_oa.sp_state,
						&planed_oa.back_home_ready,
						&planed_oa.nav_north_angle,
						&planed_oa.total_search_time,
						&error_spray[0],
						&error_spray_cord[0],
						&planed_oa.oa_necessity_flag);});
#endif

}

