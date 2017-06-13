#ifndef _RADAR_PLANED_OA_
#define _RADAR_PLANED_OA_

#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/dim2_geometry.h"

struct _s_planed_obstacle
{
   struct FloatVect2 pos;
   struct _s_polygon polygon;
   float raduis;
   bool_t obstacle_flag;
};

struct _s_oa_wp
{ 
   struct FloatVect2 s1;
   struct FloatVect2 s2;
   struct FloatVect2 s3;
   struct FloatVect2 s4;
};

struct _s_planed_oa
{ 
   bool_t test_on;
   bool_t planed_oa_ready;
   bool_t pause_flag;
   bool_t back_home_ready;
   bool_t oa_home_flag;
   bool_t heading_ready;
   bool_t sp_state;
   bool_t oa_necessity_flag;
   bool_t wp_move_done_flag;
   bool_t manual_prepare_flag;
   bool_t manual_callback_flag;

   bool_t obstacles_boundary_insert_flag;
   bool_t spray_boundary_insert_flag;
   bool_t obstacles_boundary_insert_run_flag;

   float nav_north_angle;
   
   uint32_t search_times;

   struct FloatVect2 vect_to_north;
   struct FloatVect2 vect_from_to_next_wp;
   struct FloatVect2 vect_from_to_o;
   struct FloatVect2 plane_pos;

   struct FloatVect2 pre_from_wp;
   struct FloatVect2 pre_next_wp;
   struct FloatVect2 from_wp;
   struct FloatVect2 next_wp;
   struct FloatVect2 insert_oa_wp;

   struct FloatVect2 init_start_oa_wp;
   struct FloatVect2 init_end_oa_wp;

   struct EnuCoor_i from_wp_i;
   struct EnuCoor_i next_wp_i;

   struct _s_planed_obstacle obstacles[OA_MAX_OBSTACLES_NUM];
   uint8_t obstacles_num;
   struct _s_polygon spray_area;
   struct _s_polygon flight_area;
   struct FloatVect2 flight_boundary_vertices_array[OA_MAX_BOUNDARY_VERTICES_NUM + 1];

   struct _s_oa_wp oa_wp;

   uint32_t total_search_time;

   float error_info[15];
   float error_spray_area[OA_MAX_BOUNDARY_VERTICES_NUM * 2];
   float error_flight_area[OA_MAX_BOUNDARY_VERTICES_NUM * 2 + 2];
   bool_t error_record_flag;
};

enum Obstacle_Avoidance_State
{
   release_sp = 0,
   go_to_start_wp = 1,
   reach_start_wp = 2,
   reach_frist_corner_wp = 3,   
   reach_last_corner_wp = 4,
   reach_last_wp = 5,
   reach_end_wp = 6
};

enum Obstacle_Avoidance_Wp_Search_State
{
   not_search = 0,
   on_search = 1,
   search_done = 2,
   search_error_no_path = 3,
   search_error_obstacle_invaild = 4,
   search_error_no_vaild_insert_wp = 5,
   search_error_obstacle_flag_wrong = 6,
   area_generate_error_parameter_invild = 7,
   area_generate_error_cant_gen_area = 8
};

enum Obstacle_Avoidance_Wp_Search_Direction
{
   left_side_first = 0,
   right_side_first = 1
};

enum Insert_State
{
   no_insert_wp = 0,
   insert_frist_wp = 1,
   insert_second_wp = 2
};

extern struct _s_planed_oa planed_oa;

extern enum Obstacle_Avoidance_Wp_Search_State oa_wp_search_state;
extern enum Insert_State insert_state;

extern enum Obstacle_Avoidance_Wp_Search_Direction oa_wp_search_direction;
extern enum Obstacle_Avoidance_State planed_oa_state;

void planed_oa_data_init(void);
void planed_oa_periodic_run(void);

extern void oa_error_force_recover(void);
extern void planed_oa_prepare(void);
extern void planed_oa_data_reset(void);
extern bool_t oa_task_nav_path( struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp );
extern bool_t planed_oa_search_valid(void);

#endif
