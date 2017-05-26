#ifndef _RADAR_PLANED_OA_
#define _RADAR_PLANED_OA_

/*here is the function that apply radar sensor, which can read the distance from body to obstacle defined in navigation.c
inputs: current position of body, position of obstacle
output: obstacle distance and obstacle angle;
*/

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/mission/task_manage.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/mission/task_manage.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "std.h"
#include "mcu_periph/sys_time.h"
#include "math/dim2_geometry.h"

/* _vo=v1xv2 in NED, if _v2 is antilockwise of _v1, the result is postive */
#define VECT2_CROSS_PRODUCT(_v1, _v2) ((_v1).x*(_v2).y - (_v1).y*(_v2).x)

#define FLOAT_VECT2_ENU_OF_NED(_o,_i) {    \
    (_o).x = (_i).y;        \
    (_o).y = (_i).x;        \
  }
#define ENU_OF_NED_FLOAT_VECT2_OF_BFP(_o, _i) {      \
    (_o).y = POS_FLOAT_OF_BFP((_i).x);  \
    (_o).x = POS_FLOAT_OF_BFP((_i).y);  \
  }
#define ENU_OF_NED_FLOAT_VECT2_OF_REAL(_o, _i) {      \
    (_o).y = POS_BFP_OF_REAL((_i).x);  \
    (_o).x = POS_BFP_OF_REAL((_i).y);  \
  }
#define ENU_FLOAT_VECT2_OF_BFP(_o, _i) {      \
    (_o).x = POS_FLOAT_OF_BFP((_i).x);  \
    (_o).y = POS_FLOAT_OF_BFP((_i).y);  \
  }
#define ENU_BFP_VECT2_OF_REAL(_o, _i) {       \
    (_o).x = POS_BFP_OF_REAL((_i).x);   \
    (_o).y = POS_BFP_OF_REAL((_i).y);   \
  }
#define max(a,b)	((a)>(b)?(a):(b))

#define PI            3.1415926

#define INT32_ANGLE   0.0139882
#define INT32_RAD     0.00024414   //1/2^12;

#define ANGLE_RAD     0.01745329   //PI/180.0
#define RAD_ANGLE     57.2957795   //180.0/PI

#define OA_SAFE_DIST    2.0
#define V_MAX_DIST_F    OA_SAFE_DIST * 2

#define OA_MAX_SPEED 2.0
#define OA_MAX_ACCEL 1.5

#define PROJ_NUM   OA_MAX_OBSTACLES_NUM*OA_OBSTACLE_CORNER_NUM


struct _s_planed_obstacle
{
   struct FloatVect2 pos;
   struct FloatVect2 v[OA_OBSTACLE_CORNER_NUM];
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

   struct _s_planed_obstacle o[OA_MAX_OBSTACLES_NUM];

   struct _s_oa_wp oa_wp;

   uint32_t total_search_time;
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
void planed_oa_run(void);
void creat_a_fake_oa_line(void);
extern void oa_error_force_recover(void);
extern void planed_oa_prepare(void);
extern bool_t oa_task_nav_path( struct EnuCoor_i p_start_wp, struct EnuCoor_i p_end_wp );
void get_vaild_spray_edge(void);
extern bool_t planed_oa_search_valid(void);
extern float vector_angle_calc(struct FloatVect2 *vect_a, struct FloatVect2 *vect_b);
float points_length_calc(struct FloatVect2 *point1, struct FloatVect2 *point2);
float vector_length_calc(struct FloatVect2 *vect);
extern void get_project_point(struct FloatVect2 *proj_point, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp, struct FloatVect2 *point);
uint8_t min_dist_select(float *dist, uint8_t n);
uint8_t max_dist_select(float *dist, uint8_t n);
bool_t get_start_end_oa_wp( struct FloatVect2 *start_oa_wp, struct FloatVect2 *end_oa_wp, struct _s_planed_obstacle *obstacle, uint8_t o_num, struct FloatVect2 *start_wp, struct FloatVect2 *end_wp );
void get_oa_search_direction(void);
void oa_wp_generation(void);
void guidance_h_trajectory_tracking_brake_setting_release(void);
bool_t achieve_next_oa_wp(void);
void get_task_wp(void);
extern void get_oa_from_next_wp(void);
void planed_oa_periodic_run(void);

#endif
