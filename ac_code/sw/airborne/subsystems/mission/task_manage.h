/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW2      									   *
*   AUTHOR	   :             										   *
************************************************************************
* Object        :
* Module        :
* Instance      :
* Description   :
*-----------------------------------------------------------------------
* Version:
* Date:
* Author:
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
*
*=====================================================================*/
#ifndef _TASK_MANAGE_H_
#define _TASK_MANAGE_H_

#include "state.h"
#include <inttypes.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_double.h" //TODO:for debug


#define NB_TASK 50
#define NB_RESERVE_LAND 5

#define POINT_MAX_DISTANCE 300000   //unit=cm

#define NavKillMode()  {autopilot_set_mode(AP_MODE_KILL);Flag_AC_Flight_Ready=FALSE;}

/*value: start=1; pause=2; continual=3; back_home=4;  reserve_land=5*/
enum Gcs_Task_Cmd
{
	GCS_CMD_NONE = 0,
	GCS_CMD_START = 1,
	GCS_CMD_PAUSE = 2,
	GCS_CMD_CONTI = 3,
	GCS_CMD_BHOME = 4,
	GCS_CMD_RELAND = 5,
	GCS_CMD_DLAND = 6,
	GCS_CMD_LOCK = 7
};

enum Land_Type
{
	VERTIPAD_TAKEOFF_POINT = 1,
	VERTIPAD_LAND_POINT = 2,
	TRANS_POINT = 3
};
enum Task_Type
{
	NORMAL_POLYGON = 0,
	POINTS_MISSION
};

//operation_type: add=1, update=2, delete=3
enum Operation_Type_Land
{
	LAND_TASK_ADD = 1,
	LAND_TASK_UPDATE,
	LAND_TASK_DELETE
};

/* wp_action  flight_line=1; spray_line=2; spray_convert=3;  hovering=4; termination=5;*/
enum Task_Action
{
	FLIGHT_LINE = 1,
	SPRAY_LINE,
	SPRAY_CONVERT,
	HOVERING,
	TERMINATION,
	AVOID_OBSTACLE,
	TRANSFER
};
/*
<field name="task_code" type="uint8"/>
<field name="wp_type"  type="uint8"/>
<field name="wp_start_id" type="uint8"/>
<field name="wp_end_id" type="uint8"/>
<field name="wp_action" type="uint8[]"/>
<field name="waypoints_lon" type="int32[]"/>
<field name="waypoints_lat" type="int32[]"/>
*/
struct Task_Info
{
	uint8_t task_code;
	uint8_t wp_type;
	uint16_t wp_start_id;
	uint16_t wp_end_id;
	uint8_t length_wp_action;
	uint8_t *wp_action;
	uint8_t length_wp_lon;
	int8_t *waypoints_lon;
	uint8_t length_wp_lat;
	int8_t *waypoints_lat;
};

/*
<field name="task_code" type="uint8"/>
<field name="bp_type"  type="uint8"/>
<field name="bp_start_id" type="uint8"/>
<field name="bp_end_id" type="uint8"/>
<field name="bp_points_lon" type="int32[]" unit="e8,rad"/>
<field name="bp_points_lat" type="int32[]" unit="e8,rad"/>
*/
struct bp_Info
{
	uint8_t task_code;
	uint8_t total_bp_num;
	uint8_t length_bp_lon;
	int8_t *bp_points_lon;
	uint8_t length_bp_lat;
	int8_t *bp_points_lat;
};

/*
<field name="task_code" type="uint8"/>
<field name="op_type"  type="uint8"/>
<field name="op_start_id" type="uint8"/>
<field name="op_end_id" type="uint8"/>
<field name="op_points_lon" type="int32[]" unit="e8,rad"/>
<field name="op_points_lat" type="int32[]" unit="e8,rad"/>
<field name="op_radius" type="int32[]" unit="cm"/>
*/
struct op_Info
{
	uint8_t task_code;
	uint8_t total_op_num;
	uint8_t length_op_lon;
	int8_t *op_points_lon;
	uint8_t length_op_lat;
	int8_t *op_points_lat;
};

struct _s_oa_data
{
	struct FloatVect2 spray_boundary_vertices_array[OA_MAX_BOUNDARY_VERTICES_NUM];
	uint8_t spray_boundary_vertices_num;
	struct FloatVect2 obstacles_vertices_array[OA_MAX_OBSTACLES_NUM][OA_OBSTACLE_CORNER_NUM];
	uint8_t obstacles_vertices_num;
	uint8_t obstacles_num;
	struct FloatVect2 home;
	bool_t spray_boundary_valid;
	bool_t obstacles_valid;
	bool_t home_valid;
};

/*
<field name="operation_type" type="uint8"/>
<field name="land_type" type="uint8"/>
<field name="wp_type"  type="uint8"/>
<field name="waypoints_lon" type="int32[]" unit="e8,rad"/>
<field name="waypoints_lat" type="int32[]" unit="e8,rad"/>
*/
struct Land_Info
{
	uint8_t operation_type;
	uint8_t wp_type;
	uint8_t land_type_length;
	uint8_t waypoints_length;
	uint8_t *land_type;
	int8_t  *waypoints_lon; //start add
	int8_t  *waypoints_lat;
};

struct Task_Wp
{
	struct Int32Vect2  wp_en;
	enum Task_Action   action;
	uint16_t            wp_id;
};

struct Task_Wp_Enu
{
	struct EnuCoor_i   wp_en;
	enum Task_Action   action;
	uint8_t            wp_id;
};

#define TASK_WP_LEFT_SHIFT(_id, _n) { \
	task_wp[(_id) - (_n)].wp_en.x = task_wp[(_id)].wp_en.x; \
	task_wp[(_id) - (_n)].wp_en.y = task_wp[(_id)].wp_en.y; \
	task_wp[(_id) - (_n)].action  = task_wp[(_id)].action;  \
	task_wp[(_id) - (_n)].wp_id   = task_wp[(_id)].wp_id;   }



extern struct Task_Wp task_wp[NB_TASK];
extern uint16_t nb_pending_wp;
extern struct LlaCoor_d wp0_lla;	//todo:for debug
extern struct Int32Vect2 wp_home;
extern bool_t wp_home_useful;
extern struct Int32Vect2 vertipad_takeoff;
extern struct Int32Vect2 vertipad_land;
extern bool_t p_transfer_useful;
extern enum Task_Type flight_task_type;

//extern struct Int32Vect2 wp_reserve_land[NB_RESERVE_LAND];//todo:for debug
//extern uint8_t nb_pending_reland;	//todo:for debug

extern struct Task_Wp_Enu from_wp;
extern struct Task_Wp_Enu next_wp;

#ifdef USE_PLANED_OA

#include "modules/planed_oa/planed_oa.h"
extern struct Task_Wp_Enu oa_from_wp; //use for store from oa waypoint
extern struct Task_Wp_Enu oa_next_wp; //use for store next oa waypoint
#endif

extern struct _s_oa_data oa_data;

extern void task_manage_init(void);
extern uint8_t parse_gcs_cmd( uint8_t cmd);
extern int8_t parse_add_task(struct Task_Info m_task_info);
extern int8_t parse_update_task(struct Task_Info m_task_info);
extern int8_t parse_delete_task(uint16_t wp_start_id, uint16_t wp_end_id);
extern struct Task_Info parse_get_task(uint16_t wp_start_id, uint16_t wp_end_id);
extern int8_t parse_land_task(struct Land_Info dl_land_info);
extern int8_t command_delete_all_task(void);
extern int8_t parse_add_border(struct bp_Info m_bp_info);
extern int8_t parse_add_obstacle(struct op_Info m_op_info);
extern int8_t check_oa_data_valid(void);

#endif /*_TASK_MANAGE_H_*/

/****************************** END OF FILE ***************************/
