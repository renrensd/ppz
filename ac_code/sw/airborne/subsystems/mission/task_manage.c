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

#include "subsystems/mission/task_manage.h"
#include "subsystems/mission/task_handle.h"
#include "subsystems/mission/task_process.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/monitoring/monitoring.h"  
#include "subsystems/datalink/downlink.h"
#include "math/dim2_geometry.h"

struct Task_Wp task_wp[NB_TASK];   //unfinished task, with relative waypoints/action/id
uint16_t nb_pending_wp;             //number of waypoints in  unfinished task

struct Int32Vect2 wp_home;
bool_t wp_home_useful;             //record home waypoint got
struct Int32Vect2 wp_reserve_land[NB_RESERVE_LAND];    //reserve
uint8_t nb_pending_reland;                             //reserve

struct EnuCoor_i temp_enu[20];    //only use for update_task
struct Task_Wp_Enu from_wp;       //start wp of current flight line
struct Task_Wp_Enu next_wp;       //end wp of current flight line

struct _s_oa_data oa_data;

bool_t Flag_AC_Flight_Ready;

#ifdef USE_PLANED_OA
struct Task_Wp_Enu oa_from_wp; //use for store from waypoint
struct Task_Wp_Enu oa_next_wp; //use for store next waypoint
#endif

static int8_t parse_land_task_home(struct Land_Info dl_land_info);
static int8_t parse_land_task_reserve(struct Land_Info dl_land_info, uint8_t offset);


void task_manage_init(void)
{
	nb_pending_wp = 0;           //use to sign task clean
	wp_home_useful = FALSE;
	nb_pending_reland = 0;
	
}

/***********************************************************************
* FUNCTION    : parse_gcs_cmd
* DESCRIPTION : parse gcs command: start / pause / continual / home / land
* INPUTS      : command
* RETURN      : error code
***********************************************************************/
uint8_t parse_gcs_cmd( uint8_t cmd)
{
	uint8_t response = 0;
	enum Gcs_Task_Cmd gcs_cmd = (enum Gcs_Task_Cmd)cmd;
   #if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    DOWNLINK_SEND_DEBUG_CMD(DefaultChannel, DefaultDevice, &gcs_cmd, &gcs_task_cmd, &wp_home_useful);
   #endif

  if(gcs_cmd == GCS_CMD_LOCK)
  {
  	gcs_task_cmd = GCS_CMD_LOCK;
		NavKillMode();
		gcs_cmd_interrupt = TRUE;
		return 0;
  }

	/*request first gcs_cmd is GCS_CMD_START*/
	if(GCS_CMD_NONE==gcs_task_cmd)
	{	
		if(GCS_CMD_START!=gcs_cmd)
		{
			response =2;
			return response;
		}
		else if(FALSE==wp_home_useful || 2>nb_pending_wp || FALSE==ground_check_pass)
		{
			response =3;  /*can not take off*/
			return response;
		}
	}
	if(GCS_CMD_START!=gcs_cmd && !autopilot_in_flight)
	{
		response =3;  /*not in flight, refuse command except start*/
		return response;
	}
	
	int8_t oa_ok;
	switch (gcs_cmd)
	{
	case GCS_CMD_START:
		#ifdef USE_PLANED_OA
		oa_ok = check_oa_data_valid();
		planed_oa.test_on = FALSE;
		if (oa_ok < 0)
		{
			response = search_error_obstacle_invaild;
			break;
		}
		else if(oa_ok == 0)
		{
			// no obstacles
		}
		else
		{
			planed_oa_prepare();

			if (oa_wp_search_state == area_generate_error_parameter_invild
					|| oa_wp_search_state == area_generate_error_cant_gen_area)
			{
				response = oa_wp_search_state; //area generate error
				break;
			}
			else
			{
				planed_oa.test_on = TRUE;
			}
		}
		#endif
		//pause to start need add confirm
		if ((GCS_CMD_PAUSE == gcs_task_cmd && FALSE == from_wp_useful)
				|| GCS_CMD_NONE == gcs_task_cmd
				|| GCS_CMD_START == gcs_task_cmd)
		{
			gcs_task_cmd = GCS_CMD_START;
			Flag_AC_Flight_Ready = TRUE;	//add by lg
			gcs_cmd_interrupt = TRUE;    //record command interrupt to get rid of emergency
		}
		else
		{
			response = 2; /*conflict cmd*/
		}
		break;

	case GCS_CMD_PAUSE:
		if (GCS_CMD_CONTI != gcs_task_cmd)
		{
			gcs_task_cmd = GCS_CMD_PAUSE;
			gcs_cmd_interrupt = TRUE;
			manual_pause_flag = TRUE;   //record brake signal
		}
		else
		{
			response = 2;
		}
		break;

	case GCS_CMD_CONTI:
		if (GCS_CMD_CONTI == gcs_task_cmd || GCS_CMD_PAUSE == gcs_task_cmd)
		{
			if (em_alert_grade > 2)
			{
				response = 2;
			}
			else
			{
				gcs_task_cmd = GCS_CMD_CONTI;
				gcs_cmd_interrupt = TRUE;
			}
		}
		else
		{
			response = 2;
		}
		break;

	case GCS_CMD_BHOME:
		spray_switch_flag = FALSE;
		gcs_task_cmd = GCS_CMD_BHOME;
		gcs_cmd_interrupt = TRUE;
		break;

	case GCS_CMD_RELAND:
		if (0 >= nb_pending_reland)
		{
			response = 2;
		}
		else
		{
			gcs_task_cmd = GCS_CMD_RELAND;
			gcs_cmd_interrupt = TRUE;
		}
		break;

	case GCS_CMD_DLAND:
		gcs_task_cmd = GCS_CMD_DLAND;
		gcs_cmd_interrupt = TRUE;
		break;

	case GCS_CMD_LOCK:
		gcs_task_cmd = GCS_CMD_LOCK;
		NavKillMode()
		;
		gcs_cmd_interrupt = TRUE;
		break;

	default:
		response = 1; /*parse error*/
		break;
	}
	return response;
}

/***********************************************************************
* FUNCTION    : parse_add_task
* DESCRIPTION : 
* INPUTS      : new task
* RETURN      : error code
***********************************************************************/
int8_t parse_add_task(struct Task_Info m_task_info)
{
	int8_t response = 0;
	/* waypoints space check */
	if( (nb_pending_wp + m_task_info.length_wp_action) > NB_TASK )
	{
		response = 2;  /*space is not enought*/
		return response;
	}

	/* waypoint id check(pending id < add wp_start_id) */
	uint16_t max_pending_id = get_max_pending_id();
	if( m_task_info.wp_start_id < max_pending_id )
	{
		if(m_task_info.wp_end_id==max_pending_id && check_task_wp_id(m_task_info.wp_start_id) )
		{
			response = 3;  /*add repeat*/
		}
		else
		{
			response = 5;  /*wp id not in order*/
		}
		return response;
	}

   /* wp_type: coordinate   wgs84 = 1, relative_ENU = 2 */
	uint8_t nb_wp =m_task_info.wp_end_id - m_task_info.wp_start_id + 1;
	uint8_t last_nb_pending_wp = nb_pending_wp;
	
   if( 1 == m_task_info.wp_type )
   {
   		struct LlaCoor_i lla;   /* rad, e8 */
		struct EnuCoor_i temp1_enu;
		for(uint8_t i=0; i < nb_wp; i++ )
		{
			lla.lon = *(m_task_info.waypoints_lon + i);
			lla.lat = *(m_task_info.waypoints_lat + i);	
	   		if( task_lla_to_enu_convert(&temp1_enu, &lla) )
	   		{
				VECT2_COPY(task_wp[nb_pending_wp].wp_en, temp1_enu);
				task_wp[nb_pending_wp].action = (enum Task_Action)( *(m_task_info.wp_action + i) );
				task_wp[nb_pending_wp].wp_id = m_task_info.wp_start_id + i;
				nb_pending_wp++;
	   		}
			else
			{
				response =4;
				nb_pending_wp = last_nb_pending_wp;
				break;   /* jump out the loop */
			}
		}
   	}
   
   else if( 2 == m_task_info.wp_type )
   {
   		struct EnuCoor_i temp2_enu;
   		for(uint8_t i=0; i < nb_wp; i++ )
		{
			temp2_enu.x = POS_BFP_OF_REAL(*(m_task_info.waypoints_lon + i))/1000;
			temp2_enu.y = POS_BFP_OF_REAL(*(m_task_info.waypoints_lat + i)) /1000;
			if( temp2_enu.x < POS_BFP_OF_REAL(POINT_MAX_DISTANCE/100)
				 &&temp2_enu.y < POS_BFP_OF_REAL(POINT_MAX_DISTANCE/100) )
			{
				VECT2_COPY(task_wp[nb_pending_wp].wp_en, temp2_enu);
				task_wp[nb_pending_wp].action = (enum Task_Action)( *(m_task_info.wp_action + i) );
				task_wp[nb_pending_wp].wp_id = m_task_info.wp_start_id + i;
				nb_pending_wp++;
			}
			else
			{
				response =4;
				nb_pending_wp = last_nb_pending_wp;
				break;   /* jump out the loop */				
			}
   		}
   	}

	return response;
}

/***********************************************************************
* FUNCTION    : parse_update_task
* DESCRIPTION : 
* INPUTS      : new task
* RETURN      : error code
***********************************************************************/
int8_t parse_update_task(struct Task_Info m_task_info)
{
	int8_t response = 0;
	/* waypoints id match check */
	uint8_t nb_wp =m_task_info.wp_end_id - m_task_info.wp_start_id + 1;

	int8_t wp_offset_start = get_task_wp_offset(m_task_info.wp_start_id);
	int8_t wp_offset_end = get_task_wp_offset(m_task_info.wp_end_id);
	if( wp_offset_start < 0 
		|| wp_offset_end < 0 
		|| nb_wp !=(wp_offset_end - wp_offset_start + 1) 
		|| nb_wp > 20 )
	{
		response = 2;
		return response;
	}
	
   if( 1 == m_task_info.wp_type )
   {
   		struct LlaCoor_i lla;   /* rad, e8 */
		
		for(uint8_t i=0; i < nb_wp; i++ )
		{
			lla.lon = *(m_task_info.waypoints_lon + i);
			lla.lat = *(m_task_info.waypoints_lat + i);	
	   		if( !task_lla_to_enu_convert(&temp_enu[i], &lla) )
	   		{
				response =3;
				break;   /* jump out the loop */
			}
		}
		if(!response)
		{
			for(uint8_t n=0; n <nb_wp; n++)
			{
				VECT2_COPY(task_wp[wp_offset_start+n].wp_en, temp_enu[n]);
				task_wp[wp_offset_start+n].action = (enum Task_Action)( *(m_task_info.wp_action + n) );
				task_wp[wp_offset_start+n].wp_id = m_task_info.wp_start_id + n;
			}
		}
   	}
   
   else if( 2 == m_task_info.wp_type )
   {
   		for(uint8_t i=0; i < nb_wp; i++ )
		{
			temp_enu[i].x = POS_BFP_OF_REAL(*(m_task_info.waypoints_lon + i))/1000;
			temp_enu[i].y = POS_BFP_OF_REAL(*(m_task_info.waypoints_lat + i)) /1000;
			if( temp_enu[i].x > POS_BFP_OF_REAL(POINT_MAX_DISTANCE/100)
				 ||temp_enu[i].y > POS_BFP_OF_REAL(POINT_MAX_DISTANCE/100) )
			{
				response =3;				
				break;   /* jump out the loop */				
			}
   		}
		if(!response)
		{
			for(uint8_t n=0; n <nb_wp; n++)
			{
				VECT2_COPY(task_wp[wp_offset_start+n].wp_en, temp_enu[n]);
				task_wp[wp_offset_start+n].action = (enum Task_Action)( *(m_task_info.wp_action + n) );
				//task_wp[wp_offset_start+n].wp_id = m_task_info.wp_start_id + n;
			}
		}
   	}

	return response;
}

/***********************************************************************
* FUNCTION    : parse_delete_task
* DESCRIPTION : 
* INPUTS      : the sequence of task need delete
* RETURN      : error code
***********************************************************************/
int8_t parse_delete_task(uint16_t wp_start_id, uint16_t wp_end_id)
{
	int8_t response = 0;
	int16_t wp_offset_start = get_task_wp_offset(wp_start_id);
	int16_t wp_offset_end = get_task_wp_offset(wp_end_id);
	uint16_t nb_wp = wp_end_id - wp_start_id + 1;
	uint16_t nb_left_shift = (nb_pending_wp-1) - wp_offset_end;
	if( wp_offset_start < 0 
		|| wp_offset_end < 0 
		|| nb_wp != wp_offset_end - wp_offset_start )
	{
		response = 2;
		return response;
	}
	for(uint8_t i=0; i < nb_left_shift; i++)
	{
		TASK_WP_LEFT_SHIFT(wp_offset_start+nb_wp+i, nb_wp);
	}
	
	nb_pending_wp = nb_pending_wp - nb_wp;
	return response;
}

/***********************************************************************
* FUNCTION    : parse_get_task
* DESCRIPTION : reserve
* INPUTS      : the sequence of task need reply
* RETURN      : task info
***********************************************************************/
struct Task_Info parse_get_task(uint16_t wp_start_id, uint16_t wp_end_id)
{
	struct Task_Info m_task_info;
	return m_task_info;
	
}

/*
operation_type: add=1, update=2, delete=3
land_type: home=1, reserve_land=2     
wp_type: coordinate   wgs84 = 1, relative_ENU = 2    
*/
int8_t parse_land_task(struct Land_Info dl_land_info)
{
	int8_t response =0;
	/*only parse WGS84 coordinate*/
	if(dl_land_info.wp_type != 1) 
	{
		response =2;
		return response;
	}
	
	//enum Land_Type m_land_type = (enum Land_Type)dl_land_info.land_type;

	uint8_t home_offset = 0;
	/*first land type is home,parse home waypoint*/
	if( *(dl_land_info.land_type)==HOME_LAND )
	{
		response = parse_land_task_home(dl_land_info);
		if(response)
		{
			return response;
		}
		home_offset = 1;
	}

	/*parse reserve land*/
	if( dl_land_info.land_type_length >= (1+home_offset) )
	{
		for(uint8_t i=0; i<(dl_land_info.land_type_length-home_offset); i++)
		{
			if(*(dl_land_info.land_type+i+home_offset) != RESERVE_LAND)  /*check all land_type */
			{
				response = 2;
				return response;
			}
		}
		response = parse_land_task_reserve(dl_land_info, home_offset);
	}
	
	return response;
}

static int8_t parse_land_task_home(struct Land_Info dl_land_info)
{
	int8_t response = 0;
	/*add/update home waypoint, add operation request wp_home_useful==FASLE, update is opposite*/
	if( (LAND_TASK_ADD==dl_land_info.operation_type)
		 || (LAND_TASK_UPDATE==dl_land_info.operation_type && wp_home_useful))  
	{
		if( 1 <= dl_land_info.waypoints_length )
		{
			struct EnuCoor_i enu_home;
			struct LlaCoor_i lla_home;   /* rad, e8 */
			lla_home.lon = *(dl_land_info.waypoints_lon);
			lla_home.lat = *(dl_land_info.waypoints_lat);
			if( task_lla_to_enu_convert(&enu_home, &lla_home) )
	   		{
				VECT2_COPY(wp_home, enu_home);	
				
				/*replace current pos with wp_home, if pos error <1m!!!*/
				struct EnuCoor_i temp_pos = *stateGetPositionEnu_i();
				struct EnuCoor_i diff_pos;
				VECT2_DIFF(diff_pos, temp_pos, enu_home)
				//if( abs(diff_pos.x)<POS_BFP_OF_REAL(1.0) && abs(diff_pos.y)<POS_BFP_OF_REAL(1.0) )
				{
					VECT2_COPY(wp_home, temp_pos);
				}
				wp_home_useful = TRUE;  /*home waypoint receive success*/
				oa_data.home.x = POS_FLOAT_OF_BFP(wp_home.x);
				oa_data.home.y = POS_FLOAT_OF_BFP(wp_home.y);
				oa_data.home_valid = TRUE;
			}
			else
			{
				response = 4; /*waypoint convert fail*/
			}
		}
		else
		{
			response = 3;  /*home waypoint can't accept||waypoints is empty*/
		}
	}

	/*home waypoint can't delete*/
	else
	{
		response = 5;
	}
	return response;
}

/*reserve*/
static int8_t parse_land_task_reserve(struct Land_Info dl_land_info, uint8_t offset)
{
	int8_t response = 0;
	uint8_t reserve_wp_length = dl_land_info.waypoints_length - offset;
	/*add waypoints*/
	if( LAND_TASK_ADD==dl_land_info.operation_type )
	{
		if( reserve_wp_length < (NB_RESERVE_LAND-nb_pending_reland) )
		{
			struct LlaCoor_i lla_re;   /* rad, e8 */
			struct EnuCoor_i enu_re;
			uint8_t last_nb_pending_reland = nb_pending_reland;
			for(uint8_t i=0; i<reserve_wp_length; i++ )
			{
				lla_re.lon = *(dl_land_info.waypoints_lon+i+offset);
				lla_re.lat = *(dl_land_info.waypoints_lat+i+offset);
				if( task_lla_to_enu_convert(&enu_re, &lla_re) )
		   		{
					VECT2_COPY(wp_reserve_land[nb_pending_reland], enu_re);	
					nb_pending_reland++;
				}
				else
				{
					response = 4;
					nb_pending_reland = last_nb_pending_reland;
					break;  /*convert error,jump out loop*/
				}
			}
		}
		else
		{
			response = 6;  /*reserve land waypoints space is no enought*/
		}
	}
	/*update waypoints*/
	else if( LAND_TASK_UPDATE==dl_land_info.operation_type )
	{
		if(reserve_wp_length == nb_pending_reland)
		{
			struct LlaCoor_i lla_re;   /* rad, e8 */
			for(uint8_t i=0; i<reserve_wp_length; i++ )
			{
				lla_re.lon = *(dl_land_info.waypoints_lon+i+offset);
				lla_re.lat = *(dl_land_info.waypoints_lat+i+offset);
				if( !task_lla_to_enu_convert(&temp_enu[i], &lla_re) )
		   		{
					response =4;
					break;   /* jump out the loop */
				}
			}
			if(!response)
			{
				for(uint8_t i=0; i<reserve_wp_length; i++ )
				{
					VECT2_COPY(wp_reserve_land[i], temp_enu[i]);
				}
			}
		}
		/*update waypoints number is wrong*/
		else
		{
			response = 7;
		}
	}
	else if( LAND_TASK_DELETE==dl_land_info.operation_type )
	{
		nb_pending_reland = 0; /*reset number of reland waypoint*/
	}
	return response;
}

int8_t command_delete_all_task(void)
{
	int8_t response =0;
	
	if( (autopilot_in_flight && GCS_CMD_PAUSE!=gcs_task_cmd) )
		 //||(!autopilot_in_flight && GCS_CMD_NONE!=gcs_task_cmd) )
	{
		response = 1;
		return response;
	}
	nb_pending_wp = 0;	 
	from_wp_useful = FALSE;
	next_wp.wp_id = 0;      //reset current wp_id
	return response;
}

int8_t check_oa_data_valid(void)
{
	bool_t ok = FALSE;

	ok = oa_data.spray_boundary_valid && oa_data.obstacles_valid && oa_data.home_valid;
	ok &= (oa_data.spray_boundary_vertices_num > 2);
	ok &= ((oa_data.obstacles_num * OA_OBSTACLE_CORNER_NUM) == oa_data.obstacles_vertices_num);

	if(oa_data.obstacles_num == 0)
	{
		return 0;
	}
	if(ok)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

static bool_t add_boundary_vertex(struct FloatVect2 *v, uint8_t max)
{
	if(oa_data.spray_boundary_valid)
	{
		return FALSE;
	}
	if(oa_data.spray_boundary_vertices_num >= max)
	{
		return FALSE;
	}
	if(oa_data.spray_boundary_vertices_num >= OA_MAX_BOUNDARY_VERTICES_NUM)
	{
		return FALSE;
	}
	VECT2_COPY(oa_data.spray_boundary_vertices_array[oa_data.spray_boundary_vertices_num], *v);
	++oa_data.spray_boundary_vertices_num;
	if(oa_data.spray_boundary_vertices_num == max)
	{
		oa_data.spray_boundary_valid = TRUE;
	}

	return TRUE;
}

static bool_t add_obstacles_vertex(struct FloatVect2 *v, uint8_t max)
{
	if(oa_data.obstacles_valid)
	{
		return FALSE;
	}
	if (oa_data.obstacles_vertices_num >= (max * OA_OBSTACLE_CORNER_NUM))
	{
		return FALSE;
	}
	if (oa_data.obstacles_vertices_num >= (OA_MAX_OBSTACLES_NUM * OA_OBSTACLE_CORNER_NUM))
	{
		return FALSE;
	}
	VECT2_COPY(oa_data.obstacles_vertices_array[oa_data.obstacles_vertices_num/OA_OBSTACLE_CORNER_NUM][oa_data.obstacles_vertices_num%OA_OBSTACLE_CORNER_NUM], *v);
	++oa_data.obstacles_vertices_num;
	if (oa_data.obstacles_vertices_num == (max * OA_OBSTACLE_CORNER_NUM))
	{
		oa_data.obstacles_num = max;
		oa_data.obstacles_valid = TRUE;
	}

	return TRUE;
}

int8_t parse_add_border(struct bp_Info m_bp_info)
{
	if(m_bp_info.length_bp_lat != m_bp_info.length_bp_lon)
	{
		return 4;
	}
	if(m_bp_info.length_bp_lat > OA_MAX_BOUNDARY_VERTICES_NUM)
	{
		return 4;
	}
	if(m_bp_info.total_bp_num > OA_MAX_BOUNDARY_VERTICES_NUM)
	{
		return 4;
	}

	struct LlaCoor_i lla; /* rad, e8 */
	struct EnuCoor_i temp_enu;
	for (uint8_t i = 0; i <  m_bp_info.length_bp_lon; i++)
	{
		lla.lon = *(m_bp_info.bp_points_lon + i);
		lla.lat = *(m_bp_info.bp_points_lat + i);
		if (task_lla_to_enu_convert(&temp_enu, &lla))
		{
			struct FloatVect2 v;
			v.x = POS_FLOAT_OF_BFP(temp_enu.x);
			v.y = POS_FLOAT_OF_BFP(temp_enu.y);
			if (!add_boundary_vertex(&v, m_bp_info.total_bp_num))
			{
				return 4;
			}
		}
		else
		{
			return 4;
		}
	}

	return 0;
}


int8_t parse_add_obstacle(struct op_Info m_op_info)
{
	if (m_op_info.length_op_lat != m_op_info.length_op_lon)
	{
		return 4;
	}
	if (m_op_info.length_op_lat > (OA_MAX_OBSTACLES_NUM * OA_OBSTACLE_CORNER_NUM))
	{
		return 4;
	}
	if (m_op_info.total_op_num > OA_MAX_OBSTACLES_NUM)
	{
		return 4;
	}

	struct LlaCoor_i lla; /* rad, e8 */
	struct EnuCoor_i temp_enu;
	for (uint8_t i = 0; i < m_op_info.length_op_lon; i++)
	{
		lla.lon = *(m_op_info.op_points_lon + i);
		lla.lat = *(m_op_info.op_points_lat + i);
		if (task_lla_to_enu_convert(&temp_enu, &lla))
		{
			struct FloatVect2 v;
			v.x = POS_FLOAT_OF_BFP(temp_enu.x);
			v.y = POS_FLOAT_OF_BFP(temp_enu.y);
			if (!add_obstacles_vertex(&v, m_op_info.total_op_num))
			{
				return 4;
			}
		}
		else
		{
			return 4;
		}
	}

	return 0;
}
/*
add_task error code
0 :success
1 :parse error
2 :space is not enought
3 :waypoints add repeat 
4 :waypoints paser error
5 :waypoints id is not in order
*/

/*
update_task error code
0 :success
1 :parse error
2 :id of wp is wrong
3 :waypoints paser error
*/

/*
land_task error code
0 :success
1 :number of waypoints is over 5
2 :error land type or wp_type(surport wgs84)
3 :home waypoint can't be accepted or waypoints is not only one
4 :waypoint convert fail
5 :home waypoint can't delete
6 :reserve land waypoints space is no enough
7 :update waypoints number is wrong
*/

/*
set_command id =1
0 :success
1 :parse error
2 :conflict cmd
3 :aircraft is not ready to start
*/
/****************************** END OF FILE ***************************/
