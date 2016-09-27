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


struct Task_Wp task_wp[NB_TASK];
uint8_t nb_pending_wp;

struct Int32Vect2 wp_home;
bool_t wp_home_useful;
struct Int32Vect2 wp_reserve_land[NB_RESERVE_LAND];
uint8_t nb_pending_reland;   /*number of wp_reserve_land*/

struct EnuCoor_i temp_enu[20];   //only use for update_task
struct Task_Wp_Enu from_wp;  
struct Task_Wp_Enu next_wp;          //use for get next waypoint


static int8_t parse_land_task_home(struct Land_Info dl_land_info);
static int8_t parse_land_task_reserve(struct Land_Info dl_land_info, uint8_t offset);


void task_manage_init(void)
{
	nb_pending_wp = 0;
	wp_home_useful = FALSE;
	nb_pending_reland = 0;
	
}

uint8_t parse_gcs_cmd( uint8_t cmd)
{
	uint8_t response = 0;
	enum Gcs_Task_Cmd gcs_cmd = (enum Gcs_Task_Cmd)cmd;
   #if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
    DOWNLINK_SEND_DEBUG_CMD(DefaultChannel, DefaultDevice, &gcs_cmd, &gcs_task_cmd, &wp_home_useful);
   #endif

	/*request first gcs_cmd is GCS_CMD_START*/
	if(GCS_CMD_NONE==gcs_task_cmd)
	{	
		if(GCS_CMD_START!=gcs_cmd)//"|| autopilot_in_flight)" may need start in air
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
		response =3;  /*not in flight*/
		return response;
	}
	
	switch (gcs_cmd)
	{	
		case GCS_CMD_START:
			//pause to start need add confirm
			if( (GCS_CMD_PAUSE==gcs_task_cmd && FALSE==from_wp_useful)
				 || GCS_CMD_NONE==gcs_task_cmd
				 || GCS_CMD_START==gcs_task_cmd)
			{
				gcs_task_cmd = GCS_CMD_START;
				gcs_cmd_interrupt = TRUE;
			}
			else
			{
				response = 2;  /*conflict cmd*/
			}
			break;
			
		case GCS_CMD_PAUSE:
			if(GCS_CMD_CONTI != gcs_task_cmd)
			{
				gcs_task_cmd = GCS_CMD_PAUSE;
				gcs_cmd_interrupt = TRUE;
			}
			else
			{
				response = 2;
			}
			break;
			
		case GCS_CMD_CONTI:
			if(GCS_CMD_CONTI==gcs_task_cmd || GCS_CMD_PAUSE==gcs_task_cmd)
			{
				gcs_task_cmd = GCS_CMD_CONTI;
				gcs_cmd_interrupt = TRUE;
			}
			else
			{
				response = 2;
			}
			break;
			
		case GCS_CMD_BHOME:
			gcs_task_cmd = GCS_CMD_BHOME;
			gcs_cmd_interrupt = TRUE;
			break;
			
		case GCS_CMD_RELAND:
			if( 0 >= nb_pending_reland )
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
			//TODOM add lock motors, danger!!!
			gcs_task_cmd = GCS_CMD_LOCK;
			gcs_cmd_interrupt = TRUE;
			break;

		default:
			response = 1;  /*parse error*/
			break;
	}
	return response;
}


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
	uint8_t max_pending_id = get_max_pending_id();
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

int8_t parse_delete_task(uint8_t wp_start_id, uint8_t wp_end_id)
{
	int8_t response = 0;
	int8_t wp_offset_start = get_task_wp_offset(wp_start_id);
	int8_t wp_offset_end = get_task_wp_offset(wp_end_id);
	uint8_t nb_wp = wp_end_id - wp_start_id + 1;
	uint8_t nb_left_shift = (nb_pending_wp-1) - wp_offset_end;
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

struct Task_Info parse_get_task(uint8_t wp_start_id, uint8_t wp_end_id)
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
	if( (LAND_TASK_ADD==dl_land_info.operation_type && !wp_home_useful)
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
				wp_home_useful = TRUE;  /*home waypoint receive success*/
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
	return response;
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