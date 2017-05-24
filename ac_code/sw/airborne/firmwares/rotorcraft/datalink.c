/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "subsystems/datalink/datalink.h"

#include "generated/modules.h"

#include "generated/settings.h"
#include "subsystems/datalink/downlink.h"

#include "messages.h"
//use for communication with new GCS and RC

#include "dl_protocol.h"
#if DATALINK==XBEE
#include "subsystems/datalink/xbee.h"
#endif

#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */

#ifdef GCS_V1_OPTION
#include "uplink_ac.h"
#include "uplink_gcs.h"
#include "uplink_rc.h"
#include "subsystems/datalink/datalink_ack.h"
#include "subsystems/datalink/downlink_xbee_periodic.h"

#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/gcs_nav_xbee.h"
#include "subsystems/mission/task_manage.h"
#include "subsystems/mission/task_spray_misc.h"
#endif	/* GCS_V1_OPTION */

#include "mcu_periph/uart.h"

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#if USE_GPS
#include "subsystems/gps.h"
#endif
#if defined GPS_DATALINK
#include "subsystems/gps/gps_datalink.h"
#endif

#ifdef UPGRADE_OPTION
#include "subsystems/fram/fram_if.h"
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"
#include "subsystems/ops/ops_msg_uart_def.h"
#include "subsystems/bbox/bbox_msg_if.h"
#include "subsystems/bbox/bbox_msg_def.h"
#include "subsystems/fram/fram_data.h"
#include "subsystems/fram/fram_class.h"
#include "subsystems/fram/fram_def.h"




#endif	/* UPGRADE_OPTION */

#include "subsystems/eng/eng_app_if.h"

#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"

#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "led.h"
#ifdef GCS_V1_OPTION
/*use for corret gen_message.ml generate dynamic array*/
#define DL_ADD_TASK_waypoints_lon_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+9+DL_ADD_TASK_wp_action_length(_payload))))

#define DL_ADD_TASK_waypoints_lon_cor(_payload)        ((int32_t*)(_payload+10+DL_ADD_TASK_wp_action_length(_payload)))

#define DL_ADD_TASK_waypoints_lat_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+10+DL_ADD_TASK_wp_action_length(_payload) \
	                                                     +4*DL_ADD_TASK_waypoints_lon_length_cor(_payload))))
	                                                     
#define DL_ADD_TASK_waypoints_lat_cor(_payload) ((int32_t*)(_payload+11+DL_ADD_TASK_wp_action_length(_payload) \
	                                                     +4*DL_ADD_TASK_waypoints_lon_length_cor(_payload)))


#define DL_UPDATE_TASK_waypoints_lon_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+9+DL_UPDATE_TASK_wp_action_length(_payload))))

#define DL_UPDATE_TASK_waypoints_lon_cor(_payload)        ((int32_t*)(_payload+10+DL_UPDATE_TASK_wp_action_length(_payload)))

#define DL_UPDATE_TASK_waypoints_lat_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+10+DL_UPDATE_TASK_wp_action_length(_payload) \
	                                                     +4*DL_UPDATE_TASK_waypoints_lon_length_cor(_payload))))	
	                                                     
#define DL_UPDATE_TASK_waypoints_lat_cor(_payload) ((int32_t*)(_payload+11+DL_UPDATE_TASK_wp_action_length(_payload) \
	                                                     +4*DL_UPDATE_TASK_waypoints_lon_length_cor(_payload)))	                                                     


#define DL_LAND_TASK_waypoints_lon_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+5+DL_LAND_TASK_land_type_length(_payload))))

#define DL_LAND_TASK_waypoints_lon_cor(_payload) ((int32_t*)(_payload+6+DL_LAND_TASK_land_type_length(_payload)))

#define DL_LAND_TASK_waypoints_lat_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+6+DL_LAND_TASK_land_type_length(_payload) \
	                                                     +4*DL_LAND_TASK_waypoints_lon_length_cor(_payload))))

#define DL_LAND_TASK_waypoints_lat_cor(_payload) ((int32_t*)(_payload+7+DL_LAND_TASK_land_type_length(_payload) \
	                                                     +4*DL_LAND_TASK_waypoints_lon_length_cor(_payload)))



#define DL_ADD_BORDER_INFO_bp_points_lat_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+5+4*DL_ADD_BORDER_INFO_bp_points_lon_length(_payload))))
#define DL_ADD_BORDER_INFO_bp_points_lat_cor(_payload) ((int32_t*)(_payload+6+4*DL_ADD_BORDER_INFO_bp_points_lon_length(_payload)))

#define DL_ADD_OBSTACLE_INFO_op_points_lat_length_cor(_payload) ((uint8_t)(*((uint8_t*)_payload+5+4*DL_ADD_OBSTACLE_INFO_op_points_lon_length(_payload))))
#define DL_ADD_OBSTACLE_INFO_op_points_lat_cor(_payload) ((int32_t*)(_payload+6+4*DL_ADD_OBSTACLE_INFO_op_points_lon_length(_payload)))
                                             
#endif //GCS_V1_OPTION

#ifdef CALIBRATION_OPTION
#include "calibration.h"
#endif	/* CALIBRATION_OPTION */

#define DEBUG_XBEE_COMMU 0

#define IdOfMsg(x) (x[1])
#define TypeOfMsg(x) (x[0])

#if USE_NPS
bool_t datalink_enabled = TRUE;
#endif


void dl_parse_msg(void)
{
  uint8_t msg_type = TypeOfMsg(dl_buffer);
  uint8_t msg_id = IdOfMsg(dl_buffer);

 #ifdef GCS_V1_OPTION
  //process RC uplink message 
  if(msg_type == XBEE_TYPE_RC)   
  {
    #ifndef COMM_DIRECT_CONNECT
	if(xbee_con_info.rc_con_available != TRUE)  return;
	#endif	/* COMM_DIRECT_CONNECT */

	switch (msg_id) 
	{  
		case DL_RC_MOTION_CMD:
		{
			rc_set_connect();
			rc_motion_cmd_execution(DL_RC_MOTION_CMD_motion_cmd(dl_buffer));
			break;
        }
		
		case DL_RC_SET_CMD:
		{  
			rc_set_connect();
			rc_set_cmd_parse(DL_RC_SET_CMD_set_cmd(dl_buffer));
			break;
		}
		
       case DL_HEART_BEAT_RC_STATE:
		{
			rc_set_rc_type();    //real RC communication set type
			rc_set_connect();
			send_heart_beat_A2R_msg();
			break;
		}

		#ifndef COMM_DIRECT_CONNECT
		case DL_RC_BIND_STATE: 
		{
			/*check rc serial_code*/
			bool_t code_check = TRUE;
			const char rc_serial_code[10] = RC_SERIAL_CODE;
			char *pt_serial_code = DL_RC_BIND_STATE_serial_code(dl_buffer);
			for(uint8_t i=0; i<10;i++)
			{
				if( *(pt_serial_code+i)!=rc_serial_code[i] )  
				{
					code_check = FALSE;         
					break; 
				}
			}
			//ack with bind rc message,if send fail it will continual reveice rc_bind state message
			if(code_check)
			{
				uint8_t ac_serial_code[10]=A2R_SERIAL_CODE;
				xbee_tx_header(XBEE_ACK,XBEE_ADDR_RC);		  
				DOWNLINK_SEND_BIND_RC(SecondChannel, SecondDevice, ac_serial_code);
			}
			break;
	    }
		#endif	/* COMM_DIRECT_CONNECT */
		
		#ifdef CALIBRATION_OPTION
		case DL_CALIBRATION_RESULT_RC_ACK_STATE:
		{
			rc_set_connect();
			cali_mag_state_init();
			break;
		}
		#endif
		
	    default:   
			break;
    }
  }
  
  //process android GCS uplink message 
 if(msg_type == XBEE_TYPE_GCS)   
 {
    #ifndef COMM_DIRECT_CONNECT
	if(xbee_con_info.gcs_con_available != TRUE)  return;
	#endif	/* COMM_DIRECT_CONNECT */
	
  	switch (msg_id) 
	{
       case DL_HEART_BEAT_GCS_STATE: 
		{
			gcs_vrc_set_connect();
			break;
	    }
	   
       case DL_ADD_TASK:
	   	{
			gcs_vrc_set_connect();
			
			struct Task_Info dl_task_info;
			int8_t response = 0;
			dl_task_info.task_code = DL_ADD_TASK_task_code(dl_buffer);
			dl_task_info.length_wp_action = DL_ADD_TASK_wp_action_length(dl_buffer);
			dl_task_info.length_wp_lon = DL_ADD_TASK_waypoints_lon_length_cor(dl_buffer);
			dl_task_info.length_wp_lat = DL_ADD_TASK_waypoints_lat_length_cor(dl_buffer);
			dl_task_info.wp_type = DL_ADD_TASK_wp_type(dl_buffer);			
			if( dl_task_info.length_wp_action == dl_task_info.length_wp_lon
				&& dl_task_info.length_wp_action == dl_task_info.length_wp_lat 
				&& (1==dl_task_info.wp_type || 2==dl_task_info.wp_type) )
			{
				dl_task_info.wp_type = DL_ADD_TASK_wp_type(dl_buffer);
				dl_task_info.wp_start_id = DL_ADD_TASK_wp_start_id(dl_buffer);
				dl_task_info.wp_end_id = DL_ADD_TASK_wp_end_id(dl_buffer);
				dl_task_info.length_wp_action = DL_ADD_TASK_wp_action_length(dl_buffer);
				dl_task_info.wp_action = DL_ADD_TASK_wp_action(dl_buffer);
				dl_task_info.waypoints_lon = DL_ADD_TASK_waypoints_lon_cor(dl_buffer);
				dl_task_info.waypoints_lat = DL_ADD_TASK_waypoints_lat_cor(dl_buffer);
				response = parse_add_task(dl_task_info);
			}
			else
			{
				response = 1;  
			}
			enum Task_Ack_Type task_ack_type = TASK_ADD;
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_TASK_ACK_STATE(SecondChannel, SecondDevice, 
		   	                               &dl_task_info.task_code,
		   	                               &task_ack_type,
		   	                               &response);
			break;
       }
		case DL_ADD_BORDER_INFO:
		{
			gcs_vrc_set_connect();
			struct bp_Info dl_bp_info;
			int8_t response = 0;
			dl_bp_info.task_code = DL_ADD_BORDER_INFO_task_code(dl_buffer);
			dl_bp_info.total_bp_num = DL_ADD_BORDER_INFO_bp_sum(dl_buffer);
			dl_bp_info.length_bp_lon = DL_ADD_BORDER_INFO_bp_points_lon_length(dl_buffer);
			dl_bp_info.length_bp_lat = DL_ADD_BORDER_INFO_bp_points_lat_length_cor(dl_buffer);
			if((dl_bp_info.length_bp_lon == dl_bp_info.length_bp_lat)
				&&(dl_bp_info.length_bp_lon <= OA_MAX_BOUNDARY_VERTICES_NUM))
			{
				dl_bp_info.bp_points_lon = DL_ADD_BORDER_INFO_bp_points_lon(dl_buffer);
				dl_bp_info.bp_points_lat = DL_ADD_BORDER_INFO_bp_points_lat_cor(dl_buffer);
				response = parse_add_border(dl_bp_info); 
			}
			else
			{
				response = 1;  //length error
			}
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_P_BORDER_ACK(SecondChannel, SecondDevice, 
		   	                               &dl_bp_info.task_code,
		   	                               &response);
			break;
		}

		case DL_ADD_OBSTACLE_INFO:
		{
			gcs_vrc_set_connect();
			struct op_Info dl_op_info;
			int8_t response = 0;
			dl_op_info.task_code = DL_ADD_OBSTACLE_INFO_task_code(dl_buffer);
			dl_op_info.total_op_num = DL_ADD_OBSTACLE_INFO_op_sum(dl_buffer);
			dl_op_info.length_op_lon = DL_ADD_OBSTACLE_INFO_op_points_lon_length(dl_buffer);
			dl_op_info.length_op_lat = DL_ADD_OBSTACLE_INFO_op_points_lat_length_cor(dl_buffer);
			if((dl_op_info.length_op_lat == dl_op_info.length_op_lat)
				&& (dl_op_info.length_op_lat <= (OA_MAX_OBSTACLES_NUM * OA_OBSTACLE_CORNER_NUM)))
			{
				dl_op_info.op_points_lon = DL_ADD_OBSTACLE_INFO_op_points_lon(dl_buffer);
				dl_op_info.op_points_lat = DL_ADD_OBSTACLE_INFO_op_points_lat_cor(dl_buffer);
				response = parse_add_obstacle(dl_op_info); 
			}
			else 
			{
				response = 1;  //length error
			}
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_P_OBSTACLE_ACK(SecondChannel, SecondDevice, 
		   	                               &dl_op_info.task_code,
		   	                               &response);
			break;
		}
	  	
	   case DL_UPDATE_TASK:
	   	{
			gcs_vrc_set_connect();
			
			struct Task_Info dl_task_info;
			int8_t response = 0;
			dl_task_info.task_code = DL_UPDATE_TASK_task_code(dl_buffer);
			dl_task_info.length_wp_action = DL_UPDATE_TASK_wp_action_length(dl_buffer);
			dl_task_info.length_wp_lon = DL_UPDATE_TASK_waypoints_lon_length_cor(dl_buffer);
			dl_task_info.length_wp_lat = DL_UPDATE_TASK_waypoints_lat_length_cor(dl_buffer);
			dl_task_info.wp_type = DL_UPDATE_TASK_wp_type(dl_buffer);
			if( dl_task_info.length_wp_action == dl_task_info.length_wp_lon
				&& dl_task_info.length_wp_action == dl_task_info.length_wp_lat 
				&& (1==dl_task_info.wp_type || 2==dl_task_info.wp_type) )
			{
				dl_task_info.wp_type = DL_UPDATE_TASK_wp_type(dl_buffer);
				dl_task_info.wp_start_id = DL_UPDATE_TASK_wp_start_id(dl_buffer);
				dl_task_info.wp_end_id = DL_UPDATE_TASK_wp_end_id(dl_buffer);
				dl_task_info.length_wp_action = DL_UPDATE_TASK_wp_action_length(dl_buffer);
				dl_task_info.wp_action = DL_UPDATE_TASK_wp_action(dl_buffer);
				dl_task_info.waypoints_lon = DL_UPDATE_TASK_waypoints_lon_cor(dl_buffer);
				dl_task_info.waypoints_lat = DL_UPDATE_TASK_waypoints_lat_cor(dl_buffer);
				response = parse_update_task(dl_task_info);

			}
			else
			{
				response = 1;  /*length error*/
			}
			enum Task_Ack_Type task_ack_type = TASK_UPDATE;
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_TASK_ACK_STATE(SecondChannel, SecondDevice, 
		   	                               &dl_task_info.task_code,
		   	                               &task_ack_type,
		   	                               &response);
			break;
       }

	   case DL_DELETE_TASK:
	   	{
			gcs_vrc_set_connect();
			
			uint8_t task_code = DL_DELETE_TASK_task_code(dl_buffer);
			uint8_t wp_start_id = DL_DELETE_TASK_wp_start_id(dl_buffer);
			uint8_t wp_end_id = DL_DELETE_TASK_wp_end_id(dl_buffer);
			int8_t response = 0;
			response = parse_delete_task(wp_start_id, wp_end_id);
			enum Task_Ack_Type task_ack_type = TASK_DELETE;
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_TASK_ACK_STATE(SecondChannel, SecondDevice, 
		   	                               &task_code,
		   	                               &task_ack_type,
		   	                               &response);
			break;
	   	}

	   case DL_GET_TASK:
	   	{
			gcs_vrc_set_connect();
			
			uint8_t task_code = DL_GET_TASK_task_code(dl_buffer);
			uint8_t wp_start_id = DL_GET_TASK_wp_start_id(dl_buffer);
			uint8_t wp_end_id = DL_GET_TASK_wp_end_id(dl_buffer);
			//
			break;
	   	}

	   case DL_LAND_TASK:
	   	{
			gcs_vrc_set_connect();
			
			int8_t response = 0;
			struct Land_Info dl_land_info;
			dl_land_info.operation_type = DL_LAND_TASK_operation_type(dl_buffer);
			dl_land_info.wp_type = DL_LAND_TASK_wp_type(dl_buffer);
			dl_land_info.land_type_length = DL_LAND_TASK_land_type_length(dl_buffer);
			dl_land_info.land_type = DL_LAND_TASK_land_type(dl_buffer);
			dl_land_info.waypoints_length = DL_LAND_TASK_waypoints_lon_length_cor(dl_buffer);
			if( dl_land_info.waypoints_length <= 5
				&& dl_land_info.waypoints_length == DL_LAND_TASK_waypoints_lat_length_cor(dl_buffer)
				&& dl_land_info.waypoints_length == dl_land_info.land_type_length                   )
			{
				dl_land_info.waypoints_lon = DL_LAND_TASK_waypoints_lon_cor(dl_buffer);
				dl_land_info.waypoints_lat = DL_LAND_TASK_waypoints_lat_cor(dl_buffer);
				response = parse_land_task(dl_land_info);
			}
			else
			{
				response = 1;
			}
			xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
			DOWNLINK_SEND_LAND_TASK_ACK_STATE(SecondChannel, SecondDevice, 
		   	                               &dl_land_info.operation_type,
		   	                               &response);
			break;
	   	}
	   
       case DL_SET_CONFIG:
		{  
			gcs_vrc_set_connect();
			
		   uint8_t id = DL_SET_CONFIG_parameter_id(dl_buffer);
		   uint8_t length = DL_SET_CONFIG_parameter_value_length(dl_buffer);
		   int8_t *pt_value = DL_SET_CONFIG_parameter_value(dl_buffer);
		   DlSetConfig(id, pt_value ,length);
		   send_aircraft_info_state();
		   break;
        }
		
		case DL_SET_COMMAND:
		{
			gcs_vrc_set_connect();
			
		   uint8_t id = DL_SET_COMMAND_command_id(dl_buffer);
		   //uint8_t length = DL_SET_COMMAND_command_value_length(dl_buffer);
		   uint8_t pt_value = DL_SET_COMMAND_command_value(dl_buffer);
		   int8_t response = (int8_t)(DlSetGcsCommand(id, pt_value));
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		   DOWNLINK_SEND_SET_COMMAND_ACK_STATE(SecondChannel, SecondDevice, &id, &pt_value, &response);
		   break;
		}
		
		case DL_BIND_AIRCRAFT: 
		{ 
		  /*check gcs serial_code*/
		  uint8_t code_check = TRUE;
		  //const char gcs_serial_code[] = AC_SN_CODE;
		  uint8_t *temp_pt = eng_get_product_series_number();
		  char *pt_serial_code = DL_BIND_AIRCRAFT_sn_code(dl_buffer);
		  for(uint8_t i=0; i<SIZE_OF_PRODUCT_SERIES_NUMBER;i++)
		  {    
		  	 if( *(pt_serial_code+i)!=*(temp_pt+i) )  
			 {
			 	code_check = FALSE;
			 	break; 
		  	 }
		  }
		  if(code_check)
		  {
		  		XbeeSetSuccessBind();
				gcs_vrc_set_connect();
		  }
		  else 
		  {
		  		XbeeSetFailBind();
		  }
         xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		  DOWNLINK_SEND_AIRCRAFT_BIND_ACK_STATE(SecondChannel, SecondDevice, &code_check);
		  send_aircraft_info_state();
		  break;
	   }	
		
		case DL_EMERGENCY_RECORD_ACK_STATE:
		{
			gcs_vrc_set_connect();
			
			if(DL_EMERGENCY_RECORD_ACK_STATE_response(dl_buffer)==0)
			{
				spray_bac_msg_stop();    /*gcs get record success, stop send msg timer */
			}
			break;
		}

       /*below is gcs virtual rc functions*/
		case DL_HEART_BEAT_VRC_STATE:
		{
			gcs_vrc_set_connect();
			send_heart_beat_A2VR_msg();
			break;
		}		
		case DL_GCS_VRC_MOTION_CMD:
		{
			gcs_vrc_set_connect();			
			rc_motion_cmd_execution(DL_GCS_VRC_MOTION_CMD_motion_cmd(dl_buffer));
			gcs_set_rc_type();
			break;
		}
		case DL_GCS_VRC_SET_CMD:
		{
			rc_set_cmd_parse(DL_GCS_VRC_SET_CMD_set_cmd(dl_buffer));
			gcs_set_rc_type();
			gcs_vrc_set_connect();

			//gcs_vrc_ack_timer();
			break;
		}
		
		#ifdef UPGRADE_OPTION
 		case DL_REQUEST_UPGRADE:
		{
			if(!autopilot_in_flight)
			{
				if( UPGRADE_TYPE_AC == DL_REQUEST_UPGRADE_type(dl_buffer) )
				{
					if(fram_write_swdl_mask() == 0)		//fram write swdl success.
					{
						uint8_t type = UPGRADE_TYPE_AC;
						uint8_t ug_state = UPGRADE_RES_OK;
						xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
						DOWNLINK_SEND_UPGRADE_RESPONSE(SecondChannel, SecondDevice, &type, &ug_state);
						fram_write(CL_GCS_MAC_ADDR, 0,  xbee_con_info.gcs_addr); // write GCS add to fram 
						tm_create_timer(TIMER_UPGRADE_RES_TX_TIMEOUT, (2 SECONDS), TIMER_ONE_SHOT,0);//wait 2s to reboot.
					}
					else
					{
						uint8_t type = UPGRADE_TYPE_AC;
						uint8_t ug_state = UPGRADE_RES_FAIL;
						xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
						DOWNLINK_SEND_UPGRADE_RESPONSE(SecondChannel, SecondDevice, &type, &ug_state);
					}
				}
				else if( UPGRADE_TYPE_OPS == DL_REQUEST_UPGRADE_type(dl_buffer) )  //update ops
				{
					uint8_t Param[2]={OPS_PRIO_GENERAL,OPS_UPDATE_REQ_DATA};
					ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_UPGRADE_ID,2, Param);
					
				}
				#ifdef BBOX_OPTION
				else if( UPGRADE_TYPE_BBOX == DL_REQUEST_UPGRADE_type(dl_buffer) )  //update bblox
				{
					bbox_msg_request_upgrade();
				}
				#endif	/* BBOX_OPTION */
			}
			break;
		}

		case DL_QUERY_UPGRADE_STATUS :
		{
			if(UPGRADE_TYPE_OPS==DL_QUERY_UPGRADE_STATUS_type(dl_buffer))
			{
				uint8_t Param[2]={OPS_PRIO_GENERAL,OPS_UPDATE_ASK_READY_DATA};
				ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_UPGRADE_ID,2, Param);
			}
			#ifdef BBOX_OPTION
			else if(UPGRADE_TYPE_BBOX==DL_QUERY_UPGRADE_STATUS_type(dl_buffer))
			{
				bbox_msg_ready_status();
			}
			#endif	/* BBOX_OPTION */
			break;
		}
		case DL_GCS_READY_OK:    //  GCS  ready ok
		{
			if(UPGRADE_TYPE_OPS==DL_GCS_READY_OK_type(dl_buffer))
			{
				uint8_t type = UPGRADE_TYPE_OPS;
				uint8_t ug_state = UPGRADE_RES_PASS;   //next frame
				xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
				DOWNLINK_SEND_REQUESET_FIRMWARE(SecondChannel, SecondDevice, &type, &ug_state);
			}
			#ifdef BBOX_OPTION
			else if(UPGRADE_TYPE_BBOX==DL_GCS_READY_OK_type(dl_buffer))
			{
				uint8_t type = UPGRADE_TYPE_BBOX;
				uint8_t ug_state = UPGRADE_RES_PASS;   //next frame
				xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
				DOWNLINK_SEND_REQUESET_FIRMWARE(SecondChannel, SecondDevice, &type, &ug_state);
			}
			#endif	/* BBOX_OPTION */
			break;
		}
		case DL_GCS_SEND_FIRMWARE:
		{
			if(UPGRADE_TYPE_OPS==DL_GCS_SEND_FIRMWARE_type(dl_buffer))
			{
				uint8_t Data_Length=DL_GCS_SEND_FIRMWARE_data_length(dl_buffer);
				uint8_t *Data_Param=DL_GCS_SEND_FIRMWARE_data(dl_buffer);
				ops_update_frame_trans(Data_Param,Data_Length);
			}
			#ifdef BBOX_OPTION
			else if(UPGRADE_TYPE_BBOX==DL_GCS_SEND_FIRMWARE_type(dl_buffer))
			{
				uint8_t Data_Length=DL_GCS_SEND_FIRMWARE_data_length(dl_buffer);
				uint8_t *Data_Param=DL_GCS_SEND_FIRMWARE_data(dl_buffer);
				bbox_msg_send_frame(Data_Param,Data_Length);
			}
			#endif	/* BBOX_OPTION */
			break;
		}
		
		case DL_END_UPGRADE:
		{
			uint8_t i=0;
			uint8_t Param[9];
			uint8_t check_data_length = DL_END_UPGRADE_check_data_length(dl_buffer);
			uint8_t *check_data =	DL_END_UPGRADE_check_data(dl_buffer);
			for(i=0;i<check_data_length;i++)
			{
				Param[2+i]=*(check_data+i);
			}
			if(UPGRADE_TYPE_OPS==DL_END_UPGRADE_type(dl_buffer))
			{
				Param[0]=OPS_PRIO_GENERAL;
				Param[1]=OPS_UPDATE_OVER_DATA;
				ops_comm_send_frame(OPS_REQ_ACK_NOT_NEEDED,OPS_UPGRADE_ID,9, Param);
			}
			#ifdef BBOX_OPTION
			else if(UPGRADE_TYPE_BBOX==DL_END_UPGRADE_type(dl_buffer))
			{
				Param[0]=BBOX_UPGRADE_SERVIC;
				Param[1]=BBOX_UPDATE_OVER_DATA;
				bbox_msg_update_over(9,Param);
			}
			#endif	/* BBOX_OPTION */
			break;
		}
		#endif	/* UPGRADE_OPTION */
		
	    default: break;
    }

  }
 #endif   

  //process ubuntu GCS uplink message  
 #if PERIODIC_TELEMETRY
  if( msg_type == XBEE_TYPE_DEFAULT )   //process ubuntu GCS
  {
	switch (msg_id) 
	{
		#if USE_MANU_DEBUG
		case DL_WRITE_SN :
		{
			uint8_t result = 1;
			uint8_t r_pt[DL_WRITE_SN_data_length(dl_buffer)+10];  //add 10 avoid point overrun
			uint8_t *t_pt = DL_WRITE_SN_data(dl_buffer);
			fram_sn_data_write(t_pt);
			fram_sn_data_read(r_pt);
			for(uint8_t i=0; i<DL_WRITE_SN_data_length(dl_buffer); i++)
			{
				if( *(t_pt+i)!=r_pt[i] )
				{
					result = 0;
					break;
				}
			}
			DOWNLINK_SEND_WRITE_SN_ACK(MdebugChannel, MdebugDevice, &r_pt[0], &result);
			
			fram_mag_cali_default_data_write();   //while write sn, write default mag var to fram
		}
		break;

		case DL_MC_COMMAND:
		{
			uint8_t id = DL_MC_COMMAND_mc_id(dl_buffer);
		    uint8_t value = DL_MC_COMMAND_value(dl_buffer);
			DlSetMCCommand(id, value);
	
			DOWNLINK_SEND_MC_COMMAND(MdebugChannel, MdebugDevice, &id, &value);
		}
		break;
		#endif
		
	    case  DL_PING: {
		  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
		  #if USE_MANU_DEBUG
		  DOWNLINK_SEND_PONG(MdebugChannel, MdebugDevice);
		  #endif
		  #if OPEN_PC_DATALINK
  		  DOWNLINK_SEND_PONG(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE);
		  #endif
	    }
	    break;

	    case DL_SETTING : {
	      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
	      uint8_t i = DL_SETTING_index(dl_buffer);
	      float var = DL_SETTING_value(dl_buffer);
	      DlSetting(i, var);
		   xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
		  #if OPEN_PC_DATALINK
  		  DOWNLINK_SEND_DL_VALUE(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &i, &var);
		  #endif
	    }
	    break;

	    case DL_GET_SETTING : {
	      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
	      uint8_t i = DL_GET_SETTING_index(dl_buffer);
	      float val = settings_get_value(i);
		  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
		  #if OPEN_PC_DATALINK
  		  DOWNLINK_SEND_DL_VALUE(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &i, &val);
		  #endif
	    }
	    break;

		#if defined USE_NAVIGATION
	    case DL_BLOCK : {
	      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) { break; }
	      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
	    }
	    break;

	    case DL_MOVE_WP : {
	      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
	      if (ac_id != AC_ID) { break; }
	      if (stateIsLocalCoordinateValid()) {
	        uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
	        struct LlaCoor_i lla;
	        lla.lat = DL_MOVE_WP_lat(dl_buffer);
	        lla.lon = DL_MOVE_WP_lon(dl_buffer);
	        /* WP_alt from message is alt above MSL in mm
	         * lla.alt is above ellipsoid in mm
	         */
	        lla.alt = DL_MOVE_WP_alt(dl_buffer) - state.ned_origin_i.hmsl +
	                  state.ned_origin_i.lla.alt;
	        waypoint_move_lla(wp_id, &lla);
	      }
	    }
	    break;
		#endif /* USE_NAVIGATION */
		/*
		#if def RADIO_CONTROL_TYPE_DATALINK
	    case DL_RC_3CH :
			#ifdef RADIO_CONTROL_DATALINK_LED
	      LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
			#endif //RADIO_CONTROL_DATALINK_LED
	      parse_rc_3ch_datalink(
	        DL_RC_3CH_throttle_mode(dl_buffer),
	        DL_RC_3CH_roll(dl_buffer),
	        DL_RC_3CH_pitch(dl_buffer));
	      break;
	    case DL_RC_4CH :
	      if (DL_RC_4CH_ac_id(dl_buffer) == AC_ID) {
			#ifdef RADIO_CONTROL_DATALINK_LED
	        LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
			#endif //RADIO_CONTROL_DATALINK_LED
	        parse_rc_4ch_datalink(DL_RC_4CH_mode(dl_buffer),
	                              DL_RC_4CH_throttle(dl_buffer),
	                              DL_RC_4CH_roll(dl_buffer),
	                              DL_RC_4CH_pitch(dl_buffer),
	                              DL_RC_4CH_yaw(dl_buffer));
	      }
	      break;
		#endif // RADIO_CONTROL_TYPE_DATALINK
		*/
		#if defined GPS_DATALINK
	    case DL_REMOTE_GPS :
	      // Check if the GPS is for this AC
	      if (DL_REMOTE_GPS_ac_id(dl_buffer) != AC_ID) { break; }

      // Parse the GPS
      parse_gps_datalink(
        DL_REMOTE_GPS_numsv(dl_buffer),
        DL_REMOTE_GPS_ecef_x(dl_buffer),
        DL_REMOTE_GPS_ecef_y(dl_buffer),
        DL_REMOTE_GPS_ecef_z(dl_buffer),
        DL_REMOTE_GPS_lat(dl_buffer),
        DL_REMOTE_GPS_lon(dl_buffer),
        DL_REMOTE_GPS_alt(dl_buffer),
        DL_REMOTE_GPS_hmsl(dl_buffer),
        DL_REMOTE_GPS_ecef_xd(dl_buffer),
        DL_REMOTE_GPS_ecef_yd(dl_buffer),
        DL_REMOTE_GPS_ecef_zd(dl_buffer),
        DL_REMOTE_GPS_tow(dl_buffer),
        DL_REMOTE_GPS_course(dl_buffer));
      break;
#endif
#if USE_GPS
    case DL_GPS_INJECT :
      // Check if the GPS is for this AC
      if (DL_GPS_INJECT_ac_id(dl_buffer) != AC_ID) { break; }

      // GPS parse data
      gps_inject_data(
        DL_GPS_INJECT_packet_id(dl_buffer),
        DL_GPS_INJECT_data_length(dl_buffer),
        DL_GPS_INJECT_data(dl_buffer)
        );
      break;
#endif

    case DL_GUIDED_SETPOINT_NED:
      if (DL_GUIDED_SETPOINT_NED_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t flags = DL_GUIDED_SETPOINT_NED_flags(dl_buffer);
      float x = DL_GUIDED_SETPOINT_NED_x(dl_buffer);
      float y = DL_GUIDED_SETPOINT_NED_y(dl_buffer);
      float z = DL_GUIDED_SETPOINT_NED_z(dl_buffer);
      float yaw = DL_GUIDED_SETPOINT_NED_yaw(dl_buffer);
      switch (flags) {
        case 0x00:
        case 0x02:
          /* local NED position setpoints */
          autopilot_guided_goto_ned(x, y, z, yaw);
          break;
        case 0x01:
          /* local NED offset position setpoints */
          autopilot_guided_goto_ned_relative(x, y, z, yaw);
          break;
        case 0x03:
          /* body NED offset position setpoints */
          autopilot_guided_goto_body_relative(x, y, z, yaw);
          break;
        default:
          /* others not handled yet */
          break;
      }
      break;
    default:
      break;
   }
  }
  #endif
  /* Parse modules datalink */
  //modules_parse_datalink(msg_id); //TODOM:
}
