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
#ifdef GCS_V1_OPTION
#include "uplink_ac.h"
#include "uplink_gcs.h"
#include "uplink_rc.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/datalink/datalink_ack.h"
#include "subsystems/datalink/downlink_xbee_periodic.h"
#endif
#endif

#include "mcu_periph/uart.h"

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
//#include "subsystems/radio_control.h"
#endif

#if defined GPS_DATALINK
#include "subsystems/gps/gps_datalink.h"
#endif

//#include "subsystems/radio_control/rc_datalink.h"

#include "firmwares/rotorcraft/navigation.h"

#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "led.h"
#ifdef GCS_V1_OPTION
#include "subsystems/datalink/xbee_msg_def.h"
#endif //GCS_V1_OPTION

#define DEBUG_XBEE_COMMU 0

#define IdOfMsg(x) (x[1])
#define TypeOfMsg(x) (x[0])

void dl_parse_msg(void)
{
  uint8_t msg_type = TypeOfMsg(dl_buffer);
  uint8_t msg_id = IdOfMsg(dl_buffer);

 #if DATALINK==XBEE && defined GCS_V1_OPTION
  //process RC uplink message 
  if(msg_type == XBEE_TYPE_RC)   
  { //serial_code confirm
  	if(xbee_con_info.rc_con_available != TRUE)  return;
	switch (msg_id) 
	{  /*
	    #define DL_RC_MOTION_CMD 1
        #define DL_RC_SET_CMD 2
        #define DL_RC_BIND_STATE 101
        #define DL_HEART_BEAT_RC_STATE 102
        */
        case DL_RC_MOTION_CMD:
		{
		   //need give ack message with processing result
	       uint8_t response=rc_motion_cmd_execution(DL_RC_MOTION_CMD_motion_cmd(dl_buffer));
		   //according to return var,give reponse(RC_MOTION_CMD_ACK_STATE)
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_RC);	
		   DOWNLINK_SEND_RC_MOTION_CMD_ACK_STATE(DefaultChannel, DefaultDevice, &response);
		   break;
        }
		
		case DL_RC_SET_CMD:
		{  
		   //process set cmd with reponse
		   uint8_t response=rc_set_cmd_pasre(DL_RC_SET_CMD_set_cmd(dl_buffer));
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_RC);	
		   DOWNLINK_SEND_RC_SET_CMD_ACK_STATE(DefaultChannel, DefaultDevice, &response);
		   break;
		}
		
        case DL_HEART_BEAT_RC_STATE:
		{  
		   //static float last_time=get_sys_time_float();
		   xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);	
		   send_heart_beat_A2R_msg();
		   rc_count=0;  //reset rc_count,use for check rc_lost
		   rc_lost=FALSE;
		   break;
		}
		
	    case DL_RC_BIND_STATE: 
		{ 
		  //recheck rc serial_code
		  uint8_t rc_serial_code[10]=RC_SERIAL_CODE;
		  uint8_t *pt_serial_code=DL_RC_BIND_STATE_serial_code(dl_buffer);
		  for(uint8_t i=0; i<10;i++)
		  {    
		  	 if( *(pt_serial_code+1)!=rc_serial_code[i] )  break; 
		  }
		  //ack with bind rc message,if send fail it will continual reveice rc_bind state message
		  uint8_t ac_serial_code[10]=A2R_SERIAL_CODE;
		  xbee_tx_header(XBEE_ACK,XBEE_ADDR_RC);		  
		  DOWNLINK_SEND_BIND_RC(DefaultChannel, DefaultDevice, ac_serial_code);
		  last_response=rc_set_response_pack()+2;  //when bind success,update last_response of heart_beat
		  DOWNLINK_SEND_RC_SET_CMD_ACK_STATE(DefaultChannel, DefaultDevice, &last_response);  //when connect,in sure rc get ack
		  break;
	    }
	    default:    break;
    }
  }
  
  //process android GCS uplink message 
 if(msg_type == XBEE_TYPE_GCS)   
  {
  	if(xbee_con_info.gcs_con_available != TRUE)  return;
  	switch (msg_id) 
	{  /*
	    #define DL_SET_CONFIG 1
	    #define DL_SET_COMMAND 2
	    #define DL_ADD_MISSION 3
	    #define DL_UPDATE_MISSION 4
	    #define DL_DELETE_MISSION 5
	    #define DL_GET_MISSION 6
	    #define DL_BIND_AIRCRAFT 7
	    #define DL_HEART_BEAT_GCS_STATE 101
	    #define DL_MSG_gcs_NB 8
        */
        case DL_SET_CONFIG:
		{  
		   uint8_t id = DL_SET_CONFIG_parameter_id(dl_buffer);
		   uint8_t lenght = DL_SET_CONFIG_parameter_value_length(dl_buffer);
		   int8_t *pt_value = DL_SET_CONFIG_parameter_value(dl_buffer);
		   DlSetConfig(id, pt_value ,lenght);
		   send_aircraft_info_state();
		   break;
        }
		
		case DL_SET_COMMAND:
		{  
		   uint8_t id = DL_SET_COMMAND_command_id(dl_buffer);
		   uint8_t lenght = DL_SET_COMMAND_command_value_length(dl_buffer);
		   int8_t *pt_value = DL_SET_COMMAND_command_value(dl_buffer);
		   int8_t response=DlSetCommand(id, pt_value ,lenght);
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		   DOWNLINK_SEND_SET_COMMAND_ACK_STATE(DefaultChannel, DefaultDevice, &id,&response);
		   break;
		}
		
        case DL_ADD_MISSION:
		{  
		   struct mission_info ms_info;
		   ms_info.mission_id=DL_ADD_MISSION_mission_id(dl_buffer);
		   ms_info.mission_ack_code=DL_ADD_MISSION_mission_ack_code(dl_buffer);
		   ms_info.mission_status=DL_ADD_MISSION_mission_status(dl_buffer);
		   ms_info.mission_type=DL_ADD_MISSION_mission_type(dl_buffer);
		   ms_info.duration=DL_ADD_MISSION_duration(dl_buffer);
		   ms_info.nb_backup_land=DL_ADD_MISSION_nb_backup_land(dl_buffer);
		   ms_info.nb_wp=DL_ADD_MISSION_nb_wp(dl_buffer);
		   ms_info.wp_type=DL_ADD_MISSION_wp_type(dl_buffer);
		   ms_info.wp_len=DL_ADD_MISSION_waypoints_length(dl_buffer);
           ms_info.waypoints=DL_ADD_MISSION_waypoints(dl_buffer);
		   int8_t response=mission_add_parse(ms_info);
		   uint8_t mission_ack_type=0; //mission_ack_type: add = 0, update = 1, delete = 2
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		   DOWNLINK_SEND_MISSION_ACK_STATE(DefaultChannel, DefaultDevice, 
		   	                               &ms_info.mission_ack_code,
		   	                               &mission_ack_type,
		   	                               &ms_info.mission_id,
		   	                               &response);
		   
		   break;
		}
		case DL_UPDATE_MISSION: 
		{ 
		   struct mission_info ms_info;
		   ms_info.mission_id=DL_UPDATE_MISSION_mission_id(dl_buffer);
		   ms_info.mission_ack_code=DL_UPDATE_MISSION_mission_ack_code(dl_buffer);
		   ms_info.mission_status=DL_UPDATE_MISSION_mission_status(dl_buffer);
		   ms_info.mission_type=DL_UPDATE_MISSION_mission_type(dl_buffer);
		   ms_info.duration=DL_UPDATE_MISSION_duration(dl_buffer);
		   ms_info.nb_backup_land=DL_UPDATE_MISSION_nb_backup_land(dl_buffer);
		   ms_info.nb_wp=DL_UPDATE_MISSION_nb_wp(dl_buffer);
		   ms_info.wp_type=DL_UPDATE_MISSION_wp_type(dl_buffer);
		   ms_info.wp_len=DL_UPDATE_MISSION_waypoints_length(dl_buffer);
           ms_info.waypoints=DL_UPDATE_MISSION_waypoints(dl_buffer);
		   int8_t response=mission_update_parse(ms_info);
		   uint8_t mission_ack_type=1; //mission_ack_type: add = 0, update = 1, delete = 2
		   xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		   DOWNLINK_SEND_MISSION_ACK_STATE(DefaultChannel, DefaultDevice, 
		   	                               &ms_info.mission_ack_code,
		   	                               &mission_ack_type,
		   	                               &ms_info.mission_id,
		   	                               &response);
		   break;
	    }
		case DL_DELETE_MISSION: 
		{ 
		  uint8_t mission_id=DL_DELETE_MISSION_mission_id(dl_buffer);
		  uint8_t mission_ack_code=DL_DELETE_MISSION_mission_ack_code(dl_buffer);
		  int8_t response=mission_delete(mission_id);
		  uint8_t mission_ack_type=2; //mission_ack_type: add = 0, update = 1, delete = 2 
		  xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		  DOWNLINK_SEND_MISSION_ACK_STATE(DefaultChannel, DefaultDevice, 
		   	                               &mission_ack_code,
		   	                               &mission_ack_type,
		   	                               &mission_id,
		   	                               &response);		  
		  break;
	    }		
	    case DL_GET_MISSION: 
		{ 
	      uint8_t mission_id=DL_GET_MISSION_mission_id(dl_buffer);
		  uint8_t wp_type=DL_GET_MISSION_wp_type(dl_buffer);
		  int8_t response=mission_get(mission_id, wp_type);   //not finish
		  if(response==0)  break;
		  uint8_t mission_ack_code=0;
		  uint8_t mission_ack_type=3; //mission_ack_type: add = 0, update = 1, delete = 2, get=3
		  xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
		  DOWNLINK_SEND_MISSION_ACK_STATE(DefaultChannel, DefaultDevice, 
		   	                               &mission_ack_code,
		   	                               &mission_ack_type,
		   	                               &mission_id,
		   	                               &response);	
		  break;
	    }
		case DL_BIND_AIRCRAFT: 
		{ 
		  //check gcs serial_code
		  const uint8_t gcs_serial_code[10]=GCS_SERIAL_CODE;
		  uint8_t *pt_serial_code=DL_BIND_AIRCRAFT_serial_code(dl_buffer);
		  for(uint8_t i=0; i<10;i++)
		  {    
		  	 if( *(pt_serial_code+1)!=gcs_serial_code[i] )  break; 
		  }
          send_aircraft_info_state();
		  break;
	    }		
		case DL_HEART_BEAT_GCS_STATE: 
		{ 
	      //uint8_t gcs_cur_mission_id=DL_HEART_BEAT_GCS_STATE_cur_mission_id(dl_buffer);
		  send_heart_beat_A2G_msg();
		  break;
	    }

	    default: break;
    }
  }
 #endif   

  //process ubuntu GCS uplink message  
  if( msg_type == XBEE_TYPE_DEFAULT )   //process ubuntu GCS
  {
	switch (msg_id) 
	{
	    case  DL_PING: {
		  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
	    }
	    break;

	    case DL_SETTING : {
	      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
	      uint8_t i = DL_SETTING_index(dl_buffer);
	      float var = DL_SETTING_value(dl_buffer);
	      DlSetting(i, var);
		  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
	    }
	    break;

	    case DL_GET_SETTING : {
	      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
	      uint8_t i = DL_GET_SETTING_index(dl_buffer);
	      float val = settings_get_value(i);
		  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
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
		#endif //GPS_DATALINK
		
	    default:      break;
	  }
  }

 
  /* Parse modules datalink */
  //modules_parse_datalink(msg_id);
}
