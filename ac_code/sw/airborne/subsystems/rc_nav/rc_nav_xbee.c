/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
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

/**** System include files ****/  

#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/rc_nav/rc_nav_drift.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "math.h"
#include "subsystems/datalink/datalink.h"

#ifdef OPS_OPTION
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_msg_if.h"   
#endif
#include "subsystems/monitoring/monitoring.h" 

#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif	/* DATALINK==XBEE */

#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */

#include "subsystems/mission/task_spray_misc.h"
#ifdef CALIBRATION_OPTION
#include "firmwares/rotorcraft/autopilot.h"
#include "calibration.h"
#endif	/* CALIBRATION_OPTION */

//rc_set_cmd
#define RC_SET_AUTO 0x10
#define RC_SET_MANUAL 0x01
#define RC_TAKE_OFF 0x30
#define RC_LAND 0x03
#define RC_HOME 0x40
#define RC_STOP_HOME 0x04
#define RC_LOCK 0x06
#define RC_UNLOCK 0x60
#define RC_ADD_SPRAY 0x20
#define RC_REDUCE_SPRAY 0x02
#define RC_MAINPOWER_ON 0x50
#define RC_MAINPOWER_OFF 0x05
#ifdef CALIBRATION_OPTION
#define RC_MAG_CALIBRATION_BEGIN 0x70
#define RC_MAG_CALIBRATION_END 0x07
#endif

//orientation
#define ORIENT_FOR  1
#define ORIENT_BACK -1
#define ORIENT_NONE 0

//rc_motion_cmd
#define K_FORWARD    0x25
#define K_BACKWARD   0x29
#define K_RIGHT      0x45
#define K_LEFT       0x49
#define K_UP         0x15
#define K_DOWN       0x19
#define K_HOLD       0x10
#define K_CW         0x85
#define K_CCW        0x89
#define K_HOVER      0

#define RC_MAX_COUNT  4   //lost_time_out =4/(2hz)=2s

struct rc_set rc_set_info;   //rc mode information
struct rc_motion rc_motion_info;

uint8_t rc_motion_cmd;
uint8_t rc_set_cmd;
uint8_t rc_count;   //rc lost counter
bool_t  rc_lost;


static uint8_t rc_motion_cmd_verify(uint8_t cmd);


void rc_nav_init(void)
{
    rc_motion_info_init();
	rc_set_info_init();
}

/***********************************************************************
* FUNCTION    : rc_setpoint_speed_parse
* DESCRIPTION : convert body speed to earth speed,using heading
* INPUTS      : body speed from rc
* RETURN      : none
***********************************************************************/
void rc_setpoint_speed_parse(float speed_fb,float speed_rl)
{  
   //float psi= stateGetNedToBodyEulers_f()->psi; //yaw angle,replace with nav_heading
   float psi= ANGLE_FLOAT_OF_BFP(nav_heading);
   const float s_psi = sinf(psi);
   const float c_psi = cosf(psi);
   
   float speed_x=c_psi * speed_fb - s_psi * speed_rl;
   float speed_y=s_psi * speed_fb + c_psi * speed_rl;
   
   BoundAbs(speed_x,FB_Max_Speed);
   BoundAbs(speed_y,FB_Max_Speed);
   
   guidance_h.sp.speed.x = SPEED_BFP_OF_REAL(speed_x);
   guidance_h.sp.speed.y = SPEED_BFP_OF_REAL(speed_y);
   horizontal_mode = HORIZONTAL_MODE_RC;    //request to set mode
}

/***********************************************************************
* FUNCTION    : rc_motion_cmd_verify
* DESCRIPTION : check take_off cmd and the condition for rc_motion_cmd
* INPUTS      : rc_motion_cmd
* RETURN      : error_code,pass is 0
***********************************************************************/
static uint8_t rc_motion_cmd_verify(uint8_t cmd)
{
	#ifdef DEBUG_RC
	/*on ground,execute stop spray*/
	if( cmd==K_HOLD && !autopilot_in_flight)
	{
		#ifdef OPS_OPTION
		 ops_stop_spraying(); 
		#endif
	}
	#endif
	uint8_t error_code =0;
	if(flight_mode!=nav_rc_mode)
	{//error,not in manual mode
		return error_code=10; 
	}

	/*in ground ,up_key give take_off cmd*/
	if( cmd==K_UP 
		&& !rc_set_info.locked
		&& !autopilot_in_flight 
	   #ifdef USE_MISSION 
		&& flight_state==ready
	   #else 
		&& rc_set_info.vtol==LOCKED
	   #endif   
	                               )   
	{  
		rc_set_info.vtol = TAKE_OFF;  /*give take off cmd*/
		return error_code = 0; 
	}

	/*on ground,execute stop spray*/
	if( cmd==K_HOLD && !autopilot_in_flight)
	{
		#ifdef OPS_OPTION
		 ops_stop_spraying(); 
		#endif
	}
   
	if(!autopilot_in_flight) 
	{//error,ac need take off
		return error_code=11;  
	}
	
	if(rc_set_info.home || rc_set_info.vtol==LOCKED) 
	{//error,refuse action while aircraft in home mode
		return error_code =12; 
	}
	
	if(rc_set_info.vtol==TAKE_OFF || rc_set_info.vtol==LAND)
	{//error,aircraft is taking off/landing, can not interrupt
		return error_code =13; 
	}

	return error_code;  //all pass
}	

/***********************************************************************
* FUNCTION    : rc_motion_orientation_parse
* DESCRIPTION : according to hover_cmd,set the for/backward orientation
* INPUTS      : current cmd, pointer of speed_grade
* RETURN      : void
***********************************************************************/
static void rc_motion_orientation_parse(uint8_t cmd, int8_t *speed_grade)
{
	switch(cmd)
	{
		case K_HOVER:
			rc_motion_info.orientation_flag =0;
			*speed_grade=0;
			break;
		case K_FORWARD:
			if(!rc_motion_info.orientation_flag)
			{
				rc_motion_info.orientation_fb =ORIENT_FOR;
				rc_motion_info.orientation_flag =1;
			}
			break;
		case K_BACKWARD:
			if(!rc_motion_info.orientation_flag)
			{
				rc_motion_info.orientation_fb =ORIENT_BACK;
				rc_motion_info.orientation_flag =1;
			}
			break;
		default:
			break;
	}
}

/***********************************************************************
* FUNCTION    : rc_motion_cmd_execution
* DESCRIPTION : 
* INPUTS      : current cmd
* RETURN      : unuseful
***********************************************************************/
uint8_t rc_motion_cmd_execution( uint8_t cmd )
{   
	static int8_t fb_grade = 0;
	uint8_t error_code = 0;
	rc_motion_cmd = cmd;
	
	/*check error before motion execution*/	
    error_code = rc_motion_cmd_verify(rc_motion_cmd);
	if(error_code) 
	{//something error,return
		return error_code; 
	}

    /*parse for/backward orientation*/	
    rc_motion_orientation_parse(rc_motion_cmd, &fb_grade);

	switch(rc_motion_cmd)
	{
        case  K_HOVER:
		{
			RC_HOVER();     //do hover
	        break;
        }
			
		case  K_UP: 
			RC_Climb(V_DEFAULT_GRADE);
			break;
		case  K_DOWN:
			RC_Decline(V_DEFAULT_GRADE);
			break;
			
		case  K_HOLD:
			if(nav_climb)
			{
				//RC_Z_HOLD();  replace with stop climb speed
				RC_Z_STOP();
			}
			else
			{
			  #ifdef OPS_OPTION    //back home,need stop spraying
			   ops_stop_spraying(); 
			  #endif 				
			}
			break;

		case  K_FORWARD:
			if( 0==(fb_grade+1) )
			{   //avoid reverse orientation
				break;
			}
			fb_grade++;
			BoundAbs(fb_grade,5);   //avoid out of limit grade		
			rc_motion_info.speed_fb = ( RC_FB_SPEED(fb_grade) );
			break;
		case  K_BACKWARD:
			if( 0==(fb_grade-1) )
			{   //avoid reverse orientation
				break;
			}
			fb_grade--;
			BoundAbs(fb_grade,5);   //avoid out of limit grade	
			rc_motion_info.speed_fb = ( RC_FB_SPEED(fb_grade) );
			break;
			
			
		case  K_RIGHT:
		{
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_speed_rl_drift(1);
			}
			else 
			{
				rc_motion_info.speed_rl= RL_SPEED;
			}
			break;
		}		
		case  K_LEFT:
		{
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_speed_rl_drift(-1);
			}
			else 
			{
				rc_motion_info.speed_rl= -RL_SPEED;
			}
			break;
		}

		case  K_CW:
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_rate_rotation_drift(1);
			}
			else 
			{
				rc_motion_info.rotation_rate= ROTATION_RATE;
				rc_turn_rate=RATE_BFP_OF_REAL( rc_motion_info.rotation_rate );
			}
			break;
		case  K_CCW:
			if(rc_motion_info.speed_fb!=0.0)
			{
				rc_rate_rotation_drift(-1);
			}
			else 
			{
				rc_motion_info.rotation_rate= -ROTATION_RATE;
				rc_turn_rate=RATE_BFP_OF_REAL( rc_motion_info.rotation_rate );
			}
			break;
		
		default: return error_code=7; //motion_type wrong
	}

	//execution set guidance_h_speed_sp
	rc_setpoint_speed_parse(rc_motion_info.speed_fb,rc_motion_info.speed_rl);

	//use for monitoring process
	rc_cmd_interrupt=TRUE;
	
	return error_code=0;  //process success
}



/***********************************************************************
* FUNCTION    : rc_set_cmd_pasre
* DESCRIPTION : leave take_off/mainpower/mode unused
* INPUTS      : current cmd
* RETURN      : unuseful
***********************************************************************/
 uint8_t rc_set_cmd_pasre(uint8_t cmd)
 {    
 	  rc_set_cmd=cmd;

	  switch(rc_set_cmd)
	  {    
	    #if 1
	  	   case RC_SET_AUTO:
		   	   if( AP_MODE_NAV == autopilot_mode)
		   	   {
				   if( autopilot_in_flight ) 
				   	{
						break;  /*if aircraft in flight, refuse change mode from manual to auto*/
					}		   	
				  
				   rc_set_info.mode_state = nav_gcs_mode;
				   flight_mode_enter(nav_gcs_mode);			   		
		   	   	}		   	  
			   break;
			   
		   case RC_SET_MANUAL:
		   		if( AP_MODE_NAV == autopilot_mode)
		   		{
				  #ifdef USE_MISSION
			   	   if(flight_state==taking_off || flight_state==landing)  
				   	{ 
						break;
					}
				   if(flight_mode==nav_gcs_mode && autopilot_in_flight)
				   {
				   		mode_convert_a2m = TRUE;
						spray_break_and_continual();
				   	}
				  #endif		
				  
				   rc_set_info.mode_state = nav_rc_mode;
				   flight_mode_enter(nav_rc_mode); 
		   		}
			   break;
	   #endif
		   
		   case  RC_ADD_SPRAY:
		   	#ifndef DEBUG_RC
		   	   if(flight_mode==nav_rc_mode) 
			#endif
		   	   {  
			   	   if( get_spray_switch_state() )  /*only ops is opened grade add*/
			   	   {
				   		rc_set_info.spray_grade++;
						Bound(rc_set_info.spray_grade, 0, 5);
						rc_update_ops_config_param(rc_set_info.spray_grade);
			   	   	}
			   	   #ifdef OPS_OPTION
                  ops_start_spraying();  //make sure ops is opened
			      #endif 
		   	   }
		   	   break;
		   case  RC_REDUCE_SPRAY:
		   	#ifndef DEBUG_RC
		   	   if(flight_mode==nav_rc_mode) 
			#endif
		   	   {
			   	   if(rc_set_info.spray_grade)  
			   	   {	
				   		if( get_spray_switch_state() )  /*only ops is opened grade add*/
				   		{
					   	   rc_set_info.spray_grade--;
						   //Bound(rc_set_info.spray_grade, 0, 5);
						   rc_update_ops_config_param(rc_set_info.spray_grade);
				   		}
					  #ifdef OPS_OPTION
					   if(!rc_set_info.spray_grade)
					   {
					   	    ops_stop_spraying();  //if set 0 grade,stop spray
					   }
					   else
					   {
					   		ops_start_spraying();
					   }
				     #endif 
			   	   }
		   	   }
		   	   break;			   

		   case  RC_LAND:
		   	   if(flight_mode==nav_rc_mode && autopilot_in_flight) 
			   {
					rc_set_info.vtol = LAND;
					
		           //use for monitoring process
                  rc_cmd_interrupt = TRUE;
			   }		
		   	   break;
			   
		   case  RC_HOME:
		   	   if( flight_mode==nav_rc_mode && autopilot_in_flight 
			   	  #ifdef USE_MISSION
		   	       && flight_state==cruising
			      #else
			       && rc_set_info.vtol==CRUISE
			      #endif                                            
				                                                      )
			   {
				   rc_set_info.home = TRUE;
				  #ifdef OPS_OPTION    
				   ops_stop_spraying(); //back home,make sure spray stop
				  #endif 
				   //use for monitoring process
		           rc_cmd_interrupt = TRUE;
			   }
		   	   break;
			   
		   case  RC_STOP_HOME:
		   	   if( flight_mode==nav_rc_mode && autopilot_in_flight 
			   	  #ifdef USE_MISSION
		   	       && flight_state==home
			      #else
			       && rc_set_info.home==TRUE
			      #endif                                           
				                                                       )
			   {
				   rc_set_info.home = FALSE;
				  	//use for monitoring process
		           rc_cmd_interrupt = TRUE;
			   }
		   	   break;
			   
		   case  RC_LOCK:
		   	   if(rc_set_info.vtol==LOCKED)   //request ac is not in flight cmd
		   	   {
			   	   rc_set_info.locked = TRUE;
		   	   }
		   	   break;
		   case  RC_UNLOCK:
		   	   if(rc_set_info.vtol==LOCKED)   //request ac is not in flight cmd
		   	   {
			   	   rc_set_info.locked = FALSE;
		   	   }
		   	   break;
		   #ifdef CALIBRATION_OPTION
		   case RC_MAG_CALIBRATION_BEGIN:
		   	    if(kill_throttle)
		   	    {
					cali_mag_begin();
		   	    }
				break;
		   case RC_MAG_CALIBRATION_END:
		   	    if(kill_throttle)
		   	    {
					cali_mag_end();
		   	    }
				break;
		   #endif /* CALIBRATION_OPTION */
		   
		   #if 0   
		   /*reserve function*/ 
		   case RC_MAINPOWER_ON:
	    	   rc_set_info.m_power=1;
		   	   break;
		   case RC_MAINPOWER_OFF:
	    	   rc_set_info.m_power=0;
		   	   break;
		   /*not use, instead of K_UP*/   
		   case RC_TAKE_OFF:
		   	  #ifdef USE_MISSION
		   	   if(flight_mode!=nav_rc_mode || autopilot_in_flight || flight_state!=ready)  
			   {   break;  }			   
	    	   
			  #else
			   if(flight_mode!=nav_rc_mode || autopilot_in_flight || rc_set_info.vtol!=LOCKED) 
			   {   break;  }
			  #endif
			   rc_set_info.vtol=TAKE_OFF;
		   	   break;
		   #endif

		   default:
		   	   break;
	  }
	  return TRUE;
 }


void rc_set_info_reset(void)
{
	//if(autopilot_in_flight)  rc_set_info.vtol=CRUISE;
	//else rc_set_info.vtol=LOCKED;
	rc_set_info.home = FALSE;
	//rc_set_info.spray_grade=0;
#ifdef OPS_OPTION
	ops_stop_spraying();//TODOM
#endif
}

void rc_set_info_init(void)
{
    rc_set_info.mode_state= nav_gcs_mode; //or nav_gcs_mode,use in rc_cmd mode change
	rc_set_info.vtol = LOCKED;
	rc_set_info.home = FALSE;
	rc_set_info.spray_grade= 0;
#ifdef OPS_OPTION
	 ops_stop_spraying();//TODOM
#endif
	//rc_set_info.m_power = 0;
    rc_set_info.locked = TRUE;
}

void rc_motion_info_init(void)
{
	rc_motion_info.speed_fb=0.0;
	rc_motion_info.speed_rl=0.0;
	rc_motion_info.rotation_rate=0.0;
	rc_motion_info.orientation_fb=ORIENT_NONE;
	rc_motion_info.orientation_flag=0;
}

void rc_lost_check(void)
{
	if(!rc_lost)
	{
		rc_count++;
	}
	if (rc_count>RC_MAX_COUNT)
	{
	       rc_lost = TRUE;
		   rc_count = 0;
		   #ifdef GCS_V1_OPTION

		   //unused for only heartbeat communication
		   //XbeeSetRcConFalse();   //close rc communication,wait for restart connect
		   #endif
	}
}

void rc_set_connect(void)
{
	rc_count=0;  //reset rc_count,use for check rc_lost
	rc_lost=FALSE;
}

void rc_set_rc_type(void)
{
	rc_type = REAL_RC;
}