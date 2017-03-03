/**
 * @file firmwares/rotorcraft/flight_nav.c
    gcs mode:mission run
    rc_mode:cmd motion execution
    something information to communication
 */


#include "firmwares/rotorcraft/nav_flight.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"

#include "subsystems/monitoring/monitoring.h" 
#include "subsystems/gps.h"
#ifndef USE_MISSION
#include "generated/flight_plan.h"
#endif
#include "subsystems/ops/ops_app_if.h"
#include "subsystems/ops/ops_msg_if.h"
#include "firmwares/rotorcraft/pvtolc.h"

#include "modules/system/timer_if.h"
#include "modules/mag_cali/mag_cali.h"

#include "subsystems/ins/ins_int.h"


uint8_t  flight_mode;       //nav_gcs_mode/nav_rc_mode,requested autopilot_mode==NAV

uint16_t flight_status = 0; 

//bool_t  nav_toward_wp_flag = FALSE;   //flag,use to clear old info in toward_one_wp flight
struct EnuCoor_i wp_take_off;//(wp_ms_break,wp_start) not use  record home/mission break/toward flight start waypoint
float distance2_to_takeoff;

enum Flight_State flight_state;
static uint8_t flight_step = 0;

Gcs_State task_state;

enum Rc_Type rc_type = REAL_RC;

struct config_info ac_config_info;  //aircraft config infomation,have initial default value,change by gcs setting config

static void ept_ms_run(void);
static void nav_mode_enter(void);
static void  ac_config_set_default(void);
static void rc_mode_enter(void);
static void gcs_mode_enter(void);
static void mode_convert_for_gps_type(void);



void nav_flight_init(void)
{ 
	flight_mode = nav_gcs_mode;  //default in nav_kill_mode
	task_init();            //mission initial
	rc_nav_init();             //rc info initial
	ac_config_set_default();
}


/***********************************************************************
* FUNCTION    : nav_flight
* DESCRIPTION : key RC and android gcs supported,fixed flight plan is shielded
* INPUTS      : void
* RETURN      : unuseful
***********************************************************************/
/*  request autopilot mode =NAV;
 *  in flight,gcs_mode can change to rc_mode,but exchange will be dangerous
 *  can't change autopilot mode into NAV from other mode in light,otherwise damage
 */
void nav_flight(void)
{
  static bool_t takeoff_done = FALSE;

 #ifndef DEBUG_RC
  if ( autopilot_mode != AP_MODE_NAV) 
  {
  	  nav_mode_enter();
	  return;  /*stop run*/
  }
 #endif
 
  /*caculate distance to takeoff waypoint*/
  RunOnceEvery( NAV_FREQ,
      { if(flight_state==cruising)  distance2_to_takeoff = get_dist2_to_point(&wp_take_off); } );

  /*RC/GCS timer run*/
  tm_stimulate(TIMER_TASK_RC);
  tm_stimulate(TIMER_TASK_GCS);
  
  /*run except mission from monitoring*/
  ept_ms_run();

  //mode_convert_for_gps_type();
  
  /*main nav run with flight step*/
  switch(flight_step)
  {
  	   case 0:  /*flight prepare  (lock motors -->wait GPS fixed and set ground reference)*/
  	   		flight_state = preparing;
			if( !flight_prepare(FALSE) )  
			{
				flight_step++;  /*once prepare finish, jump to next step:ready*/
			}
			
			break;
			
		case 1:  /*wait for take_off cmd*/
			flight_state = ready;

			/*gcs mode*/
			if( flight_mode==nav_gcs_mode && !autopilot_in_flight )
			{
				lock_motion(FALSE);   /*lock motor, keep safe*/ 
				
				if( auto_task_ready_check() )   /*get gcs start cmd*/
				{
					monitoring_reset_emer();
					flight_mode_enter(nav_gcs_mode);   //make sure get sp before take off,meas less
					takeoff_done = FALSE;
					flight_step++;  /*once get gcs start cmd, jump to next step: take off*/
				}
			}

			/*rc mode*/
			else if( flight_mode==nav_rc_mode && !autopilot_in_flight )
			{  
				if(rc_set_info.vtol == LAND)
				{
					if(!lock_motion(FALSE))
					{
						rc_set_info.vtol = LOCKED;  /*finish land, reset to locked*/
						rc_set_info.locked = TRUE;  /*lock, refuse other rc cmd until unlock cmd*/
					}
				}
		  	   else if(rc_set_info.vtol==TAKE_OFF) 
			   {
				   	monitoring_reset_emer();
				   	flight_mode_enter(nav_rc_mode); 	/*get rc take off cmd*/
				   	takeoff_done = FALSE;
					flight_step++;
		  	   }
			   else   /*rc_set_info.vtol==LOCKED/CRUISE*/
			   {
			   		rc_set_info.vtol = LOCKED;     //set other vtol state to LOCKED
			   		lock_motion(FALSE);   /*lock motor, keep safe*/ 
			   }
			}
			
			break;

		case 2:  /*take off motion*/
			flight_state = taking_off;

			if( (flight_mode == nav_rc_mode && rc_set_info.vtol == LAND)
				|| (flight_mode == nav_gcs_mode && gcs_task_cmd == GCS_CMD_DLAND) )
			{
				take_off_motion(TRUE);
				flight_step = 5;      /*jump from take off to land*/
				mag_cali_nav_loop(FALSE);
				takeoff_done = FALSE;
				break;
			}

			if(!takeoff_done)
			{
				if( !take_off_motion(FALSE) )	// take off done
				{
					takeoff_done = TRUE;
					record_current_waypoint(&wp_take_off);  /*record take off waypoint as wp_take_off*/
				}
			}
			else
			{
				if( mag_cali_nav_loop(TRUE) )	// mag_cali done or not necessary to cali
				{
					flight_step++;          /*take_off motion finished, next cruise step*/
				}
			}
			break;

		case 3:   /*cruise, normal flight*/
			flight_state = cruising;

			/*gcs mode*/
			if(flight_mode==nav_gcs_mode && autopilot_in_flight)
			{
				if(0)   //task interrupt ,do not doing now
				{
				}

				else   /*run gcs task*/
				{
					task_state = gcs_task_run();
					
					if( GCS_RUN_LANDING==task_state ) /*task finished success,goto landing*/
					{
						flight_step = 5;  //goto landing motion
					}
					//else task state need add !!!
				}
			}

			/*rc mode*/
			else if(flight_mode==nav_rc_mode && autopilot_in_flight)  
			{
				if(rc_set_info.vtol==TAKE_OFF)  /*take_off finished, convert to cruise*/
				{
					rc_set_info.vtol = CRUISE;		 
					rc_mode_enter(); 
				}
				else if(rc_set_info.vtol==LAND)
				{
					RC_HOVER();
					set_current_pos_to_target();
					flight_step = 5;          //goto landing
				}
				else if(rc_set_info.home)
				{
					RC_HOVER();

					/*get current height, if height lower 2m, set height 2m*/
					wp_take_off.z = stateGetPositionEnu_i()->z;
					if( wp_take_off.z < POS_BFP_OF_REAL(2.0) )
					{
						wp_take_off.z = POS_BFP_OF_REAL(2.0);
					}
					
					nav_rc_go_home(&wp_take_off,TRUE);   //reset the step
					flight_step = 4;          //goto home
				}
				//else: execution rc_cmd
			}
			
			break;

		case 4:  /*back home, only for rc mode*/
			flight_state = home;

			if(flight_mode == nav_rc_mode)
			{
				/*land cmd:return to cruise state, and set land*/
				if(rc_set_info.vtol==LAND)
				{
					flight_step = 3;  
					rc_mode_enter();
					break;
				}
				
				/*stop home,convert to cruise(hover)*/
				if(!rc_set_info.home)  /*stop home motion, return to cruise*/
				{
					flight_step = 3;  //return to cruise(mode enter)
					rc_mode_enter();
					break;
				}

				/*run back home */
				if(0 == nav_rc_go_home(&wp_take_off,FALSE))    
				{
					rc_set_info.vtol = LAND;
					rc_set_info.home = FALSE;
					flight_step++;            /*arrived home, jump to next step: land motion*/
				}
			}
			else if(flight_mode == nav_gcs_mode)
			{
				flight_step = 3;  /*gcs mode not use home motion here,jump to cruise*/
			}

			break;

		case 5: /*land motion(need set hover before land)*/
			flight_state = landing;

			if( !land_motion(FALSE) )   //doing landing
			{
				task_init();  /*reset task*/
				flight_step = 1;  /*land motion finished,jump to ready state*/
			}
			break;

		default: 
			break;  /*error step*/
	}
}


static void ept_ms_run(void)
{
	static uint8_t last_cmd = CM_NONE;
	if(flight_state==landing)
	{
		monitor_cmd = CM_LAND;
		last_cmd = monitor_cmd;
		return;
	}	
	if(last_cmd==monitor_cmd) 
	{ //same cmd,void do again 
		return;    
	} 
	
	if(flight_mode==nav_rc_mode)
	{
		if( flight_state>taking_off && (monitor_cmd+2)>=flight_state )  
		{
			flight_step = monitor_cmd + 2;    //change nav_flight step,so it will do monitor_cmd
			if(monitor_cmd==CM_HOVER)
			{
				rc_motion_cmd_execution(0);   //send hover cmd
				RC_HOVER();     //make sure do hover
			}
			else if(monitor_cmd==CM_HOME)
			{
				rc_set_info.home = TRUE;
			}
			else if(monitor_cmd==CM_LAND)
			{
				rc_set_info.vtol = LAND;
			}
				
			last_cmd = monitor_cmd;   //update last_cmd
		}
		if(monitor_cmd==CM_NONE)
		{
			if( last_cmd==CM_HOME && flight_state==home )   //make sure motion(hover/home/land) is trige by monitor_cmd,and stop it
			{
				flight_step = 3; //back to cruise
				rc_set_info.home = FALSE;
				RC_HOVER();     //do hover,wait rc_cmd
			}
			last_cmd = monitor_cmd;   //update last_cmd
		}
	}
	else if(flight_mode==nav_gcs_mode && flight_state==cruising)
	{
		switch(monitor_cmd)
		{
			case CM_NONE:
				last_cmd = monitor_cmd;
				break;
			case CM_HOVER:
				if(gcs_task_cmd < GCS_CMD_PAUSE)
				{
					gcs_task_cmd = GCS_CMD_PAUSE;
					last_cmd = monitor_cmd;
				}
				break;
			case CM_HOME:
				if(gcs_task_cmd < GCS_CMD_CONTI)
				{
					gcs_task_cmd = GCS_CMD_BHOME;
					last_cmd = monitor_cmd;
				}
				break;
			case CM_LAND:
				if(gcs_task_cmd <GCS_CMD_DLAND)
				{
					gcs_task_cmd = GCS_CMD_DLAND;
					last_cmd = monitor_cmd;
				}
				break;
			default:
				break;
		}
	}
}

static void mode_convert_for_gps_type(void)
{
	if(ins_int.ned_pos_rc_reset)
	{
		if(ins_int.gps_type == GPS_RTK)
		{
			if(!ins_int.rtk_hf_realign)
			{
				rc_set_cmd_parse(0x01);   //
			}
		}
		else if(ins_int.gps_type == GPS_UBLOX)
		{
			if(!ins_int.ublox_hf_realign)
			{ 
				rc_set_cmd_parse(0x01);   //rtk fail, set to nav_rc_mode
			}
		}
	}
}

/*flight mode change transition*/
void flight_mode_enter(uint8_t new_mode)
{
	if(flight_mode == new_mode) return;
	flight_mode = new_mode;
	
	if(flight_mode == nav_rc_mode)
	{   
		rc_mode_enter();
		if(autopilot_in_flight)     //if in_flight,hover
		{
			rc_set_info.vtol = CRUISE;
			rc_set_info.locked = FALSE;    //force to open rc cmd
			RC_HOVER();  
		}
		else
		{
			rc_set_info.vtol = LOCKED;
			//rc_set_info.locked = TRUE;   //not reset lock rc
		}
		rc_set_info.mode_state = nav_rc_mode;
	}
	
	else if(flight_mode==nav_gcs_mode)
	{
		gcs_mode_enter();
		rc_set_info.mode_state = nav_gcs_mode;
	}	
}

static void nav_mode_enter(void)
{
	/*reset flight status ,in other mode*/
	
	flight_mode = nav_gcs_mode;  //default in nav_rc_mode,how to use nav_kill_mode?
	rc_set_info_init();
	pvtol_all_reset(TRUE);  //reset all motion step
	flight_step = 0;  //restart
}

static void rc_mode_enter(void)
{
	rc_set_info_reset();        //clear rc_set_info
	guidance_h_nav_rc_enter();  //guidance_enter run
	
}

static void gcs_mode_enter(void)
{	
}

uint16_t get_flight_status(void)
{                
 /*flight_status:
    bit1: 0=ac_unready;         1=ac_ready
    bit2: 0=motors_locked;      1=motors_unlocked
    bit3: 0=on_ground;          1=in_flight
    bit4: 0=manual_mode;        1=auto_mode                         
    bit5: 0=back_home;          1=others
    bit6: 0=take_off;           1=others
    bit7: 0=landing;            1=others
    bit8: 0=spray_stop;         1=spray_open
	 other bits keep 0
  */
  flight_status = 0;   //reset 0 use for update
  if(ground_check_pass)
  {
   		flight_status |=(1<<0);
  }
  if(autopilot_motors_on)
  {
   		flight_status |=(1<<1);
  }
  if(autopilot_in_flight)
  {
  		flight_status |=(1<<2);
  }
  if(flight_mode==nav_gcs_mode)
  {
  		flight_status |=(1<<3);
  }
  if( (flight_mode==nav_gcs_mode && task_state==GCS_RUN_HOME)
  	  ||(flight_mode==nav_rc_mode && flight_state==home) )
  {
  		flight_status |=(1<<4);
  }
  if(flight_state==taking_off)
  {
  		flight_status |=(1<<5);
  }
  if(flight_state==landing)
  {
  		flight_status |=(1<<6);
  }
  if(get_spray_switch_state())
  {
  		flight_status |=(1<<7);
  }
  
  return flight_status;
	
}


//called in initial
static void  ac_config_set_default(void)
{
	ac_config_info.spray_concentration = 80;
	ac_config_info.atomization_grade = 3;
	ac_config_info.max_flight_height = 3.0;
	ac_config_info.max_flight_speed = 4.0;
	ac_config_info.spray_height = 3.0;
	ac_config_info.spray_wide = 3.0;
	ac_config_info.spray_speed = 3.0;
	ac_config_info.spray_convert_type = WAYPOINT_FORWARD;
}
