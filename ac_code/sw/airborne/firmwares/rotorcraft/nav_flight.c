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

#include "firmwares/rotorcraft/pvtolc.h"

/*use for rc drift timer update*/
#include "modules/system/timer_if.h"
#include "modules/system/timer_class.h"
#include "modules/system/timer_def.h"


uint8_t  flight_mode;       //nav_gcs_mode/nav_rc_mode,requested autopilot_mode==NAV

uint32_t flight_status = 0; 
uint32_t device_status = 0;

bool_t  auto_ms_interrupt = FALSE;    //flag,use mission running interrupt
bool_t  nav_toward_wp_flag = FALSE;   //flag,use to clear old info in toward_one_wp flight
struct EnuCoor_i wp_home_tk,wp_ms_break,wp_start;   //record home/mission break/toward flight start waypoint
float distance2_to_home;

enum Flight_State flight_state;
static uint8_t flight_step = 0;
Gcs_State task_state;

struct config_info ac_config_info;  //aircraft config infomation,have initial default value,change by gcs setting config

static void ept_ms_run(void);
static void nav_mode_enter(void);
static int8_t gps_check_before_flight(void);
static void  ac_config_set_default(void);



void nav_flight_init(void)
{ 
	flight_mode = nav_gcs_mode;  //default in nav_kill_mode
	auto_ms_interrupt = FALSE;
	nav_toward_wp_flag = FALSE;
	task_init();            //mission initial
	rc_nav_init();             //rc info initial
	ac_config_set_default();
}

/***********************************************************************
* FUNCTION    : flight_demo
* DESCRIPTION : only use in demo flight,navigation flight include fixed 
*               flight plan and key RC(request not set micro USE_MISSION)
* INPUTS      : void
* RETURN      : unuseful
***********************************************************************/
#ifndef USE_MISSION
uint8_t flight_demo(void)
{   
	//rc_drift timer update
	tm_stimulate(TIMER_TASK_RC);
	
	//request enter nav_mode
    if ( autopilot_mode!=AP_MODE_NAV) 
    {
		nav_mode_enter();
		return TRUE;
    }
	//when enter,3Dfix requested
	if( gps_check_before_flight() ) return TRUE;

	if(flight_mode==nav_gcs_mode)
	{   //we have refuse rc_mode changing into auto_mode in_flight
	    static bool_t auto_start_flag=FALSE;
		if(!auto_start_flag)    //it limit one flight finished have to restart
		{
			if( 0 )// get_mission_executable() && nav_block==2 )   //check mission exist and flight is ready
			{
				nav_block=3;  //set nav_block=start engine,begin auto flight
				auto_start_flag=TRUE;
				return FALSE;
			}
		}
		auto_nav_fp();   //run fixed flight plan
		return TRUE;
	}
	
	else if(flight_mode==nav_rc_mode)
	{
		if(rc_set_info.vtol==TAKE_OFF)   
		{   
			if( take_off_motion(FALSE) )   return TRUE;    //running take off motion
			
			record_current_waypoint(&wp_home_tk);  //record take off waypoint as wp_home_tk,but convert from flightplan will be except

			rc_mode_enter();	
			
			rc_set_info.vtol=CRUISE;  //finish take off(will stay in h=1.2m), turn to cruise
			return FALSE;
		}
		else if(rc_set_info.vtol==CRUISE)  
		{
			if(rc_set_info.home)   //check if start back home cmd
			{
				if( !(rc_set_info.home) )   //stop home(interrupt), turn to hover
				{  
					rc_mode_enter();
					RC_HOVER();   
					return FALSE;  
				}  
				else  
				{
					if(nav_toward_wp_flag)    //reset toward flight once
	              {
			  	        nav_toward_waypoint(&wp_home_tk,TRUE);   //flight to home waypoint
			  	        RC_HOVER();  
				        nav_toward_wp_flag=FALSE;
	         		}
					if(nav_toward_waypoint(&wp_home_tk,FALSE)==0)   //if arrived at home, do landing
					{   
						rc_set_info.vtol=LAND;   
						return FALSE;  
					} 
					else return TRUE;                                 //do flighting home 
				}
			}
			else return TRUE;    //doing RC motion(once motion cmd change,it will response)
		}
        else if(rc_set_info.vtol==LAND)  
		{
			if( land_motion(FALSE) ) return TRUE;   //doing landing motion
			rc_set_info.vtol=LOCKED;   //finish land, motors keep locked
			rc_set_info.locked =TRUE;  //lock take off cmd
			return FALSE;
		}
		else if(rc_set_info.vtol==LOCKED) 
		{
			if( lock_motion(FALSE) ) return TRUE;   //in locking
			return FALSE;
		}
		else return FALSE;   //something wrong 
		
	}
	
	else return TRUE;   //in nav_kill_mode
}
#endif

/***********************************************************************
* FUNCTION    : nav_flight
* DESCRIPTION : key RC and android gcs supported,fix flight plan is shielded
* INPUTS      : void
* RETURN      : unuseful
***********************************************************************/
/*  request autopilot mode =NAV;
 *  in flight,gcs_mode can change to rc_mode,but exchange will be dangerous
 *  can't change autopilot mode into NAV from other mode in light,otherwise damage
 */
void nav_flight(void)
{ 
  if ( autopilot_mode != AP_MODE_NAV) 
  {
  	  nav_mode_enter();
	  return;  /*stop run*/
  }

  /*when enter,gps 3Dfix requested*/  
  if( gps_check_before_flight() ) 
  {
  	  return;  /*stop run*/
  }

  /*caculate distance to takeoff waypoint*/
  RunOnceEvery( NAV_FREQ,
      { if(flight_state==cruising)  distance2_to_home =get_dist2_to_point(&wp_home_tk); } );

  /*RC timer run*/
  tm_stimulate(TIMER_TASK_RC);
  
  /*run except mission from monitoring*/
  ept_ms_run();
  
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
			
		case 1:  /*wait take_off cmd*/
			flight_state = ready;

			/*gcs mode*/
			if( flight_mode==nav_gcs_mode && !autopilot_in_flight )
			{
				lock_motion(FALSE);   /*lock motor, keep safe*/ 
				
				if( auto_task_ready_check() )   /*get gcs start cmd*/
				{
					flight_mode_enter(nav_gcs_mode);   //make sure get sp before take off,meas less
					flight_step++;  /*once get gcs start cmd, jump to next step: take off*/
				}
			}

			/*rc mode*/
			else if( flight_mode==nav_rc_mode && !autopilot_in_flight )
			{  
				if(rc_set_info.vtol == LAND)
				{
					rc_set_info.vtol = LOCKED;  /*finish land, reset to locked*/
					rc_set_info.locked = TRUE;  /*lock, refuse other rc cmd until unlock cmd*/
				}
		  	   else if(rc_set_info.vtol==TAKE_OFF) 
			   {
			   		flight_mode_enter(nav_rc_mode); 	/*get rc take off cmd*/
					flight_step++;
		  	   }
			   else   /*rc_set_info.vtol==LOCKED/CRUISE*/
			   {
			   		rc_set_info.vtol = LOCKED;     /*keep locked for safe*/
			   		lock_motion(FALSE);
			   }
			}
			
			break;

		case 2:  /*take off motion*/
			flight_state = taking_off;

			if( !take_off_motion(FALSE) )  //process take off motion
			{
				record_current_waypoint(&wp_home_tk);  /*record take off waypoint as wp_home_tk*/
				flight_step++;          /*take_off motion finished, next cruise step*/
			}
			break;

		case 3:   /*cruise, normal flight*/
			flight_state = cruising;

			/*gcs mode*/
			if(flight_mode==nav_gcs_mode && autopilot_in_flight)
			{
				if(0)   //task interrupt ,do not doing now
				{
					/*
					if(nav_toward_wp_flag)    //reset toward flight once
					{
						nav_toward_waypoint(&wp_ms_break,TRUE);   //flight toward mission break wp,continual mission_running
						nav_toward_wp_flag=FALSE;
					}
					if(nav_toward_waypoint(&wp_ms_break,FALSE)==0)   auto_ms_interrupt = FALSE;  
					else  break;
					*/
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
					flight_step = 5;          //goto landing
				}
				else if(rc_set_info.home)
				{
					RC_HOVER();
					flight_step = 4;          //goto home
				}
				//else: execution rc_cmd
			}
			
			break;

		case 4:  /*back home, only for rc mode*/
			flight_state = home;

			if(flight_mode == nav_rc_mode)
			{
				/*stop home,convert to cruise(hover)*/
				if(!rc_set_info.home)  /*stop home motion, return to cruise*/
				{
					flight_step = 3;  //return to cruise(mode enter)
					rc_mode_enter();
					nav_toward_wp_flag = TRUE;
				}

				/*run back home */
				if(nav_toward_wp_flag)    //reset toward flight once
				{
					nav_toward_waypoint(&wp_home_tk,TRUE); 
					nav_toward_wp_flag = FALSE;
				}
				else if(0 == nav_toward_waypoint(&wp_home_tk,FALSE))    
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
				if(nav_gcs_mode == flight_mode)
				{
					task_init();  /*reset task*/
				}
				flight_step = 1;  /*land motion finished,jump to ready state*/
			}
			break;

		default: 
			break;  /*error step*/
	}
}

static void nav_mode_enter(void)
{
	//reset flight status ,in other mode
	flight_mode = nav_gcs_mode;  //default in nav_rc_mode,how to use nav_kill_mode?
	auto_ms_interrupt = FALSE;
	nav_toward_wp_flag = FALSE;
	rc_set_info_init();
	pvtol_all_reset(TRUE);  //reset all motion step
	flight_step = 0;  //restart
}

static void ept_ms_run(void)
{
	static uint8_t last_cmd =CM_NONE;
	if(flight_state==landing)
	{
		cur_cmd =CM_LAND;
		last_cmd =cur_cmd;
		return;
	}	
	if(last_cmd==cur_cmd) 
	{ //same cmd,void do again 
		return;    
	} 
	
	if(flight_mode==nav_rc_mode)
	{
		if(flight_state>taking_off && (cur_cmd+2) >=flight_state )  
		{
			flight_step =cur_cmd+2;    //change nav_flight step,so it will do cur_cmd
			if(cur_cmd==CM_HOVER)
			{
				RC_HOVER();     //do hover
			}
			if(cur_cmd==CM_HOME)
			{
				rc_set_info.home=TRUE;
			}
			last_cmd =cur_cmd;   //update last_cmd
		}
		if(cur_cmd==CM_NONE)
		{
			if( last_cmd==CM_HOME && flight_state==home)   //make sure motion(hover/home/land) is trige by cur_cmd,and stop it
			{
				flight_step =3; //back to cruise
				rc_set_info.home=FALSE;
				RC_HOVER();     //do hover,wait rc_cmd
			}
			last_cmd =cur_cmd;   //update last_cmd
		}
	}	
}


/*flight mode change transition*/
void flight_mode_enter(uint8_t new_mode)
{
	if(flight_mode == new_mode) return;
	flight_mode=new_mode;
	
	if(flight_mode == nav_rc_mode)
	{   
		rc_set_info_reset();        //clear rc_set_info
		guidance_h_nav_rc_enter();  //guidance_enter run
		if(autopilot_in_flight)     //if in_flight,hover
		{
			record_current_waypoint(&wp_ms_break);  //record mission break waypoint,use recover mission run
			rc_set_info.vtol=CRUISE;
			RC_HOVER();  
		}
		else
		{
			rc_set_info.vtol = LOCKED;
			rc_set_info.locked = TRUE;
		}
		nav_toward_wp_flag=TRUE;
		//else prepare_flight; 		
	}
	
	else if(flight_mode==nav_gcs_mode)
	{
	    //auto_ms_interrupt=TRUE;   //not consist of
	   #ifndef USE_MISSION
	   /*
	    if(autopilot_in_flight) flight_mode=nav_rc_mode;  //refuse to enter auto if in flight( already refues in rc_set_mode
	    else  rc_set_info_reset();        //clear rc_set_info
	    */
	   #endif
	    rc_set_info_reset();        //clear rc_set_info
	    nav_toward_wp_flag=TRUE;
	}
	//else { nav_kill_mode}
	
}


void rc_mode_enter(void)
{
	rc_set_info_reset();        //clear rc_set_info
	guidance_h_nav_rc_enter();  //guidance_enter run
	
	if(autopilot_in_flight)     //if in_flight,hover
	{
		rc_set_info.vtol=CRUISE;
		RC_HOVER();  
	}
	else
	{
		rc_set_info.vtol = LOCKED;
		rc_set_info.locked = TRUE;
	}
	nav_toward_wp_flag = TRUE;	
}


uint32_t get_flight_status(void)
{                
 /*flight_status:
     bit1: 0=on_ground;          1=in_flight
	 bit2: 0=auto_mode;          1=manual_mode
	 bit3: 0=normal_mission;     1=exception_mission
	 bit4: 0=home_mode;          1=others
	 bit5: 0=spray_stop;         1=spray_open
	 bit6: 0=others;             1=take_off
	 bit7: 0=others;             1=landing
	 bit8: 0=motors_locked;      1=motors_unlocked
	 bit9: 0=mainpower_off;      1=mainpower_on
	 other bits keep 0
  */
  if(autopilot_in_flight) flight_status |=0x00000001;
  else flight_status &=0xfffffffe;
  
  if(flight_mode==nav_rc_mode) flight_status |=0x00000002;
  else flight_status &=0xfffffffd;
  
  //if(normal_mission)
  flight_status &=0xfffffffb;
  
  //if not home
  flight_status |=0x00000008;
  
  //if not spray
  flight_status &=0xffffffef;
  
  if(flight_state==taking_off)  flight_status =0x00000020;
  else flight_status &=0xffffffdf;
  
  if(flight_state==landing)  flight_status =0x00000040;
  else flight_status &=0xffffffbf;

  if(autopilot_motors_on) flight_status |=0x00000080;
  else flight_status &=0xffffff7f;

  //if mainpower_on
  flight_status |=0x00000100;

  return flight_status;
	
}

//return FALSE meas check pass
static int8_t gps_check_before_flight(void)
{
	static bool_t gps_fix_flag=FALSE;
	static uint16_t gps_fix_count=0;
	if(!gps_fix_flag)
	{
	    if ( !(GpsFixValid()) )   
		{   
			gps_fix_count=0;
	    }
	    else
		{   gps_fix_count++;
		    if( gps_fix_count==(32*5) )  //run 32Hz,continue 5s
			{
				gps_fix_flag=TRUE;  
				return FALSE;  
			}
	    }
		return TRUE;
	}
	return FALSE;  //once gps fix,no more check
}

//called in initial
static void  ac_config_set_default(void)
{
	ac_config_info.concentration = 20;
	ac_config_info.max_flight_height = 1.5;
	ac_config_info.max_flight_speed = 6.0;
	ac_config_info.spray_height = 1.2;
	ac_config_info.spray_wide = 3.0;
	ac_config_info.spray_speed = 3.0;
}
