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

#include "subsystems/gps.h"
#ifndef USE_MISSION
#include "generated/flight_plan.h"
#endif
uint8_t  flight_mode;       //nav_gcs_mode/nav_rc_mode,requested autopilot_mode=NAV
uint32_t flight_status=0;    
bool_t  auto_convert=FALSE;    //flag,use mission running interrupt
bool_t  nav_toward_wp_flag=FALSE;   //flag,use to clear old info in toward_one_wp flight
enum Flight_State flight_state;
uint8_t mission_state;         //only for debug,is mission_run return value
struct EnuCoor_i wp_home_tk,wp_ms_break,wp_start;   //record home/mission break/toward flight start waypoint
struct config_info ac_config_info;  //aircraft config infomation,have initial default value,change by gcs setting config

static int8_t gps_check_before_flight();
static void  ac_config_set_default();

/*****************************
 *****flight nav function****
 *****************************/
void nav_flight_init(void)
{  
  flight_mode=nav_gcs_mode;  //default in nav_gcs_mode
  auto_convert=FALSE;
  nav_toward_wp_flag=FALSE;
  mission_init();            //mission initial
  ac_config_set_default();
  rc_set_info_init();        //rc info initial
}

#ifndef USE_MISSION
//only use in demo flight,which is using fixed flight plan
uint8_t flight_demo(void)
{   
	//request enter nav_mode
	if ( autopilot_mode!=AP_MODE_NAV) 
	{
		//reset after in other mode
		  flight_mode=nav_gcs_mode;  //default in nav_gcs_mode,how to use nav_kill_mode?
		  auto_convert=FALSE;
		  nav_toward_wp_flag=FALSE;
		  //mission_init();
		  rc_set_info_init();
		  pvtol_all_reset(TRUE);  //reset all motion step
		  return TRUE;
	}
	//when enter,3Dfix requested
	if( gps_check_before_flight() ) return TRUE;

	if(flight_mode==nav_gcs_mode)
	{   //we have refuse rc_mode changing into auto_mode in_flight
	    static bool_t auto_start_flag=FALSE;
		if(!auto_start_flag)    //it limit one flight finished have to restart
		{
			if( get_mission_executable() && nav_block==2 )   //check mission exist and flight is ready
			{
				nav_block=3;  //set nav_block=start engine,begin auto flight
				auto_start_flag=TRUE;
				return FALSE;
			}
		}
		auto_nav_fp();   //run fixed flight plan
	}
	
	else if(flight_mode==nav_rc_mode)
	{
		if(rc_set_info.vtol==TAKE_OFF)   
		{   
			if( take_off_motion(FALSE) )   return TRUE;    //running take off motion
			record_current_waypoint(wp_home_tk);  //record take off waypoint as wp_home_tk,but convert from flightplan will be except
            RC_HOVER();   //set hover for rc motion cmd
			rc_set_info.vtol=CRUISE;  //finish take off(will stay in h=1.2m), turn to cruise
			return FALSE;
		}
		else if(rc_set_info.vtol==CRUISE)  
		{
			if(rc_set_info.home)   //check if start back home cmd
			{
				if( !(rc_set_info.home) )  {  RC_HOVER();   return FALSE;  }   //stop home(interrupt), turn to hover
				else  
				{
					if(nav_toward_wp_flag)    //reset toward flight once
	                {
			  	        nav_toward_waypoint(&wp_home_tk,TRUE);   //flight to home waypoint
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

/***************************************/
/****dynamic mission flight function****/
/***************************************/
/* !request autopilot mode =NAV;
   !in flight,gcs_mode can change to rc_mode,but exchange will be dangerous
   !can't change autopilot mode into NAV from other mode in light,otherwise damage
*/
bool_t nav_flight()
{ 
  static uint8_t step=0;
  if ( autopilot_mode!=AP_MODE_NAV) 
  {
	//reset flight status ,in other mode
	flight_mode=nav_gcs_mode;  //default in nav_gcs_mode,how to use nav_kill_mode?
	auto_convert=FALSE;
	nav_toward_wp_flag=FALSE;
	//mission_init();
	rc_set_info_init();
	pvtol_all_reset(TRUE);  //reset all motion step
	step=0;  //restart
	return TRUE;
  }

  //when enter,gps 3Dfix requested
  if( gps_check_before_flight() ) return TRUE;
  
  //uint8_t mission_state;
  switch(step) {
  	case 0:  //flight prepare  (lock motors -->wait GPS fixed and set ground reference)
  	  flight_state=preparing;
	  
      if( flight_prepare(FALSE) )  return TRUE;  //process flight_prepare,no ltp_pos setting,only use flight plan local wp as ltp_pos
      step++;
	  break;
	  
	case 1:  //wait take_off cmd
	  flight_state=ready;
	  
	  //wait take_off ready and cmd
	  if(flight_mode==nav_gcs_mode && !autopilot_in_flight)
	  {
	  	   lock_motion(FALSE);   //lock, keep safty   	   
	  	   
           if( get_mission_executable() )    //check mission if exist
		   {  
		   	  flight_mode_enter(nav_gcs_mode);   //make sure get sp before take off,meas less
		      step++;  	  
           }
	  }
	  else if(flight_mode==nav_rc_mode && !autopilot_in_flight)
	  {    
	  	   if(rc_set_info.vtol==LAND)  rc_set_info.vtol=LOCKED;  //clear land,indicate land finished
	  	   else if(rc_set_info.vtol==TAKE_OFF) 
		   {  
		   	  flight_mode_enter(nav_rc_mode); 	//make sure get setpoint pos before take off,meas less
			  step++;
	  	   }
		   else   //if(rc_set_info.vtol==LOCKED/CRUISE)
		   {  
		   	  rc_set_info.vtol=LOCKED;     //set into locked
		   	  if( lock_motion(FALSE) ) return TRUE;   //continual lock
			  return FALSE;
		   }
	  }	  
	  break;
	  
	case 2:  //taking_off   in nav_rc_mode,it will refuse other motion cmd
	  flight_state=taking_off;
	  
	  if( take_off_motion(FALSE) ) return TRUE;  //process take off motion
	  record_current_waypoint(wp_home_tk);  //record take off waypoint as wp_home_tk

	  step++;  //take_off motion finished, next
	  break;
	  
	case 3:  //normal flight
	  flight_state=cruising;
     
	  //nav_rc_mode, in crusing,allow change to land or home
	  if(flight_mode==nav_rc_mode && autopilot_in_flight)  
	  {
           if(rc_set_info.vtol==TAKE_OFF)  
		   {
		   	  rc_set_info.vtol=CRUISE;		//clear take off,indicate take_off finished 
		   	  RC_HOVER();   //set hover for rc motion cmd
           }
		   else if(rc_set_info.vtol==LAND)
		   {
		   	   step=5;   break;  //goto landing			   
		   }
		   else if(rc_set_info.home)
		   {
		   	   step=4;   break;  //goto home			   
		   }
		   //else: execution rc_cmd
		   return TRUE;  //process rc flight
	  }
	  else if(flight_mode==nav_gcs_mode && autopilot_in_flight)
	  {
	  	  if(FALSE)   //auto_convert)   //do not doing now
	  	  {
		  	 if(nav_toward_wp_flag)    //reset toward flight once
	         {
	  	        nav_toward_waypoint(&wp_ms_break,TRUE);   //flight toward mission break wp,continual mission_running
		        nav_toward_wp_flag=FALSE;
	         }
		  	 if(nav_toward_waypoint(&wp_ms_break,FALSE)==0)   auto_convert=FALSE;  
			 else  return TRUE;
	  	  }
		  
		  else   //normal mission running
		  {
		  	 mission_state=mission_run();           //processs mission flight
	  	     if(mission_state==0)  return TRUE;		//mission is running  

			 //finished success,goto landing
	         else if(mission_state==1)              
			 {    
			 	step=5;  //go to landing motion
				break;   
			 }

			 //fail doned
		     else    
		     {
		   	   //report mission_state error,should goto step 4.now we only let it land
			 	step=5;  //go to landing motion
				break;   
		     }
		  }
	  }
      break;

	case 4:  //direct home back,rc_mode is finished,but gcs mode is incomleted
	  flight_state=home;

	  if(flight_mode==nav_rc_mode && !rc_set_info.home)  //stop home, return to cruise,start from hover status
	  {
	  	  step=3;  //return to cruise
		  rc_motion_cmd_execution(0x00);  //set hover	
		  nav_toward_wp_flag=TRUE;
	  }
	  if(nav_toward_wp_flag)    //reset toward flight once
	  {
	  	  nav_toward_waypoint(&wp_home_tk,TRUE); 
		  nav_toward_wp_flag=FALSE;
	  }
	  if(nav_toward_waypoint(&wp_home_tk,FALSE)==0)    //arrived home, can do land motion
	  step++; 
	  break;

	case 5: //landing
	  flight_state=landing;
	  
	  if( land_motion(FALSE) ) return TRUE;   //doing landing
	  mission_init();
	  //if(!mission_clear_all()) return TRUE;   //finished mission,reset mission_manager.so it could accept continual mission
	  step=1;  //if land motion finished,turn to ready state
      break;
	  
	default: return FALSE;  //finish flight
  } 
  return TRUE;
  
}

/*flight mode change transition*/
void flight_mode_enter(uint8_t new_mode)
{
	if(flight_mode==new_mode) return;
	flight_mode=new_mode;
	
	if(flight_mode==nav_rc_mode)
	{   
		rc_set_info_reset();        //clear rc_set_info
		guidance_h_nav_rc_enter();  //guidance_enter run
		if(autopilot_in_flight)     //if in_flight,hover
		{
			record_current_waypoint(wp_ms_break);  //record mission break waypoint,use recover mission run
			rc_set_info.vtol=CRUISE;
            //RC_HOVER();  

			rc_motion_cmd_execution(0x00);
		}
		else
		{
			rc_set_info.vtol=LOCKED;
		}
		nav_toward_wp_flag=TRUE;
		//else prepare_flight; 		
	}
	
	else if(flight_mode==nav_gcs_mode)
	{
	    //auto_convert=TRUE;   //not consist of
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
	
}

//return FALSE meas check pass
static int8_t gps_check_before_flight()
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
	ac_config_info.concentration=20;
	ac_config_info.max_flight_height=2.0;
	ac_config_info.max_flight_speed=6.0;
	ac_config_info.spray_height=1.3;
	ac_config_info.spray_wide=3.0;
	ac_config_info.spray_speed=3.0;
}
