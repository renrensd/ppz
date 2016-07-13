/**
 * @file firmwares/rotorcraft/pvtolc.c
 *prepare,take off,landing and closure
 */
 
#include "firmwares/rotorcraft/pvtolc.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "state.h"
#include "modules/sonar/agl_dist.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/stabilization.h"

struct EnuCoor_i wp_ToL;  //waypoint for takeoff or land


/*flight prepare function,need modify*/
bool_t flight_prepare(bool_t reset)
{ 
  static uint8_t step_p=0;
  if(reset) { step_p=0; return FALSE; }
  switch(step_p) {
  	case 0:
      if( (NavKillThrottle()) )  return TRUE;  //kill throttle
      step_p++;
	  break;
	case 1:
	  #if 0
	  #else
      if ( !(GpsFixValid()) )   return TRUE;   //wait GPS fixed,it will change to wait LC connect
	  #endif
      step_p++;
	  break;
	case 2:
      //nav_reset_reference();   //set local point,use flight_plan
      step_p++;
	  break;
	case 3:	
      if( (NavKillThrottle()) )  return TRUE;  //kill throttle once more
      step_p++;
	  break;
	case 4:
	  NavAttitude(RadOfDeg(0));
      NavVerticalAutoThrottleMode(RadOfDeg(0));
      NavVerticalThrottleMode(9600*(0));
	  step_p=0;      //reset step_p,make it can rerun
	  return FALSE;  //finish flight prepare	
	  
    default: break;
  }
  return TRUE;
}


bool_t take_off_motion(bool_t reset)
{ 
  static uint8_t step_t=0;
  if(reset) { step_t=0; return FALSE; }
  switch(step_t) {
    case 0:  //reset output cmd
	  //horizontal set attitude 0
	  NavAttitude(RadOfDeg(0));
	  NavVerticalAutoThrottleMode(RadOfDeg(0));
	  //vertical set throttle 0
	  NavVerticalThrottleMode(9600*(0));
	  step_t++;
	  break;
	case 1:
	  if ((NavResurrect())) return TRUE;  //auto unlocked
	  step_t++;
	  break;
	case 2://set dynamic wp_takeoff when z <0.3,avoid airframe dump
	  if ( stateGetPositionEnu_f()->z < DISTANCE_ABOVE_GROUNG ) { 
	    wp_ToL=*stateGetPositionEnu_i();  
		NavGotoWaypoint_wp(wp_ToL);
	    NavVerticalAutoThrottleMode(RadOfDeg(0.0));
	    NavVerticalAltitudeMode(Height(1.200000), 0.);
	    return TRUE; 
	  }
	  step_t++;
	  break;
    case 3:
	  //stay takeoff waypoint,height
	  NavGotoWaypoint_wp(wp_ToL);
	  //NavVerticalClimbMode(0.5);
	  NavVerticalAltitudeMode(Height(1.200000), 0.);
	  if( (stateGetPositionEnu_f()->z  >1.0)&&(stateGetSpeedEnu_f()->z < 0.2))
	  {
	  	//NavVerticalAltitudeMode(Height(1.200000), 0.);
	  	step_t=0;   //reset
	  	return FALSE;   //finish take off motion
	  }
	  
    default: break;	  
  }
  return TRUE;

}

bool_t land_motion(bool_t reset)
{ 
  static uint8_t step_l=0;
  if(reset) { step_l=0; return FALSE; }
  switch(step_l) {
  	case 0:
	  //wp_ToL=*stateGetPositionEnu_i(); //wp_ToL should be given wp_land before this function called 
	  if( NavGetHoverSteady() )
	  {
	  	step_l++;
	  }
	  break;
	case 1:
	  if(stateGetPositionEnu_f()->z >0.600000) {   
	    //NavGotoWaypoint_wp(wp_ToL);
	    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
	    NavVerticalClimbMode(-0.3 ); 
	    return TRUE;
      }
      step_l++;
	 //break; 	let it do fast 
	case 2:
	  //NavGotoWaypoint_wp(wp_ToL);
	  NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
	  NavVerticalClimbMode(-0.15 ); 
	  step_l++;
	  break;
	case 3:
	  //not use agl_sonar detect touching ground,
	  //once on_ground,thrust will deline
	  if(stabilization_cmd[COMMAND_THRUST]>4000) return TRUE;
	  NavAttitude(RadOfDeg(0));
	  NavVerticalAutoThrottleMode(RadOfDeg(0));
	  NavVerticalThrottleMode(9600*(0));
	  step_l++;
	  break;
	case 4:	 
 	  if ( (NavKillThrottle()) )  return TRUE;
	  step_l = 0;  //reset
	  return FALSE;   //lock motors and finish      	  
	
	default: break;
  }
  return TRUE;
}

bool_t lock_motion(bool_t reset)
{ 
  static uint8_t step_o=0;
  if(reset) { step_o=0; return FALSE; }
  switch(step_o) {
	case 1:
	  NavAttitude(RadOfDeg(0));
	  NavVerticalAutoThrottleMode(RadOfDeg(0));
	  NavVerticalThrottleMode(9600*(0));
	  step_o++;
	  break;
	case 2:	 
 	  if ( (NavKillThrottle()) )  return TRUE;
	  step_o=0;  //reset
	  return FALSE;   //lock motors and finish      	  
	
	default: break;
  }
  return TRUE;
}

int8_t nav_toward_waypoint(struct EnuCoor_i *wp_end,bool_t reset)
{   
	uint8_t error_code=1;
	static uint8_t step_h=0;
	if(reset) { step_h=0; return FALSE; }
	switch(step_h)
	{
		case 0:  //set heading toward wp_end
		  nav_set_heading_towards( POS_FLOAT_OF_BFP(wp_end->x), POS_FLOAT_OF_BFP(wp_end->y) );
		  step_h++;
		  break;
		case 1:  //check heading
		  if( nav_check_heading() ) step_h++;
		  break;
		case 2:  //route flight
		  NavGotoWaypoint_wp(*wp_end);
		  NavVerticalAltitudeMode(Height(1.200000), 0.);   //POS_FLOAT_OF_BFP(wp_end->z)
		  if( nav_approaching_from(wp_end,NULL,0) )        {  step_h=0;  return error_code=0; } 
		  break;
	    default: return error_code=2;
	}
	return error_code=1;
}

void pvtol_all_reset(bool_t reset)
{
	flight_prepare(reset);
    take_off_motion(reset);
    land_motion(reset);
    lock_motion(reset);
	if(reset)   nav_toward_waypoint(&wp_ToL, reset);   //the wp_ToL is not used
}