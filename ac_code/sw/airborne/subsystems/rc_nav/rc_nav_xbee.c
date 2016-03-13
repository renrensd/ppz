/**
 * @file subsystem/rc_nav_xbee.c
    rc cmd(motion/set) parse
    something as initial/reset/check function
 */


#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#ifdef OPS_OPTION
#include "subsystems/ops/ops_msg_if.h"   
#endif

//rc_set_cmd
#define set_manual_rc 1    //0x01,
#define open_spray_rc 2    //0x02,
#define land_rc       3    //0x03,
#define home_stop_rc  4    //0x04,
#define mainpower_off_rc 5 //0x05,
#define set_auto_rc   16   //0x10,
#define stop_spray_rc 32   //0x20,
#define take_off_rc   48   //0x30,    
#define home_rc       64   //0x40,    
#define mainpower_on_rc 80 //0x50,    

#define RC_MAX_COUNT  10   //lost_time_out =10/(2hz)=5s

struct rc_set rc_set_info;   //rc mode information
uint8_t last_response;       //set cmd response record
uint8_t rc_motion_cmd;
uint8_t rc_set_cmd;
uint8_t rc_count;   //rc lost counter
bool_t  rc_lost;


/********************************
 *****rc_motion_cmd execution****
 ********************************/
//globle var,use save rc_cmd speed
float fb_speed=0.0;   //forward or backward speed
float rl_speed=0.0;   //right or left speed

static void rc_setpoint_speed_parse(float speed_fb,float speed_rl);

//convert body speed to earth speed,according to psi
static void rc_setpoint_speed_parse(float speed_fb,float speed_rl)
{  
   float psi= stateGetNedToBodyEulers_f()->psi; //yaw angle
   float s_psi = sinf(psi);
   float c_psi = cosf(psi);
   
   float speed_x=c_psi * speed_fb - s_psi * speed_rl;
   float speed_y=s_psi * speed_fb + c_psi * speed_rl;
   
   Bound(speed_x,-RC_H_Max_Speed,+RC_H_Max_Speed);
   Bound(speed_y,-RC_H_Max_Speed,+RC_H_Max_Speed);
   
   guidance_h_speed_sp.x = SPEED_BFP_OF_REAL(speed_x);
   guidance_h_speed_sp.y = SPEED_BFP_OF_REAL(speed_y);
}

uint8_t rc_motion_cmd_execution( uint8_t cmd )
{   
	rc_motion_cmd=cmd;
	//check error before motion execution
	uint8_t error_code=0;
	if(flight_mode!=nav_rc_mode) return error_code=10; //error,not in manual mode
   #ifdef USE_MISSION
    //change in ground ,up_key give take_off cmd(21=lowest,23=highest)
	if(cmd==21 && !autopilot_in_flight && flight_state==ready)   
	{  rc_set_info.vtol=TAKE_OFF;  return error_code=0; }
   #else
    //change in ground ,up_key give take_off cmd(21=lowest,23=highest)
	if(cmd==21 && !autopilot_in_flight && rc_set_info.vtol==LOCKED)   
	{  rc_set_info.vtol=TAKE_OFF;  return error_code=0; }
   #endif
	if(!autopilot_in_flight) return error_code=11;  //error,not in flight
	if(rc_set_info.home) return error_code =12; //error,aircraft is in home mode
	if(rc_set_info.vtol==LOCKED) return error_code =13;  //aircraft is not take off
	if(rc_set_info.vtol==TAKE_OFF) return error_code =14; //error,aircraft is taking off
	if(rc_set_info.vtol==LAND) return error_code =15; //error,aircraft is landing

	enum motion_type rc_motion_type=(enum motion_type)( (cmd >>4)&0x0F );   //get motion type,see in message_new.xml
	
	uint8_t rc_orientation= (cmd>>2)&0x03;       //get orientation,see in message_new.xml
	if(rc_orientation==3) return error_code=1;   //rc_orientation wrong

	
	uint8_t rc_grade= cmd&0x03;   //get speed grade

	switch(rc_motion_type)
	{
        case  hover:
		{
			if(rc_orientation!=0) return error_code=2; //hover orientation wrong
			if(rc_grade!=0) return error_code=3;// hover speed grade wrong
			RC_HOVER();   //do hover
	        break;
        }
			
		case  virtical:
		{
			if(rc_orientation==1) 
			{  
				RC_Climb(rc_grade);
				break;
			}
			else if(rc_orientation==2)
			{
				RC_Decline(rc_grade);
				break;
		    }
			else 
			{
				RC_Z_HOLD();
				break;
		    }
		}	
		case  fb_ward:
		{
			if(rc_orientation==0) return error_code=4;  //orientation wrong in fb_ward type
			else if(rc_orientation==1)
			{
				fb_speed=(RC_FORWARD(rc_grade));
				break;
			}
			else
			{
				fb_speed=(RC_BACKWARD(rc_grade));
				break;
			}			
		}
			
		case  rl_ward:
		{
			if(rc_orientation==0) return error_code=5;  //orientation wrong in rl_ward type
			else if(rc_orientation==1)
			{
				rl_speed=(RC_RIGHT(rc_grade));
				break;
			}
			else
			{
				rl_speed=(RC_LEFT(rc_grade));
				break;
			}			
		}
		
		case  rotation:   //rc_turn_rate will use when horizontal mode==HORIZONTAL_MODE_RC
		{
			if(rc_orientation==0) return error_code=6;  //orientation wrong in rotation type
			else if(rc_orientation==1)
			{
		 	    rc_turn_rate=RATE_BFP_OF_REAL( RC_TURN_RIGHT(rc_grade) ); 
				break;
			}
			else
			{
		 	    rc_turn_rate=RATE_BFP_OF_REAL( RC_TURN_LEFT(rc_grade) ); 
				break;
			}			
		}
			
		default: return error_code=7; //motion_type wrong
	}

	//execution set guidance_h_speed_sp
	rc_setpoint_speed_parse(fb_speed, rl_speed);	
	
	return error_code=0;  //process success
}



/********************************
 *******pasre rc_set_cmd********
 ********************************/
 uint8_t rc_set_cmd_pasre(uint8_t cmd)
 {    
 	  rc_set_cmd=cmd;
      uint8_t response=0;
	  bool_t success=1;
	  switch(cmd)
	  {    //in taking_off or landing motion,refuse mode change
	  	   case set_auto_rc:
		   	  #ifdef USE_MISSION
		   	   //success=0; break;  //modify later
		   	   if(autopilot_mode!=AP_MODE_NAV || flight_state==taking_off ||flight_state==landing)  { success=0;break; }
			   //if(autopilot_in_flight)  { success=0;break; }
			  #else
			   if(autopilot_mode!=AP_MODE_NAV || autopilot_in_flight) { success=0;break; }  //in flight,refuse to change			   	
			  #endif
			   rc_set_info.mode_state=nav_gcs_mode;
			   flight_mode_enter(nav_gcs_mode);
			   break;
		   case set_manual_rc:
		   	  #ifdef USE_MISSION
		   	   if(autopilot_mode!=AP_MODE_NAV || flight_state==taking_off ||flight_state==landing)  { success=0;break; }
			  #else
			   if(autopilot_mode!=AP_MODE_NAV)  { success=0;break; }
			  #endif
			   rc_set_info.mode_state=nav_rc_mode;
			   flight_mode_enter(nav_rc_mode); 
			   break;
		   case mainpower_on_rc:
		   	   //error deal
	    	   rc_set_info.m_power=1;
		   	   break;
		   case mainpower_off_rc:
		   	   //error deal
	    	   rc_set_info.m_power=0;
		   	   break;
		   case stop_spray_rc:
		   	   if(flight_mode!=nav_rc_mode)  {success=0;break;}
            #ifdef OPS_OPTION
			   ops_stop_spraying(); 
			#endif 
		   	   rc_set_info.spray=0;
		   	   break;
		   case open_spray_rc:
		   	   if(flight_mode!=nav_rc_mode)  {success=0;break;}
             #ifdef OPS_OPTION
			   ops_start_spraying();
			 #endif
	    	   rc_set_info.spray=1;						
		   	   break;
		   case take_off_rc:
		   	  #ifdef USE_MISSION
		   	   if(flight_mode!=nav_rc_mode || autopilot_in_flight || flight_state!=ready)  
			   {   success=0;  break;  }			   
	    	   
			  #else
			   if(flight_mode!=nav_rc_mode || autopilot_in_flight || rc_set_info.vtol!=LOCKED) 
			   {   success=0;  break;  }
			  #endif
			   rc_set_info.vtol=TAKE_OFF;
		   	   break;
		   case land_rc:
		   	   if(flight_mode!=nav_rc_mode || !autopilot_in_flight)  {   success=0;  break;  }		
			  #ifdef USE_MISSION
			   if( !(flight_state==cruising || flight_state==home) ) {   success=0;  break;  }
			  #else
			   if( rc_set_info.vtol!=CRUISE ) {   success=0;  break;  }
			  #endif
			   rc_set_info.vtol=LAND;
		   	   break;
		   case home_rc:
		      #ifdef USE_MISSION
		   	   if(flight_mode!=nav_rc_mode || !autopilot_in_flight || flight_state!=cruising)
			   {   success=0;  break;  }
			  #else
			   if(flight_mode!=nav_rc_mode || !autopilot_in_flight || rc_set_info.vtol!=CRUISE)
			   {   success=0;  break;  }
			  #endif
			   rc_set_info.home=1;
		   	   break;
		   case home_stop_rc:
		      #ifdef USE_MISSION
		   	   if(flight_mode!=nav_rc_mode || !autopilot_in_flight || flight_state!=home)  
			   {   success=0;  break;  }
			  #else
			   if(flight_mode!=nav_rc_mode || !autopilot_in_flight || !(rc_set_info.home) )  
			   {   success=0;  break;  }
			  #endif
			   rc_set_info.home=0;
		   	   break;

		   default: success=0;
	  }
	  
	  response=rc_set_response_pack();
	  if(success) response +=2;       //set success status
	  last_response=response;
	  return response;
 }

//communication with rc,ack response code
uint8_t rc_set_response_pack(void)
{ 
	 uint8_t response=0;
	 if(rc_set_info.mode_state==nav_gcs_mode) response =0; //set 8th bit false
	 else response =1;  //set 8th bit true

	 if(!rc_set_info.spray) response=(response<<1)+1;
	 else response =response<<1;

     if(rc_set_info.vtol==LAND) response =(response<<2)+2;
	 else if(rc_set_info.vtol==TAKE_OFF) response =(response<<2)+1;
	 else response =response<<2;  //include locked and cruising 2 state

	 if(rc_set_info.home) response =response<<1;
	 else response=(response<<1)+1;

	 if(rc_set_info.m_power) response=(response<<1)+1;
	 else response =response<<1;

	 response =response<<2;
	 
	 return response;
}

void rc_set_info_reset(void)
{
	//if(autopilot_in_flight)  rc_set_info.vtol=CRUISE;
	//else rc_set_info.vtol=LOCKED;
	rc_set_info.home=0;
	rc_set_info.spray=0;
#ifdef OPS_OPTION
	ops_stop_spraying();//TODOM
#endif
}

void rc_set_info_init(void)
{
    rc_set_info.mode_state=nav_gcs_mode;
	rc_set_info.vtol=LOCKED;
	rc_set_info.home=0;
	rc_set_info.spray=0;
#ifdef OPS_OPTION
	 ops_stop_spraying();//TODOM
#endif
	rc_set_info.m_power=0;
}

void rc_lost_check(void)
{
	rc_count++;
	if (rc_count>RC_MAX_COUNT)
	{
	       //xbee_con_info.rc_con_available==FALSE     //not change the rc con	
	       rc_lost=TRUE;
	       //in take_off,it will flight to cruise;once entered cruise,do land
	       //after landed, locked auto.
	       if(autopilot_in_flight && rc_set_info.vtol==CRUISE)   
	       {
		       rc_set_info.vtol=LAND;    //land process in this exeption case
		   }
	      
	       if(!autopilot_in_flight)
	       {
		       rc_set_info.vtol=LOCKED;
		       rc_set_cmd_pasre(0x10);  //flight_mode set auto		   	
	       }
	       //do not convert to nav_gcs_mode!!!
	       //rc_set_cmd_pasre(0x10);  //flight_mode set auto
	       //rc_set_info.mode_state=nav_gcs_mode;
	}
}