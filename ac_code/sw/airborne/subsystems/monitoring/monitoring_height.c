/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : RN R&D SW1      									   *
*   AUTHOR	   :            										   *
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
#include "subsystems/monitoring/monitoring_height.h"
#include "subsystems/monitoring/monitoring.h" 
#include "subsystems/abi.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/ins/ins_int.h"


/*===VARIABLES========================================================*/
#define SUM_RATIO 3
#define DETA_SONAR_GROUND  0.2  //20cm
#define DETA_SONAR_FLIGHT  0.6  //60cm
#define DETA_BARO          20.0 //20pa
#define FLIGHT_RANGE       500.0  //pa

#define SONAR_GROUND  0.3  //30cm
#define SONAR_VALID   3.0  //3.0m
#define BARO_MIN 60000.0  //unit =pa
#define BARO_MAX 110000.0 

#define SONAR_ERROR_DATA1  0.201
#define SONAR_ERROR_DATA2  7.649

#define DATA_FIX_MAX 20

/*---Global-----------------------------------------------------------*/
struct Height_Monitor h_moni;

static abi_event sonar_ev_mo;
static abi_event baro_ev_mo;

static void sonar_moni_cb(uint8_t __attribute__((unused)) sender_id, float distance);
static void baro_moni_cb(uint8_t __attribute__((unused)) sender_id,
                         uint32_t __attribute__((unused)) stamp, 
                         float pressure, float temp);

static inline void sonar_flight_check(void);
static inline void baro_flight_check(void);
static void sonar_dead_distance_determine(void);
static inline void sonar_range_check(void);


/*===FUNCTIONS========================================================*/

/***********************************************************************
* FUNCTIONS    : initial functions
* DESCRIPTION : 
* INPUTS      : 
* RETURN      : none
***********************************************************************/
void height_moni_init(void)
{
	height_ground_reset();
	AbiBindMsgAGL(ABI_BROADCAST, &sonar_ev_mo, sonar_moni_cb);
	AbiBindMsgBARO_ABS(ABI_BROADCAST, &baro_ev_mo, baro_moni_cb);
}

uint8_t height_ground_check(void)
{
	uint8_t check_code;
	if(h_moni.sonar_ground_check )
	{   
		h_moni.sonar_code &=0x1F;
		
		if(h_moni.sonar_code==0 || h_moni.sonar_code==2)     //normal include fix data
		{	
			h_moni.sonar_status=0;   
			//h_moni.sonar_code=0;   //reset
			ins_int.update_on_agl =TRUE;
		}
		
		else if( ((h_moni.sonar_ground_check>>3)&0x01)==1 )  //frequence error
		{	
			h_moni.sonar_status=3;  
			ins_int.update_on_agl =FALSE;
		}
		
		else                                                 //dead_distance
        {	
			h_moni.sonar_status=1;   
			ins_int.update_on_agl =FALSE;
		}		

	}
	
	if(h_moni.baro_ground_check )
	{   
		if(!h_moni.baro_code)   //normal
		{	
			h_moni.baro_status=0;   
			ins_int.baro_valid=TRUE;
		}
		else                   //set error
		{	
			h_moni.baro_status=1;   
			ins_int.baro_valid=FALSE;

		}
	}
	
    if( h_moni.sonar_ground_check && h_moni.baro_ground_check )
    {   //set height information flag  to limit flight height
        if(h_moni.baro_status)   //baro is error
        {
			flight_limit_height=BARO_LIMIT_HEIGHT;
			if(h_moni.sonar_status!=0)
			{
				check_code=2;    //set fail(sonar is error or dead)
			}
			else
			{
				check_code=1;    //set pass, only use sonar
			}
        }
		else
		{
			if(h_moni.sonar_status==3 || h_moni.sonar_status==2)
			{
				check_code=2;   //set fail
			}
			else if(h_moni.sonar_status==1)
			{
				check_code=1;   //set pass,sonar in dead_distance
			}
			else
			{
				check_code=1;   //set pass,include sonar dead_distance
			}	
		}		
		//reset sensors code to run next flight check
	    h_moni.sonar_code=0;
	    h_moni.baro_code=0;
	}
	else
	{   check_code=0;  }

	return check_code;
}


void height_flight_check(void)
{	
    sonar_flight_check();
	baro_flight_check();

	//sonar and baro all out of meas
	if( (h_moni.sonar_status==3||h_moni.sonar_status==2) && h_moni.baro_status==1)
	{
		//force to use sonar and baro,(height limit set before)
		ins_int.baro_valid =TRUE;
		ins_int.update_on_agl=TRUE;
		set_except_mission(HEIGHT_BOTH,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);
	}
	else
	{
		em[HEIGHT_BOTH].active=FALSE;
		em[HEIGHT_BOTH].finished=FALSE;
	}
}


static inline void sonar_flight_check(void)
{
   /*
	if( h_moni.sonar_code&0x04 )   
	{
		h_moni.sonar_status=3;   //fre error
		//return;
	}*/
	
	if(!above_ground)
	{
		if(h_moni.sonar_status==3)  
		{
			//on ground frequence error,need close takeoff cmd,do later!!!
			
			set_except_mission(HEIGHT_SONAR,TRUE,FALSE, TRUE,10, TRUE,FALSE,2);
			#if TEST_MSG
			sonar_bound=3;
			#endif
			return;    
		}
		
		if(h_moni.sonar_code)  
		{
			h_moni.sonar_status=1;   //in dead distance
		    h_moni.sonar_code=0;   //reset
		    ins_int.update_on_agl=FALSE;
		}
		else 
		{
			h_moni.sonar_status=0;   //in dead distance
			h_moni.sonar_code=0;   //reset
		    ins_int.update_on_agl=TRUE;
		}
		//return;  //not generate ept_ms
	}
	else //in flight
	{	
		sonar_dead_distance_determine();   //request status==1
		
	
		if( h_moni.sonar_status==2 )  //add baro_z assist
		{
			ins_int.update_on_agl=FALSE;   //refuse use sonar info update ins
			#if TEST_MSG
			sonar_bound=2;
			#endif
		}
		else if( h_moni.sonar_status==1 )
	    { 
			ins_int.update_on_agl=FALSE;
	        #if TEST_MSG
			sonar_bound=1;
			#endif
		}	
		else if( h_moni.sonar_status==3 )
	    {
			ins_int.update_on_agl=FALSE;   //refuse use sonar info update ins
			//|| (ins_int.baro_valid && ins_int.baro_z <-3.5) 
			if( h_moni.sonar_code )    
			{			
				set_except_mission(HEIGHT_SONAR,TRUE,FALSE, TRUE,10, TRUE,FALSE,2);
				#if TEST_MSG
				sonar_bound=3;
				#endif
//need test,in 2016.03.21
				//h_moni.sonar_status =0;   //reset result,try more to use it
				//h_moni.sonar_code =0;
			}
			else
			{
				//h_moni.sonar_status =0;
			}
		}
		else //normal:0
		{
			ins_int.update_on_agl=TRUE;   //normal ,use sonar info update ins
			em[HEIGHT_SONAR].active=FALSE;
			em[HEIGHT_SONAR].finished=FALSE;
			#if TEST_MSG
			sonar_bound=0;
			#endif
		}
	}
}


static inline void baro_flight_check(void)
{
	/*baro information*/
	if( h_moni.baro_status)   //error
	{
		ins_int.baro_valid =FALSE;
		flight_limit_height=BARO_LIMIT_HEIGHT;
		set_except_mission(HEIGHT_BARO,TRUE,FALSE, FALSE,0, FALSE,FALSE,1);
        #if TEST_MSG
		baro_status=1;
		#endif
	}
	else
	{
		ins_int.baro_valid =TRUE;
		flight_limit_height=FLIGHT_LIMIT_HEIGHT;
		em[HEIGHT_BARO].active=FALSE;
		em[HEIGHT_BARO].finished=FALSE;
		#if TEST_MSG
		baro_status=0;
		#endif
	}
}


static void sonar_dead_distance_determine(void)
{
	if( h_moni.sonar_status!=1 ) return;
	
    if(h_moni.sonar_code==0)
    {
		h_moni.sonar_status=0;
		ins_int.update_on_agl=TRUE;
		return;
    }
	else
	{
        if( autopilot_in_flight && stateGetPositionEnu_f()->z >1.0)
		{
			h_moni.sonar_status=3;    //dead distance turn to error
		}
		
		h_moni.sonar_code=0;   //reset code
	}
}

void height_ground_reset(void)
{
	h_moni.sonar_ground_check=FALSE;
    h_moni.baro_ground_check=FALSE;
}

/***********************************************************************
* FUNCTION    : height info update frequence check function
* DESCRIPTION : call by monitoring_task function periodic, need periodic run to avoid counter overflow
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void height_frequence_check(void)   
{
	static bool_t check_start=FALSE;
	if(!check_start)   //use to determine the time begining check
	{
		if( h_moni.sonar_update_counter && h_moni.baro_update_counter )
		{
			check_start=TRUE;
			h_moni.sonar_update_counter=0;
			h_moni.baro_update_counter=0;
			return;
		}
		else 
		{
            h_moni.sonar_code |=0x04;
			h_moni.baro_code |=0x04;
			return;
		}
	}

	if( h_moni.sonar_update_counter >6)
    {
		h_moni.sonar_code &=0xFB;   //frequence is normal
    }
	else
	{
		h_moni.sonar_code |=0x04;
		h_moni.sonar_status=3;
		#if TEST_MSG
		height_fre=1;
		#endif
	}
	
	if( h_moni.baro_update_counter >6)
    {
		h_moni.baro_code &=0xFB;   //frequence is normal
    }
	else
	{
		h_moni.baro_code |=0x04;
		h_moni.baro_status=1;
		#if TEST_MSG
		height_fre=2;
		#endif
	}
	
	h_moni.sonar_update_counter=0;
	h_moni.baro_update_counter=0;

}


/***********************************************************************
* FUNCTION    : sonar call back function
* DESCRIPTION : check sonar information,only check_range will reset sonar_code and determine sonar_status
*               dynamic check:fix_data, frequence,noise and determine range
* INPUTS      : sender_id and sonar distance
* RETURN      : none
***********************************************************************/
static void sonar_moni_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  static uint32_t last_ts1, last_ts2;
  static float last_distance;

  h_moni.sonar_update_counter++;

  h_moni.sonar_aver = (distance + h_moni.sonar_aver * SUM_RATIO)/(SUM_RATIO +1 );
  if(get_sys_time_msec() <5000)  return;  //5s later begin run
  
  //error data determine(ground and flight)
  if(distance<SONAR_ERROR_DATA1 || distance>SONAR_ERROR_DATA2)
  {   
  	  if( (get_sys_time_msec()-last_ts1) <2000 )   //requset no error data in every 2s
      {   
	  	  h_moni.sonar_error_data_counter++;  
	  }
	  else 
	  {
	  	  h_moni.sonar_error_data_counter=0; 
		  #if TEST_MSG
		  sonar_error_data=0;
		  #endif
	  }
	  last_ts1=get_sys_time_msec();
	  if( h_moni.sonar_error_data_counter >4 )
	  {   
	  	  h_moni.sonar_code |=0x01;  //error data
	  	  h_moni.sonar_error_data_counter=0;  //restart
	  	  #if TEST_MSG
		  sonar_error_data=1;
		  #endif
	  }
  }
  #if TEST_MSG
  else 
  {
  	sonar_error_data=0;
  }
  #endif

  //check fix data
  if(above_ground)
  {
	  if( data_fix_check( (int32_t)(distance*100), (int32_t)(last_distance*100), &h_moni.sonar_fix_data_counter ,DATA_FIX_MAX ) )
	  {
	  	  h_moni.sonar_code |=0x02;
		  #if TEST_MSG
	      height_fix=1;
		  #endif
	  }
	  else
	  {
	  	  #if TEST_MSG
	      height_fix=0;
		  #endif
	  }
  }

  //check noise
  float deta_distance;
  if(!autopilot_in_flight)  deta_distance=DETA_SONAR_GROUND;
  else  deta_distance=DETA_SONAR_FLIGHT;
  if( fabs(distance-last_distance) >deta_distance )
  {
  	  if( (get_sys_time_msec()-last_ts2) <2000 )   //requset no noise data in every 2s
      {   
	  	  h_moni.sonar_interval_counter++;  
	  }
	  else 
	  {
	  	  h_moni.sonar_interval_counter=0; 
		  #if TEST_MSG
		  height_noise=0;
		  #endif
		  //sonar_code reset in status determine
	  }
	  last_ts2=get_sys_time_msec();
	  if( h_moni.sonar_interval_counter >4 )
	  {   
	  	  h_moni.sonar_code |=0x08;  //error data
	  	  h_moni.sonar_interval_counter=0;  //restart
	  	  #if TEST_MSG
		  height_noise=1;
		  #endif
	  }
  }
  #if TEST_MSG
  else
  {
  	height_noise=0;
  }
  #endif
  last_distance=distance;

  //check flight range
  sonar_range_check();
  
  //another ground check
  if(!h_moni.sonar_ground_check)
  {
  	h_moni.sonar_ground_counter++;
	if(h_moni.sonar_ground_counter>50)
	{
		if( h_moni.sonar_aver >(SONAR_GROUND+DETA_SONAR_GROUND) )
		{   h_moni.sonar_code |=0x10;  } //ground average distance is out
		
		h_moni.sonar_ground_check=TRUE; //finished ground check time
		h_moni.sonar_ground_counter=0;  //reset counter
  	}	
  }  
}



/***********************************************************************
* FUNCTION    : baro call back function
* DESCRIPTION : baro_code restore error info which need reset outside, baro_status is given
* INPUTS      : sender_id, and baro pressure(pascal)
* RETURN      : none
***********************************************************************/
static void baro_moni_cb(uint8_t __attribute__((unused)) sender_id,
                         uint32_t __attribute__((unused)) stamp, 
                         float pressure, float temp)
{
  static float last_pressure, ground_pressure_aver;
  static uint32_t last_ts;
  h_moni.baro_update_counter++;

  h_moni.baro_aver = (pressure + h_moni.baro_aver * SUM_RATIO)/(SUM_RATIO +1 );
  if(get_sys_time_msec() <15000)  return;  //10s later begin run(data is not stable in front
  
  //check fix data
  if( data_fix_check( (int32_t)(pressure*100), (int32_t)(last_pressure*100), &h_moni.baro_fix_data_counter ,DATA_FIX_MAX ) )
  {
  	  h_moni.baro_code |=0x02;
	  h_moni.baro_status=1; //set fail
	  #if TEST_MSG
      height_fix=2;
	  #endif
  }
  last_pressure=pressure;

  //check noise
  float deta_baro;
  if(!autopilot_in_flight)  deta_baro =DETA_BARO;
  else  deta_baro =DETA_BARO*5;
  if( !CHECK_INTERVAL(pressure, h_moni.baro_aver, deta_baro) )
  {
	if( (get_sys_time_msec()-last_ts) <2000 )   //requset no error data in every 2s
      {   
	  	  h_moni.baro_interval_counter++;  
	  }
	  else 
	  {
	  	  h_moni.baro_interval_counter=0; 		
		  //h_moni.baro_code &=0xF7;  //reset
	  }
	  last_ts=get_sys_time_msec();
	  if( h_moni.baro_interval_counter >4 )
	  {   
	  	  h_moni.baro_code |=0x08;  //error data
	  	  h_moni.baro_status= 1;    //set fail
	  	  h_moni.baro_interval_counter=0;  //restart
	  	  #if TEST_MSG
		  height_noise=2;
		  #endif
	  }
  }

  //check range
  if(!h_moni.baro_ground_check)
  {
  	h_moni.baro_ground_counter++;
    if(h_moni.baro_ground_counter>50)
    {
		h_moni.baro_ground_check=TRUE; //finished ground check time
		h_moni.baro_ground_counter=0;  //reset counter
		if(h_moni.baro_aver >BARO_MIN && h_moni.baro_aver<BARO_MAX)   //ground bound pressure
		{
			ground_pressure_aver=h_moni.baro_aver;
		}
		else
		{
			h_moni.baro_code |=0x10;
			h_moni.baro_status=1;   //set fail
		}
    }
  }
  if(autopilot_in_flight)
  {
  	 if( h_moni.baro_aver <(ground_pressure_aver-FLIGHT_RANGE) ||
	 	 h_moni.baro_aver >(ground_pressure_aver+FLIGHT_RANGE/2) )
  	 {
	 	h_moni.baro_code |=0x10;
	    h_moni.baro_status=1;     //set fail	
	    #if TEST_MSG
		baro_flight_range=1;
		#endif
  	 }
	 else
	 {      
		//h_moni.baro_code &=0xEF;  //reset normal
	 }
  }
  
}

static inline void sonar_range_check(void)
{
	if(above_ground && (h_moni.sonar_status==0 || h_moni.sonar_status==2) )
	{
	 if(h_moni.sonar_aver >SONAR_VALID)    //sonar is out of distance,reset code
	 {
	 	h_moni.sonar_code =0x10;   //clear other error,only set range
		h_moni.sonar_status=2;
	 }
	 else  //sonar back to valid
	 {
	 	if( (h_moni.sonar_code&0x10) >>4 )  
	 	{
			h_moni.sonar_code =0;   //range error,restart to record
	 	    //h_moni.sonar_status=0; 
	 	}
		else if(h_moni.sonar_code !=0)
		{
	        if(ins_int.baro_valid && ins_int.baro_z <-3.5) 
	        {
				h_moni.sonar_code =0x10;   //clear other error,only set range
				h_moni.sonar_status=2;
	        }
			else
			{
				h_moni.sonar_status=3;   //set sonar fail,include error or out of meas
			}
		}
		else
		{
			h_moni.sonar_status=0; 
		}
	 }
	}
}
/**************** END OF FILE *****************************************/

