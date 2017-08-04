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
#define DETA_BARO          20.0 //20pa
#define FLIGHT_RANGE       500.0  //pa

#define BARO_MIN 60000.0  //unit =pa
#define BARO_MAX 110000.0

#define DATA_FIX_MAX 20

/*---Global-----------------------------------------------------------*/
struct Height_Monitor h_moni;

static abi_event baro_ev_mo;

static void baro_moni_cb(uint8_t __attribute__((unused)) sender_id,
												 uint32_t __attribute__((unused)) stamp,
												 float pressure, float temp);

static inline void baro_flight_check(void);


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
	AbiBindMsgBARO_ABS(ABI_BROADCAST, &baro_ev_mo, baro_moni_cb);
}

/***********************************************************************
* FUNCTIONS   : height_ground_check
* DESCRIPTION :  get ground check state of baro
* INPUTS      :
* RETURN      : check_code
***********************************************************************/
uint8_t height_ground_check(void)
{
	uint8_t check_code = 0;  //default is running

	if(h_moni.baro_ground_check )
	{
		if(!h_moni.baro_code)   //normal
		{
			h_moni.baro_status = 0;
			check_code = 1;  //set pass
		}
		else                   //set error
		{
			h_moni.baro_status = 1;
			check_code = 2;  //set fail
		}
	}

	return check_code;
}

/***********************************************************************
* FUNCTIONS   : height_ground_check
* DESCRIPTION : return baro error state
* INPUTS      :
* RETURN      : error_code
***********************************************************************/
uint8_t height_ground_check_code(void)
{
	if(h_moni.baro_code&0x02)
	{
		return 2;//fix data
	}
	else if(h_moni.baro_code&0x04)
	{
		return 1;//fre data
	}
	else if(h_moni.baro_code&0x08)
	{
		return 4;//noise data
	}
	else if(h_moni.baro_code&0x10)
	{
		return 3;//range data
	}
	else
	{
		return 0;
	}
}


void height_flight_check(void)
{
	baro_flight_check();
}

static inline void baro_flight_check(void)
{
	/*baro information*/
	if (!ins_int_is_baro_valid())   //error
	{
		set_except_mission(HEIGHT_BARO, TRUE, FALSE, TRUE, 0xFF, FALSE, FALSE, 2);
#if TEST_MSG
		baro_status = 1;
#endif
	}
	else
	{
		em[HEIGHT_BARO].active = FALSE;
		em[HEIGHT_BARO].finished = FALSE;
#if TEST_MSG
		baro_status = 0;
#endif
	}
}

void height_ground_reset(void)
{
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
	static bool_t check_start = FALSE;
	if(!check_start)   //use to determine the time begining check
	{
		if( h_moni.baro_update_counter )
		{
			check_start=TRUE;
			h_moni.baro_update_counter=0;
			return;
		}
		else
		{
			h_moni.baro_code |=0x04;
			return;
		}
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

	h_moni.baro_update_counter=0;

	if(h_moni.baro_status)
	{
		ins_int.baro_valid = FALSE;
	}
	else
	{
		ins_int.baro_valid = TRUE;
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
	if(get_sys_time_msec() <15000)  return;  //15s later begin run(data is not stable before

	//check fix data
	if (data_fix_check((int32_t) (pressure * 100), (int32_t) (last_pressure * 100), &h_moni.baro_fix_data_counter,
										 DATA_FIX_MAX))
	{
		h_moni.baro_code |= 0x02;
		h_moni.baro_status = 1; //set fail
#if TEST_MSG
		height_fix = 2;
#endif
		return;
	}
	h_moni.baro_code &=0xFD;
	last_pressure  =pressure;

	//check noise
	float deta_baro;
	if(!autopilot_in_flight)
	{
		deta_baro = DETA_BARO;
	}
	else
	{
		deta_baro = DETA_BARO*10.0;
	}
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
		last_ts = get_sys_time_msec();
		if( h_moni.baro_interval_counter >4 )
		{
			h_moni.baro_code |=0x08;  //error data
			h_moni.baro_status = 1;    //set fail
			h_moni.baro_interval_counter = 0;  //restart
#if TEST_MSG
			height_noise=2;
#endif
		}
	}

	//check range
	if(!h_moni.baro_ground_check)
	{
		h_moni.baro_ground_counter++;
		if(h_moni.baro_ground_counter > 50)
		{
			h_moni.baro_ground_check = TRUE; //finished ground check time
			h_moni.baro_ground_counter = 0;  //reset counter
			if(h_moni.baro_aver >BARO_MIN && h_moni.baro_aver<BARO_MAX)   //ground bound pressure
			{
				ground_pressure_aver = h_moni.baro_aver;
			}
			else
			{
				h_moni.baro_code |=0x10;
				h_moni.baro_status = 1;   //set fail
			}
		}
	}

	if(autopilot_in_flight)
	{
		if( h_moni.baro_aver < (ground_pressure_aver-FLIGHT_RANGE) ||
				h_moni.baro_aver > (ground_pressure_aver+FLIGHT_RANGE/2) )
		{
			h_moni.baro_flight_counter ++;
			if(h_moni.baro_flight_counter > 10)
			{
				h_moni.baro_code |=0x10;
				h_moni.baro_status = 1;     //set fail
				baro_flight_range = 1;
			}

		}
		else
		{
			//h_moni.baro_code &=0xEF;  //reset normal
		}
	}
	else if((!autopilot_in_flight) && (!ground_check_pass))//during selfcheck
	{
		if(h_moni.baro_ground_check == TRUE)
		{
			if( !CHECK_INTERVAL(pressure, ground_pressure_aver, DETA_BARO) )
			{
				h_moni.baro_ground_counter++;
				if(h_moni.baro_ground_counter > 10)
				{
						h_moni.baro_code |=0x10;
						h_moni.baro_status = 1;
						baro_flight_range = 1;
				}
				else
				{
					
				}
			}
			else 
			{
				h_moni.baro_ground_counter = 0;
			}	
		}
		
	}

}


/**************** END OF FILE *****************************************/

