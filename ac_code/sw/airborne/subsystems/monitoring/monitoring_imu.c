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
#include "subsystems/monitoring/monitoring_imu.h"
#include "subsystems/monitoring/monitoring.h" 
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "firmwares/rotorcraft/autopilot.h"

#if TEST_MSG
#include "subsystems/datalink/downlink.h"
#endif

/*===VARIABLES========================================================*/
#define SUM_RATIO 9
#define DETA_ACCEL 408  //0.4(2**10)   request 200/5000
#define DETA_GYRO 1638  //0.4(2**12)   request 200/5000
#define DETA_MAG  42    //0.02(2**11)    request 50/500
#define DETA_MAG_LEN2   2000000  // (2**22)   need test
#define ACC_XY  4000   //3.0
#define ACC_Z_MIN 9010  //8.8
#define ACC_Z_MAX 11060  //10.8
#define GYRO_P_Q_R 8192  //3 //2.0
#define MAG_X_Y_Z  4096  //2.0

#define DATA_FIX_MAX 100

/*---Global-----------------------------------------------------------*/
struct Imu_Monitor imu_moni;

static abi_event gyro_ev_mo;
static abi_event accel_ev_mo;
static abi_event mag_ev_mo;


static void gyro_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Rates *gyro);
static void accel_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel);
static void mag_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *mag);



/*===FUNCTIONS========================================================*/

/***********************************************************************
* FUNCTION    : accel call back function
* DESCRIPTION : use to inspect static information
* INPUTS      : sender_id,data stamp time and accel info
* RETURN      : none
***********************************************************************/
void imu_moni_init(void)
{
	imu_moni.imu_status=1;  //default ok
	imu_ground_reset();
	AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev_mo, accel_moni_cb);
	AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev_mo, gyro_moni_cb);
	AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev_mo, mag_moni_cb);	
}

uint8_t imu_ground_check(void)
{
	uint8_t check_code;
	if(imu_moni.accel_ground_check && imu_moni.gyro_ground_check && imu_moni.mag_ground_check)
	{
		if( imu_moni.imu_status==1 )
		    check_code=1;  //pass
		else 
			check_code=2;  //fail
	}
	else    //run checking
	{     
		check_code=0;    //running 
	}

	return check_code;
}



void imu_ground_reset(void)   //use for restart ground check
{
	imu_moni.accel_ground_check = FALSE;
  imu_moni.gyro_ground_check = FALSE;
	imu_moni.mag_ground_check = FALSE;
}

void imu_flight_check(void)  //only accel/gyro/mag fix_data+frequence +mag_EMI
{
	if(imu_moni.imu_status) 
	{
		#if TEST_MSG
		 fs_imu=0;
		#endif
		return;  //normal
	}

	if( !imu_moni.imu_error[0] && !imu_moni.imu_error[1] && !imu_moni.imu_error[2] ) 
	{
		em[IMU_CRITICAL].active =FALSE;
		em[IMU_CRITICAL].finished =FALSE;
		em[IMU_MAG_EMI].active =FALSE;
		em[IMU_MAG_EMI].finished =FALSE;
		//only momentary,recover do nothing
		//set_except_mission(IMU_MOMENTARY,FALSE,FALSE, FALSE,0, FALSE,FALSE,0);	

		//TODOM:need add to black_block for recording
		#if TEST_MSG
		 fs_imu=1;
		#endif
	}
	else if( !imu_moni.imu_error[0] && !imu_moni.imu_error[1] && !(imu_moni.imu_error[2]&0xEF) )
	{   
		em[IMU_CRITICAL].active =FALSE;
		em[IMU_CRITICAL].finished =FALSE;
		//set_except_mission(IMU_MAG_EMI,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,2);
		//set_except_mission(IMU_MAG_EMI,TRUE,FALSE, FALSE,0, FALSE,FALSE,2);
		//TODOM:after RTK GPS course add,need modify
		#if TEST_MSG
		 fs_imu=2;
		#endif
	}
	else 
	{   //other error:fix_data or frequence
	    //set limit height hover until other cmd
	    set_except_mission(IMU_CRITICAL,TRUE,FALSE, TRUE,0xFF, FALSE,FALSE,3);
		#if TEST_MSG
		 fs_imu=3;
		#endif
	}	

}

void imu_frequence_check(void)   //need periodic run to avoid counter overflow
{
	static bool_t check_start=FALSE;
	if(!check_start)  //use to determine the time begining check
	{
		if( imu_moni.gyro_update_counter && imu_moni.accel_update_counter && imu_moni.mag_update_counter )
		{
			check_start=TRUE;
			imu_moni.gyro_update_counter=0;
			imu_moni.accel_update_counter=0;
			imu_moni.mag_update_counter=0;
			return;
		}
		else 
		{
            imu_moni.imu_error[0] |=0x02;
			imu_moni.imu_error[1] |=0x02;
			imu_moni.imu_error[2] |=0x02;
			return;
		}
	}
	
	if( imu_moni.gyro_update_counter >100)
    {
		imu_moni.imu_error[0] &=0xFD;   //frequence is normal
    }
	else
	{
		imu_moni.imu_error[0] |=0x02;
		imu_moni.imu_status=0;
		#if TEST_MSG
		fre_imu=1;
		#endif
	}
    if( imu_moni.accel_update_counter >100)
    {
		imu_moni.imu_error[1] &=0xFD;   //frequence is normal
    }
	else
	{
		imu_moni.imu_error[1] |=0x02;
		imu_moni.imu_status=0;
		#if TEST_MSG
		fre_imu=2;
		#endif
	}
    if( imu_moni.mag_update_counter >20)
    {
		imu_moni.imu_error[2] &=0xFD;   //frequence is normal
    }
	else
	{
		imu_moni.imu_error[0] |=0x02;
		imu_moni.imu_status=0;
		#if TEST_MSG
		fre_imu=3;
		#endif
	}
	
	imu_moni.gyro_update_counter=0;
	imu_moni.accel_update_counter=0;
	imu_moni.mag_update_counter=0;

}


/***********************************************************************
* FUNCTION    : accel call back function
* DESCRIPTION : use to inspect information,dynamic check:fix data,frequence
* INPUTS      : sender_id,data stamp time and accel info
* RETURN      : none
***********************************************************************/
static void accel_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)), struct Int32Vect3 *accel)
{
  
  static struct Int32Vect3 accel_last;
  imu_moni.accel_update_counter++;    //reset to 0 in main monitoring
  
  //check fix data
  if( data_fix_check(accel->x, accel_last.x, &imu_moni.accel_fix_counter.x, DATA_FIX_MAX)||
	  data_fix_check(accel->y, accel_last.y, &imu_moni.accel_fix_counter.y, DATA_FIX_MAX)||
	  data_fix_check(accel->z, accel_last.z, &imu_moni.accel_fix_counter.z, DATA_FIX_MAX)  ) 
  {  //fix data
      imu_moni.imu_error[1] |=0x02;  //fix data
      imu_moni.imu_status=0;  //set imu fail
      #if TEST_MSG
	  fix_imu=2;
	  #endif
      return;  	
  }
  imu_moni.imu_error[1] &=0xFD;  //no fix data
  VECT3_COPY(accel_last,*accel);
  
  if(imu_moni.accel_ground_check)  return;  //finished ground check


  /*below is another ground inspect: range check and noise check*/  
  imu_moni.accel_aver.x = (accel->x + imu_moni.accel_aver.x * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.accel_aver.y = (accel->y + imu_moni.accel_aver.y * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.accel_aver.z = (accel->z + imu_moni.accel_aver.z * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.accel_ground_counter++;
  
  if(imu_moni.accel_ground_counter <1000)  return;   //give up data before 1000th
  
  else if(imu_moni.accel_ground_counter <6000)    //run ground inspect
  { 
    //data inspect counter request 5000		
	if( //check accel data is horizontal
		( abs(imu_moni.accel_aver.x)+abs(imu_moni.accel_aver.y) )<ACC_XY &&      
		 -imu_moni.accel_aver.z<ACC_Z_MAX &&
		 -imu_moni.accel_aver.z>ACC_Z_MIN                                   )
	{   //data is normal(horizontal),caculate noise
		if( !CHECK_INTERVAL(accel->x, imu_moni.accel_aver.x, DETA_ACCEL) )  
		{ imu_moni.accel_interval_counter.x++; }
		if( !CHECK_INTERVAL(accel->y, imu_moni.accel_aver.y, DETA_ACCEL) )  
		{ imu_moni.accel_interval_counter.y++; }
		if( !CHECK_INTERVAL(accel->z, imu_moni.accel_aver.z, DETA_ACCEL) )  
		{ imu_moni.accel_interval_counter.z++; }
	}
	else 
	{   //data is unnormal(not horizontal),set fail
	    imu_moni.imu_error[1] |=0x08;  //data out of range
        imu_moni.imu_status=0;  //set imu fail
	 	//device_status &=0xFFFDFFFF;  //bit 18 set 0
		imu_moni.accel_ground_check=TRUE;
		imu_moni.accel_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_range_imu=2;
		#endif
		return;    //finished ground inspect
	}
  }
  
  else //imu_moni.accel_ground_counter>=6000, check noise
  {
  	if(imu_moni.accel_interval_counter.x >500 ||
	   imu_moni.accel_interval_counter.y >500 ||
	   imu_moni.accel_interval_counter.z >500)
  	{   //data is unnormal, set fail
  	    imu_moni.imu_error[1] |=4;  //noise error
        imu_moni.imu_status=0;  //set imu fail
		//device_status &=0xFFFDFFFF;  //bit 18 set 0
		imu_moni.accel_ground_check=TRUE;
		imu_moni.accel_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_noise_imu=2;
		#endif
		return;   //finished ground inspect
  	}
	else
	{   //data is normal, set pass
	    imu_moni.imu_error[1] =0;  //normal
        //imu_moni.imu_status=1;  //use default,back to ground need reset default 1
		//device_status |=0x00020000;  //bit 18 set 1
		imu_moni.accel_ground_check=TRUE;   
		imu_moni.accel_ground_counter=0;  //reset counter
		return;   //finished ground inspect
	}
  }
  
}



/***********************************************************************
* FUNCTION    : gyro call back function
* DESCRIPTION : use to inspect gyro,dynamic check:fix data/frequence
* INPUTS      : sender_id,data stamp time and gyro info
* RETURN      : none
***********************************************************************/
static void gyro_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)), struct Int32Rates *gyro)
{
  static struct Int32Rates gyro_last;
  imu_moni.gyro_update_counter++;    //reset to 0 in main monitoring
  
  //check fix data
  if( data_fix_check(gyro->p, gyro_last.p, &imu_moni.gyro_fix_counter.x, DATA_FIX_MAX)||
	  data_fix_check(gyro->q, gyro_last.q, &imu_moni.gyro_fix_counter.y, DATA_FIX_MAX)||
	  data_fix_check(gyro->r, gyro_last.r, &imu_moni.gyro_fix_counter.z, DATA_FIX_MAX)  ) 
  {  //fix data
      imu_moni.imu_error[0] |=0x01;  //fix data
      imu_moni.imu_status = 0;  //set imu fail
      #if TEST_MSG
	  fix_imu=1;
	  #endif
      return;  	
  }
  imu_moni.imu_error[0] &=0xFE;  //no fix data
  gyro_last.p= gyro->p;
  gyro_last.q= gyro->q;
  gyro_last.r= gyro->r;  

  if(imu_moni.gyro_ground_check)  return;  //finished ground inspect


  /*below is another ground inspect: range check and noise check*/  
  imu_moni.gyro_aver.p = (gyro->p + imu_moni.gyro_aver.p * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.gyro_aver.q = (gyro->q + imu_moni.gyro_aver.q * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.gyro_aver.r = (gyro->r + imu_moni.gyro_aver.r * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.gyro_ground_counter++;

  if( imu_moni.gyro_ground_counter <1000 )  return;   //give up data before 1000th
  
  else if( imu_moni.gyro_ground_counter <6000 )    //run ground inspect
  { //data inspect counter request 5000  	
	if( //check average out of static
		abs(imu_moni.gyro_aver.p)<GYRO_P_Q_R && 
		abs(imu_moni.gyro_aver.q)<GYRO_P_Q_R &&
		abs(imu_moni.gyro_aver.r)<GYRO_P_Q_R   )
	{   //data is normal,continue caculate noise
		if( !CHECK_INTERVAL(gyro->p, imu_moni.gyro_aver.p, DETA_GYRO) )  
		{ imu_moni.gyro_interval_counter.x++; }
		if( !CHECK_INTERVAL(gyro->q, imu_moni.gyro_aver.q, DETA_GYRO) )  
		{ imu_moni.gyro_interval_counter.y++; }
		if( !CHECK_INTERVAL(gyro->r, imu_moni.gyro_aver.r, DETA_GYRO) )  
		{ imu_moni.gyro_interval_counter.z++; }
	}
	else 
	{   //data is unnormal(out of static), set fail
  	    imu_moni.imu_error[0] |=0x08; //out of range
		imu_moni.imu_status =0;	    
		//device_status &=0xFFFEFFFF;  //bit 17 set 0
		imu_moni.gyro_ground_check=TRUE;
		imu_moni.gyro_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_range_imu=1;
		#endif
		return;
	}
  }
  
  else //imu_moni.gyro_ground_counter>=6000,check noise
  {
  	if(imu_moni.gyro_interval_counter.x >200 ||
	   imu_moni.gyro_interval_counter.y >200 ||
	   imu_moni.gyro_interval_counter.z >200)
  	{   //data is unnormal, set fail
  	    imu_moni.imu_error[0] |=0x04; //noise error
		imu_moni.imu_status =0;
		//device_status &=0xFFFEFFFF;  //bit 17 set 0
		imu_moni.gyro_ground_check=TRUE;
		imu_moni.gyro_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_noise_imu=1;
		#endif
		return;
  	}
	else
	{   //data is normal,set pass
	  	imu_moni.imu_error[0] =0; //normal
		//imu_moni.imu_status=1;  //use default,back to ground need reset default 1
		//device_status |=0x00010000;  //bit 17 set 1
		imu_moni.gyro_ground_check=TRUE;   
		imu_moni.gyro_ground_counter=0;  //reset counter
		return;
	}
  }
  
}



/***********************************************************************
* FUNCTION    : mag call back function
* DESCRIPTION : use to inspect information,dynamic check:fix data,frequence,EMI
* INPUTS      : sender_id,data stamp time and mag info
* RETURN      : none
***********************************************************************/
static void mag_moni_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)), struct Int32Vect3 *mag)
{
  static struct Int32Vect3 mag_last;
  static uint32_t mag_len2_aver, last_ts;
  uint32_t mag_len2;
  imu_moni.mag_update_counter++;    //reset to 0 in main monitoring  

  //check fix data
  if( data_fix_check(mag->x, mag_last.x, &imu_moni.mag_fix_counter.x, DATA_FIX_MAX)||
	  data_fix_check(mag->y, mag_last.y, &imu_moni.mag_fix_counter.y, DATA_FIX_MAX)||
	  data_fix_check(mag->z, mag_last.z, &imu_moni.mag_fix_counter.z, DATA_FIX_MAX)  ) 
  {  //fix data
      imu_moni.imu_error[2] |=0x01;  //fix data
      imu_moni.imu_status=0;  //set imu fail
      #if TEST_MSG
	  fix_imu=3;
	  #endif
      return;  	
  }
  imu_moni.imu_error[2] &=0xFE;
  VECT3_COPY(mag_last, *mag);

  //check interference(after ground check finished)
  mag_len2 =mag->x*mag->x + mag->y*mag->y + mag->z*mag->z;
  if(imu_moni.mag_ground_check)
  {
  	  if( !CHECK_INTERVAL(mag_len2, mag_len2_aver, DETA_MAG_LEN2) )
      {	  	 
	  	 if( (get_sys_time_msec()-last_ts)<2000 )   //request time interval >2s
	  	 {
		 	 imu_moni.mag_len2_counter++;
			 #if TEST_MSG
			 mag_emi_counter = imu_moni.mag_len2_counter;
			 #endif
	  	 }
		 else 
		 {
		 	imu_moni.mag_len2_counter = 0;
			imu_moni.imu_error[2] &=0xEF;  //mag interference recover
			#if TEST_MSG
			 mag_emi=0;
			#endif

		 }
		 if(imu_moni.mag_len2_counter > 200)
		 {
		 	imu_moni.mag_len2_counter = 201;  //avoid run flow
		 	imu_moni.imu_error[2] |=0x10;  //mag interference error
			imu_moni.imu_status = 0;
			#if TEST_MSG
			 mag_emi = 1;
			#endif
		 }
		 last_ts = get_sys_time_msec();
      }
  }

  if(imu_moni.mag_ground_check)  return;   //finished ground inspect  
  

  /*below is another ground inspect: range check and noise check*/  
  imu_moni.mag_aver.x = (mag->x + imu_moni.mag_aver.x * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.mag_aver.y = (mag->y + imu_moni.mag_aver.y * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.mag_aver.z = (mag->z + imu_moni.mag_aver.z * SUM_RATIO)/(SUM_RATIO +1 );
  imu_moni.mag_ground_counter++;
  
  if( imu_moni.mag_ground_counter <100 )  return;   //give up data before 100th
  
  else if( imu_moni.mag_ground_counter <600 )  
  { 
    //data inspect counter request 500  
    
    //caculate mag_len2_aver in ground,use for interference
	mag_len2_aver = (mag_len2 + mag_len2_aver * SUM_RATIO)/(SUM_RATIO +1 );
	
	if( //check mag data is in horizon
		abs(imu_moni.mag_aver.x)<MAG_X_Y_Z && 
		abs(imu_moni.mag_aver.y)<MAG_X_Y_Z &&
		abs(imu_moni.mag_aver.z)<MAG_X_Y_Z 	  )
	{
		if( !CHECK_INTERVAL(mag->x, imu_moni.mag_aver.x, DETA_MAG) )  
		{ imu_moni.mag_interval_counter.x++; }
		if( !CHECK_INTERVAL(mag->y, imu_moni.mag_aver.y, DETA_MAG) )  
		{ imu_moni.mag_interval_counter.y++; }
		if( !CHECK_INTERVAL(mag->z, imu_moni.mag_aver.z, DETA_MAG) )  
		{ imu_moni.mag_interval_counter.z++; }
	}	
	else 
	{   //mag average out of design range
	    imu_moni.imu_error[2] |=0x08;  //out of range
		imu_moni.imu_status=0;
		//device_status &=0xFFFBFFFF;  //bit 19 set 0
		imu_moni.mag_ground_check=TRUE;
		imu_moni.mag_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_range_imu=3;
		#endif
		return;
	}
  }
  
  else //(imu_moni.mag_ground_counter>=600),check noise
  {
  	if(imu_moni.mag_interval_counter.x >50 ||
	   imu_moni.mag_interval_counter.y >50 ||
	   imu_moni.mag_interval_counter.z >50)
  	{ //noise error
  	    imu_moni.imu_error[2] |=0x04;
		imu_moni.imu_status=0;
		//device_status &=0xFFFBFFFF;  //bit 19 set 0
		imu_moni.mag_ground_check=TRUE;
		imu_moni.mag_ground_counter=0;  //reset counter
		#if TEST_MSG
		g_noise_imu=3;
		#endif
		return;
  	}
	else
	{//data pass ground check
	    imu_moni.imu_error[2] =0;
		//imu_moni.imu_status=1;  //use default,back to ground need reset default 1
		//device_status |=0x00040000;  //bit 19 set 1
		imu_moni.mag_ground_check=TRUE;   
		imu_moni.mag_ground_counter=0;  //reset counter
		return;
	}
  }
  
}

#define MAX_GYRO_OFFSET 200
#define NUM_GYRO_OFFSET_CAL 2000
static struct Int32Rates gyro_offset;
bool_t gyro_offset_caculate(struct Imu *_imu)
{
	static uint16_t cal_count = 0;
	static uint8_t noise_count = 0;
	if(cal_count < NUM_GYRO_OFFSET_CAL)
	{
		if( abs(_imu->gyro_unscaled.p) > MAX_GYRO_OFFSET
			|| abs(_imu->gyro_unscaled.q) > MAX_GYRO_OFFSET
			|| abs(_imu->gyro_unscaled.r) > MAX_GYRO_OFFSET )
		{
			noise_count++;
			if(noise_count > 3)  //noise from sensors,reset
			{
				cal_count = 0;
				RATES_ASSIGN(gyro_offset, 0, 0, 0);	
			}
		}
		else
		{  
			noise_count = 0;
			cal_count++;
			gyro_offset.p += _imu->gyro_unscaled.p;
			gyro_offset.q += _imu->gyro_unscaled.q;
			gyro_offset.r += _imu->gyro_unscaled.r;
		}
	}
	if(cal_count == NUM_GYRO_OFFSET_CAL)
	{
		struct Int32Rates gyro_offset_temp;
		gyro_offset_temp.p = gyro_offset.p/(NUM_GYRO_OFFSET_CAL);
		gyro_offset_temp.q = gyro_offset.q/(NUM_GYRO_OFFSET_CAL);
		gyro_offset_temp.r = gyro_offset.r/(NUM_GYRO_OFFSET_CAL);
		if( abs(gyro_offset_temp.p) > MAX_GYRO_OFFSET
			|| abs(gyro_offset_temp.q) > MAX_GYRO_OFFSET
			|| abs(gyro_offset_temp.r) > MAX_GYRO_OFFSET )
		{
			imu_moni.imu_error[0] |=0x08; //out of range
			imu_moni.imu_status = 0;
			#if TEST_MSG
		    g_range_imu=1;
		    #endif
			return TRUE;
		}
		else
		{
			RATES_COPY(_imu->gyro_neutral,gyro_offset_temp);
			return TRUE;
		}
	}
	if( get_sys_time_float() > 20.0 )
	{
		imu_moni.imu_error[0] |= 0x04; //noise error
		imu_moni.imu_status = 0;
		#if TEST_MSG
		g_noise_imu=1;
		#endif
		return TRUE;
	}
	
	return FALSE;
}
/**************** END OF FILE *****************************************/
