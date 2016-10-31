/**
 * @file subsystems/datalink/downlink_xbee_periodic.c
 * use for send periodic message from aircraft to gcs(android)
 */
#include "subsystems/datalink/downlink_xbee_periodic.h"
#include "generated/periodic_telemetry.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/ops/ops_app_if.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/gcs_nav_xbee.h"
#include "subsystems/mission/task_manage.h"
#include "subsystems/mission/task_process.h"
#include "uplink_ac.h"
#include "subsystems/electrical.h"

#if DATALINK == XBEE
#include "subsystems/datalink/xbee.h"
#endif	/* XBEE */
#if DATALINK == TRANSPTA
#include "subsystems/datalink/transpta.h"
#endif	/* TRANSPTA */
#include "subsystems/monitoring/monitoring.h"

//#include "subsystems/datalink/downlink.h"
//#include "uplink_ac.h"

//#define DOWNLINK_GCS_FREQUENCY 2
#define HEART_BEART_A2G_FREQUENCY 1
#define RC_INFO_PC_FREQUENCY 1
#define NAV_FLIGHT_FREQUENCY 1
//#define MISSION_REPORT__FREQUENCY 0.5
#define RC_CHECK_FREQUENCY  2

void downlink_gcs_periodic(void)  //run in DOWNLINK_GCS_FREQUENCY
{   
	//RunOnceEvery(TELEMETRY_FREQUENCY/DOWNLINK_GCS_FREQUENCY,downlink_msg_gcs());  
	RunOnceEvery( TELEMETRY_FREQUENCY/HEART_BEART_A2G_FREQUENCY, 
	              { 
					#if DATALINK==XBEE
					if(xbee_con_info.gcs_con_available==FALSE) return;
					#endif
				    send_heart_beat_A2G_msg(); } );
}

void downlink_pc_periodic(void)  //run in DOWNLINK_GCS_FREQUENCY
{   
	//RunOnceEvery(TELEMETRY_FREQUENCY/DOWNLINK_GCS_FREQUENCY,downlink_msg_gcs());  
	RunOnceEvery( TELEMETRY_FREQUENCY/RC_INFO_PC_FREQUENCY, 
	              { if(xbee_con_info.rc_con_available==FALSE) return;
				    send_rc_info_A2P_msg();                         } );
	RunOnceEvery( TELEMETRY_FREQUENCY/NAV_FLIGHT_FREQUENCY, 
	              { //if(xbee_con_info.gcs_con_available==FALSE) return;
				    send_nav_flight();                              } );

    RunOnceEvery( TELEMETRY_FREQUENCY/NAV_FLIGHT_FREQUENCY, 
	              { if(xbee_con_info.gcs_con_available==FALSE) return;
				    send_task_info_pc();                        } );
					
}

/*
    <field name="sys_time" type="uint16" unit="s"/>
    <field name="link_gcs_quality" type="uint8" unit="percent"/>  <!--reserve-->
    <field name="link_rc_quality" type="uint8" unit="percent"/>   <!--reserve-->
    <field name="link_ac_quality" type="uint8" unit="percent"/>   <!--reserve-->
    <field name="flight_status" type="uint16"/>                   
    <field name="heading" type="float" unit="deg"/>
    <field name="speed" type="int16" unit="cm/s"/>
    <field name="flight_alt" type="int16" unit="cm"/>
    <field name="pos_lng" type="int32" unit="e8,rad"/>    
    <field name="pos_lat" type="int32" unit="e8,rad"/>    
    <field name="battery_remain"  type="int8"  unit="percent"/>
    <field name="pesticides_remain" type="int8" unit="percent"/>   
    <field name="error_code" type="uint32"/>
*/
void send_heart_beat_A2G_msg(void)  
{
   uint16_t system_time = sys_time.nb_sec;
   
   uint8_t  link_gcs_quality = 100;  //the information reserve
   if(gcs_lost) 
   	{
		link_gcs_quality = 0 ;
   	}
   uint8_t  link_rc_quality = 100;   //the information reserve
   if(rc_lost) 
   	{
		link_rc_quality = 0 ;
   	}
   uint8_t  link_rssi = 100;         //the information reserve

   uint8_t gcs_cmd_state = (uint8_t)gcs_task_cmd;
   uint16_t fl_status = get_flight_status();
   
   float   heading = (stateGetNedToBodyEulers_f()->psi) * 57.2957795;
   int16_t speed = (int16_t)( stateGetHorizontalSpeedNorm_f()*100 );

   int16_t flight_alt = (int16_t)( (stateGetPositionEnu_f()->z )*100 );    
   int32_t pos_lon = (int32_t)( (int64_t)(stateGetPositionLla_i()->lon) * 17453293/100000000 ); 
   int32_t pos_lat = (int32_t)( (int64_t)(stateGetPositionLla_i()->lat) * 17453293/100000000 ); 
   //int32_t pos_lon = 198833922;
   //int32_t pos_lat = 39340866;
  /*
   static uint8_t i = 1;
   int32_t pos_lon = 198833922 + 10*i;
   int32_t pos_lat = 39340866 + 5*i;
   i++;
   if(i==20)  i=1;
  */
   
   int8_t  battery_remain = ops_info.o_bat_rep_percent; //(int8_t)((electrical.vsupply-420)*100/60);
   Bound(battery_remain, 0, 100);
   int8_t  pesticides_remain = (int8_t)(ops_info.res_cap&0x00FF);

   uint32_t error_code = em_code&0xFFFFF7FF;   //remove gcs lost error

   xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);	 //ack processing need handle later
   DOWNLINK_SEND_HEART_BEAT_AC_GCS_STATE(SecondChannel, SecondDevice, 
   	                                     &system_time, 
   	                                     &link_gcs_quality, 
   	                                     &link_rc_quality, 
   	                                     &link_rssi, 
   	                                     &gcs_cmd_state,
   	                                     &fl_status, 
   	                                     &heading,
   	                                     &speed,  
   	                                     &flight_alt, 
   	                                     &pos_lon, 
   	                                     &pos_lat, 
   	                                     &battery_remain, 
   	                                     &pesticides_remain,
   	                                     &error_code);
   
}

void send_rc_info_A2P_msg(void)
{
	#if PERIODIC_TELEMETRY
	 xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);	 //ack processing need handle later
    DOWNLINK_SEND_RC_INFO(DefaultChannel, DefaultDevice, 
	                      &rc_set_info.vtol,
	                      &rc_set_info.home,
	                      &rc_set_info.spray_grade,
	                      &rc_motion_cmd,
	                      &rc_set_cmd,
	                      &rc_motion_info.rotation_rate,
	                      &rc_motion_info.speed_fb,
	                      &rc_motion_info.speed_rl);	
	#endif
}

void send_nav_flight(void)
{
	#if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);	 //ack processing need handle later
	DOWNLINK_SEND_NAV_FLIGHT(DefaultChannel, DefaultDevice,
		                     &flight_state,
		                     &flight_mode,
		                     &task_state,
		                     &task_error_state,
		                     &rc_lost,
		                     &gcs_lost);
	#endif
}

#if 0
void send_DATA_TEST()
{
    uint8_t u8=100;
	int8_t  s8=-100;
	uint16_t u16=30000;
	int16_t  s16=-10000;
	uint32_t u32=429496729;//0xFFFFFFFF
	int32_t  s32=-100000000;
    float    f=-1000.55;
	int8_t   s3[3]={10,-20,100};
	float    f3[3]={10.555,-20.001,100.11111};
	xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_DATA_TEST(SecondChannel, SecondDevice, &u8, &s8, &u16, &s16, &u32, &s32, &f, s3, f3);
}
#endif
