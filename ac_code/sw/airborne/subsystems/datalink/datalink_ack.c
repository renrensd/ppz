
/** \file subsystems/datalink/datalink_ack.c
 *  \brief parse common status parameter,called by datalink.c
 *
 */
#include "subsystems/datalink/datalink_ack.h"
#include "uplink_ac.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/nav_flight.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/rc_nav/rc_nav_xbee.h"
#include "subsystems/mission/mission_manage.h"


void send_heart_beat_A2R_msg(void)
{   
	uint16_t system_time=sys_time.nb_sec;
	int8_t battery_remain=85;      //unit=percent, need updata from battery module
	int8_t pesticides_remain=90;   //unit=percent,need updata from spray working module
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
	DOWNLINK_SEND_HEART_BEAT_AC_RC_STATE(DefaultChannel, DefaultDevice, &system_time, 
		                                 &last_response, &battery_remain, &pesticides_remain);
}


void DlSetConfig(uint8_t id, int8_t *pt_value ,uint8_t lenght)
{   
	switch(id)
	{
		case SPRAY_HEIGHT:
			ac_config_info.spray_height=((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case SPRAY_WIDE:
			ac_config_info.spray_wide=((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case SPRAY_CONCENTRATION:
			ac_config_info.concentration=(*pt_value);
			break;
		case SPRAY_SPEED:
			ac_config_info.spray_speed=((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
		    break;
		case MAX_FLIGHT_SPEED:
			ac_config_info.max_flight_speed=((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		case MAX_FLIGHT_HEIGHT:
			ac_config_info.max_flight_height=((int16_t)(*((uint8_t*)pt_value)|*((uint8_t*)pt_value+1)<<8))/100.0;
			break;
		default: break;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
	DOWNLINK_SEND_AC_CONFIG_INFO(DefaultChannel, DefaultDevice,
		                         &ac_config_info.spray_height,
		                         &ac_config_info.spray_wide,
		                         &ac_config_info.concentration,
		                         &ac_config_info.spray_speed,
		                         &ac_config_info.max_flight_speed,
		                         &ac_config_info.max_flight_height);
}

void send_aircraft_info_state(void)
{
/*  
    <field name="engine_type" type="uint8"/>
    <field name="max_voyage" type="uint16" unit="m"/>
    <field name="battery_capacity" type="int32" unit="mAh"/>
    <field name="pesticides_capacity" type="int16" unit="0.1L"/>
    <field name="spray_wide"   type="uint16" unit="cm"/>
    <field name="spray_height" type="uint16" unit="cm"/>
    <field name="max_flight_height"   type="uint16" unit="cm"/> 
    <field name="spray_concentration"   type="uint8" unit="ml/m2"/>
    <field name="max_flight_speed"   type="uint16" unit="cm/s"/>
    <field name="spray_flight_speed"   type="uint16" unit="cm/s"/>
    <field name="sn_and_sv"   type="char[30]"/>    
 */ 
    //all need relative to the information
	enum engine_type ac_engine_type=Electricity;
	uint16_t max_vayage=500;
	int32_t  battery_capacity=16000;   //get from electrical
	int16_t  pesticides_capacity=50;   //get from electrical
	uint16_t spray_wide =ac_config_info.spray_wide*100; 
	uint16_t spray_height =ac_config_info.spray_height*100;
	uint16_t max_flight_height=ac_config_info.max_flight_height*100;
	uint8_t  spray_concentration=ac_config_info.concentration;
	uint16_t max_flight_speed=ac_config_info.max_flight_speed*100;
	uint16_t spray_flight_speed=ac_config_info.spray_speed*100;
	char     sn_and_sv[30]="EF-A1-04";	
	
	xbee_tx_header(XBEE_ACK,XBEE_ADDR_GCS);
	DOWNLINK_SEND_AIRCRAFT_INFO_STATE(DefaultChannel, DefaultDevice, 
		                              &ac_engine_type, 
		                              &max_vayage, 
		                              &battery_capacity, 
		                              &pesticides_capacity, 
		                              &spray_wide, 
		                              &spray_height, 
		                              &max_flight_height, 
		                              &spray_concentration, 
		                              &max_flight_speed, 
		                              &spray_flight_speed, 
		                              &sn_and_sv[0]);
}

uint8_t DlSetCommand(uint8_t id, int8_t *pt_value ,uint8_t lenght)
{
	uint8_t response=0;
	enum set_command command_gcs=(enum set_command)(id);
	switch(command_gcs)
	{
		case delete_all_mission:
			response=!( mission_clear_all() );
			break;
		case request_ac_info:
			send_aircraft_info_state();
		    break;
		default:  break;
	}
	return response;
}

