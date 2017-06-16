
/** @file subsystems/mission/mission_manage.c
 *  @brief mission messages parser for mission interface,and saving the mission information
 */

#include "subsystems/mission/mission_manage.h"
//#include "uplink_gcs.h"
#include <string.h>
//#include "subsystems/navigation/common_nav.h"
#include "uplink_ac.h"
#include "generated/airframe.h"
//#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/autopilot.h"

//#define INT32_T(_payload) ((int32_t)(*((uint8_t*)_payload+2)|*((uint8_t*)_payload+2+1)<<8|((uint32_t)*((uint8_t*)_payload+2+2))<<16|((uint32_t)*((uint8_t*)_payload+2+3))<<24))

#ifndef MS_SP_NB
#define MS_SP_NB 80
#endif
union ms_wp wp_space[MS_SP_NB];  //mission waypoint storage space,size default 80
struct _mission mission;
uint8_t space_id;   //ms_wp pointer hop
//uint16_t flag_mission_idx;

void mission_init(void)
{
	//mission.insert_idx = 0;  //mission insert sequence
	mission.current_idx = 1;  //the idx mission run from 1, 0 reserve
	mission.add_idx=0;
	mission.element_time = 0.;
	for(uint8_t i=0; i<MISSION_ELEMENT_NB; i++) //element_exit set false
	{
		mission.elements[i].element_exist=FALSE;
	}
	space_id=0;                //space relative length
	memset(wp_space, 0, MS_SP_NB*sizeof(union ms_wp)); //clear space
}

bool_t mission_clear_all(void)
{
	if(autopilot_in_flight) return FALSE;  //in flight ,don't allow clear mission
	mission_init();
	return TRUE;
}

// Get element,use in mission run
struct _mission_element *get_mission(void)
{
	for(uint8_t i=mission.current_idx; i<MISSION_ELEMENT_NB ; i++)
	{
		if(mission.elements[i].element_exist)  mission.add_idx=i;   //check if mission exist
		else  break;
	}
	if ( mission.current_idx==(mission.add_idx+1) )   return NULL;  //mission is run over
	else  return &(mission.elements[mission.current_idx]);          //return mission elements pointer
}

uint8_t get_mission_executable()   //check before take_off in nav_gcs_mode
{
	struct _mission_element *ele = NULL;
	if ((ele = get_mission())!=NULL)
	{
		if(ele->status==standby)  return TRUE;
	}
	return FALSE;
}

// Report mission info function
void mission_status_report(void)
{
#if PERIODIC_TELEMETRY
	static uint8_t index=1;    //use in periodic send report
	if(mission.elements[index].element_exist)
	{
		uint8_t type=mission.elements[index].type;
		uint8_t status=mission.elements[index].status;
		int32_t *pt;
		uint8_t nb_wp,data_len;
		pt=&(mission.elements[index].element.mission_path.path_p->x);
		nb_wp=mission.elements[index].element.mission_path.nb_wp;
		data_len=mission.elements[index].element.mission_path.nb_rsland;
		data_len=(data_len+nb_wp)*3;   //waypoint have x/y/z 3data

		float buf[data_len];
		for(uint8_t i=0; i<data_len; i++)
		{
			buf[i]=POS_FLOAT_OF_BFP(*(pt+i));
		}
		xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
		DOWNLINK_SEND_MISSION(DefaultChannel, DefaultDevice,
													&index,
													&type,
													&status,
													&nb_wp,
													data_len,
													&(*buf) );
		index++;
	}

	else index=1;
#endif
}


void send_current_mission(void)
{
	uint16_t system_time=sys_time.nb_sec;
	uint8_t path_idx=mission.elements[mission.current_idx].element.mission_path.path_idx;
	uint8_t mission_id=mission.current_idx;
	uint8_t mission_status=mission.elements[mission.current_idx].status;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_GCS);   //to gcs
	DOWNLINK_SEND_CURRENT_MISSION_STATE(SecondChannel, SecondDevice,
																			&system_time,
																			&path_idx,
																			&mission_id,
																			&mission_status );
#if PERIODIC_TELEMETRY
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);    //to ubuntu
	DOWNLINK_SEND_MISSION_INFO(DefaultChannel, DefaultDevice,
														 &mission_id,
														 &path_idx,
														 &mission_status,
														 &mission.element_time );
#endif
}


/***********************************
 ***** Mission Parsing functions****
 ***********************************/

//ADD_MISSION parse
int8_t mission_add_parse(struct mission_info ms_info)
{
	int8_t error_code=0;
	uint8_t ms_id = ms_info.mission_id;
	if(ms_id > MISSION_ELEMENT_NB)  return error_code=1;  //error,mission_id out of desire

	uint8_t len_data=ms_info.wp_len;
	uint8_t nb_wp=(uint8_t)ms_info.nb_wp;
	if(!nb_wp) return error_code=2;     //error,waypoints length is wrong
	uint8_t nb_rsland=(uint8_t)ms_info.nb_backup_land;
	if( (nb_wp+nb_rsland)*2!=len_data )  return error_code=2;  //error,waypoints length is wrong

	if(space_id+len_data>MS_SP_NB)  return error_code =12;     //space is not enought

	//buf data is N/E(X/Y) coordinate,save in (wp_space+space_id)
	int32_t *buf=ms_info.waypoints;
	uint8_t wp_type=ms_info.wp_type;
	len_data /=2;
	if(wp_type==1) //LLA data,need convert
	{
		struct LlaCoor_f lla[len_data];
		for( uint8_t i=0; i<len_data; i++)
		{
			lla[i].lon= ( (double)(*(buf+2*i  )) )/572957795.13;
			lla[i].lat= ( (double)(*(buf+2*i+1)) )/572957795.13;
			lla[i].alt= 17.5;  //flight_height convert to alt,use in lla to ecef convert
		}
		// if there is no valid local coordinate, do not insert mission element
		for (uint8_t j=0; j<len_data; j++)
		{
			if (!mission_point_of_lla( &( (wp_space+space_id+j)->wp_f ), &lla[j] ))  return error_code=3;  //error,lla convert to relative fail
		}
	}
	else   //relate coordinate,only deal unit
	{
		for (uint8_t j=0; j<len_data; j++)  //save to ENU space,need exchange x/y(data is NE coordinate)
		{
			(wp_space+space_id+j)->wp_f.x=( (double)(*(buf+2*j+1)) )/1000.0;
			(wp_space+space_id+j)->wp_f.y=( (double)(*(buf+2*j  )) )/1000.0;
			(wp_space+space_id+j)->wp_f.z=1.5;  //flight_height convert to alt
		}
	}

	//convert float_enu to int_enu
	for (uint8_t k=0; k<len_data; k++)  //save to ENU space,need exchange x/y
	{
		ENU_BFP_OF_REAL((wp_space+space_id+k)->wp_i,(wp_space+space_id+k)->wp_f);
	}

	//process the mission information acording to mission_type
	return( mission_add_type_parse(ms_info,ms_id,len_data) );
}

int8_t mission_add_type_parse(struct mission_info ms_info,uint8_t ms_id,uint8_t len_data)
{
	uint8_t error_code=0;
	mission.elements[ms_id].type=(enum MissionType)ms_info.mission_type;
	switch(mission.elements[ms_id].type)
	{
	case ms_path:
	case ms_home:
		mission.elements[ms_id].element.mission_path.path_p=wp_space+space_id;
		mission.elements[ms_id].element.mission_path.nb_wp=ms_info.nb_wp;
		mission.elements[ms_id].element.mission_path.nb_rsland=ms_info.nb_backup_land;
		mission.elements[ms_id].element.mission_path.path_idx=0; //initial 0
		break;
	case ms_spray_normal:
		mission.elements[ms_id].element.mission_survey.survey_p=wp_space+space_id;
		mission.elements[ms_id].element.mission_survey.nb_survey=ms_info.nb_wp;
		mission.elements[ms_id].element.mission_survey.nb_rsland=ms_info.nb_backup_land;
		mission.elements[ms_id].element.mission_survey.survey_insert=FALSE;
		mission.elements[ms_id].element.mission_survey.survey_idx=0; //initial 0
		break;
	case ms_spray_insert:
		mission.elements[ms_id].element.mission_survey.survey_p=wp_space+space_id;
		mission.elements[ms_id].element.mission_survey.nb_survey=ms_info.nb_wp;
		mission.elements[ms_id].element.mission_survey.nb_rsland=ms_info.nb_backup_land;
		mission.elements[ms_id].element.mission_survey.survey_insert=TRUE;
		mission.elements[ms_id].element.mission_survey.survey_idx=0; //initial 0
		break;
	case ms_hover:
		mission.elements[ms_id].duration=ms_info.duration;
		break;

	default:
		return error_code=4;   //mission_type is out of expection
	}

	mission.elements[ms_id].status=(enum element_status)(ms_info.mission_status);
	mission.elements[ms_id].element_exist=TRUE;
	//flag_mission_idx=flag_mission_idx|(0x0001<<ms_id); //record the mission idex,replace element_exist
	space_id=space_id+len_data; //success,add length of space_id
	return error_code=0;
}


//UPDATE_MISSION parse
int8_t mission_update_parse(struct mission_info ms_info)
{
	int8_t error_code=0;
	uint8_t ms_id=ms_info.mission_id;
	if( !mission.elements[ms_id].element_exist)  return error_code=10;   //the mission is not exist,can't update

	uint8_t ms_type=(enum MissionType)ms_info.mission_type;
	if( mission.elements[ms_id].type !=ms_type )  return  error_code=11;  //error,can not update mission_type

	//if updata less(no change waypoints),just update other information
	if(  (ms_info.nb_wp==-1) && (ms_info.nb_backup_land==-1) )
	{
		mission.elements[ms_id].status=(enum element_status)ms_info.mission_status;
		if(mission.elements[ms_id].type==ms_hover)
			mission.elements[ms_id].duration=ms_info.duration;
		return error_code=0;
	}
	//else,creat new space to save waypoints,old mission.element[ms_id] will be replace
	else
	{
		return ( mission_add_parse(ms_info) );
	}
}

//delete motion let all missions status stop
int8_t mission_delete(uint8_t mission_id)
{
	int8_t error_code=0;
	uint8_t ms_id = mission_id;
	if(ms_id > MISSION_ELEMENT_NB)  return error_code=1;
	mission.elements[ms_id].status= stop; //stop
	return error_code=0;
}

/***!!! no finish get mission,use gcs operation*****/
int8_t mission_get(uint8_t mission_id, uint8_t wp_type)
{
	uint8_t error_code=0;
	uint8_t ms_id=mission_id;
	if(!mission.elements[ms_id].element_exist)  return error_code=10;  //mission is not exist
	return error_code=20;
}
