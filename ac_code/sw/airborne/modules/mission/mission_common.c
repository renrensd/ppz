/*
 * Copyright (C) 2014 Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/mission/mission_common.c
 *  @brief messages parser for mission interface
 */

#include "modules/mission/mission_common.h"

#include <string.h>
#include "subsystems/navigation/common_nav.h"
//#include "generated/flight_plan.h"
#include "generated/airframe.h"
//#include "subsystems/datalink/datalink.h"
//#include "subsystems/datalink/downlink.h"


struct _mission mission;

#ifndef MS_SP_NB
#define MS_SP_NB 80
#endif
ms_wp wp_space[MS_SP_NB];  //storage space

uint8_t space_id;
uint16_t flag_mission_idx;

void mission_init(void)
{
  //mission.insert_idx = 0;  //mission insert sequence
  mission.current_idx = 1;  //the idx mission run from 1, 0 reserve for reland
  mission.element_time = 0.;
  space_id=0;
  flag_mission_idx=0;
  memset(space.wp_i,0,sizeof(space)); //cleat space
}


// Weak implementation of mission_element_convert (leave element unchanged)
//bool_t __attribute__((weak)) mission_element_convert(struct _mission_element *el __attribute__((unused))) { return TRUE; }


// Get element
struct _mission_element *mission_get(void)
{ 
  uint8_t i=0;
  uint16_t temp;
  do{
  	i++;
  	temp=flag_mission_idx>>i	
  }
  while(!flag_mission_idx%2);
  mission.insert_idx=i;    //record the max idx mission
  if (mission.current_idx == i) { //the mission space is empty 
    return NULL;
  }
  return &(mission.elements[mission.current_idx]);
}


// Report function
void mission_status_report(void)
{
  // build task list
  uint8_t task_list[MISSION_ELEMENT_NB];
  uint8_t i = mission.current_idx, j = 0;
  while (i != mission.insert_idx) {  //start from current_idx
    task_list[j++] = (uint8_t)mission.elements[i].type;
    i++;
  }
  if (j == 0) { task_list[j++] = 0; } // Dummy value if task list is empty
  //compute remaining time (or -1. if no time limit)
  float remaining_time = -1.;
  if (mission.elements[mission.current_idx].duration > 0.) {
    remaining_time = mission.elements[mission.current_idx].duration - mission.element_time;
  }

  // send status
  xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
  DOWNLINK_SEND_MISSION_STATUS(DefaultChannel, DefaultDevice, &remaining_time, j, task_list);
}


///////////////////////
// Parsing functions //
///////////////////////

int mission_parse_lla(void)
{  
  MissionType mission_type=(enum MissionType)(DL_FP_MISSION_misson_type(dl_buffer));
  switch(mission_type)  {  //need return result
     case MissionSurvey:
        if(!mission_parse_SURVEY_lla()) return FALSE;
	    break;
	 case MissionPath:
	 	if(!mission_parse_PATH_lla()) return FALSE;
		break;
	 case MissionHome:
	 	if(!mission_parse_HOME_lla()) return FALSE;
		break;
	 case MissionReland:
	 	if(!mission_parse_RELAND_lla()) return FALSE;
		break;
	 //case MissionSegment:
	 default: break; 		 	
  }
  return TRUE;
}

int mission_parse_PATH_lla(void)
{
  //if(DL_FP_MISSION_misson_type(dl_buffer)!=MissionPath) return FALSE; //type parse wrong
  
  uint8_t id=DL_FP_MISSION_misson_idx(dl_buffer);
  if(id>MISSION_ELEMENT_NB)  return FALSE;  //mission is out of mission stack
  
  uint8_t len=DL_FP_MISSION_wp_length(dl_buffer)/2;
  if(len!=DL_FP_MISSION_nb_path(dl_buffer))  return FALSE;     //path waypoint number check
  
  float *buf=DL_FP_MISSION_wp(dl_buffer);
  struct LlaCoor_f lla[len];
  for(uint8_t i=0;i<len;i++) {
     lla[i].lat=*(buf+2*i);
	 lla[i].lon=*(buf+2*i+1);
	 lla[i].alt=3.0;  //flight alt,modify later
  }
 
  for (uint8_t j=0; j<len; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&((space+space_id+i)->wp_f)), &lla[i]))  return FALSE;    
   }
  
  //convert float_enu to int_enu
  if (!mission_element_convert(mission.elements[id]))  return FALSE; 

  //something need to do while saving succuss
  mission.elements[id].type=MissionPath;
  mission.elements[id].element.mission_path.path_p=space+space_id;
  mission.elements[id].element.mission_path.nb=len;
  mission.elements[id].element.mission_path.path_idx=0; //initial 0  
  
  flag_mission_idx=flag_mission_idx|(0x0001<<id); //record the mission idex
  space_id=space_id+len; //add lenght of space
  
  return TRUE; //parse succuss
}

int mission_parse_SURVEY_lla(void)
{ 
  uint8_t nb_survey=0;
  uint8_t nb_path=DL_FP_MISSION_nb_path(dl_buffer);
  bool_t survey_break=FALSE;
  
  //if(DL_FP_MISSION_misson_type(dl_buffer)!=MissionSurvey) return FALSE;  //type parse wrong
  
  uint8_t id=DL_FP_MISSION_misson_idx(dl_buffer);
  if(id>MISSION_ELEMENT_NB)  return FALSE;  //mission is out of mission stack
  
  uint8_t len=DL_FP_MISSION_wp_length(dl_buffer)/2;
  uint8_t survey_info=DL_FP_MISSION_survey_info(dl_buffer);
  nb_survey= (survey_info&0x0F);
  if( (survey_info&0xF0)!=0)  {
  	 survey_break=TRUE;  //check survey mode(normal or break)
  	 if( len!=(nb_path+nb_survey+1) )  return FALSE;  //waypoint lenght check
  }
  else {
     survey_break=FALSE;
	 if( len!=(nb_path+nb_survey) )  return FALSE;  //waypoint lenght check
  }

  float *buf=DL_FP_MISSION_wp(dl_buffer);
  struct LlaCoor_f lla[len];
  for(uint8_t i=0;i<len;i++) {
     lla[i].lat=*(buf+2*i);
	 lla[i].lon=*(buf+2*i+1);
	 lla[i].alt=3.0;  //flight alt,modify later
  }

  for (uint8_t j=0; j<len; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&((space+space_id+i)->wp_f)), &lla[i]))  return FALSE;    
   }
  
  //convert float_enu to int_enu
  if (!mission_element_convert(mission.elements[id]))  return FALSE; 

  //something need to do while saving succuss
  mission.elements[id].type=MissionSurvey;
  mission.elements[id].element.mission_survey.path_p=space+space_id;
  mission.elements[id].element.mission_survey.survey_p=space+space_id+nb_path;
  mission.elements[id].element.mission_survey.path_nb=nb_path;
  mission.elements[id].element.mission_survey.survey_nb=nb_survey;
  mission.elements[id].element.mission_survey.survey_break=survey_break;
  mission.elements[id].element.mission_survey.path_idx=0; //initial 0 
  mission.elements[id].element.mission_survey.survey_idx=0; //initial 0 
  
  flag_mission_idx=flag_mission_idx|(0x0001<<id); //record the mission idex
  space_id=space_id+len; //add lenght of space
  
  return TRUE; //parse succuss

}

int mission_parse_HOME_lla(void)
{
  //if(DL_FP_MISSION_misson_type(dl_buffer)!=MissionHome) return FALSE; //type parse wrong
  
  uint8_t id=DL_FP_MISSION_misson_idx(dl_buffer);
  if(id>MISSION_ELEMENT_NB)  return FALSE;  //mission is out of mission stack
  
  uint8_t len=DL_FP_MISSION_wp_length(dl_buffer)/2;
  if(len!=DL_FP_MISSION_nb_path(dl_buffer))  return FALSE;     //path waypoint number check
  
  float *buf=DL_FP_MISSION_wp(dl_buffer);
  struct LlaCoor_f lla[len];
  for(uint8_t i=0;i<len;i++) {
     lla[i].lat=*(buf+2*i);
	 lla[i].lon=*(buf+2*i+1);
	 lla[i].alt=2.0;  //flight alt,modify later
  }
 
  for (uint8_t j=0; j<len; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&((space+space_id+i)->wp_f)), &lla[i]))  return FALSE;    
   }
  
  //convert float_enu to int_enu
  if (!mission_element_convert(mission.elements[id]))  return FALSE; 

  //something need to do while saving succuss,use path way save
  mission.elements[id].type=MissionHome;
  mission.elements[id].element.mission_path.path_p=space+space_id;
  mission.elements[id].element.mission_path.nb=len;
  mission.elements[id].element.mission_path.path_idx=0; //initial 0  
  
  flag_mission_idx=flag_mission_idx|(0x0001<<id); //record the mission idex
  space_id=space_id+len; //add lenght of space
  
  return TRUE; //parse succuss
}

int mission_parse_RELAND_lla(void)
{
  uint8_t id=DL_FP_MISSION_misson_idx(dl_buffer);
  if(id!=0)  return FALSE;  //reserve mission request idx=0
  
  uint8_t len=DL_FP_MISSION_wp_length(dl_buffer)/2;
  if(len!=DL_FP_MISSION_nb_path(dl_buffer))  return FALSE;     //waypoint number check
  
  float *buf=DL_FP_MISSION_wp(dl_buffer);
  struct LlaCoor_f lla[len];
  for(uint8_t i=0;i<len;i++) {
     lla[i].lat=*(buf+2*i);
	 lla[i].lon=*(buf+2*i+1);
	 lla[i].alt=3.0;  //flight alt,modify later
  }
 
  for (uint8_t j=0; j<len; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&((space+space_id+i)->wp_f)), &lla[i]))  return FALSE;    
   }
  
  //convert float_enu to int_enu
  if (!mission_element_convert(mission.elements[id]))  return FALSE; 

  //something need to do while saving succuss,use path way save
  mission.elements[id].type=MissionReland;
  mission.elements[id].element.mission_path.path_p=space+space_id;
  mission.elements[id].element.mission_path.nb=len;
  //mission.elements[id].element.mission_path.path_idx=0; //initial 0  
  
  flag_mission_idx=flag_mission_idx|(0x0001<<id); //record the mission idex
  space_id=space_id+len; //add lenght of space
  
  return TRUE; //parse succuss
}

int mission_parse(void)
{  
  MissionType mission_type=DL_FP_MISSION_misson_type(dl_buffer);
  switch(mission_type)  {  //need return result
     case MissionSurvey:
        //mission_parse_SURVEY();
	    break;
	 case MissionPath:
	 	//mission_parse_PATH();
		break;
	 //case MissionSegment:
	 default: break;		 	
  }
}

int mission_parse_GOTO_MISSION(void)
{
  if (DL_GOTO_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  uint8_t mission_id = DL_GOTO_MISSION_mission_id(dl_buffer);
  if (mission_id < MISSION_ELEMENT_NB) {
    mission.current_idx = mission_id;
  } else { return FALSE; }

  return TRUE;
}

int mission_parse_NEXT_MISSION(void)
{
  if (DL_NEXT_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  if (mission.current_idx == mission.insert_idx) { return FALSE; } // already at the last position

  // increment current index
  mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  return TRUE;
}

int mission_parse_END_MISSION(void)
{
  if (DL_END_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  // set current index to insert index (last position)
  mission.current_idx = mission.insert_idx;
  return TRUE;
}

