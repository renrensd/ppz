/*
 * Copyright (C) 2008-2014 The Paparazzi Team
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
 */

/**
 * @file modules/nav/nav_survey_poly_osam.c
 *     begin 1
 *  1. bool_t nav_survey_poly_osam_setup_towards_ms(struct _mission_element *el)
 *     call 2
 *  2. bool_t nav_survey_poly_osam_setup_ms(struct _mission_element *el, uint8_t Size, float Orientation, bool_t insert)
 *     continual 3
 *  3. bool_t nav_survey_poly_osam_run_ms(void)
 */

#include "modules/nav/nav_survey_poly_osam.h"

#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/nav_flight.h"  //get ac_config_info

#ifdef OPS_OPTION
#include "subsystems/ops/ops_msg_if.h"
#endif

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#ifndef POLY_OSAM_DEFAULT_SIZE
#define POLY_OSAM_DEFAULT_SIZE 5
#endif

#ifndef POLY_OSAM_DEFAULT_SWEEP
#define POLY_OSAM_DEFAULT_SWEEP 10
#endif

/// Default entry radius, if 0 default to half sweep
#ifndef POLY_OSAM_ENTRY_RADIUS
#define POLY_OSAM_ENTRY_RADIUS 0
#endif

/// if 0 never check for min radius
#ifndef POLY_OSAM_MIN_RADIUS
#define POLY_OSAM_MIN_RADIUS 1
#endif

/// if 0 default to half sweep
#ifndef POLY_OSAM_FIRST_SWEEP_DISTANCE
#define POLY_OSAM_FIRST_SWEEP_DISTANCE 0
#endif

/// maximum number of polygon corners
#ifndef POLY_OSAM_POLYGONSIZE
#define POLY_OSAM_POLYGONSIZE 10
#endif

#ifndef POLY_OSAM_USE_FULL_CIRCLE
#define POLY_OSAM_USE_FULL_CIRCLE TRUE
#endif

uint8_t Poly_Size = POLY_OSAM_DEFAULT_SIZE;
float Poly_Sweep = POLY_OSAM_DEFAULT_SWEEP;
bool_t use_full_circle = POLY_OSAM_USE_FULL_CIRCLE;

struct Point2D
{
	float x;
	float y;
};
struct Line
{
	float m;
	float b;
	float x;
};

static void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY);
static void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY);
static void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2);
static float EvaluateLineForX(float y, struct Line L);  //find the L's x,based on y

#define PolygonSize POLY_OSAM_POLYGONSIZE
#define MaxFloat   1000000000
#define MinFloat   -1000000000

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

/************** Polygon Survey **********************************************/

/** This routine will cover the enitre area of any Polygon defined in the flightplan or el which is a convex polygon.
 */

enum SurveyStatus { Init, Entry, Sweep, SweepCircle, End };
static enum SurveyStatus CSurveyStatus;
static struct Point2D SmallestCorner;
static struct Line Edges[PolygonSize];
static float EdgeMaxY[PolygonSize];
static float EdgeMinY[PolygonSize];
static float SurveyTheta;
static float dSweep;   //retative sweep wide
static float SurveyRadius;
static struct Point2D SurveyToWP;
static struct Point2D SurveyFromWP;
static struct Point2D SurveyCircle;
static uint8_t SurveyEntryWP;
static uint8_t SurveySize;
static float SurveyCircleQdr;   //working heading,unit=deg
static float MaxY;
uint16_t PolySurveySweepNum;    //count the number of sweep lines
uint16_t PolySurveySweepBackNum;
float EntryRadius;

bool_t nav_survey_poly_osam_setup_towards(uint8_t FirstWP, uint8_t Size, float Sweep_local, int SecondWP)
{
	float dx = waypoints[SecondWP].enu_f.x - waypoints[FirstWP].enu_f.x;
	float dy = waypoints[SecondWP].enu_f.y - waypoints[FirstWP].enu_f.y;
	if (dx == 0.0f)
	{
		dx = 0.000000001;
	}
	float ang = atan(dy / dx);
	//set ang between -M_PI_2 and +M_PI_2
	while( ang>M_PI_2 ) ang-=M_PI;
	while( ang<-M_PI_2 ) ang+=M_PI;

	//if values passed, use it.
	if (Size == 0)
	{
		Size = Poly_Size;
	}
	if (Sweep_local == 0)
	{
		Sweep_local = Poly_Sweep;
	}
	return nav_survey_poly_osam_setup(FirstWP, Size, Sweep_local, DegOfRad(ang));
}

bool_t nav_survey_poly_osam_setup(uint8_t EntryWP, uint8_t Size, float sw, float Orientation)
{
	SmallestCorner.x = 0;
	SmallestCorner.y = 0;
	int i = 0;
	float ys = 0;
	static struct Point2D EntryPoint;
	float LeftYInt;
	float RightYInt;
	float temp;
	float XIntercept1 = 0;
	float XIntercept2 = 0;
	float entry_distance;

	float PolySurveyEntryDistance = 0;//POLY_OSAM_FIRST_SWEEP_DISTANCE;
	float PolySurveyEntryRadius = 0;//POLY_OSAM_ENTRY_RADIUS;

	if (PolySurveyEntryDistance == 0)
	{
		entry_distance = sw / 2;
	}
	else
	{
		entry_distance = PolySurveyEntryDistance;
	}

	if (PolySurveyEntryRadius == 0)
	{
		EntryRadius = sw / 2;
	}
	else
	{
		EntryRadius = PolySurveyEntryRadius;
	}

	SurveyTheta = RadOfDeg(Orientation);
	PolySurveySweepNum = 0;
	PolySurveySweepBackNum = 0;

	SurveyEntryWP = EntryWP;
	SurveySize = Size;

	struct Point2D Corners[PolygonSize];

	CSurveyStatus = Init;

	if (Size == 0)
	{
		return TRUE;
	}

	//Don't initialize if Polygon is too big or if the orientation is not between 0 and +/-90
	if (Size <= PolygonSize && Orientation >= -90 && Orientation <= 90)
	{
		//Initialize Corners
		for (i = 0; i < Size; i++)
		{
			Corners[i].x = waypoints[i + EntryWP].enu_f.x;
			Corners[i].y = waypoints[i + EntryWP].enu_f.y;
		}

		//Rotate Corners so sweeps are parellel with x axis
		for (i = 0; i < Size; i++)
		{
			TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, 0, 0);
		}

		//Find min x and min y
		SmallestCorner.y = Corners[0].y;
		SmallestCorner.x = Corners[0].x;
		for (i = 1; i < Size; i++)
		{
			if (Corners[i].y < SmallestCorner.y)
			{
				SmallestCorner.y = Corners[i].y;
			}

			if (Corners[i].x < SmallestCorner.x)
			{
				SmallestCorner.x = Corners[i].x;
			}
		}

		//Translate Corners all exist in quad #1
		for (i = 0; i < Size; i++)
		{
			TranslateAndRotateFromWorld(&Corners[i], 0, SmallestCorner.x, SmallestCorner.y);
		}

		//Rotate and Translate Entry Point
		EntryPoint.x = Corners[0].x;
		EntryPoint.y = Corners[0].y;

		//Find max y
		MaxY = Corners[0].y;
		for (i = 1; i < Size; i++)
		{
			if (Corners[i].y > MaxY)
			{
				MaxY = Corners[i].y;
			}
		}

		//Find polygon edges
		for (i = 0; i < Size; i++)
		{
			if (i == 0)
				if (Corners[Size - 1].x == Corners[i].x)   //Don't divide by zero!
				{
					Edges[i].m = MaxFloat;
				}
				else
				{
					Edges[i].m = ((Corners[Size - 1].y - Corners[i].y) / (Corners[Size - 1].x - Corners[i].x));
				}
			else if (Corners[i].x == Corners[i - 1].x)
			{
				Edges[i].m = MaxFloat;
			}
			else
			{
				Edges[i].m = ((Corners[i].y - Corners[i - 1].y) / (Corners[i].x - Corners[i - 1].x));
			}

			//Edges[i].m = MaxFloat;
			Edges[i].b = (Corners[i].y - (Corners[i].x * Edges[i].m));
		}

		//Find Min and Max y for each line
		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size - 1]);

		if (LeftYInt > RightYInt)
		{
			EdgeMaxY[0] = LeftYInt;
			EdgeMinY[0] = RightYInt;
		}
		else
		{
			EdgeMaxY[0] = RightYInt;
			EdgeMinY[0] = LeftYInt;
		}

		for (i = 1; i < Size - 1; i++)
		{
			FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i + 1]);
			FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i - 1]);

			if (LeftYInt > RightYInt)
			{
				EdgeMaxY[i] = LeftYInt;
				EdgeMinY[i] = RightYInt;
			}
			else
			{
				EdgeMaxY[i] = RightYInt;
				EdgeMinY[i] = LeftYInt;
			}
		}

		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size - 1], Edges[0]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size - 1], Edges[Size - 2]);

		if (LeftYInt > RightYInt)
		{
			EdgeMaxY[Size - 1] = LeftYInt;
			EdgeMinY[Size - 1] = RightYInt;
		}
		else
		{
			EdgeMaxY[Size - 1] = RightYInt;
			EdgeMinY[Size - 1] = LeftYInt;
		}

		//Find amount to increment by every sweep
		if (EntryPoint.y >= MaxY / 2)
		{
			entry_distance = -entry_distance;
			EntryRadius = -EntryRadius;
			dSweep = -sw;
		}
		else
		{
			EntryRadius = EntryRadius;
			dSweep = sw;
		}

		//CircleQdr tells the plane when to exit the circle
		if (dSweep >= 0)
		{
			SurveyCircleQdr = -DegOfRad(SurveyTheta);
		}
		else
		{
			SurveyCircleQdr = 180 - DegOfRad(SurveyTheta);
		}

		//Find y value of the first sweep
		ys = EntryPoint.y + entry_distance;

		//Find the edges which intercet the sweep line first
		for (i = 0; i < SurveySize; i++)
		{
			if (EdgeMinY[i] <= ys && EdgeMaxY[i] > ys)
			{
				XIntercept2 = XIntercept1;
				XIntercept1 = EvaluateLineForX(ys, Edges[i]);
			}
		}

		//Find point to come from and point to go to
		if (fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1))
		{
			SurveyToWP.x = XIntercept1;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept2;
			SurveyFromWP.y = ys;
		}
		else
		{
			SurveyToWP.x = XIntercept2;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept1;
			SurveyFromWP.y = ys;
		}

		//Find the direction to circle
		if (ys > 0 && SurveyToWP.x > SurveyFromWP.x)
		{
			SurveyRadius = EntryRadius;
		}
		else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x)
		{
			SurveyRadius = EntryRadius;
		}
		else
		{
			SurveyRadius = -EntryRadius;
		}

		//Find the entry circle
		SurveyCircle.x = SurveyFromWP.x;
		SurveyCircle.y = EntryPoint.y + entry_distance - EntryRadius;
		//Go into entry circle state
		CSurveyStatus = Entry;
		LINE_STOP_FUNCTION;
	}

	return FALSE;
}

/*return TRUE is running,FALSE is finished(success,or error)*/
bool_t nav_survey_poly_osam_run(void)
{
	//uint8_t state;    //0:finished,1:running(success or error)
	struct Point2D C;
	struct Point2D ToP;
	struct Point2D FromP;
	float ys;
	static struct Point2D LastPoint;
	int i;
	bool_t LastHalfSweep;
	static bool_t HalfSweep = FALSE;
	float XIntercept1 = 0;
	float XIntercept2 = 0;
	float DInt1 = 0;
	float DInt2 = 0;
	float temp;
	float min_radius = POLY_OSAM_MIN_RADIUS;

	NavVerticalAutoThrottleMode(0); /* No pitch */
	NavVerticalAltitudeMode(1.0, 0.);  //fix working height

	switch (CSurveyStatus)
	{
	case Entry:
#if 0
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);
		//convert to int32
		struct EnuCoor_i CP_i0;
		struct EnuCoor_f CP_f0;
		VECT3_ASSIGN(CP_f0,C.x,C.y,0.0);
		ENU_BFP_OF_REAL(CP_i0, CP_f0);
		//follow the circle
		nav_circle(&CP_i0, POS_BFP_OF_REAL(SurveyRadius));

		if ( NavQdrCloseTo(SurveyCircleQdr) && NavCircleCountNoRewind() > 0.1
				 && stateGetPositionEnu_f->z > 1.0  )
		{
			//working height,need get from globle var!!!!
			CSurveyStatus = Sweep;
			nav_init_stage();
			//LINE_START_FUNCTION;
		}

		//replace with stay entry waypoint,and heading toward
#else
		{
			struct EnuCoor_f entry_wp_f;// toward_wp_f;
			struct EnuCoor_i entry_wp_i;
			struct Point2D entry_xy;
			entry_wp_f.x=SurveyFromWP.x;
			entry_wp_f.y=SurveyFromWP.y;
			entry_wp_f.z=1.0;//ac_config_info.spray_height;
			entry_xy.x = SurveyFromWP.x;
			entry_xy.y = SurveyFromWP.y;
			//toward_wp_f.x=SurveyToWP.x;
			//toward_wp_f.y=SurveyToWP.y;
			//toward_wp_f.z=ac_config_info.spray_height;

			//Rotate and Translate Line points into real world
			RotateAndTranslateToWorld(&entry_xy, 0, SmallestCorner.x, SmallestCorner.y);
			RotateAndTranslateToWorld(&entry_xy, SurveyTheta, 0, 0);
			//RotateAndTranslateToWorld(&toward_wp_f, 0, SmallestCorner.x, SmallestCorner.y);
			//RotateAndTranslateToWorld(&toward_wp_f, SurveyTheta, 0, 0);
			ENU_BFP_OF_REAL(entry_wp_i, entry_wp_f);
			//ENU_BFP_OF_REAL(toward_wp_i, toward_wp_f);
			NavGotoWaypoint_wp(entry_wp_i);
			//if arrived ,set heading
			if ( nav_approaching_from(&entry_wp_i, NULL, 0) )
			{
				//static bool_t set_heading=FALSE;
				//if(!set_heading)
				//set heading
				float heading_e=M_PI_2-SurveyTheta;
				if( (SurveyToWP.x-SurveyFromWP.x)<0 )
					heading_e=heading_e+M_PI;
				nav_set_heading_rad(heading_e);
				if( nav_check_heading() && (stateGetHorizontalSpeedNorm_f())<0.5)
				{
					CSurveyStatus = Sweep;  //goto next step
					nav_init_stage();
#ifdef OPS_OPTION
					ops_start_spraying();  //open spray
#endif
				}
			}
		}
#endif
		break;

	case Sweep:
		LastHalfSweep = HalfSweep;
		ToP.x = SurveyToWP.x;
		ToP.y = SurveyToWP.y;
		FromP.x = SurveyFromWP.x;
		FromP.y = SurveyFromWP.y;

		//Rotate and Translate de plane position to local world
		//C save current position
		C.x = stateGetPositionEnu_f()->x;
		C.y = stateGetPositionEnu_f()->y;
		TranslateAndRotateFromWorld(&C, SurveyTheta, 0, 0);
		TranslateAndRotateFromWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);

		//Rotate and Translate Line points into real world
		RotateAndTranslateToWorld(&ToP, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&ToP, SurveyTheta, 0, 0);

		RotateAndTranslateToWorld(&FromP, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&FromP, SurveyTheta, 0, 0);

		//follow the line
		struct EnuCoor_i start_P_i ,end_P_i;
		struct EnuCoor_f start_P_f ,end_P_f;
		VECT3_ASSIGN(start_P_f,FromP.x,FromP.y,0.0);
		VECT3_ASSIGN(end_P_f  ,ToP.x,  ToP.y,  0.0);
		ENU_BFP_OF_REAL(start_P_i,start_P_f);
		ENU_BFP_OF_REAL(end_P_i,  end_P_f);
		nav_route(&start_P_i, &end_P_i);

		if (nav_approaching_from(&end_P_i, &start_P_i, 0))
		{
			LastPoint.x = SurveyToWP.x;
			LastPoint.y = SurveyToWP.y;

			if (LastPoint.y + dSweep >= MaxY || LastPoint.y + dSweep <= 0)
			{
				//Your out of the Polygon ,finish and exit
#ifdef OPS_OPTION
				ops_stop_spraying();
				CSurveyStatus = End;
#endif
				return FALSE;
			}

			else  // Normal sweep
			{
				//Find y value of the first sweep
				HalfSweep = FALSE;
				ys = LastPoint.y + dSweep;
			}

			//Find the edges which intercet the sweep line first
			for (i = 0; i < SurveySize; i++)
			{
				if (EdgeMinY[i] < ys && EdgeMaxY[i] >= ys)
				{
					XIntercept2 = XIntercept1;
					XIntercept1 = EvaluateLineForX(ys, Edges[i]);
				}
			}

			//Find point to come from and point to go to
			DInt1 = XIntercept1 -  LastPoint.x;
			DInt2 = XIntercept2 - LastPoint.x;

			if (DInt1 * DInt2 >= 0)
			{
				if (fabs(DInt2) <= fabs(DInt1))
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else
				{
					SurveyToWP.x = XIntercept2;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept1;
					SurveyFromWP.y = ys;
				}
			}
			else
			{
				if ((SurveyToWP.x - SurveyFromWP.x) > 0 && DInt2 > 0)
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else if ((SurveyToWP.x - SurveyFromWP.x) < 0 && DInt2 < 0)
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else
				{
					SurveyToWP.x = XIntercept2;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept1;
					SurveyFromWP.y = ys;
				}
			}

			//Find the radius to circle
			if (!HalfSweep || use_full_circle)
			{
				temp = dSweep / 2;
			}
			else
			{
				temp = dSweep / 4;
			}
			//if less than min radius,limit it to be min_radius
			if (fabs(temp) < min_radius)
			{
				if (temp < 0)
				{
					temp = -min_radius;
				}
				else
				{
					temp = min_radius;
				}
			}

			//Find the direction to circle
			if (ys > 0 && SurveyToWP.x > SurveyFromWP.x)
			{
				SurveyRadius = temp;
			}
			else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x)
			{
				SurveyRadius = temp;
			}
			else
			{
				SurveyRadius = -temp;
			}

			//find x position to circle
			if (fabs(LastPoint.x - SurveyToWP.x) > fabs(SurveyFromWP.x - SurveyToWP.x))
			{
				SurveyCircle.x = LastPoint.x;
			}
			else
			{
				SurveyCircle.x = SurveyFromWP.x;
			}

			//y position to circle
			SurveyCircle.y = ys - temp;
			//xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
			//DOWNLINK_SEND_FLOW_DEG(DefaultChannel, DefaultDevice, &SurveyRadius, &dSweep,&SurveyCircle.y,&min_radius);
			//Go into circle state

			CSurveyStatus = SweepCircle;
			nav_init_stage();
#ifdef OPS_OPTION
			ops_stop_spraying();  //stop spray
#endif
			//LINE_STOP_FUNCTION;
			PolySurveySweepNum++;
			//el->element.mission_survey.survey_idx++;
		}
		break;

	case SweepCircle:
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);
		//convert to int32
		struct EnuCoor_i CP_i1;
		struct EnuCoor_f CP_f1;
		VECT3_ASSIGN(CP_f1,C.x,C.y,0.0);
		ENU_BFP_OF_REAL(CP_i1, CP_f1);
		//follow the circle
		nav_circle(&CP_i1, POS_BFP_OF_REAL(SurveyRadius));
		/*
			  float heading_s=M_PI_2-SurveyTheta;
			  if( (SurveyToWP.x - SurveyFromWP.x) < 0 )
			  {
			  	heading_s=heading_s+M_PI;
			  }
			  int32_t heading_i =ANGLE_BFP_OF_REAL(heading_s);
			  int32_t heading_des =nav_heading;

			  INT32_COURSE_NORMALIZE(heading_i);
			  INT32_COURSE_NORMALIZE(heading_des);
			  if( abs(heading_i-heading_des)<ANGLE_BFP_OF_REAL(0.06) && NavCircleCount()>0)
			  {
			  	 nav_set_heading_rad(heading_s);
		        CSurveyStatus = Sweep;
		        nav_init_stage();
				#ifdef OPS_OPTION
		        ops_start_spraying();  //open spray
		       #endif
			  }
		*/
		if (NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0)
		{
			//set heading
			float heading_s=M_PI_2-SurveyTheta;
			if( (SurveyToWP.x-SurveyFromWP.x)<0 )
				heading_s=heading_s+M_PI;
			nav_set_heading_rad(heading_s);
			CSurveyStatus = Sweep;
			nav_init_stage();
#ifdef OPS_OPTION
			ops_start_spraying();  //open spray
#endif
			//LINE_START_FUNCTION;
			//el->element.mission_survey.survey_idx++;
		}
		break;
	case Init:
		return FALSE;
	default:
		return FALSE;
	}
	return TRUE;
}


/********************************/
/***function for mission,var el**/
/********************************/
#ifdef USE_MISSION
uint8_t survey_run_ms(struct _mission_element *el)
{
	uint8_t state_sur;   //0:running,1:success,2:fail
	static bool_t flag_set_up=FALSE;
	if(!flag_set_up)
	{
		nav_survey_poly_osam_setup_towards_ms(el);
		flag_set_up=TRUE;
		return state_sur=0;
	}
	state_sur = nav_survey_poly_osam_run_ms(el);
	if(state_sur)
	{
		flag_set_up=FALSE;//reset flag
	}
	return state_sur;
}

bool_t nav_survey_poly_osam_setup_towards_ms(struct _mission_element *el)
{
	bool_t n_insert=0;
	if(el->element.mission_survey.survey_insert)  n_insert=1;

	uint8_t size=el->element.mission_survey.nb_survey - n_insert;
	struct EnuCoor_f wp_from,wp_to;
	ENU_FLOAT_OF_BFP(wp_from, *(el->element.mission_survey.survey_p+n_insert));
	ENU_FLOAT_OF_BFP(wp_to, *(el->element.mission_survey.survey_p+n_insert+1));
	float dx = wp_to.x-wp_from.x;
	float dy = wp_to.y-wp_from.y;
	if (dx == 0.0f)
	{
		dx = 0.000000001;
	}
	float ang = atan(dy / dx);
	//set ang between -M_PI_2 and +M_PI_2
	while( ang>M_PI_2 ) ang-=M_PI;
	while( ang<-M_PI_2 ) ang+=M_PI;

	//if values passed, use it.
	//if (Size == 0) {Size = Poly_Size;}
	//if (Sweep == 0) {Sweep = Poly_Sweep;}
	return nav_survey_poly_osam_setup_ms(el, size, DegOfRad(ang), n_insert);
}


bool_t nav_survey_poly_osam_setup_ms(struct _mission_element *el, uint8_t Size, float Orientation, bool_t insert)
{
	SmallestCorner.x = 0.;  //define the relative local point
	SmallestCorner.y = 0.;
	int i = 0;
	float ys = 0.;
	static struct Point2D EntryPoint;
	float LeftYInt;
	float RightYInt;
	float temp;
	float XIntercept1 = 0.;
	float XIntercept2 = 0.;
	float entry_distance;
	float sw=ac_config_info.spray_wide;   //set working wide
	/*
	float PolySurveyEntryDistance = POLY_OSAM_FIRST_SWEEP_DISTANCE;
	float PolySurveyEntryRadius = POLY_OSAM_ENTRY_RADIUS;

	if (PolySurveyEntryDistance == 0) {
	  entry_distance = sw / 2;
	} else {
	  entry_distance = PolySurveyEntryDistance;
	}

	if (PolySurveyEntryRadius == 0) {
	  EntryRadius = sw / 2;
	} else {
	  EntryRadius = PolySurveyEntryRadius;
	}*/
	entry_distance = sw / 2;  //descript the distance between border and first line
	EntryRadius = sw / 2;     //descript entry circle radius

	SurveyTheta = RadOfDeg(Orientation);
	PolySurveySweepNum = 0;
	PolySurveySweepBackNum = 0;
	SurveySize = Size;

	struct Point2D Corners[PolygonSize];  //default 10

	CSurveyStatus = Init;  //if below is wrong, CSurveyStatus = Init


	//Don't initialize if Polygon is too big or if the orientation is not between 0 and +/-90
	if (Size <= PolygonSize && Orientation >= -90 && Orientation <= 90)
	{
		//Initialize Corners
		struct EnuCoor_f wp_temp;
		for (i = 0; i < Size; i++)
		{
			ENU_FLOAT_OF_BFP(wp_temp, *(el->element.mission_survey.survey_p+insert+i));
			Corners[i].x = wp_temp.x;
			Corners[i].y = wp_temp.y;
		}

		//Rotate Corners so sweeps are parellel with x axis
		for (i = 0; i < Size; i++)
		{
			TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, 0, 0);
		}

		//Find min x and min y
		SmallestCorner.y = Corners[0].y;
		SmallestCorner.x = Corners[0].x;
		for (i = 1; i < Size; i++)
		{
			if (Corners[i].y < SmallestCorner.y)
			{
				SmallestCorner.y = Corners[i].y;
			}

			if (Corners[i].x < SmallestCorner.x)
			{
				SmallestCorner.x = Corners[i].x;
			}
		}

		//Translate Corners all exist in quad #1
		for (i = 0; i < Size; i++)
		{
			TranslateAndRotateFromWorld(&Corners[i], 0, SmallestCorner.x, SmallestCorner.y);
		}

		//Rotate and Translate Entry Point,default first point is entry point
		EntryPoint.x = Corners[0].x;
		EntryPoint.y = Corners[0].y;

		//Find max y
		MaxY = Corners[0].y;
		for (i = 1; i < Size; i++)
		{
			if (Corners[i].y > MaxY)
			{
				MaxY = Corners[i].y;
			}
		}

		//Find polygon edges
		for (i = 0; i < Size; i++)
		{
			if (i == 0)
				if (Corners[Size - 1].x == Corners[i].x)   //Don't divide by zero!
				{
					Edges[i].m = MaxFloat;
				}
				else
				{
					Edges[i].m = ((Corners[Size - 1].y - Corners[i].y) / (Corners[Size - 1].x - Corners[i].x));
				}
			else if (Corners[i].x == Corners[i - 1].x)
			{
				Edges[i].m = MaxFloat;
			}
			else
			{
				Edges[i].m = ((Corners[i].y - Corners[i - 1].y) / (Corners[i].x - Corners[i - 1].x));
			}

			//Edges[i].m = MaxFloat;
			Edges[i].b = (Corners[i].y - (Corners[i].x * Edges[i].m));
		}

		//Find Min and Max y for each line
		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size - 1]);

		if (LeftYInt > RightYInt)
		{
			EdgeMaxY[0] = LeftYInt;
			EdgeMinY[0] = RightYInt;
		}
		else
		{
			EdgeMaxY[0] = RightYInt;
			EdgeMinY[0] = LeftYInt;
		}

		for (i = 1; i < Size - 1; i++)
		{
			FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i + 1]);
			FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i - 1]);

			if (LeftYInt > RightYInt)
			{
				EdgeMaxY[i] = LeftYInt;
				EdgeMinY[i] = RightYInt;
			}
			else
			{
				EdgeMaxY[i] = RightYInt;
				EdgeMinY[i] = LeftYInt;
			}
		}

		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size - 1], Edges[0]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size - 1], Edges[Size - 2]);

		if (LeftYInt > RightYInt)
		{
			EdgeMaxY[Size - 1] = LeftYInt;
			EdgeMinY[Size - 1] = RightYInt;
		}
		else
		{
			EdgeMaxY[Size - 1] = RightYInt;
			EdgeMinY[Size - 1] = LeftYInt;
		}

		//Find amount to increment by every sweep
		if (EntryPoint.y >= MaxY / 2)
		{
			entry_distance = -entry_distance;
			EntryRadius = -EntryRadius;
			dSweep = -sw;
		}
		else
		{
			EntryRadius = EntryRadius;
			dSweep = sw;
		}

		//CircleQdr tells the plane when to exit the circle
		if (dSweep >= 0)
		{
			SurveyCircleQdr = -DegOfRad(SurveyTheta);
		}
		else
		{
			SurveyCircleQdr = 180 - DegOfRad(SurveyTheta);
		}

		//Find y value of the first sweep
		ys = EntryPoint.y + entry_distance;

		//Find the edges which intercet the sweep line first
		for (i = 0; i < SurveySize; i++)
		{
			if (EdgeMinY[i] <= ys && EdgeMaxY[i] > ys)
			{
				XIntercept2 = XIntercept1;
				XIntercept1 = EvaluateLineForX(ys, Edges[i]);
			}
		}

		//Find point to come from and point to go to
		if (fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1))
		{
			SurveyToWP.x = XIntercept1;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept2;
			SurveyFromWP.y = ys;
		}
		else
		{
			SurveyToWP.x = XIntercept2;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept1;
			SurveyFromWP.y = ys;
		}

		//Find the direction to circle
		if (ys > 0 && SurveyToWP.x > SurveyFromWP.x)
		{
			SurveyRadius = EntryRadius;
		}
		else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x)
		{
			SurveyRadius = EntryRadius;
		}
		else
		{
			SurveyRadius = -EntryRadius;
		}
#if 0
		//Find the entry circle
		SurveyCircle.x = SurveyFromWP.x;
		SurveyCircle.y = EntryPoint.y + entry_distance - EntryRadius;
		//Go into entry circle state
#else
		//Find the entry point,use SurveyFromWP
#endif
		CSurveyStatus = Entry;
		//LINE_STOP_FUNCTION;
	}

	return FALSE;   //indicate convert is finished
}


uint8_t nav_survey_poly_osam_run_ms(struct _mission_element *el)
{
	uint8_t state_poly;   //0:running,1:success,2:fail
	struct Point2D C;
	struct Point2D ToP;
	struct Point2D FromP;
	float ys;
	static struct Point2D LastPoint;
	int i;
	bool_t LastHalfSweep;
	static bool_t HalfSweep = FALSE;
	float XIntercept1 = 0;
	float XIntercept2 = 0;
	float DInt1 = 0;
	float DInt2 = 0;
	float temp;
	float min_radius = POLY_OSAM_MIN_RADIUS;

	NavVerticalAutoThrottleMode(0); /* No pitch */
	NavVerticalAltitudeMode(ac_config_info.spray_height, 0.);  //working height

	switch (CSurveyStatus)
	{
	case Entry:
#if 0
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);
		//convert to int32
		struct EnuCoor_i CP_i0;
		struct EnuCoor_f CP_f0;
		VECT3_ASSIGN(CP_f0,C.x,C.y,0.0);
		ENU_BFP_OF_REAL(CP_i0, CP_f0);
		//follow the circle
		nav_circle(&CP_i0, POS_BFP_OF_REAL(SurveyRadius));

		if ( NavQdrCloseTo(SurveyCircleQdr) && NavCircleCountNoRewind() > 0.1
				 && stateGetPositionEnu_f->z > (ac_config_info.spray_height-0.2)  )
		{
			//working height,need get from globle var!!!!
			CSurveyStatus = Sweep;
			nav_init_stage();
			//LINE_START_FUNCTION;
		}

		//replace with stay entry waypoint,and heading toward
#else
		{
			struct EnuCoor_f entry_wp_f;// toward_wp_f;
			struct EnuCoor_i entry_wp_i;
			struct Point2D entry_xy;
			entry_wp_f.x=SurveyFromWP.x;
			entry_wp_f.y=SurveyFromWP.y;
			entry_wp_f.z=ac_config_info.spray_height;
			entry_xy.x = SurveyFromWP.x;
			entry_xy.y = SurveyFromWP.y;
			//toward_wp_f.x=SurveyToWP.x;
			//toward_wp_f.y=SurveyToWP.y;
			//toward_wp_f.z=ac_config_info.spray_height;

			//Rotate and Translate Line points into real world
			RotateAndTranslateToWorld(&entry_xy, 0, SmallestCorner.x, SmallestCorner.y);
			RotateAndTranslateToWorld(&entry_xy, SurveyTheta, 0, 0);
			//RotateAndTranslateToWorld(&toward_wp_f, 0, SmallestCorner.x, SmallestCorner.y);
			//RotateAndTranslateToWorld(&toward_wp_f, SurveyTheta, 0, 0);
			ENU_BFP_OF_REAL(entry_wp_i, entry_wp_f);
			//ENU_BFP_OF_REAL(toward_wp_i, toward_wp_f);
			NavGotoWaypoint_wp(entry_wp_i);
			//if arrived ,set heading
			if ( nav_approaching_from(&entry_wp_i, NULL, 0) )
			{
				//set heading
				float heading_e=M_PI_2-SurveyTheta;
				if( (SurveyToWP.x-SurveyFromWP.x)<0 )
					heading_e=heading_e+M_PI;
				nav_set_heading_rad(heading_e);

				if( nav_check_heading() && (stateGetHorizontalSpeedNorm_f())<0.5 )
				{
					CSurveyStatus = Sweep;  //goto next step
					nav_init_stage();
					el->element.mission_survey.survey_idx++;
#ifdef OPS_OPTION
					ops_start_spraying();  //open spray
#endif
				}

			}
		}
#endif
		break;

	case Sweep:
		LastHalfSweep = HalfSweep;
		ToP.x = SurveyToWP.x;
		ToP.y = SurveyToWP.y;
		FromP.x = SurveyFromWP.x;
		FromP.y = SurveyFromWP.y;

		//Rotate and Translate de plane position to local world
		//C save current position
		C.x = stateGetPositionEnu_f()->x;
		C.y = stateGetPositionEnu_f()->y;
		TranslateAndRotateFromWorld(&C, SurveyTheta, 0, 0);
		TranslateAndRotateFromWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);

		//Rotate and Translate Line points into real world
		RotateAndTranslateToWorld(&ToP, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&ToP, SurveyTheta, 0, 0);

		RotateAndTranslateToWorld(&FromP, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&FromP, SurveyTheta, 0, 0);

		//follow the line
		struct EnuCoor_i start_P_i ,end_P_i;
		struct EnuCoor_f start_P_f ,end_P_f;
		VECT3_ASSIGN(start_P_f,FromP.x,FromP.y,0.0);
		VECT3_ASSIGN(end_P_f  ,ToP.x,  ToP.y,  0.0);
		ENU_BFP_OF_REAL(start_P_i,start_P_f);
		ENU_BFP_OF_REAL(end_P_i,  end_P_f);
		nav_route(&start_P_i, &end_P_i);

		if (nav_approaching_from(&end_P_i, &start_P_i, 0))
		{
			LastPoint.x = SurveyToWP.x;
			LastPoint.y = SurveyToWP.y;

			if (LastPoint.y + dSweep >= MaxY || LastPoint.y + dSweep <= 0)
			{
				//Your out of the Polygon ,finish and exit
				CSurveyStatus = End;
#ifdef OPS_OPTION
				ops_stop_spraying();  //stop spray
#endif
				return state_poly=1;
			}

			else  // Normal sweep
			{
				//Find y value of the first sweep
				HalfSweep = FALSE;
				ys = LastPoint.y + dSweep;
			}

			//Find the edges which intercet the sweep line first
			for (i = 0; i < SurveySize; i++)
			{
				if (EdgeMinY[i] < ys && EdgeMaxY[i] >= ys)
				{
					XIntercept2 = XIntercept1;
					XIntercept1 = EvaluateLineForX(ys, Edges[i]);
				}
			}

			//Find point to come from and point to go to
			DInt1 = XIntercept1 -  LastPoint.x;
			DInt2 = XIntercept2 - LastPoint.x;

			if (DInt1 * DInt2 >= 0)
			{
				if (fabs(DInt2) <= fabs(DInt1))
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else
				{
					SurveyToWP.x = XIntercept2;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept1;
					SurveyFromWP.y = ys;
				}
			}
			else
			{
				if ((SurveyToWP.x - SurveyFromWP.x) > 0 && DInt2 > 0)
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else if ((SurveyToWP.x - SurveyFromWP.x) < 0 && DInt2 < 0)
				{
					SurveyToWP.x = XIntercept1;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept2;
					SurveyFromWP.y = ys;
				}
				else
				{
					SurveyToWP.x = XIntercept2;
					SurveyToWP.y = ys;

					SurveyFromWP.x = XIntercept1;
					SurveyFromWP.y = ys;
				}
			}

			//Find the radius to circle
			if (!HalfSweep || use_full_circle)
			{
				temp = dSweep / 2;
			}
			else
			{
				temp = dSweep / 4;
			}
			//if less than min radius,limit it to be min_radius
			if (fabs(temp) < min_radius)
			{
				if (temp < 0)
				{
					temp = -min_radius;
				}
				else
				{
					temp = min_radius;
				}
			}

			//Find the direction to circle
			if (ys > 0 && SurveyToWP.x > SurveyFromWP.x)
			{
				SurveyRadius = temp;
			}
			else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x)
			{
				SurveyRadius = temp;
			}
			else
			{
				SurveyRadius = -temp;
			}

			//find x position to circle
			if (fabs(LastPoint.x - SurveyToWP.x) > fabs(SurveyFromWP.x - SurveyToWP.x))
			{
				SurveyCircle.x = LastPoint.x;
			}
			else
			{
				SurveyCircle.x = SurveyFromWP.x;
			}

			//y position to circle
			SurveyCircle.y = ys - temp;
			//xbee_tx_header(XBEE_NACK,XBEE_ADDR_PC);
			//DOWNLINK_SEND_FLOW_DEG(DefaultChannel, DefaultDevice, &SurveyRadius, &dSweep,&SurveyCircle.y,&min_radius);
			//Go into circle state

			CSurveyStatus = SweepCircle;
			nav_init_stage();
#ifdef OPS_OPTION
			ops_stop_spraying();  //stop spray
#endif
			//LINE_STOP_FUNCTION;
			PolySurveySweepNum++;
			el->element.mission_survey.survey_idx++;
		}
		break;

	case SweepCircle:
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
		RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);
		//convert to int32
		struct EnuCoor_i CP_i1;
		struct EnuCoor_f CP_f1;
		VECT3_ASSIGN(CP_f1,C.x,C.y,0.0);
		ENU_BFP_OF_REAL(CP_i1, CP_f1);
		//follow the circle
		nav_circle(&CP_i1, POS_BFP_OF_REAL(SurveyRadius));

		if (NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0)
		{
			//set heading
			float heading_s=M_PI_2-SurveyTheta;
			if( (SurveyToWP.x-SurveyFromWP.x)<0 )
				heading_s=heading_s+M_PI;
			nav_set_heading_rad(heading_s);
			CSurveyStatus = Sweep;
			nav_init_stage();
#ifdef OPS_OPTION
			ops_start_spraying();  //open spray
#endif
			//LINE_START_FUNCTION;
			el->element.mission_survey.survey_idx++;
		}
		break;
	case Init:
		return state_poly=2;
	default:
		return state_poly=2;
	}
	return state_poly=0;  //runing
}
#endif


/*
  Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
*/
void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
	float temp;

	p->x = p->x - transX;
	p->y = p->y - transY;

	temp = p->x;
	p->x = p->x * cosf(Zrot) + p->y * sinf(Zrot);
	p->y = -temp * sinf(Zrot) + p->y * cosf(Zrot);
}

/// Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
	float temp = p->x;

	p->x = p->x * cosf(Zrot) - p->y * sinf(Zrot);
	p->y = temp * sinf(Zrot) + p->y * cosf(Zrot);

	p->x = p->x + transX;
	p->y = p->y + transY;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
	*x = ((L2.b - L1.b) / (L1.m - L2.m));
	*y = L1.m * (*x) + L1.b;
}


float EvaluateLineForX(float y, struct Line L)
{
	return ((y - L.b) / L.m);
}
