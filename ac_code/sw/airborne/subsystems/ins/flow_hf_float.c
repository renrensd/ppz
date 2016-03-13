/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ins/flow_hf_float.c
 *
 * Horizontal filter (x,y) to estimate position and velocity of flow data(body coor).
 *
 */

#include "subsystems/ins/flow_hf_float.h"
#include "subsystems/ins/hf_float.h"  //using struct HfilterFloat
//#include "math/pprz_algebra_float.h"
#include "state.h"
#include <stdlib.h>
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"


/** initial covariance diagonal */
#define INIT_FPXX 1.

//TODO: proper measurement noise
#ifndef FHFF_R_POS
#define FHFF_R_POS   2.
#endif

#ifndef FHFF_R_SPEED
#define FHFF_R_SPEED 6.
#endif

/*
#ifndef HFF_UPDATE_SPEED
#define HFF_UPDATE_SPEED TRUE
#endif
*/

struct FloatVect2 local_flow_pos;
/* flow measurement noise */
float Rflow_pos, Rflow_vel;

/* output filter states */
struct HfilterFloat flow_hff_state;

static void flow_hff_init_x(float init_x, float init_xdot);
static void flow_hff_init_y(float init_y, float init_ydot);

static void flow_hff_update_x(struct HfilterFloat *hff_work, float x_meas, float Rpos);
static void flow_hff_update_y(struct HfilterFloat *hff_work, float y_meas, float Rpos);

static void flow_hff_update_xdot(struct HfilterFloat *hff_work, float vel, float Rvel);
static void flow_hff_update_ydot(struct HfilterFloat *hff_work, float vel, float Rvel);


void flow_hff_init(float init_x, float init_xdot, float init_y, float init_ydot)
{
  //Rflow_pos = HFF_R_POS/100.0;  //convert to meters
  //Rflow_vel = HFF_R_SPEED/100.0;
  flow_hff_init_x(init_x, init_xdot);
  flow_hff_init_y(init_y, init_ydot);
  VECT2_COPY(local_flow_pos, *stateGetPositionNed_f());
  flow_hff_state.rollback=true;// init ok
}

static void flow_hff_init_x(float init_x, float init_xdot)
{
  flow_hff_state.x     = init_x;
  flow_hff_state.xdot  = init_xdot;
  int i, j;
  for (i = 0; i < 2; i++) {
    for (j = 0; j < 2; j++) {
      flow_hff_state.xP[i][j] = 0.;
    }
    flow_hff_state.xP[i][i] = INIT_FPXX;
  }
}

static void flow_hff_init_y(float init_y, float init_ydot)
{
  flow_hff_state.y     = init_y;
  flow_hff_state.ydot  = init_ydot;
  int i, j;
  for (i = 0; i < 2; i++) {
    for (j = 0; j < 2; j++) {
      flow_hff_state.yP[i][j] = 0.;
    }
    flow_hff_state.yP[i][i] = INIT_FPXX;
  }
}

void hff_update_flow(struct FloatVect2 *pos_flow, struct FloatVect2 *speed_flow ,unsigned char qual)
{
  //b2_hff_lost_counter = 0;
  if(!flow_hff_state.rollback) flow_hff_init(0.0, 0.0, 0.0, 0.0);  //in case not init
  Rflow_pos = (FHFF_R_POS + (255-qual)/80.0)/100.0;  //meas noise estimate
  Rflow_vel = (FHFF_R_SPEED + (255-qual)/40.0)/100.0;

  /* update filter state with measurement */
  flow_hff_update_x(&flow_hff_state, pos_flow->x, Rflow_pos);
  flow_hff_update_y(&flow_hff_state, pos_flow->y, Rflow_pos);
//#if HFF_UPDATE_SPEED
  flow_hff_update_xdot(&flow_hff_state, speed_flow->x, Rflow_vel);
  flow_hff_update_ydot(&flow_hff_state, speed_flow->y, Rflow_vel);
//#endif
  
}


void flow_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel)
{
  flow_hff_state.x = pos.x;
  flow_hff_state.y = pos.y;
  flow_hff_state.xdot = vel.x;
  flow_hff_state.ydot = vel.y;
}


/*
 *
 * Update position
 *
 *

 H = [1 0];
 R = 0.1;
 // state residual
 y = pos_measurement - H * Xm;
 // covariance residual
 S = H*Pm*H' + R;
 // kalman gain
 K = Pm*H'*inv(S);
 // update state
 Xp = Xm + K*y;
 // update covariance
 Pp = Pm - K*H*Pm;
*/
void flow_hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos)
{
  flow_hff_update_x(&flow_hff_state, pos.x, Rpos.x);
  flow_hff_update_y(&flow_hff_state, pos.y, Rpos.y);
}

static void flow_hff_update_x(struct HfilterFloat *hff_work, float x_meas, float Rpos)
{
  //b2_hff_x_meas = x_meas;

  const float y  = x_meas;  // - hff_work->x; x_meas is increment
  const float S  = hff_work->xP[0][0] + Rpos;
  const float K1 = hff_work->xP[0][0] * 1 / S;
  const float K2 = hff_work->xP[1][0] * 1 / S;

  hff_work->x     = hff_work->x     + K1 * y;
  hff_work->xdot  = hff_work->xdot  + K2 * y;

  const float P11 = (1. - K1) * hff_work->xP[0][0];
  const float P12 = (1. - K1) * hff_work->xP[0][1];
  const float P21 = -K2 * hff_work->xP[0][0] + hff_work->xP[1][0];
  const float P22 = -K2 * hff_work->xP[0][1] + hff_work->xP[1][1];

  hff_work->xP[0][0] = P11;
  hff_work->xP[0][1] = P12;
  hff_work->xP[1][0] = P21;
  hff_work->xP[1][1] = P22;
  //runOnceEvery(25, DOWNLINK_SEND_FLOW_DEG(DefaultChannel, DefaultDevice, &S, &K1,&K2,&hff_work->x));
}

static void flow_hff_update_y(struct HfilterFloat *hff_work, float y_meas, float Rpos)
{
  //b2_hff_y_meas = y_meas;

  const float y  = y_meas;  // - hff_work->y;
  const float S  = hff_work->yP[0][0] + Rpos;
  const float K1 = hff_work->yP[0][0] * 1 / S;
  const float K2 = hff_work->yP[1][0] * 1 / S;

  hff_work->y     = hff_work->y     + K1 * y;
  hff_work->ydot  = hff_work->ydot  + K2 * y;

  const float P11 = (1. - K1) * hff_work->yP[0][0];
  const float P12 = (1. - K1) * hff_work->yP[0][1];
  const float P21 = -K2 * hff_work->yP[0][0] + hff_work->yP[1][0];
  const float P22 = -K2 * hff_work->yP[0][1] + hff_work->yP[1][1];

  hff_work->yP[0][0] = P11;
  hff_work->yP[0][1] = P12;
  hff_work->yP[1][0] = P21;
  hff_work->yP[1][1] = P22;
}


/*
 *
 * Update velocity
 *
 *

 H = [0 1];
 R = 0.1;
 // state residual
 yd = vx - H * Xm;
 // covariance residual
 S = H*Pm*H' + R;
 // kalman gain
 K = Pm*H'*inv(S);
 // update state
 Xp = Xm + K*yd;
 // update covariance
 Pp = Pm - K*H*Pm;
*/
void flow_hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel)
{
  flow_hff_update_xdot(&flow_hff_state, vel.x, Rvel.x);
  flow_hff_update_ydot(&flow_hff_state, vel.y, Rvel.y);
}

static void flow_hff_update_xdot(struct HfilterFloat *hff_work, float vel, float Rvel)
{
  //b2_hff_xd_meas = vel;

  const float yd = vel - hff_work->xdot;
  const float S  = hff_work->xP[1][1] + Rvel;
  const float K1 = hff_work->xP[0][1] * 1 / S;
  const float K2 = hff_work->xP[1][1] * 1 / S;

  hff_work->x     = hff_work->x     + K1 * yd;
  hff_work->xdot  = hff_work->xdot  + K2 * yd;

  const float P11 = -K1 * hff_work->xP[1][0] + hff_work->xP[0][0];
  const float P12 = -K1 * hff_work->xP[1][1] + hff_work->xP[0][1];
  const float P21 = (1. - K2) * hff_work->xP[1][0];
  const float P22 = (1. - K2) * hff_work->xP[1][1];

  hff_work->xP[0][0] = P11;
  hff_work->xP[0][1] = P12;
  hff_work->xP[1][0] = P21;
  hff_work->xP[1][1] = P22;
}

static void flow_hff_update_ydot(struct HfilterFloat *hff_work, float vel, float Rvel)
{
  //b2_hff_yd_meas = vel;

  const float yd = vel - hff_work->ydot;
  const float S  = hff_work->yP[1][1] + Rvel;
  const float K1 = hff_work->yP[0][1] * 1 / S;
  const float K2 = hff_work->yP[1][1] * 1 / S;

  hff_work->y     = hff_work->y     + K1 * yd;
  hff_work->ydot  = hff_work->ydot  + K2 * yd;

  const float P11 = -K1 * hff_work->yP[1][0] + hff_work->yP[0][0];
  const float P12 = -K1 * hff_work->yP[1][1] + hff_work->yP[0][1];
  const float P21 = (1. - K2) * hff_work->yP[1][0];
  const float P22 = (1. - K2) * hff_work->yP[1][1];

  hff_work->yP[0][0] = P11;
  hff_work->yP[0][1] = P12;
  hff_work->yP[1][0] = P21;
  hff_work->yP[1][1] = P22;
}
