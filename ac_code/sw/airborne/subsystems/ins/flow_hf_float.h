/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file subsystems/ins/flow_hf_float.h
 *
 * Horizontal filter (x,y) to estimate position and velocity using in flow module.
 *
 */

#ifndef FLOW_HF_FLOAT_H
#define FLOW_HF_FLOAT_H
//#include "subsystems/ins/hf_float.h"  //using struct HfilterFloat


extern struct HfilterFloat flow_hff_state;
extern struct FloatVect2 local_flow_pos;

extern void flow_hff_init(float init_x, float init_xdot, float init_y, float init_ydot);
extern void hff_update_flow(struct FloatVect2 *pos_flow, struct FloatVect2 *speed_flow ,unsigned char qual);
extern void flow_hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos);
extern void flow_hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel);
extern void flow_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel);

#endif /* FLOW_HF_FLOAT_H */
