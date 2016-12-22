/*
 * ins_ublox.h
 *
 *  Created on: Dec 20, 2016
 *      Author: jay
 */

#ifndef SW_AIRBORNE_MODULES_INS_INS_UBLOX_H_
#define SW_AIRBORNE_MODULES_INS_INS_UBLOX_H_

#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "filters/sgdf_filter.h"

#define NED_VEL_SGDF_WIN_SIZE	(3)

struct _s_ins_ublox
{
	struct NedCoor_f ned_acc;
	struct NedCoor_f ublox_ned_vel;
	struct EcefCoor_f ublox_ecef_pos;
	struct NedCoor_f ned_vel;
	struct NedCoor_f ned_pos;
	struct NedCoor_f ned_pos_last;
	DECLARE_SGDF(vel_x_sgdf, NED_VEL_SGDF_WIN_SIZE);
	DECLARE_SGDF(vel_y_sgdf, NED_VEL_SGDF_WIN_SIZE);
	DECLARE_SGDF(vel_z_sgdf, NED_VEL_SGDF_WIN_SIZE);
	struct LtpDef_f ltpDef;
	bool_t ltpDef_initialized;
	bool_t ublox_stable;
	bool_t ublox_signal_stable;
	uint16_t ublox_stable_count;
	bool_t ublox_stable_first_time;
	bool_t ublox_update;
	uint16_t ublox_update_lost_count;
};

extern struct _s_ins_ublox ins_ublox;

void ins_ublox_init(void);
void ins_ublox_event(void);
void ins_ublox_periodic(void);

#endif /* SW_AIRBORNE_MODULES_INS_INS_UBLOX_H_ */
