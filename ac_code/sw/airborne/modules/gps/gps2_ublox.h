/*
 * gps_ublox.h
 *
 *  Created on: Dec 19, 2016
 *      Author: jay
 */

#ifndef SW_AIRBORNE_MODULES_GPS_GPS2_UBLOX_H_
#define SW_AIRBORNE_MODULES_GPS_GPS2_UBLOX_H_

#include "ubx_parser.h"

#define GPS2_UBLOX_UPDATE_FERQ	(5)

struct _s_gps2_ublox
{
	struct _s_ubx_parser parser;
	struct link_device *dev;

	uint8_t have_velned;
};

extern struct _s_gps2_ublox gps2_ublox;
extern struct GpsState gps2;

void gps2_ublox_init(void);
void gps2_ublox_event(void);
void gps2_ublox_periodic(void);
bool_t gps2_ublox_check_update(void);

#endif /* SW_AIRBORNE_MODULES_GPS_GPS_UBLOX_H_ */
