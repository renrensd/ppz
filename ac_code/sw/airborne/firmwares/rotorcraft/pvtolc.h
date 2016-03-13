/**
 * @file firmwares/rotorcraft/pvtolc.h
 *prepare,take off,landing and closure
 */

#ifndef PVTOLC_H
#define PVTOLC_H

#include "math/pprz_geodetic_int.h"

#include "std.h"

extern bool_t flight_prepare(bool_t reset);
extern bool_t take_off_motion(bool_t reset);
extern bool_t land_motion(bool_t reset);
extern bool_t lock_motion(bool_t reset);
extern int8_t nav_toward_waypoint(struct EnuCoor_i *wp_end, bool_t reset);
extern void pvtol_all_reset(bool_t reset);

#endif /* END OF RC_NAV_H */
