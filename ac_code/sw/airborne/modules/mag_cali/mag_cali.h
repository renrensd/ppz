/*
 * Copyright (C) 2016  Lijie <lij@efficien.cn>
 */

/**
 * @file modules/mag_cali/mag_cali.h
 * @brief Calibration of magnetometer.
 * This module is based on circle fitting).
 */

#ifndef MAG_CALI_H
#define MAG_CALI_H

#include "std.h"
#include "subsystems/imu.h"
#include "math/pprz_geodetic_int.h"

#define MAG_CALI_GRAB_COUNT_MAX				(100)
#define MAG_CALI_AUTO_ROTATE_TIME		(10)
#define MAG_CALI_CIRCLE_SPLITE_NUM	(10)
#define MAG_CALI_GRAB_NUM	(MAG_CALI_CIRCLE_SPLITE_NUM)
#define MAG_CALI_CONVERGENCE_COUNT	(100)

enum _e_mag_cali_status
{
	MAG_CALI_IDLE = 0,
	MAG_CALI_INI,
	MAG_CALI_GRAB,
	MAG_CALI_HEADING_ALIGN,
	MAG_CALI_CALC1,
	MAG_CALI_CALC2,
	MAG_CALI_CALC3
};

struct MagCali
{
	bool_t manual_enable;
	bool_t manual_enable_prev;
	bool_t manual_fram_erase;
	bool_t manual_fram_erase_prev;
	enum _e_mag_cali_status state;
	bool_t need_cali;
	bool_t auto_cali;
	bool_t cali_ok;
	bool_t cali_ok_last;
	bool_t persistent_store;
	bool_t persistent_read;
	int32_t nav_heading_ini;
	bool_t manufacture_cali;

	uint8_t grab_tick[MAG_CALI_GRAB_NUM];
	float grab_sum[MAG_CALI_GRAB_NUM][2];
	uint8_t grab_index;
	uint8_t grab_index_lock;
	uint16_t convergence_tick;
	float gain[2];
	float offset[2];
	float declination;
	bool_t declination_ok;
	struct FloatVect3  mag_h_cali;
};

extern void mag_cali_init(void);
extern void mag_cali_periodic(void);
extern void mag_cali_event(void);

extern bool_t mag_cali_load_to_imu(void);
extern void mag_cali_imu_scale(struct Imu *_imu);
extern bool_t mag_cali_nav_loop(bool_t run);

extern bool mag_cali_manufacture_start(void);
extern void mag_cali_manufacture_stop(void);

extern struct MagCali mag_cali;

#endif
