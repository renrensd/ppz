/*
 * Copyright (C) 2016  Lijie <lij@efficien.cn>
 */

/**
 * @file modules/acc_cali/acc_cali.h
 * @brief Calibration of accelarometer.
 * This module is based on sphere fitting).
 */

#ifndef ACC_CALI_H
#define ACC_CALI_H

#include "std.h"

enum _e_acc_cali_state
{
	ACC_CALI_IDLE = 0,
	ACC_CALI_INI = 1,
	ACC_CALI_GRAB_PX = 2,
	ACC_CALI_GRAB_NX = 3,
	ACC_CALI_GRAB_PY = 4,
	ACC_CALI_GRAB_NY = 5,
	ACC_CALI_GRAB_NZ = 6,
	ACC_CALI_GRAB_PZ = 7,
	ACC_CALI_CALC = 8
};

struct AccCali
{
	bool_t enable;
	bool_t enable_prev;
	bool_t cali_success;
	enum _e_acc_cali_state state;
	float acc_phy_nc[3];
	float acc_norm;
	float acc_norm_filter;
	bool_t is_body_static;
	float acc_offset[3];
	float acc_gain[3];
	float acc_6point[6][3];
	uint32_t acc_cali_tick;
	uint32_t check_body_static_tick;
	float acc_NEUTRAL[3];
	float acc_SENS[3];
};

extern struct AccCali acc_cali;

extern void acc_cali_init(void);
extern void acc_cali_periodic(void);
extern void acc_cali_event(void);

extern void sensors_acc_cali_start(void);
extern void sensors_acc_cali_stop(void);
extern bool_t acc_cali_load_to_imu(void);

extern void set_acc_cali_enable(uint8_t value);



#endif
