/*
 * Copyright (C) 2009-2014 The Paparazzi Team
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
 * @file subsystems/settings.c
 * Persistent settings interface.
 *
 */

#include "subsystems/settings.h"
#include "generated/settings.h"

#ifndef NPS_SIMU
#ifndef USE_PERSISTENT_SETTINGS
#define USE_PERSISTENT_SETTINGS FALSE
#endif
#endif

#ifdef WDG_OPTION
#include "wdg.h"
#endif

#ifdef FRAM_OPTION
#include "subsystems/fram/fram_if.h"
#endif	/* FRAM_OPTION */

struct PersistentSettings pers_settings;

/** flag for setting feedback.
 * if TRUE, was stored sucessfully.
 * Also settings still need a variable,
 * pure function call not possible yet.
 */
bool_t settings_store_flag;

bool_t settings_clear_flag;


void settings_init(void)
{
#if USE_PERSISTENT_SETTINGS
#ifdef FRAM_OPTION
	if( fram_ac_param_read((void *)&pers_settings, sizeof(struct PersistentSettings)) != 0)
	{
		return;  // return -1 ?
	}
#else
	if (persistent_read((void *)&pers_settings, sizeof(struct PersistentSettings)))
	{
		return;  // return -1 ?
	}
#endif	/* FRAM_OPTION */

	/* from generated/settings.h */
	persistent_settings_load();
#endif
#ifdef FRAM_OPTION
	fram_acc_cali_get();
	fram_mag_cali_get();
#endif
}

/** store settings marked as persistent to flash
 * @return 0 on success
 */
int32_t settings_store(void)
{
#if USE_PERSISTENT_SETTINGS
	if (settings_store_flag)
	{
#ifdef WDG_OPTION
		wdg_enable_systick_feed();
#endif
		/* from generated/settings.h */
		persistent_settings_store();

#ifdef FRAM_OPTION
		if( fram_ac_param_write((void *)&pers_settings, sizeof(struct PersistentSettings)) == 0)
		{
			/* persistent write was successful */
			settings_store_flag = TRUE;
#ifdef WDG_OPTION
			wdg_disable_systick_feed();
#endif
			return 0;
		}
#else
		if (!persistent_write((void *)&pers_settings, sizeof(struct PersistentSettings)))
		{
			/* persistent write was successful */
			settings_store_flag = TRUE;
#ifdef WDG_OPTION
			wdg_disable_systick_feed();
#endif
			return 0;
		}
#endif	/* FRAM_OPTION */
	}
#endif
	settings_store_flag = FALSE;
	return -1;
}

/** clear all persistent settings from flash
 * @return 0 on success
 */
int32_t settings_clear(void)
{
#if USE_PERSISTENT_SETTINGS
	if (settings_clear_flag)
	{
#ifdef WDG_OPTION
		wdg_enable_systick_feed();
#endif

#ifdef FRAM_OPTION

#else
		if (!persistent_clear())
		{
			/* clearing all persistent settings was successful */
			settings_clear_flag = TRUE;
#ifdef WDG_OPTION
			wdg_disable_systick_feed();
#endif
			return 0;
		}
#endif	/* FRAM_OPTION */
	}
#endif
	settings_clear_flag = FALSE;
	return -1;
}
