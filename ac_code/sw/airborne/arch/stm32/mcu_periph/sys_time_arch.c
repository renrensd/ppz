/*
 * Copyright (C) 2009-2011 The Paparazzi Team
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
 * @file arch/stm32/mcu_periph/sys_time_arch.c
 * @ingroup stm32_arch
 *
 * STM32 timing functions.
 *
 */

#include "mcu_periph/sys_time.h"
#include "libopencm3/cm3/systick.h"

#ifdef SYS_TIME_LED
#include "led.h"
#endif

#ifdef OPS_OPTION
#include "subsystems/ops/uart_ops_if.h"
#include "subsystems/ops/ops_msg_if.h"
#endif	/* OPS_OPTION */

#ifndef USE_OCM3_SYSTICK_INIT
#define USE_OCM3_SYSTICK_INIT 1
#endif

#ifdef SYS_TIMER_OPTION
#include "modules/system/timer_if.h"
#endif

#ifdef WDG_OPTION
#include "wdg.h"
#endif


void sys_tick_handler(void);
void tm_tick_polling(void);


/** Initialize SysTick.
 * Generate SysTick interrupt every sys_time.resolution_cpu_ticks
 */
void sys_time_arch_init(void)
{
  /* run cortex systick timer with 72MHz (FIXME only 72 or does it work with 168MHz???) */
#if USE_OCM3_SYSTICK_INIT
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
#endif
  sys_time.cpu_ticks_per_sec = AHB_CLK;

  /* cpu ticks per desired sys_time timer step */
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

#if USE_OCM3_SYSTICK_INIT
  /* The timer interrupt is activated on the transition from 1 to 0,
   * therefore it activates every n+1 clock ticks.
   */
  systick_set_reload(sys_time.resolution_cpu_ticks - 1);

  systick_interrupt_enable();
  systick_counter_enable();
#endif
}

/***********************************************************************
*  Name        : tm_tick_polling
*  Description : 
*  Parameter   : void
*  Returns     : void
***********************************************************************/
void tm_tick_polling(void)
{
  sys_time.nb_tick++;

  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;
  if (sys_time.nb_sec_rem >= sys_time.cpu_ticks_per_sec) 
  {
    sys_time.nb_sec_rem -= sys_time.cpu_ticks_per_sec;
    sys_time.nb_sec++;

#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
  }
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
}

// FIXME : nb_tick rollover ???
//
// 97 days at 512hz
// 12 hours at 100khz
//
void sys_tick_handler(void)
{
	tm_tick_polling();
	tm_task_tick_polling();
#ifdef OPS_OPTION
	uart_ops_polling();
#endif	/* OPS_OPTION */

	#ifdef WDG_OPTION
	wdg_systick_feed();
	#endif
}

