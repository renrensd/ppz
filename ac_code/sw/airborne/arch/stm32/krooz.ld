/*
 * Hey Emacs, this is a -*- makefile -*-
 *
 * Copyright (C) 2012 Sergey Krukowski <softsr@yahoo.de>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/* Linker script for Krooz (STM32F405, 1024K flash, 192K RAM). */

/* Define memory regions. */
MEMORY
{
    /* Only 128K (SRAM1 and SRAM2) are accessible by all AHB masters. */
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
    bkpram (rwx) : ORIGIN = 0x2001C000, LENGTH = 16K
    
    /* Leaving 128k of space at the end of rom for persistent settings */
    rom_boot (rx) : ORIGIN = 0x08000000, LENGTH = 32K
    rom (rx) : ORIGIN = 0x08008000, LENGTH = 864K
    ccmram : org = 0x10000000, len = 64k
    bkpsram : org = 0x40024000, len = 4k
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f4.ld
