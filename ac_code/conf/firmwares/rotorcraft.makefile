# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2010 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_ROTORCRAFT=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/rotorcraft

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

SRC_ARCH=arch/$(ARCH)

ROTORCRAFT_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOARD)


ap.ARCHDIR = $(ARCH)

#NPS_OPTION=yes

######################################################################
##
## COMMON ROTORCRAFT ALL TARGETS (AP + NPS)
##
$(TARGET).CFLAGS += -I$(PAPARAZZI_SRC)/sw/ext/libstm32

$(TARGET).CFLAGS += $(ROTORCRAFT_INC)
$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
$(TARGET).CFLAGS += -DPERIPHERALS_AUTO_INIT
ifndef NPS_OPTION
$(TARGET).CFLAGS += -DFAULT_OPTION
$(TARGET).CFLAGS += -DWDG_OPTION
$(TARGET).CFLAGS += -DUPGRADE_OPTION
# $(TARGET).CFLAGS += -DBBOX_OPTION=1
# $(TARGET).CFLAGS += -DOPEN_PC_DATALINK=1
$(TARGET).CFLAGS += -DFRAM_OPTION
$(TARGET).CFLAGS += -DENG_OPTION
$(TARGET).CFLAGS += -DGCS_V1_OPTION
# $(TARGET).CFLAGS += -DCOMM_DIRECT_CONNECT
# $(TARGET).CFLAGS += -DCALIBRATION_OPTION
$(TARGET).CFLAGS += -DSYS_TIMER_OPTION
$(TARGET).CFLAGS += -DQMC5883_OPTION
# $(TARGET).CFLAGS += -DHMC5983_OPTION
# $(TARGET).CFLAGS += -DDEBUG_VRC
endif
$(TARGET).srcs   += mcu.c
$(TARGET).srcs   += $(SRC_ARCH)/mcu_arch.c

# frequency of main periodic
PERIODIC_FREQUENCY ?= 512
$(TARGET).CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)

ifdef AHRS_PROPAGATE_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
endif

ifdef AHRS_CORRECT_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)
endif

ifdef AHRS_MAG_CORRECT_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_MAG_CORRECT_FREQUENCY=$(AHRS_MAG_CORRECT_FREQUENCY)
endif


#
# Systime
#
$(TARGET).srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

ifeq ($(ARCH), linux)
# seems that we need to link against librt for glibc < 2.17
$(TARGET).LDFLAGS += -lrt
endif

 include $(CFG_SHARED)/fram.makefile

ifndef NPS_OPTION
#
# add libstm32
#
 include $(PAPARAZZI_SRC)/sw/ext/libstm32/libstm32.makefile

#
# OPS_SYSTEM
#
 include $(CFG_SHARED)/ops.makefile

#
# AUTOFLIGHT_XBEE  (use dynamic mission replay flight plan,add key RC control)
#
 include $(CFG_ROTORCRAFT)/autoflight_xbee.makefile 

#
# MONITORING_SYSTEM
#
 include $(CFG_SHARED)/monitoring.makefile

#
# ENG SUBSYSTEM
#
 include $(CFG_SHARED)/eng.makefile

$(TARGET).srcs += modules/system/timer.c
$(TARGET).srcs += modules/system/type_conv.c
$(TARGET).srcs += modules/system/tools.c

endif



#
# Math functions
#

ifneq ($(TARGET), fbw)
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c
$(TARGET).srcs += math/my_math.c
$(TARGET).srcs += math/dim2_algebra.c
$(TARGET).srcs += math/dim2_geometry.c

$(TARGET).srcs += subsystems/settings.c
$(TARGET).srcs += $(SRC_ARCH)/subsystems/settings_arch.c
endif

$(TARGET).srcs += subsystems/actuators.c
$(TARGET).srcs += subsystems/commands.c

ifneq ($(TARGET), fbw)
$(TARGET).srcs += state.c

#
# BARO_BOARD (if existing/configured)
#
include $(CFG_SHARED)/baro_board.makefile


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization.c
$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_none.c
$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
#$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_h_ref.c
$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
#$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v_ref.c
#$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v_adapt.c
#$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_flip.c

include $(CFG_ROTORCRAFT)/navigation.makefile
else
$(TARGET).CFLAGS += -DFBW=1
endif

ifneq ($(TARGET), fbw)
	ifeq ($(RTOS), chibios-libopencm3)
	 $(TARGET).srcs += $(SRC_FIRMWARE)/main_chibios_libopencm3.c
	 $(TARGET).srcs += $(SRC_FIRMWARE)/chibios-libopencm3/chibios_init.c
	else
	 $(TARGET).srcs += $(SRC_FIRMWARE)/main.c
	endif
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot.c
else
$(TARGET).srcs += $(SRC_FIRMWARE)/main_fbw.c
endif

######################################################################
##
## COMMON HARDWARE SUPPORT FOR ALL TARGETS
##

ifneq ($(TARGET), fbw)
$(TARGET).srcs += mcu_periph/i2c.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
endif

include $(CFG_SHARED)/uart.makefile


#
# Electrical subsystem / Analog Backend
#
$(TARGET).CFLAGS += -DUSE_ADC
$(TARGET).srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
$(TARGET).srcs   += subsystems/electrical.c


######################################################################
##
## HARDWARE SUPPORT FOR ALL NON-SIMULATION TARGETS (ap)
##

# baro has variable offset amplifier on booz board
ifeq ($(BOARD), booz)
ns_CFLAGS += -DUSE_DAC
ns_srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c
endif

#
# Interrupts
#
ifeq ($(ARCH), lpc21)
ns_srcs += $(SRC_ARCH)/armVIC.c
endif

ifeq ($(ARCH), stm32)
ns_srcs += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif


#
# LEDs
#
ns_CFLAGS += -DUSE_LED
ifneq ($(SYS_TIME_LED),none)
ns_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif

ifeq ($(ARCH), stm32)
ns_srcs += $(SRC_ARCH)/led_hw.c
endif

ifeq ($(BOARD), ardrone)
ns_srcs += $(SRC_BOARD)/gpio_ardrone.c
endif

#
# add other subsystems to rotorcraft firmware in airframe file:
#
# telemetry

# add debug port datalink for manufacture
# include $(CFG_ROTORCRAFT)/telemetry_manu_debug.makefile 


# include $(CFG_ROTORCRAFT)/telemetry_bbox.makefile


# radio_control
# actuators
# imu
# gps
# ahrs
# ins
#


######################################################################
##
## Final Target Allocations
##

ap.CFLAGS 		+= $(ns_CFLAGS)
ap.srcs 		+= $(ns_srcs)
fbw.CFLAGS 		+= $(ns_CFLAGS)
fbw.srcs 		+= $(ns_srcs)
