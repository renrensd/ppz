# Hey Emacs, this is a -*- makefile -*-
#
# krooz_sd_chibios.makefile
#
#

BOARD=krooz
BOARD_VERSION=sd
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
ARCH_DIR=stm32
RTOS=chibios-libopencm3
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/STM32F407xG_CCM_CHIBIOS.ld

# include Makefile.chibios-libopencm3 instead of Makefile.stm32
$(TARGET).MAKEFILE = chibios-libopencm3

# default flash mode is via usb dfu bootloader
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL

#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED ?= none
AHRS_ALIGNER_LED ?= 2
GPS_LED ?= none
SYS_TIME_LED ?= 1

#
# default uart configuration
#

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B9600


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
#ACTUATORS ?= actuators_pwm
