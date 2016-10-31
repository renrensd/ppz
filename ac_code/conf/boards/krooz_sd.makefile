# Hey Emacs, this is a -*- makefile -*-
#
# krooz_sd.makefile
#
#
#

BOARD=krooz
BOARD_VERSION=sd
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny
$(TARGET).LDSCRIPT=$(SRC_ARCH)/krooz.ld

# default flash mode is via usb dfu bootloader
# possibilities: DFU-UTIL, SWD, STLINK

FLASH_MODE ?= DFU-UTIL
# $(TARGET).CFLAGS+=-DLUFTBOOT
# $(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000

#
#
# some default values shared between different firmwares
#
#

#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED ?= none
AHRS_ALIGNER_LED ?= 2
GPS_LED ?= 3
SYS_TIME_LED ?= 1

#
# default uart configuration
#

MODEM_PORT ?= UART2
MODEM_BAUD ?= B115200

GPS_PORT ?= UART4
GPS_BAUD ?= B115200

OPS_PORT ?= UART6
OPS_BAUD ?= B57600

