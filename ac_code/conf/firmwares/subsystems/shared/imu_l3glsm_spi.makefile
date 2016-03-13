# Hey Emacs, this is a -*- makefile -*-
#
# L3GLSM IMU via SPI
# Should  @ conf/firmwares/subsystems/shared/imu_l3glsm_spi.makefile
#

include $(CFG_SHARED)/spi_master.makefile

# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_L3GLSM_CFLAGS  = -DUSE_IMU
endif

IMU_L3GLSM_CFLAGS += -DIMU_TYPE_H=\"imu/imu_l3glsm_spi.h\"
IMU_L3GLSM_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_L3GLSM_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_l3glsm_spi.c
IMU_L3GLSM_SRCS   += peripherals/l3glsm.c
IMU_L3GLSM_SRCS   += peripherals/l3glsm_spi.c


# set default SPI device and slave index
IMU_L3GLSM_SPI_DEV ?= spi2
IMU_L3GLSM_SPI_SLAVE_IDX ?= SPI_SLAVE2


ifeq ($(TARGET), ap)
ifndef IMU_L3GLSM_SPI_DEV
$(error Error: IMU_L3GLSM_SPI_DEV not configured!)
endif
ifndef IMU_L3GLSM_SPI_SLAVE_IDX
$(error Error: IMU_L3GLSM_SPI_SLAVE_IDX not configured!)
endif
endif

# convert spix to upper/lower case
IMU_L3GLSM_SPI_DEV_UPPER=$(shell echo $(IMU_L3GLSM_SPI_DEV) | tr a-z A-Z)
IMU_L3GLSM_SPI_DEV_LOWER=$(shell echo $(IMU_L3GLSM_SPI_DEV) | tr A-Z a-z)

IMU_L3GLSM_CFLAGS += -DIMU_L3GLSM_SPI_DEV=$(IMU_L3GLSM_SPI_DEV_LOWER)
IMU_L3GLSM_CFLAGS += -DUSE_$(IMU_L3GLSM_SPI_DEV_UPPER)
IMU_L3GLSM_CFLAGS += -DIMU_L3GLSM_SPI_SLAVE_IDX=$(IMU_L3GLSM_SPI_SLAVE_IDX)
IMU_L3GLSM_CFLAGS += -DUSE_$(IMU_L3GLSM_SPI_SLAVE_IDX)

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_L3GLSM_CFLAGS)
$(TARGET).srcs += $(IMU_L3GLSM_SRCS)
endif


#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
