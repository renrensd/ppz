# Hey Emacs, this is a -*- makefile -*-
#
# The IMU system integrated into Lisa/MX V2.1 based on Aspirin V2.2. Major
# difference is that the orientation of the chips is bit different and we need
# to compensate for that.
#  </section>
#
#


# for fixedwing firmware and ap only
ifeq ($(TARGET), ap)
  IMU_ADIV1_CFLAGS  = -DUSE_IMU
endif

ADIV1_I2C_DEV ?= i2c1
IMU_ADIV1_CFLAGS += -DUSE_I2C1
ifeq ($(TARGET), ap)
ifndef ADIV1_I2C_DEV
$(error Error: ADIV1_I2C_DEV not configured!)
endif
endif

IMU_ADIV1_CFLAGS += -DIMU_TYPE_H=\"imu/imu_adiv1.h\"
IMU_ADIV1_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_ADIV1_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_adiv1.c
IMU_ADIV1_SRCS   += $(SRC_ARCH)/subsystems/imu/imu_adiv1_arch.c
IMU_ADIV1_SRCS   += peripherals/adxl345_spi.c
IMU_ADIV1_SRCS   += peripherals/adxrs290_spi.c
IMU_ADIV1_SRCS   += peripherals/hmc58xx.c

include $(CFG_SHARED)/spi_master.makefile

IMU_ADIV1_CFLAGS += -DADIV1_ACCEL_SPI_SLAVE_IDX=SPI_SLAVE1
IMU_ADIV1_CFLAGS += -DADIV1_GYRO_XY_SPI_SLAVE_IDX=SPI_SLAVE3
IMU_ADIV1_CFLAGS += -DADIV1_GYRO_Z_SPI_SLAVE_IDX=SPI_SLAVE2
IMU_ADIV1_CFLAGS += -DADIV1_SPI_DEV=spi1

IMU_ADIV1_CFLAGS += -DUSE_SPI1
# Slave select configuration
IMU_ADIV1_CFLAGS += -DUSE_SPI_SLAVE1
IMU_ADIV1_CFLAGS += -DUSE_SPI_SLAVE2
IMU_ADIV1_CFLAGS += -DUSE_SPI_SLAVE3

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_ADIV1_CFLAGS)
$(TARGET).srcs += $(IMU_ADIV1_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile
