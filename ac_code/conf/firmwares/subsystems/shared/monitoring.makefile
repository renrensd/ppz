#
#


#$(TARGET).CFLAGS += -DUSE_MONITORING

$(TARGET).srcs += subsystems/monitoring/monitoring.c 
$(TARGET).srcs += subsystems/monitoring/monitoring_imu.c 
$(TARGET).srcs += subsystems/monitoring/monitoring_height.c
$(TARGET).srcs += subsystems/monitoring/monitoring_misc.c