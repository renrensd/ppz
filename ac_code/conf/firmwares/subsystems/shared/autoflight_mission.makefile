#

#$(TARGET).CFLAGS += -DUSE_MISSION
#$(TARGET).srcs += subsystems/mission/mission_manage.c subsystems/mission/mission_process.c

$(TARGET).srcs += subsystems/mission/task_manage.c subsystems/mission/task_handle.c
$(TARGET).srcs += subsystems/mission/task_process.c 
$(TARGET).srcs += subsystems/mission/task_spray_convert.c 

