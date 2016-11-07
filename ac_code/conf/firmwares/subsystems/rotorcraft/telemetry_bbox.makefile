#
# Expected from board file or overriden as xml param :
#
#

# $(TARGET).CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=can_tp
# $(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=can_tp
  $(TARGET).CFLAGS += -DBBOX_OPTION=1
  $(TARGET).CFLAGS += -DOPEN_PC_DATALINK=1
 include $(CFG_SHARED)/bbox.makefile

$(TARGET).srcs += subsystems/datalink/can_transport.c 

