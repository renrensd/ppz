#
# Expected from board file or overriden as xml param :
#
#

# $(TARGET).CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=can_tp
# $(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=can_tp


 include $(CFG_SHARED)/bbox.makefile

$(TARGET).srcs += subsystems/datalink/can_transport.c 

