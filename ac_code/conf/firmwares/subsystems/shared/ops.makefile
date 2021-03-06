#
# Expected from board file or overriden as xml param :
#
# OPS_PORT
# OPS_BAUD
#

OPS_PORT_LOWER=$(shell echo $(OPS_PORT) | tr A-Z a-z)
PATH_LIBSTM32=$(PAPARAZZI_SRC)/sw/ext/libstm32


$(TARGET).CFLAGS += -DUSE_$(OPS_PORT)
$(TARGET).CFLAGS += -D$(OPS_PORT)_BAUD=$(OPS_BAUD)  
$(TARGET).CFLAGS += -I$(PATH_LIBSTM32)

$(TARGET).CFLAGS += -DOPS_DEVICE=$(OPS_PORT_LOWER) -DOPS_UART=$(OPS_PORT_LOWER)
$(TARGET).srcs += subsystems/ops/ops_app.c subsystems/ops/ops_comm.c subsystems/ops/ops_msg.c subsystems/ops/uart_ops.c
$(TARGET).srcs += modules/system/fifo.c

