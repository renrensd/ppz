# XBee modems in API mode
#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

TRANSPTA_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
TRANSPTA_MODEM_PORT_UPPER=$(shell echo $(MODEM_PORT) | tr a-z A-Z)

$(TARGET).CFLAGS += -DUSE_$(TRANSPTA_MODEM_PORT_UPPER)
$(TARGET).CFLAGS += -D$(TRANSPTA_MODEM_PORT_UPPER)_BAUD=$(MODEM_BAUD) -DTRANSPTA_BAUD=$(MODEM_BAUD)
$(TARGET).CFLAGS += -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(TRANSPTA_MODEM_PORT_LOWER)
$(TARGET).CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE_PTA=$(TRANSPTA_MODEM_PORT_LOWER) -DPTA_UART=$(TRANSPTA_MODEM_PORT_LOWER) -DPPRZ_UART=$(TRANSPTA_MODEM_PORT_LOWER)
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT_PTA=pta_tp -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=TRANSPTA

$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/telemetry.c
$(TARGET).srcs += subsystems/datalink/transpta.c 
$(TARGET).srcs += subsystems/datalink/pprz_transport.c 

ap.srcs += $(SRC_FIRMWARE)/datalink.c 
ap.srcs += $(SRC_FIRMWARE)/rotorcraft_telemetry.c
ap.srcs += subsystems/datalink/datalink_ack.c subsystems/datalink/downlink_xbee_periodic.c

