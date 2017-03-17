#
# Expected from board file or overriden as xml param :
#
#

$(TARGET).srcs += subsystems/fram/fram.c subsystems/fram/fm25v.c subsystems/fram/fram_data.c

FRAM_CFLAGS += -DFRAM_SPI_SLAVE_IDX=SPI_SLAVE4
FRAM_CFLAGS += -DFRAM_SPI_DEV=spi2

FRAM_CFLAGS += -DUSE_SPI2=1
# Slave select configuration
FRAM_CFLAGS += -DUSE_SPI_SLAVE4=1

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(FRAM_CFLAGS)
$(TARGET).srcs += $(FRAM_SRCS)
endif

