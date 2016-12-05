#
# Expected from board file or overriden as xml param :
#
PATH_LIBSTM32=$(PAPARAZZI_SRC)/sw/ext/libstm32
SRC_LIBSTM32=$(PAPARAZZI_SRC)/sw/ext/libstm32/Libraries/STM32F4xx_StdPeriph_Driver/src
LIB_LIBSTM32=$(PAPARAZZI_SRC)/sw/ext/libstm32/Libraries/STM32F4xx_StdPeriph_Driver/inc
PATH_FATFS=$(PAPARAZZI_SRC)/sw/ext/fatfs/src

$(TARGET).CFLAGS += -I$(PATH_LIBSTM32) -I$(LIB_LIBSTM32) -I$(PATH_LIBSTM32)/Libraries/CMSIS/Include -I$(PATH_LIBSTM32)/Libraries/CMSIS/ST/STM32F4xx
$(TARGET).CFLAGS += -I$(PATH_FATFS)

$(TARGET).CFLAGS += -DLIBSTM32_OPTION
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_gpio.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_rcc.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_sdio.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_dma.c
$(TARGET).srcs += $(SRC_LIBSTM32)/misc.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_syscfg.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_wwdg.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_dbgmcu.c
$(TARGET).srcs += $(SRC_LIBSTM32)/stm32f4xx_can.c

$(TARGET).srcs += $(PATH_LIBSTM32)/system_stm32f4xx.c $(PATH_LIBSTM32)/stm32f4xx_it.c $(PATH_LIBSTM32)/sdio_sd.c

$(TARGET).srcs += $(PATH_FATFS)/ff.c $(PATH_FATFS)/diskio.c $(PATH_FATFS)/option/ccsbcs.c
#$(TARGET).srcs += $(PATH_LIBSTM32)/calibration.c
$(TARGET).srcs += $(PATH_LIBSTM32)/wdg.c
$(TARGET).srcs += $(PATH_LIBSTM32)/can_drv.c

