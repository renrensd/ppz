#
# Expected from board file or overriden as xml param :
#
#

$(TARGET).CFLAGS += -DENG_OPTION
$(TARGET).srcs += subsystems/eng/eng_app.c

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(ENG_CFLAGS)
$(TARGET).srcs += $(ENG_SRCS)
endif

