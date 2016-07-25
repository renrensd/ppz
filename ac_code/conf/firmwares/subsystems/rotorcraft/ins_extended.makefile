#
# extended INS with vertical filter using sonar in a better way (flap ground)
#

$(TARGET).CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_int.h\"
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/ins_int.c

#  vertical filter float version,using vf_extended_float in a better way
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/vf_extended_float.c
$(TARGET).CFLAGS += -DUSE_VFF_EXTENDED=1

# horizontal filter float version
$(TARGET).CFLAGS += -DUSE_HFF=1
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/hf_float.c
$(TARGET).srcs += $(SRC_SUBSYSTEMS)/ins/flow_hf_float.c
