#
# autoflight subsysytem,similiar to flight plan,use to generate flight mission and navigate flight
#

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/autoflight_rc.makefile
include $(CFG_SHARED)/autoflight_mission.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/nav_flight.c $(SRC_FIRMWARE)/pvtolc.c
