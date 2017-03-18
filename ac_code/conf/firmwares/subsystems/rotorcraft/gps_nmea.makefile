# Hey Emacs, this is a -*- makefile -*-

# NMEA GPS unit

NMEA_GPS_PORT_LOWER=$(shell echo $(GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS=1
ap.CFLAGS += -DGPS_LINK=$(NMEA_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)
ap.CFLAGS += -DUSE_GPS_NMEA

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_nmea.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_nmea.c

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

