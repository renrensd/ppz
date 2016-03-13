/**
 * @file subsystems/datalink/downlink_xbee_periodic.h
 * use for send periodic message from aircraft to gcs(android)
 */
#ifndef DOWNLINK_GCS_PERIODIC_H
#define DOWNLINK_GCS_PERIODIC_H

extern void downlink_gcs_periodic(void);
extern void send_heart_beat_A2G_msg(void);
extern void downlink_pc_periodic(void);

#endif /* DOWNLINK_GCS_PERIODIC_H */
