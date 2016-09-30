/* Automatically generated by gen_messages from /home/lijie/work/codes/ef01_ac/ac_code/conf/messages_new.xml */
/* Version v5.8.0_stable */
/* Please DO NOT EDIT */
/* Macros to send and receive messages of class rc */
#ifndef _VAR_MESSAGES_rc_H_
#define _VAR_MESSAGES_rc_H_
#include "subsystems/datalink/transport.h"
#include "mcu_periph/link_device.h"
#define DL_RC_MOTION_CMD 1
#define PPRZ_MSG_ID_RC_MOTION_CMD 1
#define DL_RC_SET_CMD 2
#define PPRZ_MSG_ID_RC_SET_CMD 2
#define DL_RC_BIND_STATE 101
#define PPRZ_MSG_ID_RC_BIND_STATE 101
#define DL_HEART_BEAT_RC_STATE 102
#define PPRZ_MSG_ID_HEART_BEAT_RC_STATE 102
#define DL_CALIBRATION_RESULT_RC_ACK_STATE 103
#define PPRZ_MSG_ID_CALIBRATION_RESULT_RC_ACK_STATE 103
#define DL_MSG_rc_NB 5

#define DOWNLINK_SEND_RC_MOTION_CMD(_trans, _dev, motion_cmd) pprz_msg_send_RC_MOTION_CMD(&((_trans).trans_tx), &((_dev).device), AC_ID, motion_cmd)
static inline void pprz_msg_send_RC_MOTION_CMD(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_motion_cmd) {
	if (trans->check_available_space(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */))) {
	  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */));
	  trans->start_message(trans->impl, dev, 0+1 +2 /* msg header overhead */);
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, &ac_id);
	  trans->put_named_byte(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_RC_MOTION_CMD, "RC_MOTION_CMD");
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, (void *) _motion_cmd);
	  trans->end_message(trans->impl, dev);
	} else
	  trans->overrun(trans->impl, dev);
}

#define DOWNLINK_SEND_RC_SET_CMD(_trans, _dev, set_cmd) pprz_msg_send_RC_SET_CMD(&((_trans).trans_tx), &((_dev).device), AC_ID, set_cmd)
static inline void pprz_msg_send_RC_SET_CMD(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_set_cmd) {
	if (trans->check_available_space(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */))) {
	  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */));
	  trans->start_message(trans->impl, dev, 0+1 +2 /* msg header overhead */);
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, &ac_id);
	  trans->put_named_byte(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_RC_SET_CMD, "RC_SET_CMD");
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, (void *) _set_cmd);
	  trans->end_message(trans->impl, dev);
	} else
	  trans->overrun(trans->impl, dev);
}

#define DOWNLINK_SEND_RC_BIND_STATE(_trans, _dev, serial_code) pprz_msg_send_RC_BIND_STATE(&((_trans).trans_tx), &((_dev).device), AC_ID, serial_code)
static inline void pprz_msg_send_RC_BIND_STATE(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, char *_serial_code) {
	if (trans->check_available_space(trans->impl, dev, trans->size_of(trans->impl, 0+0+10*1 +2 /* msg header overhead */))) {
	  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, 0+0+10*1 +2 /* msg header overhead */));
	  trans->start_message(trans->impl, dev, 0+0+10*1 +2 /* msg header overhead */);
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, &ac_id);
	  trans->put_named_byte(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_RC_BIND_STATE, "RC_BIND_STATE");
	  trans->put_bytes(trans->impl, dev, DL_TYPE_CHAR, DL_FORMAT_ARRAY, 1 * 10, (void *) _serial_code);
	  trans->end_message(trans->impl, dev);
	} else
	  trans->overrun(trans->impl, dev);
}

#define DOWNLINK_SEND_HEART_BEAT_RC_STATE(_trans, _dev, signal) pprz_msg_send_HEART_BEAT_RC_STATE(&((_trans).trans_tx), &((_dev).device), AC_ID, signal)
static inline void pprz_msg_send_HEART_BEAT_RC_STATE(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_signal) {
	if (trans->check_available_space(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */))) {
	  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */));
	  trans->start_message(trans->impl, dev, 0+1 +2 /* msg header overhead */);
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, &ac_id);
	  trans->put_named_byte(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_HEART_BEAT_RC_STATE, "HEART_BEAT_RC_STATE");
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, (void *) _signal);
	  trans->end_message(trans->impl, dev);
	} else
	  trans->overrun(trans->impl, dev);
}

#define DOWNLINK_SEND_CALIBRATION_RESULT_RC_ACK_STATE(_trans, _dev, ack) pprz_msg_send_CALIBRATION_RESULT_RC_ACK_STATE(&((_trans).trans_tx), &((_dev).device), AC_ID, ack)
static inline void pprz_msg_send_CALIBRATION_RESULT_RC_ACK_STATE(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_ack) {
	if (trans->check_available_space(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */))) {
	  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, 0+1 +2 /* msg header overhead */));
	  trans->start_message(trans->impl, dev, 0+1 +2 /* msg header overhead */);
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, &ac_id);
	  trans->put_named_byte(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_CALIBRATION_RESULT_RC_ACK_STATE, "CALIBRATION_RESULT_RC_ACK_STATE");
	  trans->put_bytes(trans->impl, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, 1, (void *) _ack);
	  trans->end_message(trans->impl, dev);
	} else
	  trans->overrun(trans->impl, dev);
}


#define DL_RC_MOTION_CMD_motion_cmd(_payload) ((uint8_t)(*((uint8_t*)_payload+2)))

#define DL_RC_SET_CMD_set_cmd(_payload) ((uint8_t)(*((uint8_t*)_payload+2)))

#define DL_RC_BIND_STATE_serial_code_length(_payload) (10)
#define DL_RC_BIND_STATE_serial_code(_payload) ((char*)(_payload+2))

#define DL_HEART_BEAT_RC_STATE_signal(_payload) ((uint8_t)(*((uint8_t*)_payload+2)))

#define DL_CALIBRATION_RESULT_RC_ACK_STATE_ack(_payload) ((uint8_t)(*((uint8_t*)_payload+2)))
#endif // _VAR_MESSAGES_rc_H_
