/*
 * ubx_parser.h
 *
 *  Created on: Dec 19, 2016
 *      Author: jay
 */

#ifndef SW_AIRBORNE_MODULES_GPS_UBX_PARSER_H_
#define SW_AIRBORNE_MODULES_GPS_UBX_PARSER_H_

#define UBX_MAX_PAYLOAD_LEN	255

#define UBX_ID_FIELD_LEN	2
#define UBX_LEN_FIELD_LEN	2

#define UBX_SYNC_CHAR_1	0xB5
#define UBX_SYNC_CHAR_2 0x62

#define UBX_NAV_CLASS		0x01

#define UBX_POSLLH_ID		0x02
#define UBX_SOL_ID			0x06
#define UBX_VELNED_ID		0x12
#define UBX_TIMEUTC_ID	0x21

#define UBX_POSLLH_LEN		28
#define UBX_SOL_LEN				52
#define UBX_VELNED_LEN		36
#define UBX_TIMEUTC_LEN		20

struct _s_ubx_POSLLH
{
	unsigned long iTOW;
	signed long lon;
	signed long lat;
	signed long height;
	signed long hMSL;
	unsigned long hAcc;
	unsigned long vAcc;
};

struct _s_ubx_SOL
{
	unsigned long iTOW;
	signed long fTOW;
	signed short week;
	unsigned char gpsFix;
	unsigned char flags;
	signed long ecefX;
	signed long ecefY;
	signed long ecefZ;
	unsigned long pAcc;
	signed long ecefVX;
	signed long ecefVY;
	signed long ecefVZ;
	unsigned long sAcc;
	unsigned short pDOP;
	unsigned char reserved1;
	unsigned char numSV;
	unsigned char reserved2[4];
};

struct _s_ubx_VELNED
{
	unsigned long iTOW;
	signed long velN;
	signed long velE;
	signed long velD;
	unsigned long speed;
	unsigned long gSpeed;
	signed long heading;
	unsigned long sAcc;
	unsigned long cAcc;
};

struct _s_ubx_TIMEUTC
{
	unsigned long iTOW;
	unsigned long tAcc;
	signed long nano;
	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	unsigned char valid;
};

enum _e_ubx_parse_state
{
	UBX_PARSE_SEARCH_SYNC_1 = 0,
	UBX_PARSE_SYNC_2,
	UBX_PARSE_CLASS,
	UBX_PARSE_ID,
	UBX_PARSE_LEN_1,
	UBX_PARSE_LEN_2,
	UBX_PARSE_PAYLOAD,
	UBX_PARSE_CK_1,
	UBX_PARSE_CK_2,
	UBX_PARSE_FETCH
};

enum _e_ubx_parse_err
{
	UBX_PARSE_ERR_NONE = 0,
	UBX_PARSE_ERR_LEN,
	UBX_PARSE_ERR_CK,
	UBX_PARSE_ERR_SYNC,
	UBX_PARSE_ERR_FETCH,
	UBX_PARSE_ERR_UNKNOWN
};

struct _s_ubx_parser
{
	struct _s_ubx_POSLLH POSLLH;
	struct _s_ubx_SOL SOL;
	struct _s_ubx_VELNED VELNED;
	struct _s_ubx_TIMEUTC TIMEUTC;

	bool_t POSLLH_available;
	bool_t SOL_available;
	bool_t VELNED_available;
	bool_t TIMEUTC_available;

	uint8_t CLASS;
	uint8_t ID;
	uint16_t LEN;

	uint8_t buffer[UBX_MAX_PAYLOAD_LEN];
	uint8_t index;
	uint8_t ck_a;
	uint8_t ck_b;

	enum _e_ubx_parse_state state;
	enum _e_ubx_parse_err err_last;
	uint32_t err_count;
};

static inline void UBX_parser_init(struct _s_ubx_parser *parser)
{
	parser->POSLLH_available = FALSE;
	parser->SOL_available = FALSE;
	parser->TIMEUTC_available = FALSE;

	parser->err_count = 0;
	parser->state = UBX_PARSE_SEARCH_SYNC_1;
}

static inline uint8_t UBX_message_fetch(struct _s_ubx_parser *parser)
{
	uint16_t i;
	uint8_t *addr;
	uint16_t len = 0;
	bool_t *available = 0;

	switch (parser->CLASS)
	{
	case UBX_NAV_CLASS:
		switch (parser->ID)
		{
		case UBX_POSLLH_ID:
			addr = (uint8_t *) &parser->POSLLH;
			available = &parser->POSLLH_available;
			len = UBX_POSLLH_LEN;
			break;
		case UBX_SOL_ID:
			addr = (uint8_t *) &parser->SOL;
			available = &parser->SOL_available;
			len = UBX_SOL_LEN;
			break;
		case UBX_VELNED_ID:
			addr = (uint8_t *) &parser->VELNED;
			available = &parser->VELNED_available;
			len = UBX_VELNED_LEN;
			break;
		case UBX_TIMEUTC_ID:
			addr = (uint8_t *) &parser->TIMEUTC;
			available = &parser->TIMEUTC_available;
			len = UBX_TIMEUTC_LEN;
			break;
		default:
			return UBX_PARSE_ERR_FETCH;
			break;
		}

		if( parser->LEN != len )
		{
			return UBX_PARSE_ERR_LEN;
		}
		for(i=0;i<len;++i)
		{
			addr[i] = parser->buffer[i];
		}
		*available = TRUE;

		break;
	default:
		return UBX_PARSE_ERR_FETCH;
		break;
	}

	return UBX_PARSE_ERR_NONE;
}

static inline void UBX_message_parse(struct _s_ubx_parser *parser, uint8_t c)
{
	if (parser->state < UBX_PARSE_CK_1)
	{
		parser->ck_a += c;
		parser->ck_b += parser->ck_a;
	}

	switch (parser->state)
	{
	case UBX_PARSE_SEARCH_SYNC_1:
		if (c == UBX_SYNC_CHAR_1)
		{
			parser->state++;
		}
		break;
	case UBX_PARSE_SYNC_2:
		if (c != UBX_SYNC_CHAR_2)
		{
			parser->err_last = UBX_PARSE_ERR_SYNC;
			goto _error;
		}
		parser->ck_a = 0;
		parser->ck_b = 0;
		parser->state++;
		break;
	case UBX_PARSE_CLASS:
		parser->CLASS = c;
		parser->state++;
		break;
	case UBX_PARSE_ID:
		parser->ID = c;
		parser->state++;
		break;
	case UBX_PARSE_LEN_1:
		parser->LEN = c;
		parser->state++;
		break;
	case UBX_PARSE_LEN_2:
		parser->LEN |= (c << 8);
		if (parser->LEN > UBX_MAX_PAYLOAD_LEN)
		{
			parser->err_last = UBX_PARSE_ERR_LEN;
			goto _error;
		}
		parser->index = 0;
		parser->state++;
		break;
	case UBX_PARSE_PAYLOAD:
		parser->buffer[parser->index] = c;
		parser->index++;
		if (parser->index >= parser->LEN)
		{
			parser->state++;
		}
		break;
	case UBX_PARSE_CK_1:
		if (c != parser->ck_a)
		{
			parser->err_last = UBX_PARSE_ERR_CK;
			goto _error;
		}
		parser->state++;
		break;
	case UBX_PARSE_CK_2:
		if (c != parser->ck_b)
		{
			parser->err_last = UBX_PARSE_ERR_CK;
			goto _error;
		}
		else
		{
			parser->err_last = UBX_message_fetch(parser);
			if (parser->err_last != UBX_PARSE_ERR_NONE)
			{
				goto _error;
			}
			else
			{
				goto _restart;
			}
		}
		break;
	case UBX_PARSE_FETCH:
		break;
	default:
		parser->err_last = UBX_PARSE_ERR_UNKNOWN;
		goto _error;
	}
	return;

	_error: parser->err_count++;
	_restart: parser->state = UBX_PARSE_SEARCH_SYNC_1;
	return;
}

/*
static inline void UBX_message_fetch(struct _s_ubx_parser *parser, uint8_t *msg, uint16_t msg_len)
{
	uint16_t i;
	uint8_t *addr;
	uint16_t len = 0;

	if(msg[0] == UBX_NAV_CLASS)
	{
		if(msg[1] == UBX_POSLLH_ID)
		{
			addr = (uint8_t *)&parser->POSLLH;
			len = UBX_POSLLH_LEN;
		}
		else if(msg[1] == UBX_SOL_ID)
		{
			addr = (uint8_t *)&parser->SOL;
			len = UBX_SOL_LEN;
		}
		else if(msg[1] == UBX_TIMEUTC_ID)
		{
			addr = (uint8_t *)&parser->TIMEUTC;
			len = UBX_TIMEUTC_LEN;
		}

		if(len == 0)
		{
			return;
		}

		if( (msg_len - (UBX_ID_FIELD_LEN+UBX_LEN_FIELD_LEN)) != len )
		{
			return;
		}

		msg += UBX_ID_FIELD_LEN+UBX_LEN_FIELD_LEN;
		for(i=0;i<len;++i)
		{
			addr[i] = msg[i];
		}
	}
}

static inline uint8_t *UBX_message_parse(struct _s_ubx_parser *parser, uint8_t *buffer, uint16_t buffer_len)
{
	uint16_t i;
	uint16_t message_len;
	uint8_t sync_find = 0;
	uint8_t CK_A,CK_B;

	if(buffer_len < 4)
	{
		return 0;
	}

	for(i=0;i<buffer_len;i+=2)
	{
		if((buffer[i] == UBX_SYNC_CHAR_1) && (buffer[i+1] == UBX_SYNC_CHAR_2))
		{
			buffer = buffer + i + 2;
			buffer_len = buffer_len - i - 2;
			message_len = buffer_len;
			sync_find = 1;
			break;
		}
	}
	if(!sync_find)
	{
		return 0;
	}

	for(i=0;i<buffer_len;i+=2)
	{
		if((buffer[i] == UBX_SYNC_CHAR_1) && (buffer[i+1] == UBX_SYNC_CHAR_2))
		{
			message_len = i;
			break;
		}
	}

	if(message_len < 4)
	{
		return 0;
	}

	CK_A = CK_B = 0;
	for(i=0;i<message_len-2;++i)
	{
		CK_A = CK_A + buffer[i];
		CK_B = CK_B + CK_A;
	}
	if((buffer[message_len-2] == CK_A) && (buffer[message_len-1] == CK_B))
	{
		UBX_message_fetch(parser, buffer, message_len - 2);
	}

	return buffer + message_len;
}

static inline void UBX_rx_callback(struct _s_ubx_parser *parser, uint8_t *msg_buffer, uint16_t rxLength)
{
	uint8_t *buffer;
	uint16_t remain_len;

	if(rxLength < 4)
	{
		return;
	}

	if(rxLength > UBX_MAX_PAYLOAD_LEN)
	{
		return;
	}

	remain_len = rxLength;
	buffer = msg_buffer;
	while(1)
	{
		buffer = UBX_message_parse(parser, buffer, remain_len);
		if(buffer == 0)
		{
			return;
		}
		else
		{
			remain_len = msg_buffer + rxLength - buffer;
		}
	}
}
*/

#endif /* SW_AIRBORNE_MODULES_GPS_UBX_PARSER_H_ */
