#ifndef _XBEE_MSG_DEF_H_
#define _XBEE_MSG_DEF_H_

/**** Definition of constants ****/


/**** Definition of macros ****/
enum XBEE_MSG_ID
{
	XBEE_DEV_MANAGE_SERVEICE = 0x01,
		
};

enum XBEE_TYPE_PARAM
{
	XBEE_TYPE_DEFAULT = 0x00,
	XBEE_TYPE_AIRCRAFT,
	XBEE_TYPE_GCS,
	XBEE_TYPE_RC,
	XBEE_TYPE_LC
};

enum XBEE_ACK_PARAM
{
	XBEE_NACK = 0x00,
	XBEE_ACK
};

enum XBEE_ADDR_PARAM
{
	XBEE_ADDR_BC = 0x00,
		
	XBEE_ADDR_GCS = 0x10,

	XBEE_ADDR_RC = 0x50,

	XBEE_ADDR_PC = 0x60
};

#endif /*_XBEE_MSG_DEF_H_*/
/****************************** END OF FILE ***************************/

