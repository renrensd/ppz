#ifndef _COMM_MSG_DEF_H_
#define _COMM_MSG_DEF_H_

/**** Definition of constants ****/


/**** Definition of macros ****/
enum COMM_MSG_ID
{
	COMM_DEV_MANAGE_SERVEICE = 0x01,
		
};

enum COMM_TYPE_PARAM
{
	COMM_TYPE_DEFAULT = 0x00,
	COMM_TYPE_AIRCRAFT,
	COMM_TYPE_GCS,
	COMM_TYPE_RC,
	COMM_TYPE_LC
};

enum COMM_ACK_PARAM
{
	COMM_NACK = 0x00,
	COMM_ACK
};

enum COMM_ADDR_PARAM
{
	COMM_ADDR_BC = 0x00,		
	COMM_ADDR_GCS = 0x10,
	COMM_ADDR_AC = 0x20,
	COMM_ADDR_RC = 0x50,
	COMM_ADDR_PC = 0x60
};

#endif /*_COMM_MSG_DEF_H_*/
/****************************** END OF FILE ***************************/

