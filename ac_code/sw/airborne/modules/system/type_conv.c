/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				   *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									   *
*   AUTHOR	   :            										   *
************************************************************************
* Object        : 
* Module        : 
* Instance      : 
* Description   : 
*-----------------------------------------------------------------------
* Version: 
* Date: 
* Author: 
***********************************************************************/
/*-History--------------------------------------------------------------
* Version       Date    Name    Changes and comments
* 
*=====================================================================*/

/**** System include files ****/
#include "std.h"


/*---Public include files---------------------------------------------*/

/*---Private include files--------------------------------------------*/
#include "type_conv.h"   
#include "type_conv_if.h"   




/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/***********************************************************************
*  Name        : type_conv_bcd_to_hex
*  Description : change bcd code to hex code        
*  Parameter   : uint8_t bcd
*  Returns     : uint8_t hex
***********************************************************************/
uint8_t type_conv_bcd_to_hex(uint8_t bcd)
{
    uint8_t hex;
    uint8_t temp;

    temp = bcd >> 0x04;
    hex = (temp << 0x03) + (temp << 0x01) + (bcd & 0x0F);

    return hex;
}

/***********************************************************************
*  Name        : type_conv_hex_to_bcd
*  Description : change hex code to bcd code        
*  Parameter   : uint8_t hex
*  Returns     : uint8_t bcd
***********************************************************************/
uint8_t type_conv_hex_to_bcd(uint8_t hex)
{
	if(hex <= 99)
	{
		return ((hex/10)<<4)+(hex%10);
	}
    return 0xFF;
}

/***********************************************************************
*  Name        : type_conv_hex_to_ascii
*  Description : change hex code to ascii code        
*  Parameter   : uint8_t hex
*  Returns     : uint8_t ascii
***********************************************************************/
uint8_t type_conv_hex_to_ascii(uint8_t hex)
{
    uint8_t ascii;

    if(hex <= 0x09 && hex >= 0x00)
    {
        ascii = hex + 0x30;
    }
    else
    {
        if( hex <= 0x0F && hex >= 0x0A)
        {
            ascii = hex + 0x37;
        }
        else
        {
            ascii = 0x00;  // need  modify to the wrong value
        }
    }
    return ascii;
}

/***********************************************************************
*  Name        : type_conv_ascii_to_hex
*  Description : change hex code to ascii code        
*  Parameter   : uint8_t ascii
*  Returns     : uint8_t hex
***********************************************************************/
uint8_t type_conv_ascii_to_hex(uint8_t ascii)
{
    uint8_t hex;

    if(ascii <= 0x39 && ascii >= 0x30)
    {
        hex = ascii - 0x30;
    }
    else
    {
        if(ascii <= 0x46 && ascii >= 0x41)
        {
            hex = ascii - 0x37;
        }
        else 
        {
            if (ascii <= 0x66 && ascii >= 0x61)
            {
                hex = ascii - 0x57;
            }
            else
            {
                hex = 0x00; // need  modify to the wrong value
            }
        }
    }
    
    return hex;
}

/***********************************************************************
*  Name        : type_conv_u16_to_u8
*  Description : change uint16_t to uint8_t
*  Parameter   : uint16_t , uint8_t*
*  Returns     : void 
***********************************************************************/
void type_conv_u16_to_u8(uint16_t u16, uint8_t* u8)
{
    *u8++ = (uint8_t)(u16 >> 0x08);

    *u8 = (uint8_t)u16;
}

/***********************************************************************
*  Name        : type_conv_u8_to_u16
*  Description : change uint8_t to uint16_t
*  Parameter   : uint8_t*
*  Returns     : uint16_t
***********************************************************************/
uint16_t type_conv_u8_to_u16(uint8_t *u8)
{
    uint16_t u16;

    u16 = (*u8 << 0x08) + *(u8 + 0x01);

    return u16;
}    
/**********************************************************************
 *  Function:      void byte_to_hex_string
 *  Description:   convert from uint8_t to hex string.			  
 *  Argument(s):   dst   - pointer to string
 *                 	data - uint8_t to be convert
 *  Return:        void.
 ***********************************************************************/
void byte_to_hex_string(uint8_t * dst, uint8_t data)
{
    uint8_t temp = 0;
    temp = (data & 0xf0) >> 4;
    if(temp > 9)
    {
        dst[0] = 'A' - 10 + temp;
    }
    else
    {
        dst[0] = '0' + temp;
    }
    temp = data & 0x0f;
    if(temp > 9)
    {
        dst[1] = 'A' - 10 + temp;
    }
    else
    {
        dst[1] = '0' + temp;
    }
}
/**********************************************************************
 *  Function:      void byte_to_string
 *  Description:   convert from uint8_t to string.			  
 *  Argument(s):   dst   - pointer to string
 *                 	data - uint8_t to be convert
 *  Return:        string length.
 ***********************************************************************/
uint8_t byte_to_string(uint8_t * dst, uint8_t data)
{
    uint8_t i = 0;
    //100
    data = data % 1000;
    dst[i] = '0' + data / 100;
    if(i > 0 || '0' != dst[0])
    {
        i ++;
    }
    //10
    data = data % 100;
    dst[i] = '0' + data / 10;
    if(i > 0 || '0' != dst[0])
    {
        i ++;
    }
    //1
    dst[i] = '0' + data % 10;
    i++;
    dst[i] = 0;
    return i;
    
}

/**********************************************************************
 *  Function:      void byte_to_string2
 *  Description:   convert from uint8_t to string. Eg:56 to "056". 			  
 *  Argument(s):   dst   - pointer to string
 *                 	data - uint8_t to be convert
 *  Return:        string length.
 ***********************************************************************/
uint8_t byte_to_string2(uint8_t * dst, uint8_t data)
{
    uint8_t i = 0;
    //100
    dst[i] = '0' + data / 100;
	i++;
    //10
    data = data % 100;
    dst[i] = '0' + data / 10;
	i++;
    //1
    dst[i] = '0' + data % 10;
	i++;
    dst[i] = 0;
    return i;
    
}
/*---Private----------------------------------------------------------*/




/**************** END OF FILE *****************************************/
