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
#include "..\..\CONFIG\INC\CONFIG.H"
#include "..\..\CONFIG\INC\TYPES.H"   


/*---Public include files---------------------------------------------*/

/*---Private include files--------------------------------------------*/
#include "..\INC\TYPE_CONV.H"   
#include "..\INC\TYPE_CONV_IF.H"   




/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/***********************************************************************
*  Name        : type_conv_bcd_to_hex
*  Description : change bcd code to hex code        
*  Parameter   : U8 bcd
*  Returns     : U8 hex
***********************************************************************/
U8 type_conv_bcd_to_hex(U8 bcd)
{
    U8 hex;
    U8 temp;

    temp = bcd >> 0x04;
    hex = (temp << 0x03) + (temp << 0x01) + (bcd & 0x0F);

    return hex;
}

/***********************************************************************
*  Name        : type_conv_hex_to_bcd
*  Description : change hex code to bcd code        
*  Parameter   : U8 hex
*  Returns     : U8 bcd
***********************************************************************/
U8 type_conv_hex_to_bcd(U8 hex)
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
*  Parameter   : U8 hex
*  Returns     : U8 ascii
***********************************************************************/
U8 type_conv_hex_to_ascii(U8 hex)
{
    U8 ascii;

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
*  Parameter   : U8 ascii
*  Returns     : U8 hex
***********************************************************************/
U8 type_conv_ascii_to_hex(U8 ascii)
{
    U8 hex;

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
*  Description : change U16 to U8
*  Parameter   : U16 , U8*
*  Returns     : void 
***********************************************************************/
void type_conv_u16_to_u8(U16 u16, U8* u8)
{
    *u8++ = (U8)(u16 >> 0x08);

    *u8 = (U8)u16;
}

/***********************************************************************
*  Name        : type_conv_u8_to_u16
*  Description : change U8 to U16
*  Parameter   : U8*
*  Returns     : U16
***********************************************************************/
U16 type_conv_u8_to_u16(U8 *u8)
{
    U16 u16;

    u16 = (*u8 << 0x08) + *(u8 + 0x01);

    return u16;
}    
/**********************************************************************
 *  Function:      void byte_to_hex_string
 *  Description:   convert from U8 to hex string.			  
 *  Argument(s):   dst   - pointer to string
 *                 	data - U8 to be convert
 *  Return:        void.
 ***********************************************************************/
void byte_to_hex_string(U8 * dst, U8 data)
{
    U8 temp = 0;
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
 *  Description:   convert from U8 to string.			  
 *  Argument(s):   dst   - pointer to string
 *                 	data - U8 to be convert
 *  Return:        string length.
 ***********************************************************************/
U8 byte_to_string(U8 * dst, U8 data)
{
    U8 i = 0;
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

/*---Private----------------------------------------------------------*/




/**************** END OF FILE *****************************************/
