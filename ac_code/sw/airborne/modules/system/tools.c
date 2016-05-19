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
#include "..\INC\TOOLS.H"   
#include "..\INC\TOOLS_IF.H"   




/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/

/***********************************************************************
*  Name        : get_ram_bitband
*  Description : get mcu_ram_bitband address
*  Parameter   : U8 *a  the address of the variable
*  Returns     : None
***********************************************************************/
RAM_BIT_BAND_TYPE* get_ram_bitband( U8 *a )
{
    return( RAM_BIT_BAND_TYPE* )( 0x22000000 + ( ( int )a - 0x20000000 ) * 32 );
}  


U8 get_the_maximum(U8 *src,U8 size)
{
    U32 i=0;
    U8 max = src[0];
    for(i=1;i<size;i++)
    {
       if(max <src[i])
       {
           max = src[i];
       }
    }
    return max;
}


U8 get_the_minimum(U8 *src,U8 size)
{
    U32 i=0;
    U8 min = src[0];
    for(i=1;i<size;i++)
    {
       if(min >src[i])
       {
           min = src[i];
       }
    }
    return min;
}

/*---Private----------------------------------------------------------*/




/**************** END OF FILE *****************************************/
