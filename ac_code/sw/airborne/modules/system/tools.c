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
#include "tools.h"
#include "tools_if.h"




/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/

uint8_t get_the_maximum(uint8_t *src,uint8_t size)
{
	uint32_t i=0;
	uint8_t max = src[0];
	for(i=1; i<size; i++)
	{
		if(max <src[i])
		{
			max = src[i];
		}
	}
	return max;
}


uint8_t get_the_minimum(uint8_t *src,uint8_t size)
{
	uint32_t i=0;
	uint8_t min = src[0];
	for(i=1; i<size; i++)
	{
		if(min >src[i])
		{
			min = src[i];
		}
	}
	return min;
}

/***********************************************************************
*  Name        : get_size_of_string
*  Description : get size of string.
*  Parameter   : src   - string address
*  Returns     : size of string.
***********************************************************************/
uint8_t get_size_of_string(const uint8_t * src)
{
	uint8_t size = 0;
	while(*src != 0)
	{
		size ++;
		src ++;
	}
	return size;
}


bool_t  is_same_in_array(uint8_t * array,uint8_t size)
{
	uint8_t index;
	for(index = size -1 ; index>0 ; index--)
	{
		if(array[index] != array[index -1] )
		{
			return FALSE;
		}
	}
	return TRUE;
}
/*---Private----------------------------------------------------------*/




/**************** END OF FILE *****************************************/
