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
#include "..\INC\FIFO_U16_IF.H" 
#include "..\INC\FIFO_U16.H"   


/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/



/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
/*******************************************************************************
**  FUNCTION      : fifo_init_u16                                         
**  DESCRIPTION   : This function init u16 fifo buffer
**  PARAMETERS    : FIFO_U16_TYPE *this
**                  U16 *pBuf
**                  U16 nSize  
**  RETURN        : void                                                          
*******************************************************************************/
void fifo_init_u16(FIFO_U16_TYPE *this, U16 *pBuf, U16 nSize)
{
    this->mpFIFOBuffer = pBuf;
    this->mFIFOHead = 0x00;
	this->mFIFOTail = 0x00;
	this->mFIFOSize = nSize;
}

/*******************************************************************************
**  FUNCTION      : fifo_input_u16                                         
**  DESCRIPTION   : This function input u16 to u16 fifo buffer
**  PARAMETERS    : FIFO_U16_TYPE *this
**                  U16 input_data
**  RETURN        : BOOL                                                          
*******************************************************************************/
BOOL fifo_input_u16(FIFO_U16_TYPE *this, U16 input_data)
{
  	BOOL ret_val = FALSE;
    if (!fifo_u16_is_full(this))
    {
	    if (this->mFIFOTail < this->mFIFOSize)
	    {
	    	this->mpFIFOBuffer[this->mFIFOTail] = input_data;
	    	this->mFIFOTail++;

			if(this->mFIFOTail >= this->mFIFOSize)
            {
                this->mFIFOTail = 0;
            }
	        ret_val = TRUE;
		}		
	}
	return (ret_val);
}

/*******************************************************************************
**  FUNCTION      : fifo_output_u16                                         
**  DESCRIPTION   : This function output u16 from u16 fifo buffer
**  PARAMETERS    : FIFO_U16_TYPE *this
**                  U16 *pdata
**  RETURN        : BOOL                                                          
*******************************************************************************/
BOOL fifo_output_u16(FIFO_U16_TYPE *this, U16 *pdata)
{
  	BOOL ret_val = FALSE;
    if (!fifo_u16_is_empty(this))
    {
		if (this->mFIFOHead < this->mFIFOSize)
	    {
	    	*pdata = this->mpFIFOBuffer[this->mFIFOHead];
	    	this->mFIFOHead++;
            
            if(this->mFIFOHead >= this->mFIFOSize)
            {
			    this->mFIFOHead = 0;
            }
		    ret_val = TRUE;
		}
	}	
	return (ret_val);
}

/*******************************************************************************
**  FUNCTION      : fifo_u16_is_empty                                         
**  DESCRIPTION   : This function judge u16 fifo buffer is empty or not
**  PARAMETERS    : FIFO_U16_TYPE *this
**  RETURN        : BOOL                                                          
*******************************************************************************/
BOOL fifo_u16_is_empty(FIFO_U16_TYPE *this)
{
  	BOOL ret_val;
    if (this->mFIFOHead == this->mFIFOTail)
    {
    	ret_val = TRUE;
    }
	else
	{
		ret_val = FALSE;
	}
	return (ret_val);
}

/*---Private----------------------------------------------------------*/
/*******************************************************************************
**  FUNCTION      : fifo_u16_is_full                                         
**  DESCRIPTION   : This function judge u16 fifo buffer is full or not
**  PARAMETERS    : FIFO_U16_TYPE *this
**  RETURN        : BOOL                                                          
*******************************************************************************/
static BOOL fifo_u16_is_full(FIFO_U16_TYPE *this)
{
	BOOL ret_val = FALSE;

	if (this->mFIFOHead > 0)
	{
		if (this->mFIFOTail == (this->mFIFOHead - 1))
		{
			ret_val = TRUE;
		}
	}
	else if (this->mFIFOTail == (this->mFIFOSize - 1))
	{
		ret_val = TRUE;
	}
	
	return (ret_val);
}
/**************** END OF FILE *****************************************/
