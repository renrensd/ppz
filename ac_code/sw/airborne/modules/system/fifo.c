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
#include "types.h" 

/*---Public include files---------------------------------------------*/

/*---Private include files--------------------------------------------*/
#include "fifo_if.h" 
#include "fifo.h"   

/*===VARIABLES========================================================*/

/*---Global-----------------------------------------------------------*/

/*---Private----------------------------------------------------------*/
 
/*===FUNCTIONS========================================================*/

/*---Global-----------------------------------------------------------*/
 
/*******************************************************************************
**  FUNCTION      : fifo_io_init                                         
**  DESCRIPTION   : This function initialize FIFO
**  PARAMETERS    : p_this
**                  pBuf
**                  nSize  
**  RETURN        : void                                                          
*******************************************************************************/
void fifo_io_init( FIFO_TYPE *p_this, U8 *pBuf, U16 nSize)
{
    p_this->mpFIFOBuffer = pBuf;
    p_this->mFIFOHead = 0x00;
	p_this->mFIFOTail = 0x00;
	p_this->mFIFOSize = nSize;
}

/*******************************************************************************
**  FUNCTION      : fifo_input_frame                                         
**  DESCRIPTION   : This function input a frame to fifo buffer
**  PARAMETERS    : p_this
**                  pFrame
**                  nBytes  
**  RETURN        : void                                                          
*******************************************************************************/
BOOL fifo_input_frame( FIFO_TYPE *p_this , U8 *pFrame , U8 length)
{
  	U8 i;
  	U8 *pTemp = pFrame;

    if(!fifo_have_enough_space(p_this, length))
    {
        return FALSE;
    }

    if(!fifo_input_byte(p_this, length))
    {
        return FALSE;
    }
    
    for (i = 0x00; i < length; i++) //input (1 +nbyte) byte to FIFO
    {    
    	if (!fifo_input_byte(p_this, *pTemp++))
    	{
    	    return FALSE;
    	}
    }
    return TRUE;
}

/*******************************************************************************
**  FUNCTION      : fifo_output_frame                                         
**  DESCRIPTION   : This function output a frame from FIFO buffer
**  PARAMETERS    : p_this
**                  pFrame
**                  nBytes  
**  RETURN        : void                                                          
*******************************************************************************/
BOOL fifo_output_frame( FIFO_TYPE *p_this, U8 *pFrame, U8 *pLength)
{
  	U8 i;
  	U8 tempVal;
    U16 tempLength;
//    U8 tempFrameLength;
    U8 *pTemp = pFrame;

    tempLength = fifo_current_length(p_this);  

    /*frist byte is the length of current frame*/
    /*if(fifo_output_byte(p_this, &tempFrameLength))
    {
        *pTemp++ = tempFrameLength;
    }
    else
    {
        return FALSE;
    }*/

    if(!fifo_output_byte(p_this, pLength))
    {
        return FALSE;
    }
        
        
    if(*pLength > tempLength || *pLength == 0x00)
    {
       p_this->mFIFOHead = p_this->mFIFOTail;
       return FALSE;
    }
    else
    {
        for (i = 0x00; i < *pLength; i++)
        {
        	if (fifo_output_byte(p_this, &tempVal))
        	{
        		*pTemp++ = tempVal;
        	}
    		else
    		{
    			return FALSE;
    		}
        }        
    }

	return TRUE;
}


/*---Private----------------------------------------------------------*/

/*******************************************************************************
**  FUNCTION      : fifo_is_empty                                         
**  DESCRIPTION   : This function check whether the FIFO is empty.
**  PARAMETERS    : p_this:
**  RETURN        : BOOL                                                          
*******************************************************************************/
BOOL fifo_is_empty(FIFO_TYPE *p_this  )
{
    if(p_this->mFIFOHead == p_this->mFIFOTail)
    {
        return TRUE;
    }
	else
	{
		return FALSE;
	}	
}

/*******************************************************************************
**  FUNCTION      : fifo_is_full                                         
**  DESCRIPTION   : This function check whether the FIFO is full.
**  PARAMETERS    : p_this
**  RETURN        : BOOL                                                          
*******************************************************************************/
static BOOL fifo_is_full(FIFO_TYPE *p_this  )
{
	if (p_this->mFIFOHead > 0x00)
	{
		if (p_this->mFIFOTail == (p_this->mFIFOHead - 0x01))
		{
			return TRUE;
		}
	}
	else if (p_this->mFIFOTail == (p_this->mFIFOSize - 0x01))
	{
		return TRUE;
	}
	
	return FALSE;
}

/*******************************************************************************
**  FUNCTION      : fifo_input_byte                                         
**  DESCRIPTION   : This function input a byte into FIFO.
**  PARAMETERS    : p_this
**                  byte
**  RETURN        : BOOL
*******************************************************************************/
static BOOL fifo_input_byte( FIFO_TYPE *p_this , unsigned char byte )
{
    if (!fifo_is_full(p_this))
    {
        if (p_this->mFIFOTail < p_this->mFIFOSize)
	    {
            p_this->mpFIFOBuffer[p_this->mFIFOTail] = byte;
            p_this->mFIFOTail++;

            if(p_this->mFIFOTail >= p_this->mFIFOSize)
            {
                p_this->mFIFOTail = 0x00;
            }
            return TRUE;
	    }
    }
    return FALSE;
}

/*******************************************************************************
**  FUNCTION      : fifo_input_byte                                         
**  DESCRIPTION   : This function input a byte into FIFO.
**  PARAMETERS    : p_this
**                  byte
**  RETURN        : BOOL
*******************************************************************************/
static BOOL fifo_output_byte(FIFO_TYPE *p_this, U8 *pdata)
{
    if (!fifo_is_empty(p_this))
    {
		if (p_this->mFIFOHead < p_this->mFIFOSize)
	    {
	    	*pdata = p_this->mpFIFOBuffer[p_this->mFIFOHead];
	    	p_this->mFIFOHead++;
            
            if(p_this->mFIFOHead >= p_this->mFIFOSize)
            {
			    p_this->mFIFOHead = 0x00;
            }
		    return TRUE;
		}
	}	
	return FALSE;
}

/*******************************************************************************
**  FUNCTION      : fifo_current_length                                         
**  DESCRIPTION   : This function return the current length of FIFO buffer
**  PARAMETERS    : p_this
**  RETURN        : U16
*******************************************************************************/
static U16 fifo_current_length(FIFO_TYPE *p_this  )
{
  	U16 retVal;

    if(p_this->mFIFOTail > p_this->mFIFOHead)
    {
        retVal = p_this->mFIFOTail - p_this->mFIFOHead;
    }
    else
    {
        if (p_this->mFIFOHead == p_this->mFIFOTail)
        {
            retVal = 0x00; 
        }
        else
        {
            retVal = p_this->mFIFOSize - (p_this->mFIFOHead - p_this->mFIFOTail);   
        }
    }
	return (retVal);
}

/*******************************************************************************
**  FUNCTION      : fifo_have_enough_space                                        
**  DESCRIPTION   : This function judge fifo buffer have enough space to
                    the current length of FIFO buffer
**  PARAMETERS    : p_this
**  RETURN        : U16
*******************************************************************************/
static U16 fifo_have_enough_space(FIFO_TYPE *p_this , U8 nBytes)
{
    /*p_this buffer have one byte canot be used.
      we should input one length byte + n data byte to FIFO buffer*/
    if((p_this->mFIFOSize - fifo_current_length(p_this) - 0x01) >=  (nBytes + 0x01))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**************** END OF FILE *****************************************/

