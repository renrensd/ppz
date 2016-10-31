/***********************************************************************
*   Copyright (C) Shenzhen Efficien Tech Co., Ltd.				       *
*				  All Rights Reserved.          					   *
*   Department : R&D SW      									       *
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

/**** Definition of constants ****/


/**** Definition of types ****/ 


/**** Definition of macros ****/
#ifndef _FRAM_CLASS_H_
#define _FRAM_CLASS_H_

#define FRAM_PASS_1
#define DUMMY_GAP(x)	fram_##x
#define CONDITIONAL_CLASSES

#endif //_FRAM_CLASS_H_

#ifdef FRAM_PASS_9
    #undef  FRAM_PASS_9
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START 
    #define CLASS(c,q,s,d)
    #define FRAM_END 
#endif

#ifdef FRAM_PASS_8
    #undef  FRAM_PASS_8
    #define FRAM_PASS_9
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       const unsigned int object_base[CL_QTY] = {
    #define CLASS(c,q,s,d)     offsetof(struct fram_base_struct,fram_##c),
    #define FRAM_END         };
#endif


#ifdef FRAM_PASS_7
    #undef  FRAM_PASS_7
    #define FRAM_PASS_8
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       const unsigned char object_quantity[CL_QTY] = {
    #define CLASS(c,q,s,d)      q,
    #define FRAM_END         };
#endif

#ifdef FRAM_PASS_6
    #undef  FRAM_PASS_6
    #define FRAM_PASS_7
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       const unsigned int object_size[CL_QTY] = {
    #define CLASS(c,q,s,d)      s,
    #define FRAM_END         };
#endif


#ifdef FRAM_PASS_5   
    #undef  FRAM_PASS_5
    #define FRAM_PASS_6
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       struct fram_base_struct {
    #define CLASS(c,q,s,d)      unsigned char fram_##c[QUANTITY_##c * SIZE_##c]; 
    #define FRAM_END         };    
	
#endif

#ifdef FRAM_PASS_4
    #undef  FRAM_PASS_4
    #define FRAM_PASS_5
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       const uint8_t* const cl_data_array[]={  
    #define CLASS(c,q,s,d)    d,
    #define FRAM_END         };
#endif

#ifdef FRAM_PASS_31
    #undef  FRAM_PASS_31
    #define FRAM_PASS_4
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START           
    #define CLASS(c,q,s,d)     extern const uint8_t d[];
    #define FRAM_END       
#endif

#ifdef FRAM_PASS_3
    #undef  FRAM_PASS_3
    #define FRAM_PASS_31
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       enum FRAM_SIZE {       
    #define CLASS(c,q,s,d)       SIZE_##c = s,
    #define FRAM_END         };
#endif

#ifdef FRAM_PASS_2
    #undef  FRAM_PASS_2
    #define FRAM_PASS_3
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       enum FRAM_QUANTITY {       
    #define CLASS(c,q,s,d)       QUANTITY_##c = q,
    #define FRAM_END         };
#endif

#ifdef FRAM_PASS_1
    #undef  FRAM_PASS_1
#ifdef FRAM_MACROS
    #define FRAM_PASS_2
#else
    #define FRAM_PASS_7
#endif
    #undef  FRAM_START
    #undef  CLASS
    #undef  FRAM_END
    #define FRAM_START       enum FRAM_IDS {
    #define CLASS(c,q,s,d)       c,
    #define FRAM_END         CL_QTY };   
#endif
