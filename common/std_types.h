/*********************************************************************************************************************************
 *  Filename: std_types.h
 *
 *  Description: Type Abstraction for AVR
 *
 *  Created on: Dec 27, 2019
 *
 *  Author: Shabrawy
 ********************************************************************************************************************************/

#ifndef STD_TYPES_H_
#define STD_TYPES_H_

/*Boolean data types*/
typedef unsigned char bool;

/*Boolean VALUES*/
#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef LOW
#define LOW (0U)
#endif

#ifndef HIGH
#define HIGH (1U)
#endif

#define NULL_PTR ((void *)0)

typedef unsigned char uint8;
typedef signed char sint8;
typedef unsigned short uint16;
typedef signed short sint16;
typedef unsigned long uint32;
typedef signed long sint32;
typedef unsigned long long uint64;
typedef signed long long sint64;
typedef float float32;
typedef double float64;



#endif /* STD_TYPES_H_ */
