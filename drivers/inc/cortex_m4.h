/*
 * cortex_m4.h
 *
 *  Created on: Apr 20, 2020
 *      Author: Shabrawy
 */

#ifndef INC_CORTEX_M4_H_
#define INC_CORTEX_M4_H_

#include "../../common/std_types.h"

#define __IO volatile
#define NO_OF_PRIO_BITS 4
/*********************************************************************************************************************************
 *                    								NVIC Registers Defintion     						                                 *
 ********************************************************************************************************************************/


/*NVIC Registers*/

typedef struct {
	__IO uint32 ISER[8];
	uint32		RESERVED0[24];
	__IO uint32 ICER[8];
	uint32		RESERVED1[24];
	__IO uint32 ISPR[8];
	uint32		RESERVED2[24];
	__IO uint32 ICPR[8];
	uint32		RESERVED3[24];
	__IO uint32 IABR[8];
	uint32		RESERVED4[56];
	__IO uint32 IPR[60];
	uint32		RESERVED5[644];
	__IO uint32 STIR;
}NVIC_TypeDef;


#define NVIC_BASE		(0xE000E100)
#define NVIC			((NVIC_TypeDef*)NVIC_BASE)

/*********************************************************************************************************************************
 *                    								IRQ Priority Defintion     						                                 *
 ********************************************************************************************************************************/

#define NVIC_IRQ_PRIO0                  0
#define NVIC_IRQ_PRIO1                  1
#define NVIC_IRQ_PRIO2                  2
#define NVIC_IRQ_PRIO3                  3
#define NVIC_IRQ_PRIO4                  4
#define NVIC_IRQ_PRIO5                  5
#define NVIC_IRQ_PRIO6                  6
#define NVIC_IRQ_PRIO7                  7
#define NVIC_IRQ_PRIO8                  8
#define NVIC_IRQ_PRIO9                  9
#define NVIC_IRQ_PRIO10                 10
#define NVIC_IRQ_PRIO11                 11
#define NVIC_IRQ_PRIO12                 12
#define NVIC_IRQ_PRIO13                 13
#define NVIC_IRQ_PRIO14                 14
#define NVIC_IRQ_PRIO15                 15


#endif /* INC_CORTEX_M4_H_ */
