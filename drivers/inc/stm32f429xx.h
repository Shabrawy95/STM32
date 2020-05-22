 /********************************************************************************************************************************
 * File Name: stm32f429xx.h
 *
 * Description: source file for I2C module driver
 *
 * Author: Shabrawy
 ********************************************************************************************************************************/

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include "../../common/std_types.h"
#include "../../common/common_macros.h"

#include "cortex_m4.h"
/*********************************************************************************************************************************
 *                    								 Misc. Definition   						                             *
 ********************************************************************************************************************************/

#define __IO volatile
#define ENABLED 1
#define DISABLED 0



/*********************************************************************************************************************************
 *                    							IRQ Numbers   						                             *
 ********************************************************************************************************************************/


typedef enum{
	/*Cortex M-4 Specific Exception Numbers*/
	NonMaskableInt_IRQn         = -14,
	MemoryManagement_IRQn       = -12,
	BusFault_IRQn               = -11,
	UsageFault_IRQn             = -10,
	SVCall_IRQn                 = -5,
	DebugMonitor_IRQn           = -4,
	PendSV_IRQn                 = -2,
	Systick_IRQn                = -1,
	/*STM32 Specific Interrupt Numbers*/
	WWDG_IRQn                   =  0,
	PVD_IRQn                    =  1,
	TAMP_STAMP_IRQn             =  2,
	RTC_WKUP_IRQn               =  3,
	FLASH_IRQn                  =  4,
	RCC_IRQn                    =  5,
	EXTI0_IRQn                  =  6,
	EXTI1_IRQn                  =  7,
	EXTI2_IRQn                  =  8,
	EXTI3_IRQn                  =  9,
	EXTI4_IRQn                  = 10,
	DMA1_Stream0_IRQn           = 11,
	DMA1_Stream1_IRQn           = 12,
	DMA1_Stream2_IRQn           = 13,
	DMA1_Stream3_IRQn           = 14,
	DMA1_Stream4_IRQn           = 15,
	DMA1_Stream5_IRQn           = 16,
	DMA1_Stream6_IRQn           = 17,
	ADC_IRQn                    = 18,
	CAN1_TX_IRQn                = 19,
	CAN1_RX0_IRQn               = 20,
	CAN1_RX1_IRQn               = 21,
	CAN1_SCE_IRQn               = 22,
	EXTI9_5_IRQn                = 23,
	TIM1_BRK_TIM9_IRQn          = 24,
	TIM1_UP_TIM10_IRQn          = 25,
	TIM1_TRG_COM_TIM11_IRQn     = 26,
	TIM1_CC_IRQn                = 27,
	TIM2_IRQn                   = 28,
	TIM3_IRQn                   = 29,
	TIM4_IRQn                   = 30,
	I2C1_EV_IRQn                = 31,
	I2C1_ER_IRQn                = 32,
	I2C2_EV_IRQn                = 33,
	I2C2_ER_IRQn                = 34,
	SPI1_IRQn                   = 35,
	SPI2_IRQn                   = 36,
	USART1_IRQn                 = 37,
	USART2_IRQn                 = 38,
	USART3_IRQn                 = 39,
	EXTI15_10_IRQn              = 40,
	RTC_Alarm_IRQn              = 41,
	OTG_FS_WKUP_IRQn            = 42,
	TIM8_BRK_TIM12_IRQn         = 43,
	TIM8_UP_TIM13_IRQn          = 44,
	TIM8_TRG_COM_TIM14_IRQn     = 45,
	TIM8_CC_IRQn                = 46,
	DMA1_Stream7_IRQn           = 47,
	FSMC_IRQn                   = 48,
	SDIO_IRQn                   = 49,
	TIM5_IRQn                   = 50,
	SPI3_IRQn                   = 51,
	UART4_IRQn                  = 52,
	UART5_IRQn                  = 53,
	TIM6_DAC_IRQn               = 54,
	TIM7_IRQn                   = 55,
	DMA2_Stream0_IRQn           = 56,
	DMA2_Stream1_IRQn           = 57,
	DMA2_Stream2_IRQn           = 58,
	DMA2_Stream3_IRQn           = 59,
	DMA2_Stream4_IRQn           = 60,
	ETH_IRQn                    = 61,
	ETH_WKUP_IRQn               = 62,
	CAN2_TX_IRQn                = 63,
	CAN2_RX0_IRQn               = 64,
	CAN2_RX1_IRQn               = 65,
	CAN2_SCE_IRQn               = 66,
	OTG_FS_IRQn                 = 67,
	DMA2_Stream5_IRQn           = 68,
	DMA2_Stream6_IRQn           = 69,
	DMA2_Stream7_IRQn           = 70,
	USART6_IRQn                 = 71,
	I2C3_EV_IRQn                = 72,
	I2C3_ER_IRQn                = 73,
	OTG_HS_EP1_OUT_IRQn         = 74,
	OTG_HS_EP1_IN_IRQn          = 75,
	OTG_HS_WKUP_IRQn            = 76,
	OTG_HS_IRQn                 = 77,
	DCMI_IRQn                   = 78,
	CRYP_IRQn                   = 79,
	HASH_RNG_IRQn               = 80,
	FPU_IRQn                    = 81,
	UART7_IRQn                  = 82,
	UART8_IRQn                  = 83,
	SPI4_IRQn                   = 84,
	SPI5_IRQn                   = 85,
	SPI6_IRQn                   = 86,
	SAI1_IRQn                   = 87,
	LTDC_IRQn                   = 88,
	LCTDC_ER_IRQn               = 89,
	DMA2D_IRQn                  = 90

}IRQn_Type;



typedef enum{
	DISABLE,
	ENABLE
}IRQn_EN_DIS;


/*********************************************************************************************************************************
 *                    							Peripheral Register Structure Definition   						                             *
 ********************************************************************************************************************************/

typedef struct {
	__IO uint32 MODER;
	__IO uint32 OTYPER;
	__IO uint32 OSPEEDR;
	__IO uint32 PUPDR;
	__IO uint32 IDR;
	__IO uint32 ODR;
	__IO uint32 BSSR;
	__IO uint32 LCKR;
	__IO uint32 AFR[2];

}GPIO_TypeDef;


typedef struct {
	__IO uint32 CR;
	__IO uint32 PLLCFGR;
	__IO uint32 CFGR;
	__IO uint32 CIR;
	__IO uint32 AHB1RSTR;
	__IO uint32 AHB2RSTR;
	__IO uint32 AHB3RSTR;
    uint32	    RESERVED0;
	__IO uint32 APB1RSTR;
	__IO uint32 APB2RSTR;
    uint32	    RESERVED1[2];
	__IO uint32 AHB1ENR;
	__IO uint32 AHB2ENR;
	__IO uint32 AHB3ENR;
    uint32	    RESERVED2;
	__IO uint32 APB1ENR;
	__IO uint32 APB2ENR;
    uint32 		RESERVED3[2];
	__IO uint32 AHB1LPENR;
	__IO uint32 AHB2LPENR;
	__IO uint32 AHB3LPENR;
	uint32	    RESERVED4;
	__IO uint32 APB1LPENR;
	__IO uint32 APB2LPENR;
	uint32 		RESERVED5[2];
	__IO uint32 BDCR;
	__IO uint32 CSR;
	uint32 		RESERVED6[2];
	__IO uint32 SSCGR;
	__IO uint32 PLLI2SCFGR;
	__IO uint32 PLLSAICFGR;
	__IO uint32 RCC_DCKCFGR;

}RCC_TypeDef;

typedef struct{
	__IO uint32 IMR;
	__IO uint32 EMR;
	__IO uint32 RTSR;
	__IO uint32 FTSR;
	__IO uint32 SWIER;
	__IO uint32 PR;
}EXTI_TypeDef;

typedef struct{
	__IO uint32 MEMRMP;
	__IO uint32 PMC;
	__IO uint32 EXTICR[4];
	uint32 		RESERVED[2];
	__IO uint32 CMPCR;
}SYSCFG_TypeDef;

typedef struct{
	__IO uint32 CR1;
	__IO uint32 CR2;
	__IO uint32 SR;
	__IO uint32 DR;
	__IO uint32 CRCPR;
	__IO uint32 RXCRCR;
	__IO uint32 TXCRCR;
	__IO uint32 I2SCFGR;
	__IO uint32 I2SPR;
}SPI_TypeDef;


typedef struct{
	__IO uint32 CR1;
	__IO uint32 CR2;
	__IO uint32 OAR1;
	__IO uint32 OAR2;
	__IO uint32 DR;
	__IO uint32 SR1;
	__IO uint32 SR2;
	__IO uint32 CCR;
	__IO uint32 TRISE;
	__IO uint32 FLTR;
}I2C_TypeDef;

typedef struct{
	__IO uint32 SR;
	__IO uint32 DR;
	__IO uint32 BRR;
	__IO uint32 CR1;
	__IO uint32 CR2;
	__IO uint32 CR3;
	__IO uint32 GTPR;
}USART_TypeDef;

/*********************************************************************************************************************************
 *                    								 Memory Address Defintions    						                             *
 ********************************************************************************************************************************/

#define FLASH_BASE_ADDRESS				0x08000000U
#define ROM_BASE_ADDRESS				0x1FFF0000U
#define OTP_BASE_ADDRESS				0x1FFF7800U
#define OPTION_BYTES_BASE_ADDRESS		0x1FFFC000U
#define SRAM1_BASE_ADDRESS				0x20000000U
#define SRAM2_BASE_ADDRESS				0x2001C000U
#define SRAM3_BASE_ADDRESS				0x20020000U

#define SRAM							SRAM1_BASE_ADDRESS

/*********************************************************************************************************************************
 *                    								 Bus Address Defintions    						                             *
 ********************************************************************************************************************************/
#define PERIPH_BASE					    0x40000000U
#define AHB1PERIPH_BASE					(PERIPH_BASE+0x00020000U)
#define AHB2PERIPH_BASE					(PERIPH_BASE+0x10000000U)
#define AHB3PERIPH_BASE					(PERIPH_BASE+0x60000000U)
#define APB1PERIPH_BASE					(PERIPH_BASE+0x00000000U)
#define APB2PERIPH_BASE					(PERIPH_BASE+0x00010000U)

/*********************************************************************************************************************************
 *                    								 APB1 Peripheral Address Defintions    						                             *
 ********************************************************************************************************************************/

#define TIM2_BASE						(APB1PERIPH_BASE+0x0000U)
#define TIM3_BASE						(APB1PERIPH_BASE+0x0400U)
#define TIM4_BASE						(APB1PERIPH_BASE+0x0800U)
#define TIM5_BASE						(APB1PERIPH_BASE+0x0C00U)
#define TIM6_BASE						(APB1PERIPH_BASE+0x1000U)
#define TIM7_BASE						(APB1PERIPH_BASE+0x1400U)
#define TIM12_BASE						(APB1PERIPH_BASE+0x1800U)
#define TIM13_BASE						(APB1PERIPH_BASE+0x1C00U)
#define TIM14_BASE						(APB1PERIPH_BASE+0x2000U)
#define RTC_BASE                        (APB1PERIPH_BASE+0x2800U)
#define WWDG_BASE                       (APB1PERIPH_BASE+0x2C00U)
#define IWDG_BASE                       (APB1PERIPH_BASE+0x3000U)
#define I2Sext_BASE                     (APB1PERIPH_BASE+0x3400U)
#define SPI2_BASE                       (APB1PERIPH_BASE+0x3800U)
#define SPI3_BASE                       (APB1PERIPH_BASE+0x3C00U)
#define I2S3ext_BASE                    (APB1PERIPH_BASE+0x4000U)
#define USART2_BASE                     (APB1PERIPH_BASE+0x4400U)
#define USART3_BASE                     (APB1PERIPH_BASE+0x4800U)
#define UART4_BASE                      (APB1PERIPH_BASE+0x4C00U)
#define UART5_BASE                      (APB1PERIPH_BASE+0x5000U)
#define I2C1_BASE                       (APB1PERIPH_BASE+0x5400U)
#define I2C2_BASE                       (APB1PERIPH_BASE+0x5800U)
#define I2C3_BASE                       (APB1PERIPH_BASE+0x5C00U)
#define CAN1_BASE                       (APB1PERIPH_BASE+0x6400U)
#define CAN2_BASE                       (APB1PERIPH_BASE+0x6800U)
#define PWR_BASE                        (APB1PERIPH_BASE+0x7000U)
#define DAC_BASE                        (APB1PERIPH_BASE+0x7400U)
#define UART7_BASE						(APB1PERIPH_BASE+0x7800U)
#define UART8_BASE                      (APB1PERIPH_BASE+0x7C00U)




/*********************************************************************************************************************************
 *                    								 APB2 Peripheral Address Defintions    						                             *
 ********************************************************************************************************************************/
#define TIM1_BASE						(APB2PERIPH_BASE+0x0000U)
#define TIM8_BASE						(APB2PERIPH_BASE+0x0400U)
#define USART1_BASE						(APB2PERIPH_BASE+0x1000U)
#define USART6_BASE						(APB2PERIPH_BASE+0x1400U)
#define ADC1_BASE						(APB2PERIPH_BASE+0x2000U)
#define ADC2_BASE						(APB2PERIPH_BASE+0x2100U)
#define ADC3_BASE						(APB2PERIPH_BASE+0x2200U)
#define ADC123_COMMON					(APB2PERIPH_BASE+0x2300U)
#define SDIO_BASE						(APB2PERIPH_BASE+0x2C00U)
#define SPI1_BASE						(APB2PERIPH_BASE+0x3000U)
#define SPI4_BASE                       (APB2PERIPH_BASE+0x3400U)
#define SYSCFG_BASE                     (APB2PERIPH_BASE+0x3800U)
#define EXTI_BASE                       (APB2PERIPH_BASE+0x3C00U)
#define TIM9_BASE                       (APB2PERIPH_BASE+0x4000U)
#define TIM10_BASE                      (APB2PERIPH_BASE+0x4400U)
#define TIM11_BASE                      (APB2PERIPH_BASE+0x4800U)
#define SPI5_BASE                    	(APB2PERIPH_BASE+0x5000U)
#define SPI6_BASE                     	(APB2PERIPH_BASE+0x5400U)
#define SAI1_BASE                     	(APB2PERIPH_BASE+0x5800U)
#define LTDC_BASE                       (APB2PERIPH_BASE+0x6800U)


/*********************************************************************************************************************************
 *                    								 AHB1 Peripheral Address Defintions    						                             *
 ********************************************************************************************************************************/

#define	GPIOA_BASE						(AHB1PERIPH_BASE + 0x0000U)
#define	GPIOB_BASE                      (AHB1PERIPH_BASE + 0x0400U)
#define	GPIOC_BASE                      (AHB1PERIPH_BASE + 0x0800U)
#define	GPIOD_BASE                      (AHB1PERIPH_BASE + 0x0C00U)
#define	GPIOE_BASE                      (AHB1PERIPH_BASE + 0x1000U)
#define	GPIOF_BASE                      (AHB1PERIPH_BASE + 0x1400U)
#define	GPIOG_BASE                      (AHB1PERIPH_BASE + 0x1800U)
#define	GPIOH_BASE                      (AHB1PERIPH_BASE + 0x1C00U)
#define	GPIOI_BASE                      (AHB1PERIPH_BASE + 0x2000U)
#define	GPIOJ_BASE                      (AHB1PERIPH_BASE + 0x2400U)
#define	GPIOK_BASE                      (AHB1PERIPH_BASE + 0x2800U)
#define CRC_BASE                        (AHB1PERIPH_BASE + 0x3000U)
#define RCC_BASE                        (AHB1PERIPH_BASE + 0x3800U)
#define FLASH_IF_BASE                   (AHB1PERIPH_BASE + 0x3C00U)
#define BKPSRAM_BASE                    (AHB1PERIPH_BASE + 0x4000U)
#define DMA1_BASE                       (AHB1PERIPH_BASE + 0x6000U)
#define DMA2_BASE                       (AHB1PERIPH_BASE + 0x6400U)
#define ETHERNET_BASE                   (AHB1PERIPH_BASE + 0x8000U)
#define DMA2D_BASE                      (AHB1PERIPH_BASE + 0xB000U)

#define USB_OTG_HS_BASE         		(AHB1PERIPH_BASE + 0x20000U)

/*********************************************************************************************************************************
 *                    								 AHB2 Peripheral Address Defintions    						                             *
 ********************************************************************************************************************************/

#define USB_OTG_FS_BASE					(AHB2PERIPH_BASE + 0x00000U)
#define DCMI_BASE						(AHB2PERIPH_BASE + 0x50000U)
#define CRYP_BASE						(AHB2PERIPH_BASE + 0x60000U)
#define HASH_BASE						(AHB2PERIPH_BASE + 0x60400U)
#define RNG_BASE						(AHB2PERIPH_BASE + 0x60800U)

/*********************************************************************************************************************************
 *                    								 AHB3 Peripheral Address Defintions    						                             *
 ********************************************************************************************************************************/

#define FMSC_BASE						(AHB3PERIPH_BASE)

/*********************************************************************************************************************************
 *                    								 Peripheral Structure casting   						                             *
 ********************************************************************************************************************************/

#define	GPIOA                  ((GPIO_TypeDef*)GPIOA_BASE)
#define	GPIOB                  ((GPIO_TypeDef*)GPIOB_BASE)
#define	GPIOC                  ((GPIO_TypeDef*)GPIOC_BASE)
#define	GPIOD                  ((GPIO_TypeDef*)GPIOD_BASE)
#define	GPIOE                  ((GPIO_TypeDef*)GPIOE_BASE)
#define	GPIOF                  ((GPIO_TypeDef*)GPIOF_BASE)
#define	GPIOG                  ((GPIO_TypeDef*)GPIOG_BASE)
#define	GPIOH                  ((GPIO_TypeDef*)GPIOH_BASE)
#define	GPIOI                  ((GPIO_TypeDef*)GPIOI_BASE)
#define	GPIOJ                  ((GPIO_TypeDef*)GPIOJ_BASE)
#define	GPIOK				   ((GPIO_TypeDef*)GPIOK_BASE)

#define RCC					   ((RCC_TypeDef*)RCC_BASE)
#define EXTI				   ((EXTI_TypeDef*)EXTI_BASE)
#define SYSCFG				   ((SYSCFG_TypeDef*)SYSCFG_BASE)

#define SPI1				   ((SPI_TypeDef* )SPI1_BASE)
#define SPI2				   ((SPI_TypeDef* )SPI2_BASE)
#define SPI3      			   ((SPI_TypeDef* )SPI3_BASE)
#define SPI4     			   ((SPI_TypeDef* )SPI4_BASE)
#define SPI5    	   	       ((SPI_TypeDef* )SPI5_BASE)
#define SPI6   				   ((SPI_TypeDef* )SPI6_BASE)

#define I2C1				   ((I2C_TypeDef* )I2C1_BASE)
#define I2C2				   ((I2C_TypeDef* )I2C2_BASE)
#define I2C3				   ((I2C_TypeDef* )I2C3_BASE)

#define USART1		    	   ((USART_TypeDef* )USART1_BASE)
#define USART2          	   ((USART_TypeDef* )USART2_BASE)
#define USART3          	   ((USART_TypeDef* )USART3_BASE)
#define UART4           	   ((USART_TypeDef* )UART4_BASE)
#define UART5           	   ((USART_TypeDef* )UART5_BASE)
#define USART6                 ((USART_TypeDef* )USART6_BASE)
#define UART7           	   ((USART_TypeDef* )UART7_BASE)
#define UART8           	   ((USART_TypeDef* )UART8_BASE)


/*********************************************************************************************************************************
 *                    								 Peripheral Bit position Macros  						                             *
 ********************************************************************************************************************************/

/*       SPI        */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE 	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*		I2C		*/
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE         3
#define I2C_CR1_ENARP           4
#define I2C_CR1_ENPEC           5
#define I2C_CR1_ENGC            6
#define I2C_CR1_NOSTRETCH       7
#define I2C_CR1_START           8
#define I2C_CR1_STOP            9
#define I2C_CR1_ACK             10
#define I2C_CR1_POS             11
#define I2C_CR1_PEC             12
#define I2C_CR1_ALERT           13
#define I2C_CR1_SWRST           15


#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN         9
#define I2C_CR2_ITBUFEN         10
#define I2C_CR2_DMAEN           11
#define I2C_CR2_LADT            12

#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD		    1
#define I2C_OAR1_ADDMODE        15

#define I2C_OAR2_ENDUAL			0
#define I2C_OAR2_ADD2		    1


#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF        	    2
#define I2C_SR1_ADD10           3
#define I2C_SR1_STOPF           4
#define I2C_SR1_RXNE            6
#define I2C_SR1_TXE             7
#define I2C_SR1_BERR            8
#define I2C_SR1_ARLO            9
#define I2C_SR1_AF              10
#define I2C_SR1_OVR             11
#define I2C_SR1_PECERR          12
#define I2C_SR1_TIMEOUT         14
#define I2C_SR1_SMBALERT



#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA       	    2
#define I2C_SR2_GENCALL         4
#define I2C_SR2_SMBDEFAULT      5
#define I2C_SR2_SMBHOST         6
#define I2C_SR2_DUALF           7
#define I2C_SR2_PEC             8


#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS       	    15  /*********************/

#define I2C_TRISE_TRISE			0

#define I2C_FLTR_DNF			0
#define I2C_FLTR_ANOFF			4

/*			USART		*/

#define USART_SR_PE			0
#define USART_SR_FE         1
#define USART_SR_NF         2
#define USART_SR_ORE        3
#define USART_SR_IDLE       4
#define USART_SR_RXNE       5
#define USART_SR_TC         6
#define USART_SR_TXE        7
#define USART_SR_LBD        8
#define USART_SR_CTS        9


#define USART_BRR_DIV_Fraction		0
#define USART_BRR_DIV_Mantissa		4

#define USART_CR1_SBK				0
#define USART_CR1_RWU               1
#define USART_CR1_RE                2
#define USART_CR1_TE                3
#define USART_CR1_IDLEIE            4
#define USART_CR1_RXNEIE            5
#define USART_CR1_TCIE              6
#define USART_CR1_TXEIE             7
#define USART_CR1_PEIE              8
#define USART_CR1_PS                9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15


#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14


#define USART_CR3_EIE				0
#define USART_CR3_IREN              1
#define USART_CR3_IRLP              2
#define USART_CR3_HDSEL             3
#define USART_CR3_NACK              4
#define USART_CR3_SCEN              5
#define USART_CR3_DMAR              6
#define USART_CR3_DMAT              7
#define USART_CR3_RTSE              8
#define USART_CR3_CTSE              9
#define USART_CR3_CTSIE             10
#define USART_CR3_ONEBIT            11

#define USART_GTPR_PSC				0
#define USART_GTPR_GT				8


#endif /* INC_STM32F429XX_H_ */
