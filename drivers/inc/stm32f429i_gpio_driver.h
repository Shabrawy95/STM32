 /********************************************************************************************************************************
 * Module: GPIO
 *
 * File Name: stm32f429i_gpio_driver.h
 *
 * Description: Header file for GPIO Driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/

#ifndef STM32F4XX_HAL_DRIVER_INC_STM32F429I_GPIO_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_STM32F429I_GPIO_DRIVER_H_

/*MCU Specific Header File*/
#include "stm32f429xx.h"

/*********************************************************************************************************************************
 *                    								  GPIO Type Defintion     						                                 *
 ********************************************************************************************************************************/

/*For enable peripheral clock in RCC register API*/
typedef enum{
	GPIO_DIS,
	GPIO_EN

}GPIO_CLK;

/*Datatype for gpio pin mode configuration*/
typedef enum {
	GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_FUNC, GPIO_ANALOG
}GPIO_pinMode;

/*Datatype for gpio pin output type configuration*/
typedef enum {
	GPIO_PUSH_PULL, GPIO_OPEN_DRAIN
}GPIO_pinOutputType;


/*Datatype for gpio pin output speed configuration*/
typedef enum {
	GPIO_LOW, GPIO_MEDIUM, GPIO_HIGH, GPIO_VERY_HIGH
}GPIO_pinOutputSpeed;

/*Datatype for gpio pin pullup/down configuration*/
typedef enum {
	GPIO_RES_DISABLED, GPIO_PULLUP, GPIO_PULLDOWN
}GPIO_pinResistorType;

/*Datatype for gpio alternate function configurations
 * used when pin mode is alternate function*/
typedef enum{
	GPIO_SYS = 0,
	GPIO_TIM1_OR_2 = 1 ,
	GPIO_TIM3_TO_5 = 2,
	GPIO_TIM8_TO_11 = 3,
	GPIO_I2C1_TO_3 = 4,
	GPIO_SPI1_TO_6 = 5,
	GPIO_SPI2_OR_3 = 6,
	GPIO_SAL1 = 6,
	GPIO_USART1_TO_3 = 7,
	GPIO_USART4_TO_8 = 8,
	GPIO_CAN1_OR_2 = 9,
	GPIO_LTDC = 9,
	GPIO_TIM12_TO_14 = 9,
	GPIO_OTG_FS = 10,
	GPIO_OTG_HS = 10,
	GPIO_ETH = 11,
	GPIO_FMC = 12,
	GPIO_SDIO = 12,
	GPIO_OTG_HS_1 = 12,
	GPIO_DCMI = 13,
	GPIO_LTDC_1 = 14,
	GPIO_EVENTOUT = 15
}GPIO_pinAltFunction;

typedef enum{
	GPIO_INTERRUPT_DISABLED,
	GPIO_INTERRUPT_ENABLED
}GPIO_InterruptEn;

typedef enum{
	GPIO_EDGE_DISABLED, GPIO_INT_RISING, GPIO_INT_FALLING, GPIO_INT_RISING_AND_FALLING
}GPIO_InterruptEdge;

/*Datatype for gpio pin lock*/
typedef enum{
	GPIO_UNLOCKED, GPIO_LOCKED
}GPIO_lockState;

/*Datatyoe for port number from 0 to 10*/
typedef enum{
	GPIOA_NUM,
	GPIOB_NUM,
	GPIOC_NUM,
	GPIOD_NUM,
	GPIOE_NUM,
	GPIOF_NUM,
	GPIOG_NUM,
	GPIOH_NUM,
	GPIOI_NUM,
	GPIOJ_NUM,
	GPIOK_NUM

}GPIO_portNumber;

/* Main structure used for GPIO init function*/
typedef struct{
	GPIO_pinMode mode;
	GPIO_pinOutputType outputType;
	GPIO_pinOutputSpeed outputSpeed;
	GPIO_pinResistorType resType;
	GPIO_pinAltFunction altFunction;
	GPIO_lockState lockState;
	GPIO_InterruptEn pinInterrupt;
	GPIO_InterruptEdge interruptEdge;
	uint8 pinNum;
	GPIO_portNumber portNum;
}GPIO_InstanceConfigType;

typedef struct{
	/*Base adress of GPIO port*/
	GPIO_TypeDef* Instance;
	/*Config for this instance*/
	GPIO_InstanceConfigType Instance_Config;
}GPIO_PeripheralConfiguration;

/*********************************************************************************************************************************
 *                    								  GPIO PORT Defintion     						                                 *
 ********************************************************************************************************************************/

/*Port address*/
#define GPIO_PORT_A 		GPIOA
#define GPIO_PORT_B         GPIOB
#define GPIO_PORT_C         GPIOC
#define GPIO_PORT_D         GPIOD
#define GPIO_PORT_E         GPIOE
#define GPIO_PORT_F         GPIOF
#define GPIO_PORT_G         GPIOG
#define GPIO_PORT_H         GPIOH
#define GPIO_PORT_I         GPIOI
#define GPIO_PORT_J         GPIOJ
#define GPIO_PORT_K         GPIOK


#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1<<14))

/*To enable clock for different gpio ports in RCC register*/
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()   (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()   (RCC->AHB1ENR |= (1<<10))

/*To disable clock for different gpio ports in RCC register*/
#define GPIOA_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DIS()   (RCC->AHB1ENR &= ~(1<<10))

/*To reset registers of a specific port*/
#define GPIOA_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<0));\
	(RCC->AHB1RSTR &= ~(1<<0));\
	}
#define GPIOB_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<1));\
	(RCC->AHB1RSTR &= ~(1<<1));\
	}
#define GPIOC_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<2));\
	(RCC->AHB1RSTR &= ~(1<<2));\
	}
#define GPIOD_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<3));\
	(RCC->AHB1RSTR &= ~(1<<3));\
	}
#define GPIOE_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<4));\
	(RCC->AHB1RSTR &= ~(1<<4));\
	}
#define GPIOF_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<5));\
	(RCC->AHB1RSTR &= ~(1<<5));\
	}
#define GPIOG_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<6));\
	(RCC->AHB1RSTR &= ~(1<<6));\
	}
#define GPIOH_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<7));\
	(RCC->AHB1RSTR &= ~(1<<7));\
	}
#define GPIOI_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<8));\
	(RCC->AHB1RSTR &= ~(1<<8));\
	}
#define GPIOJ_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<9));\
	(RCC->AHB1RSTR &= ~(1<<9));\
	}
#define GPIOK_REG_RESET()   \
{\
	(RCC->AHB1RSTR |= (1<<10));\
	(RCC->AHB1RSTR &= ~(1<<10));\
	}
/*********************************************************************************************************************************
 *                    								 Function Prototypes    						                                 *
 ********************************************************************************************************************************/


void GPIO_Init(GPIO_PeripheralConfiguration* Config);

void GPIO_DeInit(GPIO_TypeDef* GPIOx);

void GPIO_PeriClockControl(GPIO_TypeDef* GPIOx, GPIO_CLK a_CLK);

uint8 GPIO_readPin(GPIO_TypeDef* GPIOx, uint8 pinNum);

uint16 GPIO_readPort(GPIO_TypeDef* GPIOx);


void GPIO_writePin(GPIO_TypeDef* GPIOx, uint8 pinNum, uint8 val);

void GPIO_writePort(GPIO_TypeDef* GPIOx, uint16 val);

void GPIO_togglePin(GPIO_TypeDef* GPIOx, uint8 pinNum);

void GPIO_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi);

void GPIO_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority);


void GPIO_IRQHandling(uint8 pinNum);



#endif /* STM32F4XX_HAL_DRIVER_INC_STM32F429I_GPIO_DRIVER_H_ */
