 /********************************************************************************************************************************
 * Module: GPIO
 *
 * File Name: stm32f429i_gpio_driver.c
 *
 * Description: source file for GPIO Driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/

#include "../Inc/stm32f429i_gpio_driver.h"
#include "../Inc/stm32f429i_rcc_driver.h"



GPIO_InstanceConfigType GPIO_InstanceConfig;
static GPIO_TypeDef* Instance_Registers = NULL_PTR;

/*********************************************************************************************************************************
 *                    								 ISRs    						                                 *
 ********************************************************************************************************************************/

void EXTI1_IRQHandler(void)
{

}
void EXTI2_IRQHandler(void)
{

}
void EXTI3_IRQHandler(void)
{

}
void EXTI4_IRQHandler(void)
{

}
void EXTI9_5_IRQHandler(void)
{

}
void EXTI15_10_IRQHandler(void)
{

}

/*********************************************************************************************************************************
 *                    								 Function definitions    						                                 *
 ********************************************************************************************************************************/

void GPIO_Init(GPIO_PeripheralConfiguration* Config){
	GPIO_InstanceConfig = Config->Instance_Config;
	Instance_Registers = Config->Instance;
	/*Mode*/
	GPIO_PeriClockControl(Config->Instance, GPIO_EN); /*Implicitly enabling clock*/
	Instance_Registers->MODER = (Instance_Registers->MODER & (~(3<<2*GPIO_InstanceConfig.pinNum)))\
			|(GPIO_InstanceConfig.mode<<2*GPIO_InstanceConfig.pinNum);
	/*Output type*/
	Instance_Registers->OTYPER = (Instance_Registers->OTYPER & (~(1<<GPIO_InstanceConfig.pinNum)))\
			|(GPIO_InstanceConfig.outputType<<GPIO_InstanceConfig.pinNum);
	/*Output Speed*/
	Instance_Registers->OSPEEDR = (Instance_Registers->OSPEEDR & (~(3<<2*GPIO_InstanceConfig.pinNum)))\
			|(GPIO_InstanceConfig.outputSpeed<<2*GPIO_InstanceConfig.pinNum);
	/*Resistor Type*/
	Instance_Registers->PUPDR = (Instance_Registers->PUPDR & (~(3<<2*GPIO_InstanceConfig.pinNum)))\
			|(GPIO_InstanceConfig.resType<<2*GPIO_InstanceConfig.pinNum);

	/*If input configure interrupts (Enabled or disabled)*/
	if(GPIO_INPUT == GPIO_InstanceConfig.mode)
	{
		if (GPIO_INTERRUPT_ENABLED == GPIO_InstanceConfig.pinInterrupt)
		{
			/*Enable SYSCFG register before configuring its registers*/
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[(uint8)GPIO_InstanceConfig.pinNum/4] = \
					((SYSCFG->EXTICR[(uint8)GPIO_InstanceConfig.pinNum/4])\
					&(~(15<<4*(GPIO_InstanceConfig.pinNum%4))))\
					|(GPIO_InstanceConfig.portNum<<4*(GPIO_InstanceConfig.pinNum%4));

			/*Configure interrupt rising or falling edge or both*/
			if(GPIO_INT_RISING == GPIO_InstanceConfig.interruptEdge)
			{
				EXTI->FTSR &= ~(1<<GPIO_InstanceConfig.pinNum);
				EXTI->RTSR |= (1<<GPIO_InstanceConfig.pinNum);
			}
			else if(GPIO_INT_FALLING == GPIO_InstanceConfig.interruptEdge)
			{
				EXTI->RTSR &= ~(1<<GPIO_InstanceConfig.pinNum);
				EXTI->FTSR |= (1<<GPIO_InstanceConfig.pinNum);

			}
			else if(GPIO_INT_RISING_AND_FALLING == GPIO_InstanceConfig.interruptEdge)
			{
				EXTI->RTSR |= (1<<GPIO_InstanceConfig.pinNum);
				EXTI->FTSR |= (1<<GPIO_InstanceConfig.pinNum);
			}
			else
			{
				/*No action required*/
			}
			/*Enable interrupt delivery*/
			EXTI->IMR = (EXTI->IMR & (~(1<<GPIO_InstanceConfig.pinNum)))|(1<<GPIO_InstanceConfig.pinNum);

		}
		else
		{
			/*No action required*/
		}
	}
	else if (GPIO_ALT_FUNC == GPIO_InstanceConfig.mode)
	{
		Instance_Registers->AFR[(uint8)GPIO_InstanceConfig.pinNum/8] = \
				(Instance_Registers->AFR[(uint8)GPIO_InstanceConfig.pinNum/8]\
						& (~(15<<4*(GPIO_InstanceConfig.pinNum%8))))\
						|(GPIO_InstanceConfig.altFunction<<4*(GPIO_InstanceConfig.pinNum%8));

	}
	else
	{
		/*No action required*/
	}



}

//typedef enum{
//	GPIO_INT_RISING, GPIO_INT_FALLING, GPIO_INT_RISING_AND_FALLING
//}GPIO_InterruptEdge;
//GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_FUNC
void GPIO_DeInit(GPIO_TypeDef* GPIOx){
	if(GPIOA == GPIOx)
	{
		GPIOA_REG_RESET();
	}
	else if(GPIOB == GPIOx)
	{
		GPIOB_REG_RESET();
	}
	else if(GPIOC == GPIOx)
	{
		GPIOC_REG_RESET();
	}
	else if(GPIOD == GPIOx)
	{
	GPIOD_REG_RESET();
	}
	else if(GPIOE == GPIOx)
	{
		GPIOE_REG_RESET();
	}
	else if(GPIOF == GPIOx)
	{
		GPIOF_REG_RESET();
	}
	else if(GPIOG == GPIOx)
	{
		GPIOG_REG_RESET();
	}
	else if(GPIOH == GPIOx)
	{
		GPIOH_REG_RESET();
	}
	else if(GPIOI == GPIOx)
	{
		GPIOI_REG_RESET();
	}
	else if(GPIOJ == GPIOx)
	{
		GPIOJ_REG_RESET();
	}
	else if(GPIOK == GPIOx)
	{
		GPIOK_REG_RESET();
	}
	else{
		/*No action required*/
	}
}

void GPIO_PeriClockControl(GPIO_TypeDef* GPIOx, GPIO_CLK a_CLK){
	if(GPIO_EN == a_CLK)
	{
		if(GPIOA == GPIOx)
		{
			GPIOA_PCLK_EN();
		}
		else if(GPIOB == GPIOx)
		{
			GPIOB_PCLK_EN();
		}
		else if(GPIOC == GPIOx)
		{
			GPIOC_PCLK_EN();
		}
		else if(GPIOD == GPIOx)
		{
		GPIOD_PCLK_EN();
		}
		else if(GPIOE == GPIOx)
		{
			GPIOE_PCLK_EN();
		}
		else if(GPIOF == GPIOx)
		{
			GPIOF_PCLK_EN();
		}
		else if(GPIOG == GPIOx)
		{
			GPIOG_PCLK_EN();
		}
		else if(GPIOH == GPIOx)
		{
			GPIOH_PCLK_EN();
		}
		else if(GPIOI == GPIOx)
		{
			GPIOI_PCLK_EN();
		}
		else if(GPIOJ == GPIOx)
		{
			GPIOJ_PCLK_EN();
		}
		else if(GPIOK == GPIOx)
		{
			GPIOK_PCLK_EN();
		}
		else{
			/*No action required*/
		}
	}
	else if (GPIO_DIS == a_CLK)
	{
		if(GPIOA == GPIOx)
		{
			GPIOA_PCLK_DIS();
		}
		else if(GPIOB == GPIOx)
		{
			GPIOB_PCLK_DIS();
		}
		else if(GPIOC == GPIOx)
		{
			GPIOC_PCLK_DIS();
		}
		else if(GPIOD == GPIOx)
		{
			GPIOD_PCLK_DIS();
		}
		else if(GPIOE == GPIOx)
		{
			GPIOE_PCLK_DIS();
		}
		else if(GPIOF == GPIOx)
		{
			GPIOF_PCLK_DIS();
		}
		else if(GPIOG == GPIOx)
		{
			GPIOG_PCLK_DIS();
		}
		else if(GPIOH == GPIOx)
		{
			GPIOH_PCLK_DIS();
		}
		else if(GPIOI == GPIOx)
		{
			GPIOI_PCLK_DIS();
		}
		else if(GPIOJ == GPIOx)
		{
			GPIOJ_PCLK_DIS();
		}
		else if(GPIOK == GPIOx)
		{
			GPIOK_PCLK_DIS();
		}
		else{
			/*No action required*/
		}
	}
	else
	{
		/*No action required*/
	}
}

uint8 GPIO_readPin(GPIO_TypeDef* GPIOx, uint8 pinNum){
	return (((uint8)GPIOx->IDR & (1<<pinNum))>>pinNum);
}

uint16 GPIO_readPort(GPIO_TypeDef* GPIOx){
	return (uint16)GPIOx->IDR;
}


void GPIO_writePin(GPIO_TypeDef* GPIOx, uint8 pinNum, uint8 val){
	GPIOx->ODR = (GPIOx->ODR & (~(1<<pinNum))) | (val<<pinNum);
}

void GPIO_writePort(GPIO_TypeDef* GPIOx, uint16 val){
	GPIOx->ODR = val;
}

void GPIO_togglePin(GPIO_TypeDef* GPIOx, uint8 pinNum){
	GPIOx->ODR  ^= (1<<pinNum);
}

void GPIO_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi){
	if(ENABLE == EnOrDi)
	{
		/*Enable Interrupt*/
		NVIC->ISER[(uint8)IRQ_Num/32] = (NVIC->ISER[(uint8)IRQ_Num/32] & (~(1<<IRQ_Num%32)))|(1<<IRQ_Num%32);


	}
	else if (DISABLE == EnOrDi)
	{
		/*Disable Interrupt*/
		NVIC->ICER[(uint8)IRQ_Num/32] = (NVIC->ICER[(uint8)IRQ_Num/32] & (~(1<<IRQ_Num%32)))|(1<<IRQ_Num%32);
	}
	else
	{
		/*No action required*/
	}
}

void GPIO_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority)
{
	/*Configure interrupt priority*/
	NVIC->IPR[(uint8)IRQ_Num/4] = (NVIC->IPR[(uint8)IRQ_Num/4] \
			& (~(15<<((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)))))|(IRQ_Priority << ((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)));
}

void GPIO_IRQHandling(uint8 pinNum){
	/*Clear pending bit in pending register at EXTI side(after handling interrupt )since at
	 * NVIC side it\s cleared before executing ISR*/
	if(EXTI->PR & (1<<pinNum))/*If bit is set, clear it  by writing one*/
	{
		EXTI->PR |= (1<<pinNum);
	}
}
