/********************************************************************************************************************************
 * Module: UART
 *
 * File Name: stm32f429i_uart_driver.c
 *
 * Description: source file for UART module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/
#include "../Inc/stm32f429i_uart_driver.h"
#include "../Inc/stm32f429i_rcc_driver.h"

USART_InstanceConfigType USART_InstanceConfig;
static USART_TypeDef* Instance_Registers = NULL_PTR;

/*Array of callback pointers*/
__IO static void (*g_CallbackPtr[NO_OF_CALLBACK_EVENTS]) (USART_PeripheralConfiguration* Config)= \
		{NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR};


/*********************************************************************************************************************************
 *                    								 Function Defintions    						                             *
 ********************************************************************************************************************************/

/*
 * typedef enum{
	RCC_AHB_BUS,
	RCC_APB1_BUS,
	RCC_APB2_BUS
}RCC_BusSpeed;
		*/


void USART_Init(USART_PeripheralConfiguration* Config)
{
	uint32 temp;
	USART_InstanceConfig = Config->Instance_Config;
	Instance_Registers = Config->Instance;
	/*Enable USART in RCC to be able to configure registers*/
	USART_PeriClockControl(Config->Instance, USART_InstanceConfig.USARTEnable);
	/*Receiver Mode: Active or Mute*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFFD)|(USART_InstanceConfig.RX_Mode<<1);
	/*Receiver Enable or disable*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFFB)|(USART_InstanceConfig.RXEnable<<2);
	/*Transmitter Enable or disable*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFF7)|(USART_InstanceConfig.TXEnable<<3);

	/*Idle, RX, TX, Transmission complete interrupt config*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFF0F)|(USART_InstanceConfig.Idle_Interrupt<<4)\
			|(USART_InstanceConfig.RX_Interrupt<<5)|(USART_InstanceConfig.TransmitComplete_Interrupt<<6)\
			|(USART_InstanceConfig.TX_Interrupt<<7);

	/*Parity Enable or disable*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFBFF)|(USART_InstanceConfig.ParityEnable<<10);

	/*If Parity is enabled configure parity type, even or od
	 * then configure parity interrupt*/
	if(USART_PARITY_ENABLE == USART_InstanceConfig.ParityEnable)
	{
		/*Even or odd parity*/
		Instance_Registers->CR1 = (Instance_Registers->CR1 &  0xFFFFFDFF)|(USART_InstanceConfig.ParityType<<9);
		/*Enable or disable parity interrupt*/
		Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFEFF)|(USART_InstanceConfig.ParityError_Interrupt<<8);
	}
	else
	{
		/*No action required*/
	}
	/*Wakeup method: Idle line or Address Mark*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFF7FF)|(USART_InstanceConfig.WakeupMethod<<11);
	/*Word length: 8 or 9 bit*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFEFFF)|(USART_InstanceConfig.WordLength<<12);
	/*Over sampling: by 8 or 16*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFF7FFF)|(USART_InstanceConfig.OverSamplingMode<<15);
	if(Config->Instance != UART4 && Config->Instance != UART5)
	{
		//USART_CK_PIN_ENABLE
		/*Enable or disable clock pin (synch. mode)*/
		Instance_Registers->CR2 = (Instance_Registers->CR1 & 0xFFFFF7FF)|(USART_InstanceConfig.ClockPin);
		/*Configure this bits only in Synch Mode*/
		if(USART_CK_PIN_ENABLE == USART_InstanceConfig.ClockPin)
		{
			/*Last bit clock pulse output on ck in synchronous mode or not*/
			Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFEFF)|(USART_InstanceConfig.LastClockPulse);
			/*Configure phase and polarity*/
			Instance_Registers->CR2  = (Instance_Registers->CR2  & 0xFFFFF9FF)|(USART_InstanceConfig.USART_PolandPhaseMode<<9);
		}

		/*Smart card mode enable or disable*/
		Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFFDF)|(USART_InstanceConfig.SmartCardMode<<5);
		/*Configure smart card nack bit in case of parity error only if smart card mode is enabled*/
		if(USART_SMARTCARD_MODE_ENABLE == USART_InstanceConfig.SmartCardMode)
		{
			Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFFEF)|(USART_InstanceConfig.SmartCardNackEnable<<5);

		}


		/*Configure hardware flow control (not available for UART 4 & 5)*/
		Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFCFF)|(USART_InstanceConfig.CTS_Enable<<9)\
				|(USART_InstanceConfig.RTS_Enable<<8);
		/*Configure CTS interrupt only if CTS is enabled*/
		if(USART_CTS_ENABLE == USART_InstanceConfig.CTS_Enable)
		{
			Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFBFF)|(USART_InstanceConfig.CTS_Interrupt<<10);
		}

	}

	/*Address if UART node*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFFF8)|(USART_InstanceConfig.Addr & 0x7);
	/*Configure Stop bits, 0.5 and 1.5 not available for UART 4, 5*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFCFFF)|(USART_InstanceConfig.StopBit<<12);

	/*Lin mode enable or disable*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFBFF)|(USART_InstanceConfig.LinMode<<14);
	/*Confgure this bits only if lin mode is enabled*/
	if(USART_LIN_MODE_ENABLE == USART_InstanceConfig.LinMode)
	{
		/*Lin break detection length*/
		Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFFDF)|(USART_InstanceConfig.LinBreakLength <<5);
		/*Line break interrupt enable or disable*/
		Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFFBF)|(USART_InstanceConfig.LinBreak_Interrupt);
	}

	/*Error interrupt Enable or disable*/
	Instance_Registers->CR3 =(Instance_Registers->CR3 & 0xFFFFFFFE)|(USART_InstanceConfig.Error_Interrupt);
	/*IrDA mode enable or disable*/
	Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFFFD)|(USART_InstanceConfig.IrDAMode<<1);
	/*Only configure Irda low power or normal power if Irda is disabled DUHH!!!*/
	if(USART_IRDA_MODE_ENABLE == USART_InstanceConfig.IrDAMode)
	{
		/*Low or normal power*/
		Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFFFB)|(USART_InstanceConfig.IrDAPowerMode<<2);
	}
	/*Half duplex mode enable or disable*/
	Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFFF7)|(USART_InstanceConfig.HalfDuplexEnable<<3);
	/*DMA transmitter/receiver enable or disable*/
	Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFFF3F)|(USART_InstanceConfig.TX_Dma<<7)\
			|(USART_InstanceConfig.RX_Dma<<6);
	/*Configure One sample bit method*/
	Instance_Registers->CR3 = (Instance_Registers->CR3 & 0xFFFFF7FF)|(USART_InstanceConfig.SampleBitMethod<<11);
	/***************Configure Baud rate Here*************/


	if (Config->Instance == USART1 || Config->Instance == USART6) /*APB2*/
	{
		temp =(25*RCC_getPclkval(RCC_APB2_BUS))/(2*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))\
				*USART_InstanceConfig.Baud);/*USARTDIV * 100*/

		Instance_Registers->BRR = (((uint16)(temp/100))<<4); /*Mantissa*/
		if( GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8) == 1) /*Over sampling by 8 keep bit 3 clear
		i.e & 0x7*/
		Instance_Registers->BRR =(Instance_Registers->BRR  & 0xFFFFFFF0)|\
				(((uint16)(((temp - (((uint16)(temp/100))*100))*(8*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))))+50)/100)&0x7);
		else
		{/*Over sampling by 16  & 0xF*/
			Instance_Registers->BRR =(Instance_Registers->BRR  & 0xFFFFFFF0)|\
					(((uint16)(((temp - (((uint16)(temp/100))*100))*(8*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))))+50)/100)&0xF);
		}
	}
	else 	/*Apb1 bus for the rest*/
	{
		temp =(25*RCC_getPclkval(RCC_APB1_BUS))/(2*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))\
				*USART_InstanceConfig.Baud);/*USARTDIV * 100*/

		Instance_Registers->BRR = (((uint16)(temp/100))<<4); /*Mantissa*/
		if( GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8) == 1) /*Over sampling by 8 keep bit 3 clear
		i.e & 0x7*/
		Instance_Registers->BRR =(Instance_Registers->BRR  & 0xFFFFFFF0)|\
				(((uint16)(((temp - (((uint16)(temp/100))*100))*(8*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))))+50)/100)&0x7);
		else
		{/*Over sampling by 16  & 0xF*/
			Instance_Registers->BRR =(Instance_Registers->BRR  & 0xFFFFFFF0)|\
					(((uint16)(((temp - (((uint16)(temp/100))*100))*(8*(2-GET_BIT(Instance_Registers->CR1, USART_CR1_OVER8))))+50)/100)&0xF);
		}
	}
	/*Finally Enable UART peripheral*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFDFFF)|(USART_InstanceConfig.USARTEnable<<13);

}

void USART_DeInit(USART_TypeDef* USARTx)
{
	if(USART1 == USARTx)
	{
		USART1_PCLK_DIS();
	}
	else if(USART2 == USARTx)
	{
		USART2_PCLK_DIS();
	}
	else if(USART3 == USARTx)
	{
		USART3_PCLK_DIS();
	}
	else if(UART4 == USARTx)
	{
		USART4_PCLK_DIS();
	}
	else if(UART5 == USARTx)
	{
		USART5_PCLK_DIS();
	}
	else if(USART6 == USARTx)
	{
		USART6_PCLK_DIS();
	}
	else if(UART7 == USARTx)
	{
		USART7_PCLK_DIS();
	}
	else if(UART8 == USARTx)
	{
		USART8_PCLK_DIS();
	}
	else{
		/*No action required*/
	}
}


void USART_PeriClockControl(USART_TypeDef* USARTx, USART_CLK a_CLK)
{
	if (USART_EN == a_CLK)
	{
		if(USART1 == USARTx)
		{
			USART1_PCLK_EN();
		}
		else if(USART2 == USARTx)
		{
			USART2_PCLK_EN();
		}
		else if(USART3 == USARTx)
		{
			USART3_PCLK_EN();
		}
		else if(UART4 == USARTx)
		{
			USART4_PCLK_EN();
		}
		else if(UART5 == USARTx)
		{
			USART5_PCLK_EN();
		}
		else if(USART6 == USARTx)
		{
			USART6_PCLK_EN();
		}
		else if(UART7 == USARTx)
		{
			USART7_PCLK_EN();
		}
		else if(UART8 == USARTx)
		{
			USART8_PCLK_EN();
		}
		else{
			/*No action required*/
		}
	}
	else if (USART_DIS == a_CLK)
	{
		if(USART1 == USARTx)
		{
			USART1_PCLK_DIS();
		}
		else if(USART2 == USARTx)
		{
			USART2_PCLK_DIS();
		}
		else if(USART3 == USARTx)
		{
			USART3_PCLK_DIS();
		}
		else if(UART4 == USARTx)
		{
			USART4_PCLK_DIS();
		}
		else if(UART5 == USARTx)
		{
			USART5_PCLK_DIS();
		}
		else if(USART6 == USARTx)
		{
			USART6_PCLK_DIS();
		}
		else if(UART7 == USARTx)
		{
			USART7_PCLK_DIS();
		}
		else if(UART8 == USARTx)
		{
			USART8_PCLK_DIS();
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


void USART_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi)
{
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

void USART_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority)
{
	/*Configure interrupt priority*/
	NVIC->IPR[(uint8)IRQ_Num/4] = (NVIC->IPR[(uint8)IRQ_Num/4] \
			& (~(15<<((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)))))|(IRQ_Priority << ((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)));
}




/*Blocking*/
void USART_SendData(USART_TypeDef* USARTx, uint16* pTXBuffer, uint32 len)
{
	char* tx_tempBuffer = (char*)pTXBuffer; /*To increment a uint16* ptxbuffer by one byte only*/
	while(len > 0)
	{
		while(BIT_IS_CLEAR(USARTx->SR, USART_SR_TXE));
		if(USART_8_DATA_BITS == GET_BIT(USARTx->CR1, USART_CR1_M))
		{
			if(USART_PARITY_DISABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				USARTx->DR = *(uint8*)tx_tempBuffer;
			}
			else if (USART_PARITY_ENABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				USARTx->DR = (*(uint8*)tx_tempBuffer)&0x7F; /*Get 7 bits*/
			}
			else{
				/*No action Required*/
			}
			tx_tempBuffer++;
			len--;
		}
		else if(USART_9_DATA_BITS == GET_BIT(USARTx->CR1, USART_CR1_M))
		{
			if(USART_PARITY_DISABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				USARTx->DR = (*(uint16*)pTXBuffer)&0x01FF; /*Get 9 data bits*/
				pTXBuffer++;
				len--;
			}
			else if (USART_PARITY_ENABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				USARTx->DR = (*(uint8*)tx_tempBuffer); /*Get 8 bits, the 9th bit is parity, register is 9 bits*/
				tx_tempBuffer++;
				len--;
			}
			else{
				/*No action Required*/
			}

		}

	}
	/*Wait till transmission completes, you can disable transmitter TE bit here to save power*/
	while(BIT_IS_CLEAR(USARTx->SR, USART_SR_TC));
}

void USART_ReceiveData(USART_TypeDef* USARTx, uint16* pRXBuffer, uint32 len)
{
	char* Rx_tempBuffer = (char*)pRXBuffer; /*To increment a uint16* pRXBuffer by one byte only*/
	while(len > 0)
	{
		while(BIT_IS_CLEAR(USARTx->SR, USART_SR_RXNE));
		if(USART_8_DATA_BITS == GET_BIT(USARTx->CR1, USART_CR1_M))
		{
			if(USART_PARITY_DISABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				*(uint8*)Rx_tempBuffer = USARTx->DR &0xFF; /*get 8 bits*/
			}
			else if (USART_PARITY_ENABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				*(uint8*)Rx_tempBuffer = USARTx->DR&0x7F; /*Get 7 bits*/
			}
			else{
				/*No action Required*/
			}
			Rx_tempBuffer++;
			len--;
		}
		else if(USART_9_DATA_BITS == GET_BIT(USARTx->CR1, USART_CR1_M))
		{
			if(USART_PARITY_DISABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{
				(*(uint16*)pRXBuffer) = USARTx->DR & 0x01FF; /*Get 9 data bits*/
				pRXBuffer++;
				len--;
			}
			else if (USART_PARITY_ENABLE == GET_BIT(USARTx->CR1, USART_CR1_PCE))
			{

				(*(uint8*)Rx_tempBuffer)= USARTx->DR; /*Get 8 bits, the 9th bit is parity, register is 9 bits*/
				Rx_tempBuffer++;
				len--;
			}
			else{
				/*No action Required*/
			}

		}

	}
}

void USART_IRQHandling(USART_PeripheralConfiguration* Config)
{
	char* tx_tempBuffer = (char*)Config->Ptr_TXBuffer; /*To increment a uint16* ptxbuffer by one byte only*/
	char* Rx_tempBuffer = (char*)Config->Ptr_RXBuffer; /*To increment a uint16* pRXBuffer by one byte only*/

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_TC )\
			&& BIT_IS_SET(Config->Instance->CR1, USART_CR1_TCIE))
	{
		if(USART_STATE_BUSY_TX == Config->TxState ){
			/*If length is zero close, clear TC flag and close transmission*/
			if(!Config->TX_TransferLen)
			{
				Config->Instance->SR &=~ (1 << USART_SR_TC);
				/*Set state to ready for next transmission*/
				Config->TxState = USART_STATE_READY;
				/*Reset buffer length and buffer pointer*/
				Config->TX_TransferLen = 0;
				Config->Ptr_TXBuffer = NULL_PTR;
				if(g_CallbackPtr[USART_TC_COMPLETE] != NULL_PTR)
				{
					(*g_CallbackPtr[USART_TC_COMPLETE])(Config);
				}
			}
		}

	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_TXE )\
			&& BIT_IS_SET(Config->Instance->CR1, USART_CR1_TXEIE))
	{
		if(USART_STATE_BUSY_TX == Config->TxState)
		{
			if(Config->TX_TransferLen > 0)
			{
				if(USART_8_DATA_BITS == GET_BIT(Config->Instance->CR1, USART_CR1_M))
				{
					if(USART_PARITY_DISABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						Config->Instance->DR = *(uint8*)tx_tempBuffer;
					}
					else if (USART_PARITY_ENABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						Config->Instance->DR = (*(uint8*)tx_tempBuffer)&0x7F; /*Get 7 bits*/
					}
					else{
						/*No action Required*/
					}
					tx_tempBuffer++;
					Config->TX_TransferLen--;
				}
				else if(USART_9_DATA_BITS == GET_BIT(Config->Instance->CR1, USART_CR1_M))
				{
					if(USART_PARITY_DISABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						Config->Instance->DR = (*(uint16*)Config->Ptr_TXBuffer)&0x01FF; /*Get 9 data bits*/
						Config->Ptr_TXBuffer++;
						Config->TX_TransferLen--;
					}
					else if (USART_PARITY_ENABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						Config->Instance->DR = (*(uint8*)tx_tempBuffer); /*Get 8 bits, the 9th bit is parity, register is 9 bits*/
						tx_tempBuffer++;
						Config->TX_TransferLen--;
					}
					else{
						/*No action Required*/
					}

				}
				if(g_CallbackPtr[USART_TX_COMPLETE] != NULL_PTR)
				{
					(*g_CallbackPtr[USART_TX_COMPLETE])(Config);
				}
			}
			if(Config->TX_TransferLen == 0)
			{
				/*At end of transmission disable interrupt*/
				Config->Instance->CR1 &=~ (1<<USART_CR1_TXEIE);
			}
		}


	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_RXNE )\
			&& BIT_IS_SET(Config->Instance->CR1, USART_CR1_RXNEIE))
	{
		if(USART_STATE_BUSY_RX == Config->RxState)
		{
			if(Config->RX_TransferLen > 0)
			{
				if(USART_8_DATA_BITS == GET_BIT(Config->Instance->CR1, USART_CR1_M))
				{
					if(USART_PARITY_DISABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						*(uint8*)Rx_tempBuffer = Config->Instance->DR &0xFF; /*get 8 bits*/
					}
					else if (USART_PARITY_ENABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						*(uint8*)Rx_tempBuffer = Config->Instance->DR&0x7F; /*Get 7 bits*/
					}
					else{
						/*No action Required*/
					}
					Rx_tempBuffer++;
					Config->RX_TransferLen--;
				}
				else if(USART_9_DATA_BITS == GET_BIT(Config->Instance->CR1, USART_CR1_M))
				{
					if(USART_PARITY_DISABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{
						(*(uint16*)Config->Ptr_RXBuffer) = Config->Instance->DR & 0x01FF; /*Get 9 data bits*/
						Config->Ptr_RXBuffer++;
						Config->RX_TransferLen--;
					}
					else if (USART_PARITY_ENABLE == GET_BIT(Config->Instance->CR1, USART_CR1_PCE))
					{

						(*(uint8*)Rx_tempBuffer)= Config->Instance->DR; /*Get 8 bits, the 9th bit is parity, register is 9 bits*/
						Rx_tempBuffer++;
						Config->RX_TransferLen--;
					}
					else{
						/*No action Required*/
					}

				}

			}
			if(Config->RX_TransferLen == 0)
			{
				/*At end of transmission disable interrupt*/
				Config->Instance->CR1 &=~ (1<<USART_CR1_RXNEIE);
				/*Set state to ready for next transmission*/
				Config->RxState = USART_STATE_READY;
				/*Reset buffer length and pointer*/
				Config->Ptr_RXBuffer = NULL_PTR;
				Config->RX_TransferLen = 0;
				/*Whole transmission completed*/
				if(g_CallbackPtr[USART_RX_COMPLETE] != NULL_PTR)
				{
					(*g_CallbackPtr[USART_RX_COMPLETE])(Config);
				}
			}
		}
	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_ORE )\
			&& BIT_IS_SET(Config->Instance->CR1, USART_CR1_RXNEIE))
	{
		/*Clear flag in callaback API by reading DR*/
		if(g_CallbackPtr[USART_OVR_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_OVR_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_IDLE )\
			&& BIT_IS_SET(Config->Instance->CR1, USART_CR1_IDLEIE))
	{
		/*Clear flag then Callback*/
		Config->Instance->SR &=~ (1<<USART_SR_IDLE);
		if(g_CallbackPtr[USART_IDLE_LINE] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_IDLE_LINE])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_CTS )\
			&& BIT_IS_SET(Config->Instance->CR3, USART_CR3_CTSIE))
	{
		/*Clear flag then Callback*/
		Config->Instance->SR &=~ (1<<USART_SR_CTS);
		if(g_CallbackPtr[USART_CTS_CHANGE] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_CTS_CHANGE])(Config);
		}

	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_NF )\
			&& BIT_IS_SET(Config->Instance->CR3, USART_CR3_EIE))
	{
		/*Clear flag in callaback API by reading DR*/
		if(g_CallbackPtr[USART_NOISE_DETECTED] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_NOISE_DETECTED])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_FE )\
			&& BIT_IS_SET(Config->Instance->CR3, USART_CR3_EIE))
	{
		/*Clear flag in callaback API by reading DR*/
		if(g_CallbackPtr[USART_FRAME_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_FRAME_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR,USART_SR_ORE )\
			&& BIT_IS_SET(Config->Instance->CR3, USART_CR3_EIE))
	{
		/*Clear flag in callaback API by reading DR*/
		if(g_CallbackPtr[USART_OVR_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[USART_OVR_ERROR])(Config);
		}
	}

}

/* non Blocking*/
USART_State USART_SendDataIT(USART_PeripheralConfiguration* Config, uint16* pTXBuffer, uint32 len)
{
	USART_State Busy_state = Config->TxState;
	if(Busy_state != USART_STATE_BUSY_TX) /*Only configure new buffers when USART is ready, to not overwrite ones we're sending*/
	{
		/*Save info*/
		Config->Ptr_TXBuffer = pTXBuffer;
		Config->TX_TransferLen = len;

		/*Mark state as busy*/
		Config->TxState = USART_STATE_BUSY_TX;

		if(BIT_IS_CLEAR(Config->Instance->CR2, USART_CR1_TXEIE))/*If TX interrupt is not set, enable it*/
		{
			SET_BIT(Config->Instance->CR2, USART_CR1_TXEIE);
		}
		else
		{
			/*No action required*/
		}
		if(BIT_IS_CLEAR(Config->Instance->CR2, USART_CR1_TCIE))/*If TX Complete interrupt is not set, enable it*/
		{
			SET_BIT(Config->Instance->CR2, USART_CR1_TCIE);
		}
		else
		{
			/*No action required*/
		}
	}
	else
	{
		/*No action required*/

	}

	return Busy_state;
}

USART_State USART_ReceiveDataIT(USART_PeripheralConfiguration* Config, uint16* pRXBuffer, uint32 len)
{
	USART_State Busy_state = Config->RxState;
	if(Busy_state != USART_STATE_BUSY_RX) /*Only configure new buffers when USART is ready, to not overwrite ones we're receiving*/
	{
		/*Save info*/
		Config->Ptr_RXBuffer = pRXBuffer;
		Config->RX_TransferLen = len;

		/*Mark state as busy*/
		Config->RxState = USART_STATE_BUSY_RX;

		if(BIT_IS_CLEAR(Config->Instance->CR2, USART_CR1_RXNEIE))/*If RX interrupt is not set, enable it*/
		{
			SET_BIT(Config->Instance->CR2, USART_CR1_RXNEIE);
		}
		else
		{
			/*No action required*/
		}
	}
	else
	{
		/*No action required*/

	}

	return Busy_state;
}



void USART_SetCallback(void (*a_Ptr)(USART_PeripheralConfiguration* Config),
		USART_EventCallBack Event_Set)
{
	/*Set call back for multiple events and error*/
	if(Event_Set == USART_TX_COMPLETE)
	{
		g_CallbackPtr[USART_TX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == USART_RX_COMPLETE)
	{
		g_CallbackPtr[USART_RX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == USART_OVR_ERROR)
	{
		g_CallbackPtr[USART_OVR_ERROR] = a_Ptr;

	}
	else if (Event_Set == USART_TC_COMPLETE)
	{
		g_CallbackPtr[USART_TC_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == USART_CTS_CHANGE)
	{
		g_CallbackPtr[USART_CTS_CHANGE] = a_Ptr;

	}
	else if (Event_Set == USART_IDLE_LINE)
	{
		g_CallbackPtr[USART_IDLE_LINE] = a_Ptr;

	}
	else if (Event_Set == USART_FRAME_ERROR)
	{
		g_CallbackPtr[USART_FRAME_ERROR] = a_Ptr;

	}

	else if (Event_Set == USART_NOISE_DETECTED)
	{
		g_CallbackPtr[USART_NOISE_DETECTED] = a_Ptr;

	}
	else
	{
		/*No action required*/
	}

}
