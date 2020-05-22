 /********************************************************************************************************************************
 * Module: SPI
 *
 * File Name: stm32f429i_spi_driver.c
 *
 * Description: source file for SPI module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/
#include "../Inc/stm32f429i_Spi_driver.h"
#include "../Inc/stm32f429i_rcc_driver.h"




static void SPI_HandleTXInterrupt(SPI_PeripheralConfiguration* Config);
static void SPI_HandleRXInterrupt(SPI_PeripheralConfiguration* Config);
static void SPI_HandleOVRErrorInterrupt(SPI_PeripheralConfiguration* Config);

SPI_InstanceConfigType SPI_InstanceConfig;
static SPI_TypeDef* Instance_Registers = NULL_PTR;
/*Array of callback pointers*/
__IO static void (*g_CallbackPtr[NO_OF_CALLBACK_EVENTS]) (SPI_PeripheralConfiguration* Config)= \
		{NULL_PTR, NULL_PTR, NULL_PTR};
/*********************************************************************************************************************************
 *                    								 Function Defintions    						                                 *
 ********************************************************************************************************************************/

void SPI_Init(SPI_PeripheralConfiguration* Config){
	/*Configure polarity, Phase, and Mode(Master or Slave)*/
	SPI_InstanceConfig = Config->Instance_Config; /*To ease access to config struct within the big peripheral struct*/
	Instance_Registers = Config->Instance; /*To access registers for that specific SPI Channel*/
	SPI_PeriClockControl(Config->Instance, SPI_InstanceConfig.SPIEnable);
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFFC)|(SPI_InstanceConfig.SPI_PolandPhaseMode);
	/*To Configure CPOL and CPHA bits in SPI_CR1*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFFB) | ((SPI_InstanceConfig.Config_Mode)<<SPI_CR1_MSTR);
	/*To Configure SPI in master or slave mode*/

	/*Configure Direction, DataSize, BaudRate, LSBFIRST*/
	/*TO GET BIDIMODE, BIDIOE, RXONLY*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFF3BFF)|((SPI_InstanceConfig.Bidirection_Unidirection)<<SPI_CR1_BIDIMODE)\
			|((SPI_InstanceConfig.BI_Mode)<<SPI_CR1_BIDIOE)|((SPI_InstanceConfig.Uni_Mode)<<SPI_CR1_RXONLY);
	/*TO Get Baudrate BR[2:0]*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFC7)|((SPI_InstanceConfig.Freq_Divide)<<SPI_CR1_BR);
	/*To get Frame Length*/
	Instance_Registers->CR1 =(Instance_Registers->CR1 & 0xFFFFF7FF)|((SPI_InstanceConfig.Frame_Length)<<SPI_CR1_DFF);
	/*To set which bit first LSB or MSB*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFF7F)|((SPI_InstanceConfig.Frame_Order)<<SPI_CR1_LSBFIRST);
	/*To CRCEN, CRCNEXT bits*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFCFFF)|((SPI_InstanceConfig.CRC_Enable)<<SPI_CR1_CRCEN)\
			|((SPI_InstanceConfig.CRC_Next)<<SPI_CR1_CRCNEXT);
	/*Configure NSS (Slave Select Pin)*/
	if(SPI_InstanceConfig.Config_Mode == SPI_MASTER_CONFIG)
	{
		if(SPI_InstanceConfig.SSM_Management == SPI_SSM_ENABLED)
		{
			/*Enable SSM bit, and pull NSS to high by setting SSI bit to one, since it's recommendded that NSS
			 * pin is high for master mode*/
			Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFCFF)|((SPI_InstanceConfig.SSM_Management)<<SPI_CR1_SSM)\
					|(1<<SPI_CR1_SSI); /* |(1<<8) to pull SSI bit to VCC since Master*/
		}
		else if (SPI_InstanceConfig.SSM_Management == SPI_SSM_DISABLED)
		{
			/*Hardware Slave management, Here I Must configure GPIO for Slave that drives pin low later*/
			Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFCFF); /*Disabled SSM Bit and SSI value doens't matter
			SSM disabled SINCE FFFF FCFF where C disabled bit of SSM and SSI */
		}
		else
		{
			/*No action required*/
		}
	}
	else if (SPI_InstanceConfig.Config_Mode == SPI_SLAVE_CONFIG)
	{
		if(SPI_InstanceConfig.SSM_Management == SPI_SSM_ENABLED)
		{
			/*Enable SSM bit, and pull NSS to low by setting SSI bit to zero, since  NSS
			 * pin must be pulled low for slave mode*/
			Instance_Registers->CR1 = ((Instance_Registers->CR1 & 0xFFFFFCFF)|((SPI_InstanceConfig.SSM_Management)<<SPI_CR1_SSM))\
					&(~(1<<SPI_CR1_SSI)); /* (~(1<<8)) to pull SSI bit to GND since Slave*/
		}
		else if (SPI_InstanceConfig.SSM_Management == SPI_SSM_DISABLED)
		{
			Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFCFF); /*Disabled SSM Bit and SSI value doens't matter
			SSM disabled SINCE FFFF FCFF where C disabled bit of SSM and SSI */

		}
		else
		{
			/*No action required*/
		}
	}
	else
	{
		/*No action Required*/
	}

	/******************************CR2 Config************************************/
	/*Enable or disable TX, RX and Error interrupts*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFF1F)|((SPI_InstanceConfig.TX_Interrupt)<<SPI_CR2_TXEIE)\
			|((SPI_InstanceConfig.RX_Interrupt)<<SPI_CR2_RXNEIE)|((SPI_InstanceConfig.Err_Interrupt)<<SPI_CR2_ERRIE);
	/*Configure frame format, SSOUtput, TXDMAEN, RXDMAEN*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFFE8)|((SPI_InstanceConfig.Frame_Format)<<SPI_CR2_FRF)\
			|((SPI_InstanceConfig.SS_Output)<<SPI_CR2_SSOE)|((SPI_InstanceConfig.TX_Dma)<<SPI_CR2_TXDMAEN)|(SPI_InstanceConfig.RX_Dma);

	/*FinallyConfigure SPE enable Bit to enable or disable SPI*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFBF)|((SPI_InstanceConfig.SPIEnable)<<SPI_CR1_SPE);

}



void SPI_PeriClockControl(SPI_TypeDef* SPIx, SPI_CLK a_CLK)
{
	if(SPI_EN == a_CLK)
	{
		if(SPI1 == SPIx)
		{
			SPI1_PCLK_EN();
		}
		else if(SPI2 == SPIx)
		{
			SPI2_PCLK_EN();
		}
		else if(SPI3 == SPIx)
		{
			SPI3_PCLK_EN();
		}
		else if(SPI4 == SPIx)
		{
			SPI4_PCLK_EN();
		}
		else if(SPI5 == SPIx)
		{
			SPI5_PCLK_EN();
		}
		else if(SPI6 == SPIx)
		{
			SPI6_PCLK_EN();
		}
		else{
			/*No action required*/
		}
	}
	else if (SPI_DIS == a_CLK)
	{
		if(SPI1 == SPIx)
		{
			SPI1_PCLK_DIS();
		}
		else if(SPI2 == SPIx)
		{
			SPI2_PCLK_DIS();
		}
		else if(SPI3 == SPIx)
		{
			SPI3_PCLK_DIS();
		}
		else if(SPI4 == SPIx)
		{
			SPI4_PCLK_DIS();
		}
		else if(SPI5 == SPIx)
		{
			SPI5_PCLK_DIS();
		}
		else if(SPI6 == SPIx)
		{
			SPI6_PCLK_DIS();
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

void SPI_DeInit(SPI_TypeDef* SPIx)
{
	if(SPI1 == SPIx)
	{
		SPI1_REG_RESET();
	}
	else if(SPI2 == SPIx)
	{
		SPI2_REG_RESET();
	}
	else if(SPI3 == SPIx)
	{
		SPI3_REG_RESET();
	}
	else if(SPI4 == SPIx)
	{
		SPI4_REG_RESET();
	}
	else if(SPI5 == SPIx)
	{
		SPI5_REG_RESET();
	}
	else if(SPI6 == SPIx)
	{
		SPI6_REG_RESET();
	}
	else{
		/*No action required*/
	}
}


/*********/

/*Blocking APIs*/
void SPI_SendData(SPI_TypeDef* SPIx, uint8* pTXBuffer, uint32 len)
{
	while(len > 0)
	{
		/*Wait till TXE is empty, since initially it may not be empty*/
		while(BIT_IS_CLEAR(SPIx->SR, SPI_SR_TXE));
		if(SPI_8_BIT_FRAME == GET_BIT(SPIx->CR1, SPI_CR1_DFF))
		{
			SPIx->DR = *((uint8 *)pTXBuffer);
			len --;
			(uint8 *)pTXBuffer++;

		}
		else if(SPI_16_BIT_FRAME == GET_BIT(SPIx->CR1, SPI_CR1_DFF))
		{
			SPIx->DR = *((uint16 *)pTXBuffer);
			len -=2;
			(uint16 *)pTXBuffer++;
		}
		else
		{
			/*No action required*/
		}

	}
}

void SPI_EnableOrDisable(SPI_TypeDef* SPIx, SPI_CLK a_Clk)
{
	/*Configure SPE enable Bit to enable or disable SPI*/
	SPIx->CR1 = (SPIx->CR1 & 0xFFFFFFBF)|(a_Clk<<SPI_CR1_SPE);

}

void SPI_ReceiveData(SPI_TypeDef* SPIx, uint8* pRXBuffer, uint32 len)
{
	while(len > 0)
	{
		/*Wait till TXE is empty, since initially it may not be empty*/
		while(BIT_IS_CLEAR(SPIx->SR, SPI_SR_RXNE));
		if(SPI_8_BIT_FRAME == GET_BIT(SPIx->CR1, SPI_CR1_DFF))
		{
		    *((uint8 *)pRXBuffer) = (uint8)SPIx->DR;
			len --;
			(uint8 *)pRXBuffer++;

		}
		else if(SPI_16_BIT_FRAME == GET_BIT(SPIx->CR1, SPI_CR1_DFF))
		{
			*((uint16 *)pRXBuffer) = (uint16)SPIx->DR;
			len -=2;
			(uint16 *)pRXBuffer++;
		}
		else
		{
			/*No action required*/
		}

	}
}

SPI_State SPI_SendDataIT(SPI_PeripheralConfiguration* Config, uint8* pTXBuffer, uint32 len)
{
	SPI_State Busy_State = Config->TxState;
	if(Busy_State != SPI_STATE_BUSY_TX) /*Only configure new buffers when spi is ready, to not overwrite ones we're sending*/
	{
		/*Save info*/
		Config->Ptr_TXBuffer = pTXBuffer;
		Config->TX_TransferLen = len;

		/*Mark state as busy*/
		Config->TxState = SPI_STATE_BUSY_TX;

		if(BIT_IS_CLEAR(Config->Instance->CR2, SPI_CR2_TXEIE))/*If TX interrupt is not set, enable it*/
		{
			SET_BIT(Config->Instance->CR2, SPI_CR2_TXEIE);
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

	return Busy_State;
}

SPI_State SPI_ReceiveDataIT(SPI_PeripheralConfiguration* Config, uint8* pRXBuffer, uint32 len)
{
	SPI_State Busy_State = Config->RxState;

	if(Busy_State != SPI_STATE_BUSY_RX) /*Only configure new buffers when spi is ready, to not overwrite ones we're receiving*/
	{
		/*Save info*/
		Config->Ptr_RXBuffer = pRXBuffer;
		Config->RX_TransferLen = len;

		/*Mark state as busy*/
		Config->RxState = SPI_STATE_BUSY_RX;

		if(BIT_IS_CLEAR(Config->Instance->CR2, SPI_CR2_RXNEIE))/*If RX interrupt is not set, enable it*/
		{
			SET_BIT(Config->Instance->CR2, SPI_CR2_RXNEIE);
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

	return Busy_State;
}

void SPI_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi)
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

void SPI_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority)
{
	/*Configure interrupt priority*/
	NVIC->IPR[(uint8)IRQ_Num/4] = (NVIC->IPR[(uint8)IRQ_Num/4] \
			& (~(15<<((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)))))|(IRQ_Priority << ((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)));
}


void SPI_IRQHandling(SPI_PeripheralConfiguration* Config)
{
	if(BIT_IS_SET(Config->Instance->SR, SPI_SR_TXE) && BIT_IS_SET(Config->Instance->CR2, SPI_CR2_TXEIE))
	{
		SPI_HandleTXInterrupt(Config);
	}

	if(BIT_IS_SET(Config->Instance->SR, SPI_SR_RXNE) && BIT_IS_SET(Config->Instance->CR2, SPI_CR2_RXNEIE))
	{
		SPI_HandleRXInterrupt(Config);
	}

	/*Here we're only implementing Overrun error, may implement other errors if needed*/
	if(BIT_IS_SET(Config->Instance->SR, SPI_SR_OVR) && BIT_IS_SET(Config->Instance->CR2, SPI_CR2_ERRIE))
	{
		SPI_HandleOVRErrorInterrupt(Config);
	}
}

void SPI_SetCallback(void (*a_Ptr)(SPI_PeripheralConfiguration* Config),
		SPI_EventCallBack Event_Set)
{
	/*Set call back for multiple events*/
	if(Event_Set == SPI_TX_COMPLETE)
	{
		g_CallbackPtr[SPI_TX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == SPI_RX_COMPLETE)
	{
		g_CallbackPtr[SPI_RX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == SPI_OVR_ERROR)
	{
		g_CallbackPtr[SPI_OVR_ERROR] = a_Ptr;

	}
	else
	{
		/*No action required*/
	}
}



/*This API will only be called if TXE and TXEIE are set and an interrupt happens
 * it's a common handler for master and slave
 */
static void SPI_HandleTXInterrupt(SPI_PeripheralConfiguration* Config){
	if(SPI_8_BIT_FRAME == GET_BIT(Config->Instance->CR1, SPI_CR1_DFF))
	{
		Config->Instance->DR = *((uint8 *)Config->Ptr_TXBuffer);
		Config->TX_TransferLen --;
		(uint8 *)Config->Ptr_TXBuffer++;

	}
	else if(SPI_16_BIT_FRAME == GET_BIT(Config->Instance->CR1, SPI_CR1_DFF))
	{
		Config->Instance->DR = *((uint16 *)Config->Ptr_TXBuffer);
		Config->TX_TransferLen -=2;
		(uint16 *)Config->Ptr_TXBuffer++;
	}
	else
	{
		/*No action required*/
	}

	if(Config->TX_TransferLen == 0)
	{
		/*Disable interrupt for end of communication and do call back function*/
		CLEAR_BIT(Config->Instance->CR2, SPI_CR2_TXEIE);
		/*Reset buffers*/
		Config->Ptr_TXBuffer = NULL_PTR;
		/*Reset length*/
		Config->TX_TransferLen = 0;
		/*Return state to ready*/
		Config->TxState = SPI_STATE_READY;
		if(g_CallbackPtr[SPI_TX_COMPLETE] != NULL_PTR)
		{
			(*g_CallbackPtr[SPI_TX_COMPLETE])(Config);
		}
	}


}

/*This API will only be called if OVR and ERREIE are set and an interrupt happens*/
static void SPI_HandleRXInterrupt(SPI_PeripheralConfiguration* Config){
	if(SPI_8_BIT_FRAME == GET_BIT(Config->Instance->CR1, SPI_CR1_DFF))
	{
		*((uint8 *)Config->Ptr_RXBuffer) = (uint8)Config->Instance->DR;
		Config->RX_TransferLen --;
		(uint8 *)Config->Ptr_RXBuffer++;

	}
	else if(SPI_16_BIT_FRAME == GET_BIT(Config->Instance->CR1, SPI_CR1_DFF))
	{
		*((uint16 *)Config->Ptr_RXBuffer) = (uint16)Config->Instance->DR;
		Config->RX_TransferLen -=2;
		(uint16 *)Config->Ptr_RXBuffer++;
	}
	else
	{
		/*No action required*/
	}

	if(Config->RX_TransferLen == 0)
	{
		/*Disable interrupt for end of communication and do call back function*/
		CLEAR_BIT(Config->Instance->CR2, SPI_CR2_RXNEIE);
		/*Reset buffers*/
		Config->Ptr_RXBuffer = NULL_PTR;
		/*Reset length*/
		Config->RX_TransferLen = 0;
		/*Return state to ready*/
		Config->RxState = SPI_STATE_READY;
		if(g_CallbackPtr[SPI_RX_COMPLETE] != NULL_PTR)
		{
			(*g_CallbackPtr[SPI_RX_COMPLETE])(Config);
		}

	}

}

static void SPI_HandleOVRErrorInterrupt(SPI_PeripheralConfiguration* Config){
	/*Clear overrun flag then use callback to inform app
	 * cleared by reading DR then SR*/
	uint8 temp;
	/*Make sure data in DR when overrun happens is not used in TX(overrun happens on receiving another byte
	 * when byte in DR is not yet read)*/
	if(Config->RxState != SPI_STATE_BUSY_TX)
	{
		temp = Config->Instance->DR;
		temp = Config->Instance->SR;
	}
	else
	{
		/*No action required*/
	}
	/*If MCU is transmitting don't clear flag(since we don't want to lose data in DR)
	 * but use callback function to inform app and app clears ovr on its own by using
	 * SPI_ClearOVRFlag() API
	 * this call back is called in both cases*/
	if(g_CallbackPtr[SPI_OVR_ERROR] != NULL_PTR)
	{
		(*g_CallbackPtr[SPI_OVR_ERROR])(Config);
	}
}


/*Below functions are only called by app layer in case of aborting transmission abruptly or
 * for clearing OVR */
void SPI_CloseTransmission(SPI_PeripheralConfiguration* Config)
{
	/*Disable interrupt for end of communication and do call back function*/
	CLEAR_BIT(Config->Instance->CR2, SPI_CR2_TXEIE);
	/*Reset buffers*/
	Config->Ptr_TXBuffer = NULL_PTR;
	/*Reset length*/
	Config->TX_TransferLen = 0;
	/*Return state to ready*/
	Config->TxState = SPI_STATE_READY;
}
void SPI_CloseReception(SPI_PeripheralConfiguration* Config)
{
	/*Disable interrupt for end of communication and do call back function*/
	CLEAR_BIT(Config->Instance->CR2, SPI_CR2_RXNEIE);
	/*Reset buffers*/
	Config->Ptr_RXBuffer = NULL_PTR;
	/*Reset length*/
	Config->RX_TransferLen = 0;
	/*Return state to ready*/
	Config->RxState = SPI_STATE_READY;
}

/*Called by application layer when it's called back in handle error interrupt,
 * if state is busy txing*/
void SPI_ClearOVRFlag(SPI_TypeDef* SPIx)
{
	uint8 temp;

	temp = SPIx->DR;
	temp = SPIx->SR;
}

