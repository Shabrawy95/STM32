/********************************************************************************************************************************
 * Module: I2C
 *
 * File Name: stm32f429i_i2c_driver.c
 *
 * Description: source file for I2C module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/

#include "../Inc/stm32f429i_i2c_driver.h"
#include "../Inc/stm32f429i_rcc_driver.h"



static void I2C_HandleTXInterrupt(I2C_PeripheralConfiguration* Config);
static void I2C_HandleRXInterrupt(I2C_PeripheralConfiguration* Config);

I2C_InstanceConfigType I2C_InstanceConfig;
static I2C_TypeDef* Instance_Registers = NULL_PTR;
/*Array of callback pointers*/
__IO static void (*g_CallbackPtr[NO_OF_CALLBACK_EVENTS]) (I2C_PeripheralConfiguration* Config)= \
		{NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR, NULL_PTR};

static inline void I2C_CLR_ADDR_FLAG(I2C_PeripheralConfiguration* Config)
{
	uint16 dummyRead;


	/*MSL = 1 -> MSTR
	 *MSL = 0 -> SLAVE */
	if(BIT_IS_SET(Config->Instance->SR2, I2C_SR2_MSL))
	{	/*I2C is Master*/
		if(Config->Instance_State ==  I2C_STATE_BUSY_RX)
		{
			if(Config->RxSize == 1)
			{
				/*if receving only one byte, disable ack and generate stop and then clear addr and read dr
				 * when RXNE is set*/
				CLEAR_BIT(Config->Instance->CR1, I2C_CR1_ACK);
				/*Clear ADDR*/
			}
			/*Straight Away Clear ADDR, after disabling ack if rxsize is 1
			 * or straightway if rxsize >1, or */
			dummyRead = Config->Instance->SR1;
			dummyRead = Config->Instance->SR2;

		}
		else /*Busy in Tx or whatever*/
		{
			/*Straight Away Clear ADDR, If in Tx Mode*/
			dummyRead = Config->Instance->SR1;
			dummyRead = Config->Instance->SR2;
		}
	}
	else if (BIT_IS_CLEAR(Config->Instance->SR2, I2C_SR2_MSL))
	{
		/* I2C is slave
		 * Straight Away Clear ADDR if in slave no problem*/
		dummyRead = Config->Instance->SR1;
		dummyRead = Config->Instance->SR2;
	}

}


/*********************************************************************************************************************************
 *                    								 Function Defintions    						                             *
 ********************************************************************************************************************************/
void I2C_Init(I2C_PeripheralConfiguration* Config){
	Instance_Registers = Config->Instance;
	I2C_InstanceConfig = Config->Instance_Config;
	/*Implicitly enable clock in RCC*/
	I2C_PeriClockControl(Config->Instance, I2C_InstanceConfig.I2CEnable);
	/*Enable or disable Clock stretching*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFF7F)|(I2C_InstanceConfig.clockStretching<<7);
	/*General Call Enable or Disable*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFBF)|(I2C_InstanceConfig.GeneralCall<<6);
	/*SMBUS MODE and Type*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 &0xFFFFFFFD)|(I2C_InstanceConfig.SMBUSMode<<1);
	if(I2C_InstanceConfig.SMBUSMode == SMBUS_MODE)
	{	/*IF SMBUS mode is enabled configure type (device or host)*/
		Instance_Registers->CR1 = (Instance_Registers->CR1 &0xFFFFFFF7)|(I2C_InstanceConfig.SMBusType<<3);
		/*Enable or disable ARP*/
		Instance_Registers->CR1 = (Instance_Registers->CR1 &0xFFFFFFEF)|(I2C_InstanceConfig.ARPEnable<<4);

	}
	else
	{
		/*No action Required*/
	}
	/*Analog and digital noise filter config.*/
	Instance_Registers->FLTR = (Instance_Registers->FLTR & 0xFFFFFFE0)|(I2C_InstanceConfig.AnalogNoiseFiler<<4)\
			|(I2C_InstanceConfig.DigitalNoiseFiler);
	/*Enable or disable PEC calculation*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFFDF) | (I2C_InstanceConfig.PacketErrorChecking);
	/*Configure address mode, bit 14 should always be set according to RM*/
	Instance_Registers->OAR1 = (Instance_Registers->OAR1 & 0xFFFF3FFF)|(I2C_InstanceConfig.AddressMode <<15)\
			|(1<<14);
	/*First address*/
	if(I2C_InstanceConfig.AddressMode == I2C_10_BIT)
	{
		Instance_Registers->OAR1 = (Instance_Registers->OAR1 & 0xFFFFFC00) | (I2C_InstanceConfig.I2C_Address1 & 0x3FF);
	}
	else if (I2C_InstanceConfig.AddressMode == I2C_7_BIT)
	{
		Instance_Registers->OAR1 = (Instance_Registers->OAR1 & 0xFFFFFF01) | ((I2C_InstanceConfig.I2C_Address1 & 0x7F)<<1);
	}
	else
	{
		/*No Action required*/
	}

	/*If dual addressing is enabled*/
	if(I2C_InstanceConfig.DualAddress == I2C_DUAL_ADDRESS_ENABLE)
	{
		Instance_Registers->OAR2 |= (1<<0); /*Enable dual address*/
		Instance_Registers->OAR1 = (Instance_Registers->OAR1 & 0xFFFFFF01) | ((I2C_InstanceConfig.I2C_Address2 & 0x7F)<<1);

	}
	else
	{
		Instance_Registers->OAR2 &=~ (1<<0); /*Disable dual address*/

	}
	/*Setting clock input to I2C, max is APB1 bus freq*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFFFE0) | (I2C_InstanceConfig.FREQUENCY);


	if(I2C_InstanceConfig.SCLSpeed <= I2C_SCL_SPEEd_SM) /*Standard Mode*/
	{
		/*Master mode:  standard mode*/
		Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFF7FFF) | (I2C_STANDARD_MODE << 15);
		/*Configuring T rise*/
		Instance_Registers->TRISE = (Instance_Registers->TRISE & 0xFFFFFFC0)|(I2C_InstanceConfig.FREQUENCY +1);

		/* T High = T Low, clock speed is in KHZ e.g 100khz
		 * Thigh = CCR * TPCLK1
		 * Tscl = 1/clock_speed
		 * tscl = Thigh + TLow
		 * thigh = tlow =  tscl/2 = 1/2clock_speed
		 * CCR = 1/tpclk1 * thigh = fpclk1* 1/2*clock_speed
		 * ((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.ClockSpeed<<2))**/

		/*In standard mode duty is always one to one
		 * Below line is for CCR value*/
		Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFFF000)|(uint16)((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.SCLSpeed*2));

	}
	else if(I2C_InstanceConfig.SCLSpeed > I2C_SCL_SPEEd_SM) /*Fast mode*/
	{	/*Master mode:  Fast mode*/
		Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFF7FFF) | (I2C_FAST_MODE << 15);
		/*Set duty cycle 2:1 or 16:9*/
		Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFFBFFF) | (I2C_InstanceConfig.FastModeDuty << 14);
		/*Configuring T rise
		 * Trise = max trise * fpclk1 +1
		 * 		 = 1000ns * freq * (1000000 for mhz) +1 =
		 * 		 = freq in unis +1 */
		Instance_Registers->TRISE = (Instance_Registers->TRISE & 0xFFFFFFC0)|(I2C_InstanceConfig.FREQUENCY+1);
		if(I2C_InstanceConfig.FastModeDuty == I2C_2_to_1)
		{
			/* T High = T Low, clock speed is in KHZ e.g 100khz
			 * Thigh = CCR * TPCLK1
			 * Tscl = 1/clock_speed
			 * tscl = Thigh + TLow
			 * tlow = 2 t high
			 * tscl = 3 t high
			 * thigh = 0.5 * tlow  =   tscl/3 = 1/3clock_speed
			 * CCR = 1/tpclk1 * thigh = fpclk1* 1/3*clock_speed
			 * ((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.ClockSpeed*3))**/
			Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFFF000) |(uint16)((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.SCLSpeed*3));
		}
		else if (I2C_InstanceConfig.FastModeDuty == I2C_16_to_9)
		{
			/* T High = T Low, clock speed is in KHZ e.g 100khz
			 * Thigh = 9 * CCR * TPCLK1
			 * Tlow = 16 * CCR * TPCLK1
			 * CCR = 1/9 * fpclk1* thigh
			 * t high + tlow = tscl = 9/25 * tscl
			 * CCR = 1/9 * fpclk1* 9/25 tscl
			 * CCR = fpclk * 1/25 tscl
			 * CCR = ((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.ClockSpeed*25)
			 *   */
			Instance_Registers->CCR = (Instance_Registers->CCR & 0xFFFFF000) |(uint16)((I2C_InstanceConfig.FREQUENCY *1000000)/(I2C_InstanceConfig.SCLSpeed*25));

		}
		else{
			/*No action required*/
		}
	}
	else
	{
		/*No action required*/
	}
	/*Configuring DMA Requests, and buffer, event, error interrupts enable or disable*/
	Instance_Registers->CR2 = (Instance_Registers->CR2 & 0xFFFFE0FF)|(I2C_InstanceConfig.DMARequestEnable <<11)|(I2C_InstanceConfig.BufferInterrupt <<10)\
			|(I2C_InstanceConfig.EventInterrupt <<9)|(I2C_InstanceConfig.ErrorInterrupt <<8);
	/*Enable I2C*/
	Instance_Registers->CR1 =  (Instance_Registers->CR1 & 0xFFFFFFFE)|I2C_InstanceConfig.I2CEnable;
	/*Enable or disable Ack only after PE bit (I2CEnable) is enabled*/
	Instance_Registers->CR1 = (Instance_Registers->CR1 & 0xFFFFFBFF)|(I2C_InstanceConfig.Acknowledge<<10);
}


void I2C_DeInit(I2C_TypeDef* I2Cx)
{
	if(I2C1 == I2Cx)
	{
		I2C1_REG_RESET();
	}
	else if(I2C2 == I2Cx)
	{
		I2C2_REG_RESET();
	}
	else if(I2C3 == I2Cx)
	{
		I2C3_REG_RESET();
	}
	else{
		/*No action required*/
	}
}

void I2C_PeriClockControl(I2C_TypeDef* I2Cx, I2C_CLK a_CLK)
{
	if(I2C_EN == a_CLK)
	{
		if(I2C1 == I2Cx)
		{
			I2C1_PCLK_EN();
		}
		else if(I2C2 == I2Cx)
		{
			I2C2_PCLK_EN();
		}
		else if(I2C3 == I2Cx)
		{
			I2C3_PCLK_EN();
		}
		else{
			/*No action required*/
		}
	}
	else if (I2C_DIS == a_CLK)
	{
		if(I2C1 == I2Cx)

		{
			I2C1_PCLK_DIS();
		}
		else if(I2C2 == I2Cx)
		{
			I2C2_PCLK_DIS();
		}
		else if(I2C3 == I2Cx)
		{
			I2C3_PCLK_DIS();
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




/*********/

///*Blocking*/
//void I2C_SendData(I2C_TypeDef* I2Cx, uint8* pTXBuffer, uint32 len);
//
//void I2C_ReceiveData(I2C_TypeDef* I2Cx, uint8* pRXBuffer, uint32 len);
//
//
///* non Blocking*/
//I2C_State I2C_SendDataIT(I2C_PeripheralConfiguration* Config, uint8* pTXBuffer, uint32 len);
//
//I2C_State I2C_ReceiveDataIT(I2C_PeripheralConfiguration* Config, uint8* pRXBuffer, uint32 len);

/*******/

void I2C_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi)
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

void I2C_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority)
{
	/*Configure interrupt priority*/
	NVIC->IPR[(uint8)IRQ_Num/4] = (NVIC->IPR[(uint8)IRQ_Num/4] \
			& (~(15<<((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)))))|(IRQ_Priority << ((8*(IRQ_Num%4))+(8-NO_OF_PRIO_BITS)));
}


void I2C_MasterSendData(I2C_PeripheralConfiguration* Config, uint8* pTXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr)/*Sr is repeated start*/
{
	/*Generate Start Condition*/
	Config->Instance->CR1 |= (1<<I2C_CR1_START);
	/*Used by master to wait till start is generated successfully*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_SB));
	/*Send address phase with R/W = 0 (Write)*/
	Config->Instance->DR = (slaveAddr<<1)& (~(1<<0)); /*1<<0 to clear bit for writing*/
	/*Used to confirm that address is sent succefully*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_ADDR));
	/*Clear address flag by reading sr1 then sr2 */
	I2C_CLR_ADDR_FLAG(Config);
	/*Send data until Len is 0*/
	while(len >0)
	{	/*Wait till TXE is set (Data register is empty)*/
		while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_TXE));
		Config->Instance->DR = *((uint8*)pTXBuffer);
		len--;
		(uint8*)pTXBuffer++;
	}
	/*Wait till TXE is set (Data register is empty), redundant in IMO since BTF implies that*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_TXE));
	/*Wait till BTF is set (Data register is empty, Shift register is empty)*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_BTF));


	if(Sr == I2C_NO_REPEATED_START)
	{
		/*Generate stop bit only if repeated start is zero*/
		Config->Instance->CR1 |=(1<<I2C_CR1_STOP);
	}
	else
	{
		/*No action required*/
	}
}

void I2C_MasterReceiveData(I2C_PeripheralConfiguration* Config, uint8* pRXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr)
{
	/*Generate Start Condition*/
	Config->Instance->CR1 |= (1<<I2C_CR1_START);
	/*Used by master to wait till start is generated successfully*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_SB));
	/*Send address phase with R/W = 1 (Read)*/
	Config->Instance->DR = (slaveAddr<<1)|(1<<0); /*1<<0 to set bit for reading*/
	/*Used to confirm that address is sent succefully*/
	while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_ADDR));

	if(len == 1)
	{
		/*if receving only one byte, disable ack and generate stop and then clear addr and read dr
		 * when RXNE is set*/
		/*Clear Ack, then Clear address flag by reading sr1 then sr2 to start receiving byte*/
		I2C_CLR_ADDR_FLAG(Config); /*Ack is cleared inside function*/

		/*Wait till RXNE is set to read byte*/
		while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_RXNE));

		if(Sr == I2C_NO_REPEATED_START)
		{
			/*Generate stop bit only if repeated start is zero*/
			Config->Instance->CR1 |=(1<<I2C_CR1_STOP);
		}
		else
		{
			/*No action required*/
		}

		*((uint8*)pRXBuffer) = Config->Instance->DR;

		/*If acking was enabled in instance config then re enable it before leaving API*/
		if(Config->Instance_Config.Acknowledge == I2C_ACK_ENABLE)
		{
			/*Re enable acking after finishing data reception,
			 * for other receptions*/
			SET_BIT(Config->Instance->CR1, I2C_CR1_ACK);
		}
		return;

	}

	if(len > 1)
	{

		/*Clear address flag by reading sr1 then sr2 to start receiving bytes*/
		I2C_CLR_ADDR_FLAG(Config);
		while(len > 0)
		{

			/*Wait till RXNE is set to read byte*/
			while(BIT_IS_CLEAR(Config->Instance->SR1,I2C_SR1_RXNE));
			if(len == 2)
			{
				/*if receving last byte, disable ack and generate stop and then clear addr and read dr
				 * when RXNE is set*/
				CLEAR_BIT(Config->Instance->CR1, I2C_CR1_ACK);
				if(Sr == I2C_NO_REPEATED_START)
				{
					/*Generate stop bit only if repeated start is zero*/
					Config->Instance->CR1 |=(1<<I2C_CR1_STOP);
				}
				else
				{
					/*No action required*/
				}
			}
			*((uint8*)pRXBuffer) = Config->Instance->DR;
			len--;
			(uint8*)pRXBuffer++;
		}

	}
	/*If acking was enabled in instance config then re enable it before leaving API*/
	if(Config->Instance_Config.Acknowledge == I2C_ACK_ENABLE)
	{
		/*Re enable acking after finishing data reception,
		 * for other receptions*/
		SET_BIT(Config->Instance->CR1, I2C_CR1_ACK);
	}

}

/* non Blocking*/
I2C_State I2C_MasterSendDataIT(I2C_PeripheralConfiguration* Config, uint8* pTXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr)
{
	I2C_State Busy_State = Config->Instance_State;
	if((Busy_State != I2C_STATE_BUSY_TX) || (Busy_State != I2C_STATE_BUSY_RX))
	{
		/*Save Tx Buffer*/
		Config->Ptr_TXBuffer = pTXBuffer;
		/*Save slave address*/
		Config->devAddress = slaveAddr;
		/*Save TX buffer length*/
		Config->TX_TransferLen = len;
		/*Set state to busy TX*/
		Config->Instance_State = I2C_STATE_BUSY_TX;
		/*Set repeated start config*/
		Config->Sr = Sr;
		/*Generate Start Condition*/
		Config->Instance->CR1 |= (1<<I2C_CR1_START);
		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITEVTEN))
		{
			/*Enable Event interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITEVTEN);
		}
		else
		{
			/*No action required*/
		}

		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITBUFEN))
		{
			/*Enable Buffer interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITBUFEN);
		}
		else
		{
			/*No action required*/
		}


		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITERREN))
		{
			/*Enable Error interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITERREN);
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
	/*Return State*/
	return  Busy_State;
;

}
I2C_State I2C_MasterReceiveDataIT(I2C_PeripheralConfiguration* Config, uint8* pRXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr)
{
	I2C_State Busy_State = Config->Instance_State;
	if((Busy_State != I2C_STATE_BUSY_RX) || (Busy_State != I2C_STATE_BUSY_TX))
	{
		/*Save Rx Buffer*/
		Config->Ptr_RXBuffer = pRXBuffer;
		/*Save slave address*/
		Config->devAddress = slaveAddr;
		/*Save TX buffer length*/
		Config->RX_TransferLen = len;
		/*Set state to busy RX*/
		Config->Instance_State = I2C_STATE_BUSY_RX;
		/*Set repeated start config*/
		Config->Sr = Sr;
		/**/
		Config->RxSize = len;
		/*Generate Start Condition*/
		Config->Instance->CR1 |= (1<<I2C_CR1_START);

		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITEVTEN))
		{
			/*Enable Event interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITEVTEN);
		}
		else
		{
			/*No action required*/
		}

		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITBUFEN))
		{
			/*Enable Buffer interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITBUFEN);
		}
		else
		{
			/*No action required*/
		}


		if(BIT_IS_CLEAR(Config->Instance->CR2, I2C_CR2_ITERREN))
		{
			/*Enable Error interrupt*/
			SET_BIT(Config->Instance->CR2, I2C_CR2_ITERREN);
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
	/*Return State*/
	return Busy_State;


}

void I2C_SlaveSendData(I2C_TypeDef* I2Cx, uint8 val)
{
	I2Cx->DR = val;
}
uint8 I2C_SlaveReceiveData(I2C_TypeDef* I2Cx)
{
	return (uint8)I2Cx->DR;
}

/*Event IRQ handling for Master and Slave*/
void I2C_EventIRQHandling(I2C_PeripheralConfiguration* Config)
{
	/*If SB is set the start is sent, check for event interrupt enable,
	 * SB Flag*/
	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_SB)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN))
	{
		/*1- Handle for SB interrupt, SB flag is valid in master mode only, slave's SB is always zero*/

		/*Start is sent succefully so execute address phase here*/
		if(Config->Instance_State ==  I2C_STATE_BUSY_TX)
		{
			/*Send address phase with R/W = 0 (Write)*/
			Config->Instance->DR = (Config->devAddress<<1)& (~(1<<0)); /*1<<0 to clear bit for writing*/
		}
		else if (Config->Instance_State ==  I2C_STATE_BUSY_RX)
		{
			/*Send address phase with R/W = 1 (Read)*/
			Config->Instance->DR = (Config->devAddress<<1)|(1<<0); /*1<<0 to set bit for reading*/
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

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_ADDR)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN))
	{
		/*2- Handle for Address phase sent interrupt(ADDR = 1), ADDR flag is valid in master mode only
		 * Master: Address sent successfully
		 * Slave: Address is matched*/

		/*ADDR = 1 and clock is stretched until cleared
		 * Just clear ADDR Here,
		 *Clear address flag by reading sr1 then sr2 to start receiving bytes*/
		I2C_CLR_ADDR_FLAG(Config);
	}
	else
	{
		/*No action Required*/
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_BTF)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN))
	{
		/*3- Handle for BTF event interrupt*/

		/*If application state is busy in TX Sending data,
		 * in this Case BTF is used to check last byte is sent,
		 * and if no repeated start then generate stop*/
		if(Config->Instance_State ==  I2C_STATE_BUSY_TX)
		{
			/*Make sure that TXE is also set(Last byte is sent DR + SR empty)*/
			if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_TXE))
			{
				/*IF BTF = 1, TXE = 1, check length,
				 * ig length is equal to zero then Close transmission*/
				if(Config->TX_TransferLen == 0)
				{
					/* 1- Generate Stop */
					if(Config->Sr == I2C_NO_REPEATED_START)
					{
						/*Generate stop bit only if repeated start is zero*/
						Config->Instance->CR1 |=(1<<I2C_CR1_STOP);
					}
					/* 2- Close transmission (Disable interrupts, reset config struct) */
					CLEAR_BIT(Config->Instance->CR2, I2C_CR2_ITEVTEN);
					CLEAR_BIT(Config->Instance->CR2, I2C_CR2_ITBUFEN);
					Config->Instance_State = I2C_STATE_READY;
					Config->Ptr_TXBuffer = NULL_PTR;
					Config->TX_TransferLen = 0;

					/* 3- Notify app of transmission completion */
					if(g_CallbackPtr[I2C_TX_COMPLETE] != NULL_PTR)
					{
						(*g_CallbackPtr[I2C_TX_COMPLETE])(Config);
					}
				}

			}
			else{
				/*No action required*/
			}
		}
		else if (Config->Instance_State ==  I2C_STATE_BUSY_RX)
		{
			/*Nothing to do here when BTF is equal to One*/
		}
		else
		{
			/*No action Required*/
		}


	}
	else
	{
		/*No action required*/
	}

	/*Stop flag is only applicable for Slave, Master will never have Stop flag be set,
	 * so below code block won't be executed for master*/
	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_STOPF)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN))
	{
		/*4- Handle for Stop Condition event interrupt
		 * Note: Stop condition flag is applicable only in slave
		 * STOPF = 1, Cleared by reading SR1 then writing CR1
		 * Reading SR1 is already done in the if condition*/
		Config->Instance->CR1 |= 0; /*making or with zero to not change contents
									but to write to CR1 to clear STOPF*/
		if(g_CallbackPtr[I2C_STOP_DETECTED] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_STOP_DETECTED])(Config);
		}

	}
	else
	{
		/*No action required*/
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_TXE)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITBUFEN))
	{
		/*5- Handle for Transmit Data register empty interrupt, TXE = 1
		 *Do Data transmission Here*/

		/*Only transmit data when Master:
		 * Check if Master Using MSL bit in SR2
		 * MSL = 0 -> Device is slave
		 * MSL = 1 -> Device is Master*/
		if(BIT_IS_SET(Config->Instance->SR2, I2C_SR2_MSL))
		{
			if(Config->Instance_State ==  I2C_STATE_BUSY_TX)
			{
				I2C_HandleTXInterrupt(Config);
			}
		}
		else if(BIT_IS_CLEAR(Config->Instance->SR2, I2C_SR2_MSL))
		{
			/*Slave TX, Data request from Master TXE = 1, send data from slave
			 * Check first that I2C is in Slave transmitter mode, i think it's
			 * redundant cause TXE will never be set for Slave Receiver
			 * Master reading from slave*/
			/*TRA = 1: Transmitter
			 * TRA = 0: receiver
			 * TRA is determined by RW bit of address phase*/
			if(BIT_IS_SET(Config->Instance->SR2, I2C_SR2_TRA))
			{
				/*If Transmitter mode, use callback function*/
				if(g_CallbackPtr[I2C_DATA_REQUEST] != NULL_PTR)
				{
					(*g_CallbackPtr[I2C_DATA_REQUEST])(Config);
				}
			}

		}

	}
	else
	{
		/*No action required*/
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_RXNE)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITEVTEN)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITBUFEN))
	{
		/*Only receive data when Master:
		 * Check if Master Using MSL bit in SR2
		 * MSL = 0 -> Device is slave
		 * MSL = 1 -> Device is Master*/
		if(BIT_IS_SET(Config->Instance->SR2, I2C_SR2_MSL))
		{
			/*The device is master*/
			/*5- Handle for Receive Data register Full interrupt*, RXNE = 1*/
			if(Config->Instance_State ==  I2C_STATE_BUSY_RX)
			{
				I2C_HandleRXInterrupt(Config);
			}
		}
		else if(BIT_IS_CLEAR(Config->Instance->SR2, I2C_SR2_MSL))
		{
			/*Data write from Master,RXNE = 1 -> DR is full, slave receives data
			 * * Check first that I2C is in Slave receiver mode, i think it's
			 * redundant cause RXNE will never be set for Slave Transmitter*/

			/*TRA = 1: Transmitter
			 * TRA = 0: receiver
			 * TRA is determined by RW bit of address phase*/
			if(BIT_IS_CLEAR(Config->Instance->SR2, I2C_SR2_TRA))
			{
				/*If Transmitter mode, use callback function*/
				if(g_CallbackPtr[I2C_DATA_RECEIVE] != NULL_PTR)
				{
					(*g_CallbackPtr[I2C_DATA_RECEIVE])(Config);
				}
			}

		}
		else
		{
			/*No action required*/
		}
	}

}

static void I2C_HandleTXInterrupt(I2C_PeripheralConfiguration* Config)
{
	/*Transmit only when state is Busy TX, I think if condition is redundant*/
	if(Config->TX_TransferLen > 0)
	{
		/*1. Load data to DR*/
		Config->Instance->DR = *((uint8*)(Config->Ptr_TXBuffer));
		/*2. Decrement TX length*/
		Config->TX_TransferLen--;
		/*3. Increment Poiner*/
		Config->Ptr_RXBuffer++;
	}
}

static void I2C_HandleRXInterrupt(I2C_PeripheralConfiguration* Config)
{
	/*Check RxSize and not length*/
	if(Config->RxSize == 1)
	{

		*((uint8*)Config->Ptr_RXBuffer) = Config->Instance->DR;
		Config->RX_TransferLen--;


	}

	if(Config->RxSize > 1)
	{

		if(Config->RX_TransferLen == 2)
		{

			CLEAR_BIT(Config->Instance->CR1, I2C_CR1_ACK);

		}
		*((uint8*)Config->Ptr_RXBuffer) = Config->Instance->DR;
		Config->RX_TransferLen--;
		(uint8*)Config->Ptr_RXBuffer++;
	}

	if(Config->RX_TransferLen == 0)
	{
		/*End of transmission*/

		/*1. Generate Stop condition*/
		if(Config->Sr == I2C_NO_REPEATED_START)
		{
			/*Generate stop bit only if repeated start is zero*/
			Config->Instance->CR1 |=(1<<I2C_CR1_STOP);
		}
		/* 2- Close Reception (Disable interrupts, reset config struct) */
		CLEAR_BIT(Config->Instance->CR2, I2C_CR2_ITEVTEN);
		CLEAR_BIT(Config->Instance->CR2, I2C_CR2_ITBUFEN);
		Config->Instance_State = I2C_STATE_READY;
		Config->Ptr_RXBuffer = NULL_PTR;
		Config->RX_TransferLen = 0;
		Config->RxSize = 0;
		/*If acking was enabled in instance config then re enable it before leaving API*/
		if(Config->Instance_Config.Acknowledge == I2C_ACK_ENABLE)
		{
			/*Re enable acking after finishing data reception,
			 * for other receptions*/
			SET_BIT(Config->Instance->CR1, I2C_CR1_ACK);
		}
		/*3. Notify App of Rx Complettion*/
		if(g_CallbackPtr[I2C_RX_COMPLETE] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_RX_COMPLETE])(Config);
		}


	}
}



void I2C_ErrorIRQHandling(I2C_PeripheralConfiguration* Config)
{
	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_BERR)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITERREN))
	{

		/* 1- Clear Bus Error and notify App
		 * Cleared by writing Zero */
		Config->Instance->SR1 &= ~ (1<<I2C_SR1_BERR);
		if(g_CallbackPtr[I2C_BUS_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_BUS_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_ARLO)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITERREN))
	{

		/* 2- Clear Arbitration loss Error and notify App
		 * Cleared by writing Zero */
		Config->Instance->SR1 &= ~ (1<<I2C_SR1_ARLO);
		if(g_CallbackPtr[I2C_ARBITRATION_LOSS_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_ARBITRATION_LOSS_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_AF)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITERREN))
	{

		/* 3- Clear Ack Failure Error and notify App
		 * Cleared by writing Zero */
		Config->Instance->SR1 &= ~ (1<<I2C_SR1_AF);
		if(g_CallbackPtr[I2C_ACK_FAILURE_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_ACK_FAILURE_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_OVR)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITERREN))
	{

		/* 4- Clear Overrun Error and notify App
		 * Cleared by writing Zero */
		Config->Instance->SR1 &= ~ (1<<I2C_SR1_OVR);
		if(g_CallbackPtr[I2C_OVERRUN_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_OVERRUN_ERROR])(Config);
		}
	}

	if(BIT_IS_SET(Config->Instance->SR1,I2C_SR1_TIMEOUT)\
			&& BIT_IS_SET(Config->Instance->CR2, I2C_CR2_ITERREN))
	{

		/* 5- Clear Timeout Error and notify App
		 * Cleared by writing Zero */
		Config->Instance->SR1 &= ~ (1<<I2C_SR1_TIMEOUT);
		if(g_CallbackPtr[I2C_TIMEOUT_ERROR] != NULL_PTR)
		{
			(*g_CallbackPtr[I2C_TIMEOUT_ERROR])(Config);
		}
	}
}

void I2C_SetCallback(void (*a_Ptr)(I2C_PeripheralConfiguration* Config),
		I2C_EventCallBack Event_Set)
{

	/*Set call back for multiple events*/
	if(Event_Set == I2C_TX_COMPLETE)
	{
		g_CallbackPtr[I2C_TX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == I2C_RX_COMPLETE)
	{
		g_CallbackPtr[I2C_RX_COMPLETE] = a_Ptr;

	}
	else if (Event_Set == I2C_STOP_DETECTED)
	{
		g_CallbackPtr[I2C_STOP_DETECTED] = a_Ptr;

	}
	else if (Event_Set == I2C_BUS_ERROR)
	{
		g_CallbackPtr[I2C_BUS_ERROR] = a_Ptr;

	}
	else if (Event_Set == I2C_ARBITRATION_LOSS_ERROR)
	{
		g_CallbackPtr[I2C_ARBITRATION_LOSS_ERROR] = a_Ptr;

	}
	else if (Event_Set == I2C_DATA_REQUEST)
	{
		g_CallbackPtr[I2C_DATA_REQUEST] = a_Ptr;

	}
	else if (Event_Set == I2C_DATA_RECEIVE)
	{
		g_CallbackPtr[I2C_DATA_RECEIVE] = a_Ptr;

	}
	else if (Event_Set == I2C_ACK_FAILURE_ERROR)
	{
		g_CallbackPtr[I2C_ACK_FAILURE_ERROR] = a_Ptr;

	}
	else if (Event_Set == I2C_OVERRUN_ERROR)
	{
		g_CallbackPtr[I2C_OVERRUN_ERROR] = a_Ptr;

	}
	else if (Event_Set == I2C_TIMEOUT_ERROR)
	{
		g_CallbackPtr[I2C_TIMEOUT_ERROR] = a_Ptr;

	}
	else
	{
		/*No action required*/
	}



}


