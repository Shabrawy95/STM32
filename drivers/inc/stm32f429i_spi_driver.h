 /********************************************************************************************************************************
 * Module: SPI
 *
 * File Name: stm32f429i_spi_driver.h
 *
 * Description: header file for SPI module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/



#ifndef STM32F4XX_HAL_DRIVER_INC_STM32F429I_SPI_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_STM32F429I_SPI_DRIVER_H_

/*MCU Specific Header File*/
#include "stm32f429xx.h"

#define SPI_BUSY 				1
#define SPI_NOT_BUSY 			0
#define NO_OF_CALLBACK_EVENTS	3
/*********************************************************************************************************************************
 *                    								 Type defintions   						                                 *
 ********************************************************************************************************************************/

/*Type defintion to select SPI to configure*/
typedef enum{
	/*Use SPI 1 and configure it*/
	SPI1_PERIPHERAL,
	/*Use SPI 2 and configure it*/
	SPI2_PERIPHERAL,
	/*Use SPI 3 and configure it*/
	SPI3_PERIPHERAL,
	/*Use SPI 4 and configure it*/
	SPI4_PERIPHERAL,
	/*Use SPI 5 and configure it*/
	SPI5_PERIPHERAL,
	/*Use SPI 6 and configure it*/
	SPI6_PERIPHERAL
}SPI_PeripheralSelect;

/*******************************************************SPI_CR1 Types************************************************************/

/*Type defintion to enable disable SPI*/
typedef enum{
	SPI_DIS,
	SPI_EN

}SPI_CLK;

/*Type definition to set SPI direction mode, Bidirectional or unidirectional*/
typedef enum{
	/*Unidirectional is enabled here*/
	SPI_UNI_ENABLE,
	/*Bidirectional Enable*/
	SPI_BIDI_ENABLE

}SPI_Bidirection;

/*Type definition to set SPI transfer mode when spi is configured as Bidirectional */
typedef enum{
	/*In case of Unidirectional it's not used*/
	SPI_BIDI_UNUSED = 0,
	/*RX only mode, input enabled, output is disabled*/
	SPI_BIDI_RXONLY=0,
	/*TX only mode, input is disabled, output is enabled*/
	SPI_BIDI_TXONLY = 1

}SPI_BidirectionTransferMode;

/*Type definition to enable or disable SPI CRC Calculation
 *  This bit should be written only when SPI is disabled
 *  (SPE = ‘0’) for correct operation*/
typedef enum{
	/*CRC Calculation Disabled*/
	SPI_CRC_DISABLE,
	/*CRC Calculation Enabled*/
	SPI_CRC_ENABLE
}SPI_CRCEnable;

/*Type definition to select SPI CRC Transfer Next
 *
 *  When the SPI is configured in full duplex or transmitter only modes, CRCNEXT must be
 *	written as soon as the last data is written to the SPI_DR register.
 *	When the SPI is configured in receiver only mode, CRCNEXT must be set after the
 *	second last data reception.
 *	This bit should be kept cleared when the transfers are managed by DMA
 * */
typedef enum{
	/*Data Phase*/
	SPI_NO_CRC_PHASE,
	/*Next transfer is CRC (CRC Phase)*/
	SPI_CRC_PHASE
}SPI_CRCNext;

/*Type definition to set SPI Frame length in transmission and reception
 * This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation*/
typedef enum{
	SPI_8_BIT_FRAME,
	SPI_16_BIT_FRAME
}SPI_FrameLength;

/*Type definition to set SPI Frame bit order in transmission and reception
 * This bit should not be changed when communication is ongoing*/
typedef enum{
	/*Transmit most significant bit first*/
	SPI_MSB_FIRST,
	/*Transmit least significant bit first*/
	SPI_LSB_FIRST
}SPI_FrameBitOrder;

/*Type definition to set SPI transfer mode when spi is configured as unidirectional */
typedef enum{
	/*In case of Bidirectional mode it's not used*/
	SPI_UNIDI_UNUSED = 0,
	/*Full Duplex mode, input enabled, output is enabled*/
	SPI_UNIDI_FULL_DUPLEX = 0,
	/*RX only mode, input is enabled, output is disabled*/
	SPI_UNIDI_RXONLY = 1

}SPI_UnidirectionTransferMode;

/* Type definition to enable software management of slaves,
 * when enabled, NSS pin value is ignored and instead
 * it takes value in SSI bit which is written by software(programmer)*/
typedef enum{
	/*Value is set from NSS pin normally*/
	SPI_SSM_DISABLED,
	/*Slave select value is in SSI Bit*/
	SPI_SSM_ENABLED
}SPI_SWSlaveManagement;

/* Type definition to ste SPi frequency prescaler, frequency
 * is frequency of the bus on which the SPI hangs
 *
 * These bits should not be changed when communication is ongoing.*/
typedef enum{
	/*Divide clock of bus by 2*/
	SPI_FPCLK_2,
	/*Divide clock of bus by 4*/
	SPI_FPCLK_4,
	/*Divide clock of bus by 8*/
	SPI_FPCLK_8,
	/*Divide clock of bus by 16*/
	SPI_FPCLK_16,
	/*Divide clock of bus by 32*/
	SPI_FPCLK_32,
	/*Divide clock of bus by 64*/
	SPI_FPCLK_64,
	/*Divide clock of bus by 128*/
	SPI_FPCLK_128,
	/*Divide clock of bus by 256*/
	SPI_FPCLK_256,
}SPI_FREQUENCY;

/*Type definition to select configuration mode (Master configuration or Slave configuration)
 *
 * This bit should not be changed when communication is ongoing*/
typedef enum{
	/*SPI Slave configuration mode*/
	SPI_SLAVE_CONFIG,
	/*SPI Master configuration mode*/
	SPI_MASTER_CONFIG
}SPI_ConfigurationMode;

/*Type definition to set SPI polarity and phase
 *
 *These bits should not be changed when communication is ongoing*/
typedef enum{
	/*MODE 0: CPOL 0 (Clock is zero when idle) CPHA 0 (sample on leading edge)*/
	SPI_POL_PHASE_MODE_0,
	/* MODE 1: CPOL 0 (Clock is zero when idle) CPHA 1 (sample on trailing edge)*/
	SPI_POL_PHASE_MODE_1,
	/*MODE 2: CPOL 1 (Clock is One when idle) CPHA 0 (sample on leading edge)*/
	SPI_POL_PHASE_MODE_2,
	/*MODE 3: CPOL 1 (Clock is One when idle) CPHA 1 (sample on trailing edge)*/
	SPI_POL_PHASE_MODE_3
}SPI_PolarityAndPhaseMode;

/*******************************************************SPI_CR2 Types************************************************************/

/*Type definition to enable and disable SPI TX interrupt*/
typedef enum{
	/*TX interrupt Disabled*/
	SPI_TX_INT_DISABLED,
	/*TX interrupt enabled, used to generate an interrupt request when the TXE flag is set*/
	SPI_TX_INT_ENABLED
}SPI_TransmitInterrupt;

/*Type definition to enable and disable SPI RX interrupt*/
typedef enum{
	/*RX interrupt Disabled*/
	SPI_RX_INT_DISABLED,
	/*RX interrupt enabled,Used to generate an interrupt request when the RXNE flag is set. */
	SPI_RX_INT_ENABLED
}SPI_ReceiveInterrupt;

/*Type definition to enable and disable SPI Error interrupt
 *
 * This bit controls the generation of an interrupt when an error condition occurs )(CRCERR,
 *  OVR, MODF in SPI mode, FRE in TI mode and UDR, OVR, and FRE in I2S mode)*/
typedef enum{
	/*Error interrupt Disabled*/
	SPI_ERROR_INT_DISABLED,
	/*Error interrupt Enabled*/
	SPI_ERROR_INT_ENABLED

}SPI_ErrorInterrupt;

/*Type defintion to set SPI frame format (TI or Motorola)*/
typedef enum{
	/*SPI Motorola Mode*/
	SPI_MOTOROLA_FRAME,
	/*SPI TI Mode*/
	SPI_TI_FRAME
}SPI_FrameFormat;

/*type defintion to enable and disable SS output, not used in SPI TI mode*/
typedef enum{
	/* SS output is disabled in master mode and the cell can work in multimaster configuration*/
	SPI_SS_OUTPUT_DISABLED,
	/* SS output is enabled in master mode and when the cell is enabled. The cell cannot work
	in a multimaster environment.*/
	SPI_SS_OUTPUT_ENABLED
}SPI_SSOutput;

/*Type definition to enable/disable TX buffer DMA request being made whenever the TXE flag is set.*/
typedef enum{
	/*Tx buffer DMA disabled*/
	SPI_TX_DMA_DISABLED,
	/*Tx buffer DMA enabled*/
	SPI_TX_DMA_ENABLED
}SPI_TransmitDMA;

/*Type definition to enable/disable RX buffer DMA request being made whenever the RXNE flag is set.*/
typedef enum{
	/*Rx buffer DMA disabled*/
	SPI_RX_DMA_DISABLED,
	/*Rx buffer DMA enabled*/
	SPI_RX_DMA_ENABLED
}SPI_ReceiveDMA;

/*Type defintion to hold SPI state
 * applicaiton has to check state before initiating any SPI transaction,
 * proceed only state is ready*/
typedef enum{
	/*SPI initialized and ready to use*/
	SPI_STATE_READY = 0,
	/*Data reception process is ongoing*/
	SPI_STATE_BUSY_RX = 1,
	/*Data transmission process is ongoing*/
	SPI_STATE_BUSY_TX = 2
}SPI_State;

/*Events for callback function*/

typedef enum{
	SPI_TX_COMPLETE ,
	SPI_RX_COMPLETE,
	SPI_OVR_ERROR
}SPI_EventCallBack;

/*Type defintion  for Config Struct*/
typedef struct{
	/*select SPI to configure*/
	SPI_PeripheralSelect Peripheral_Select;
	/*Enable or disable SPI*/
	SPI_CLK SPIEnable;
	/*Enable Unidirectional or bidirectional mode*/
	SPI_Bidirection Bidirection_Unidirection;
	/*Select mode in case of Bidirectional (TX only or RX only)*/
	SPI_BidirectionTransferMode BI_Mode;
	/*Enable or disable CRC Calculation*/
	SPI_CRCEnable CRC_Enable;
	/*SPI CRC transfer Next (Data phase or CRC phase)*/
	SPI_CRCNext CRC_Next;
	/*Frame Lenght 8 or 16 bit*/
	SPI_FrameLength Frame_Length;
	/*MSB o rLSB First*/
	SPI_FrameBitOrder Frame_Order;
	/*select mode in case of Unidirectional (Full duplex or RX only)*/
	SPI_UnidirectionTransferMode Uni_Mode;
	/*Configure Software Slave management, enable(NSS pin value is ignored and SSI bit is used instead) */
	SPI_SWSlaveManagement SSM_Management;
	/*Select Spi Frequency prescaler*/
	SPI_FREQUENCY Freq_Divide;
	/*Use master or Slave configuration*/
	SPI_ConfigurationMode Config_Mode;
	/*Select polarity and phase mode*/
	SPI_PolarityAndPhaseMode SPI_PolandPhaseMode;
	/*Tx interrupt enable or disable*/
	SPI_TransmitInterrupt TX_Interrupt;
	/*Rx interrupt enable or disable*/
	SPI_ReceiveInterrupt RX_Interrupt;
	/*Error interrupt enable or disable*/
	SPI_ErrorInterrupt Err_Interrupt;
	/*Select spi frame format (Ti or Motorola)*/
	SPI_FrameFormat Frame_Format;
	/*Enable or disable SS_Output*/
	SPI_SSOutput SS_Output;
	/*Enable or disable TX DMA*/
	SPI_TransmitDMA TX_Dma;
	/*Enable or disable RX DMA*/
	SPI_ReceiveDMA RX_Dma;
}SPI_InstanceConfigType;



/*Struct contains the config struct and varius other configuration,
 * this struct is used to init each spi peripheral seprately*/

typedef struct{
	/*Instance of SPI mapped to it's base address struct of registers*/
	SPI_TypeDef* Instance;
	/*config struct defined above for that specific instance*/
	SPI_InstanceConfigType Instance_Config;
	/*Pointer to transfer buffer passed by application to send data out of*/
	uint8* Ptr_TXBuffer;
	/*TX Transfer Length*/
	uint32 TX_TransferLen;
	/*Pointer to receive buffer passed by application to receive data in*/
	uint8* Ptr_RXBuffer;
	/*RX Transfer Length*/
	uint32 RX_TransferLen;
	/*Tx State*/
	SPI_State TxState;
	/*Rx state*/
	SPI_State RxState;


}SPI_PeripheralConfiguration;



/*********************************************************************************************************************************
 *                    						 Base Address and Clock Enable defintions   						                                 *
 ********************************************************************************************************************************/


/*SPI Base address*/

#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3
#define SPI_4 SPI4
#define SPI_5 SPI5
#define SPI_6 SPI6


/*To enable clock for different SPI peripherals in RCC register*/
#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()   (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()   (RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN()   (RCC->APB2ENR |= (1<<21))

/*To disable clock for different SPI peripherals in RCC register*/
#define SPI1_PCLK_DIS()   (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DIS()   (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DIS()   (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DIS()   (RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DIS()   (RCC->APB2ENR &= ~(1<<20))
#define SPI6_PCLK_DIS()   (RCC->APB2ENR &= ~(1<<21))

/*To reset registers of a specific peripheral*/
#define SPI1_REG_RESET() \
{\
  RCC->APB2RSTR |= (1<<12); \
  RCC->APB2RSTR &= ~(1<<12);\
 }
#define SPI2_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<14); \
  RCC->APB1RSTR &= ~(1<<14); \
  }
#define SPI3_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<15); \
  RCC->APB1RSTR &= ~(1<<15); \
  }
#define SPI4_REG_RESET() \
{\
 RCC->APB2RSTR |= (1<<13); \
 RCC->APB2RSTR &= ~(1<<13); \
 }
#define SPI5_REG_RESET() \
{ \
 RCC->APB2RSTR |= (1<<20);  \
 RCC->APB2RSTR &= ~(1<<20); \
 }
#define SPI6_REG_RESET() \
{\
 RCC->APB2RSTR |= (1<<21); \
 RCC->APB2RSTR &= ~(1<<21); \
 }

/*********************************************************************************************************************************
 *                    								 Function Prototypes    						                                 *
 ********************************************************************************************************************************/

/**************/
void SPI_PeriClockControl(SPI_TypeDef* SPIx, SPI_CLK a_CLK);

void SPI_Init(SPI_PeripheralConfiguration* Config);

void SPI_DeInit(SPI_TypeDef* SPIx);


/*********/

/*Blocking*/
void SPI_SendData(SPI_TypeDef* SPIx, uint8* pTXBuffer, uint32 len);

void SPI_ReceiveData(SPI_TypeDef* SPIx, uint8* pRXBuffer, uint32 len);


/* non Blocking*/
SPI_State SPI_SendDataIT(SPI_PeripheralConfiguration* Config, uint8* pTXBuffer, uint32 len);

SPI_State SPI_ReceiveDataIT(SPI_PeripheralConfiguration* Config, uint8* pRXBuffer, uint32 len);

/*******/


void SPI_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi);

void SPI_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority);


void SPI_IRQHandling(SPI_PeripheralConfiguration* Config);

void SPI_SetCallback(void (*a_Ptr)(SPI_PeripheralConfiguration* Config),
		SPI_EventCallBack Event_Set);

void SPI_CloseTransmission(SPI_PeripheralConfiguration* Config);
void SPI_CloseReception(SPI_PeripheralConfiguration* Config);
void SPI_ClearOVRFlag(SPI_TypeDef* SPIx);


#endif /* STM32F4XX_HAL_DRIVER_INC_STM32F429I_SPI_DRIVER_H_ */
