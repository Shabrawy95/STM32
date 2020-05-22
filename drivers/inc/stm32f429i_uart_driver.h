 /********************************************************************************************************************************
 * Module: UART
 *
 * File Name: stm32f429i_uart_driver.h
 *
 * Description: header file for UART module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/

#ifndef INC_STM32F429I_UART_DRIVER_H_
#define INC_STM32F429I_UART_DRIVER_H_

/*MCU Specific Header File*/
#include "stm32f429xx.h"


#define NO_OF_CALLBACK_EVENTS	8

/*********************************************************************************************************************************
 *                    								 Type Defintions    						                                 *
 ********************************************************************************************************************************/

#define UNUSED	0

/*Type definition to enable or disable USART*/
typedef enum{
	/*Disable USART*/
	USART_DIS,
	/*Enable USART*/
	USART_EN
}USART_CLK;

/*Type definition for receiver active or mute*/
typedef enum{
	/*Receiver in active mode*/
	USART_RECEIVER_ACTIVE,
	/*Receiver in Mute mode*/
	USART_RECEIVER_MUTE
}USART_RecevierMode;

/*Type definition for receiver enable/disable*/
typedef enum{
	/*Receiver disable*/
	USART_RECEIVER_DISABLE,
	/*Receiver enable*/
	USART_RECEIVER_ENABLE
}USART_ReceiverEn;

/*Type definition for Transmitter enable/disable*/
typedef enum{
	/*Transmitter disable*/
	USART_TRANSMITTER_DISABLE,
	/*Transmitter enable*/
	USART_TRANSMITTER_ENABLE
}USART_TransmitterEn;


/*Type definition for Idle interrupt enable/disable*/
typedef enum{
	/*Idle interrupt disabled*/
	USART_IDLE_INTERRUPT_DISABLE,
	/*Idle interrupt Enabled*/
	USART_IDLE_INTERRUPT_ENABLE
}USART_IdleInterrupt;

/*Type definition to enable and disable USART TX interrupt*/
typedef enum{
	/*TX interrupt Disabled*/
	USART_TX_INT_DISABLE,
	/*TX interrupt enabled, used to generate an interrupt request when the TXE flag is set*/
	USART_TX_INT_ENABLE
}USART_TransmitInterrupt;

/*Type definition to enable and disable USART RX interrupt*/
typedef enum{
	/*RX interrupt Disabled*/
	USART_RX_INT_DISABLE,
	/*RX interrupt enabled, Used to generate an interrupt request when the RXNE or ORE flags are set. */
	USART_RX_INT_ENABLE
}USART_ReceiveInterrupt;

/*Type definition to enable and disable USART TX Complete interrupt*/
typedef enum{
	/*TX Complete interrupt Disabled*/
	USART_TC_INT_DISABLE,
	/*TX Complete interrupt enabled, used to generate an interrupt request when the TCE flag is set*/
	USART_TC_INT_ENABLE
}USART_TransmitCompleteInterrupt;

/*Type definition to enable and disable USART Parity Error interrupt*/
typedef enum{
	/*PE interrupt Disabled*/
	USART_PE_INT_DISABLE,
	/*PE interrupt enabled, used to generate an interrupt request when the PE flag is set*/
	USART_PE_INT_ENABLE
}USART_ParityErrorInterrupt;

/*Type definition for parity Type*/
typedef enum{
	/*Even Parity*/
	USART_EVEN_PARITY,
	/*Odd Parity*/
	USART_ODD_PARITY
}USART_ParityType;

/*Type definition for parity Enable/Disable*/
typedef enum{
	/*disable Parity checking and generation*/
	USART_PARITY_DISABLE,
	/*enable Parity checking and generation*/
	USART_PARITY_ENABLE
}USART_ParityEn;

/*Type definition for USART wakeup Method*/
typedef enum{
	/*Idle Line*/
	USART_WAKEUP_IDLE_LINE,
	/*Address Mark*/
	USART_WAKEUP_ADDR_MARK
}USART_WakeupMethod;

/*Type defintion for Word length select*/
typedef enum{
	/*1 Start, 8 Data, n stop bits, if PCE is enabled 8th bit of data bits is parity*/
	USART_8_DATA_BITS,
	/*1 Start, 9 Data, n stop bits, if PCE is enabled 9th bit of data bits is parity*/
	USART_9_DATA_BITS
}USART_WordLength;

/*Type defintion for oversampling mode
 * Over sampling by 8 is not available in smartcard, IRDA, LIN modes
 * I.E when SCEN or IREN or LINEN are set then Oversampling is forced to be zero*/
typedef enum{
	/*OverSampling by 16*/
	USART_OVERSAMPLING_BY_16,
	/*OverSampling by 8*/
	USART_OVERSAMPLING_BY_8
}USART_OverSamplingMode;

/*Type definitions for LIN Break detection Length*/
typedef enum{
	/*10 bit break detection*/
	USART_10BIT_BREAK_DETECTION,
	/*11 bit break detection*/
	USART_11BIT_BREAK_DETECTION
}USART_LinBreakDetectionLength;

/*Type definition to enable and disable USART lin break detection interrupt*/
typedef enum{
	/*lin break detection interrupt Disabled*/
	USART_LBD_INT_DISABLED,
	/*lin break detection interrupt enabled, Used to generate an interrupt request when the LBD Flag is set. */
	USART_LBD_INT_ENABLED
}USART_LinBreakDetectionInterrupt;

/*Type defintion for Kast bit clock pulse (Syncronous mode)*/
typedef enum{
	/*Last data bit clock pulse is not output on CK pin in synch. mode*/
	USART_LAST_BIT_CLOCK_PULSE_DISABLED,
	/*Last data bit clock pulse is output on CK pin in synch. mode*/
	USART_LAST_BIT_CLOCK_PULSE_ENABLED
}USART_LastBitClockPulse;

/*Type defintion for USART Synch. mode (Not available for UART4, 5)*/
typedef enum{
	/*MODE 0: CPOL 0 (Clock is zero when idle) CPHA 0 (sample on leading edge)*/
	USART_POL_PHASE_MODE_0,
	/* MODE 1: CPOL 0 (Clock is zero when idle) CPHA 1 (sample on trailing edge)*/
	USART_POL_PHASE_MODE_1,
	/*MODE 2: CPOL 1 (Clock is One when idle) CPHA 0 (sample on leading edge)*/
	USART_POL_PHASE_MODE_2,
	/*MODE 3: CPOL 1 (Clock is One when idle) CPHA 1 (sample on trailing edge)*/
	USART_POL_PHASE_MODE_3
}USART_PolarityAndPhaseMode;

/*Type defintion for USART Clock Enable (Synch. Mode only) (Not available for UART4, 5)*/
typedef enum{
	/*CK pin enable*/
	USART_CK_PIN_DISABLE,
	/*CK pin disable*/
	USART_CK_PIN_ENABLE
}USART_ClockPinEn;

/*Type definition for Stop bit number config
 * 0.5 and 1.5 stop bits are not available for (UART4 and UART5)*/
typedef enum{
	/*One stop bit*/
	USART_ONE_STOP_BIT,
	/*Half stop bit*/
	USART_HALF_STOP_BIT,
	/*Two stop bits*/
	USART_TWO_STOP_BIT,
	/*One and aHalf stop bit*/
	USART_ONE_AND_HALF_STOP_BIT
}USART_StopBit;

/*Type defintion to enable/disable Lin Mode
 * Lin mode enables capability to send LIN Synch Breaks (13 low bits) Using SBK bit in
 * USART CR1 and to detect LIN Synch breaks*/
typedef enum{
	/*Lin mode disable*/
	USART_LIN_MODE_DISABLE,
	/*Lin mode enable*/
	USART_LIN_MODE_ENABLE
}USART_LinMode;

/*Type definition for USART Error interrupt Enable/Disable
 * Enable interrupt in case of Framing Error, Overrun Error,
 * or Noise Flag (FE = 1, ORE = 1, NF=1 in SR) while DMAR = 1 in CR3 register*/
typedef enum{
	/*Error interrupt disabled*/
	USART_ERROR_INTERRUPT_DISABLE,
	/*Error interrupt Enabled*/
	USART_ERROR_INTERRUPT_ENABLE
}USART_ErrorInterrupt;

/*Type defintion for IrDA mode enable/disable*/
typedef enum{
	/*IrDA mode disable*/
	USART_IRDA_MODE_DISABLE,
	/*IrDA mode enable*/
	USART_IRDA_MODE_ENABLE
}USART_IrDAMode;


/*Type defintion for IrDA mode low-power or Normal Mode*/
typedef enum{
	/*select low power mode when IrDA mode is enabled*/
	USART_IRDA_LOW_POWER,
	/*select Normal power mode when IrDA mode is enabled*/
	USART_IRDA_NORMAL
}USART_IrDALowPower;

/*Type definition for single wire half duplex mode enable/disable*/
typedef enum{
	/*Half duplex mode not selected*/
	USART_HALF_DUPLEX_DISABLE,
	/*Half duplex mode selected*/
	USART_HALF_DUPLEX_ENABLE
}USART_HalfDuplex;

/*Type defintion for Smart Card Nack Enable/Disable (Not Available for UART4 and UART5)*/
typedef enum{
	/*Nack Transmission in case of parity error is disabled*/
	USART_SMARTCARD_NACK_DISABLE,
	/*Nack Transmission in case of parity error is enabled*/
	USART_SMARTCARD_NACK_ENABLE
}USART_SmartCardNack;

/*Type definition for Smart card mode enable/disabe (Not Available for UART4 and UART5)*/
typedef enum{
	/*Smart Card mode disable*/
	USART_SMARTCARD_MODE_DISABLE,
	/*Smart Card  mode enable*/
	USART_SMARTCARD_MODE_ENABLE
}USART_SmartCardMode;



/*Type definition for DMA mode for transmission Enable/Disable*/
typedef enum{
	/*DMA mode for transmission disable*/
	USART_TX_DMA_DISABLE,
	/*DMA mode for transmission Enable*/
	USART_TX_DMA_ENABLE
}USART_TransmitDMA;

/*Type definition for DMA mode for reception Enable/Disable*/
typedef enum{
	/*DMA mode for reception disable*/
	USART_RX_DMA_DISABLE,
	/*DMA mode for reception Enable*/
	USART_RX_DMA_ENABLE
}USART_ReceiveDMA;

/*Type definition for USART Clear to send flow control Enable/Disable  (Not Available for UART4 and UART5)*/
typedef enum{
	/*Clear to send flow control disable*/
	USART_CTS_DISABLE,
	/*Clear to send flow control enable, data sent when CTS is 0*/
	USART_CTS_ENABLE
}USART_ClearToSend;

/*Type definition for USART Request to send flow control Enable/Disable  (Not Available for UART4 and UART5)*/
typedef enum{
	/*Request to send flow control disable*/
	USART_RTS_DISABLE,
	/*Request to send flow control enable, data can be received when RTS = 0*/
	USART_RTS_ENABLE
}USART_RequestToSend;

/*Type definition for Clear to send interrupt enable/disable
 * Interrupt whenever CTS = 1 in SR
 * I.e when CTS = 1 transmitter can't send data
 *  (Not Available for UART4 and UART5)*/
typedef enum{
	/*Clear to send interrupt enable*/
	USART_CTS_INTERRUPT_DISABLE,
	/*Clear to send interrupt disable*/
	USART_CTS_INTERRUPT_ENABLE
}USART_ClearToSendInterrupt;


/*Type definition for One sample bit method enable/disable
 * when one sample bit method is enabled noise Flag (NF) is disabled
 * ONEBIT feature applies only to data bits and doesn't apply to start bit*/
typedef enum{
	/*Default*/
	USART_3_SAMPLE_BIT,
	/*One sample bit and noise flag is disabled*/
	USART_1_SAMPLE_BIT
}USART_OneSampleBitMethod;

typedef enum{
	/*USART initialized and ready to use*/
	USART_STATE_READY = 0,
	/*Data reception process is ongoing*/
	USART_STATE_BUSY_RX = 1,
	/*Data transmission process is ongoing*/
	USART_STATE_BUSY_TX = 2
}USART_State;

typedef enum{
	/*Transmission of current byte is done*/
	USART_TX_COMPLETE ,
	/*Reception of current byte is done*/
	USART_RX_COMPLETE,
	/*Overrun while receiving*/
	USART_OVR_ERROR,
	/*Transmission complete (whole transmisison not a byte)*/
	USART_TC_COMPLETE,
	/*CTS input changed*/
	USART_CTS_CHANGE,
	/*Idle line detected after transmission of bytes*/
	USART_IDLE_LINE,
	/*Framing error*/
	USART_FRAME_ERROR,
	/*Noise Flag is set*/
	USART_NOISE_DETECTED
}USART_EventCallBack;



typedef struct{
	/*Enable or disable USART*/
	USART_CLK USARTEnable;
	/*Receiver active or mute*/
	USART_RecevierMode RX_Mode;
	/*Receiver enable/disable*/
	USART_ReceiverEn RXEnable;
	/*Transmitter enable/disable*/
	USART_TransmitterEn TXEnable;
	/*Idle interrupt enable/disable*/
	USART_IdleInterrupt Idle_Interrupt;
	/*Enable and disable USART TX interrupt*/
	USART_TransmitInterrupt TX_Interrupt;
	/*Enable and disable USART RX interrupt*/
	USART_ReceiveInterrupt RX_Interrupt;
	/*Enable and disable USART TX Complete interrupt*/
	USART_TransmitCompleteInterrupt TransmitComplete_Interrupt;
	/*Enable and disable USART Parity Error interrupt*/
	USART_ParityErrorInterrupt ParityError_Interrupt;
	/*Parity Type*/
	USART_ParityType ParityType;
	/*Parity Enable/Disable*/
	USART_ParityEn ParityEnable;
	/*USART wakeup Method*/
	USART_WakeupMethod WakeupMethod;
	/*Word length select*/
	USART_WordLength WordLength;
	/*Oversampling mode*/
	USART_OverSamplingMode OverSamplingMode;
	/*LIN Break detection Length*/
	USART_LinBreakDetectionLength LinBreakLength;
	/*Enable and disable USART lin break detection interrupt*/
	USART_LinBreakDetectionInterrupt LinBreak_Interrupt;
	/*Last bit clock pulse Enable or disable(Syncronous mode)*/
	USART_LastBitClockPulse LastClockPulse;
	/*USART Synch. mode (polarity and Phase) (Not available for UART4, 5)*/
	USART_PolarityAndPhaseMode USART_PolandPhaseMode;
	/*USART Clock Enable (Synch. Mode only) (Not available for UART4, 5)*/
	USART_ClockPinEn ClockPin;
	/*Stop bit number config
	 * 0.5 and 1.5 stop bits are not available for (UART4 and UART5)*/
	USART_StopBit StopBit;
	/*TEnable/disable Lin Mode*/
	USART_LinMode LinMode;
	/*USART Error interrupt Enable/Disable*/
	USART_ErrorInterrupt Error_Interrupt;
	/*IrDA mode enable/disable*/
	USART_IrDAMode IrDAMode;
	/*IrDA mode low-power or Normal Mode*/
	USART_IrDALowPower IrDAPowerMode;
	/*Single wire half duplex mode enable/disable*/
	USART_HalfDuplex HalfDuplexEnable;
	/*Smart Card Nack Enable/Disable (Not Available for UART4 and UART5)*/
	USART_SmartCardNack SmartCardNackEnable;
	/*Smart card mode enable/disable  (Not Available for UART4 and UART5)*/
	USART_SmartCardMode SmartCardMode;
	/*DMA mode for transmission Enable/Disable*/
	USART_TransmitDMA TX_Dma;
	/*DMA mode for reception Enable/Disable*/
	USART_ReceiveDMA RX_Dma;
	/*USART Clear to send flow control Enable/Disable  (Not Available for UART4 and UART5)*/
	USART_ClearToSend CTS_Enable;
	/*USART Request to send flow control Enable/Disable  (Not Available for UART4 and UART5)*/
	USART_RequestToSend RTS_Enable;
	/*Clear to send interrupt enable/disable  (Not Available for UART4 and UART5)*/
	USART_ClearToSendInterrupt CTS_Interrupt;
	/*One sample bit method enable/disable*/
	USART_OneSampleBitMethod SampleBitMethod;
	/*Baudrate of USART*/
	uint32 Baud;
	/*Address of USART node (for multiprocessor communication in mute mode)*/
	uint8 Addr;

}USART_InstanceConfigType;


typedef struct{
	USART_TypeDef* Instance;
	USART_InstanceConfigType Instance_Config;
	/*Pointer to transfer buffer passed by application to send data out of*/
	uint16* Ptr_TXBuffer;
	/*TX Transfer Length*/
	uint32 TX_TransferLen;
	/*Pointer to receive buffer passed by application to receive data in*/
	uint16* Ptr_RXBuffer;
	/*RX Transfer Length*/
	uint32 RX_TransferLen;
	/*Tx State*/
	USART_State TxState;
	/*Rx state*/
	USART_State RxState;

}USART_PeripheralConfiguration;


/*********************************************************************************************************************************
 *                    								 Macro Defintions    						                                 *
 ********************************************************************************************************************************/

/*USART Baud Rates*/
#define USART_STD_BAUD_1200             1200
#define USART_STD_BAUD_2400             2400
#define USART_STD_BAUD_9600             9600
#define USART_STD_BAUD_19200            19200
#define USART_STD_BAUD_38400            38400
#define USART_STD_BAUD_57600            57600
#define USART_STD_BAUD_115200           115200
#define USART_STD_BAUD_230400           230400
#define USART_STD_BAUD_460800           460800
#define USART_STD_BAUD_921600			921600
#define USART_STD_BAUD_2M				2000000
#define USART_STD_BAUD_3M				3000000

/*To enable clock for different USART peripherals in RCC register*/
#define USART1_PCLK_EN()				RCC->APB2ENR |= (1<<4)
#define USART2_PCLK_EN()				RCC->APB1ENR |= (1<<17)
#define USART3_PCLK_EN()				RCC->APB1ENR |= (1<<18)
#define USART4_PCLK_EN()				RCC->APB1ENR |= (1<<19)
#define USART5_PCLK_EN()				RCC->APB1ENR |= (1<<20)
#define USART6_PCLK_EN()				RCC->APB2ENR |= (1<<5)
#define USART7_PCLK_EN()				RCC->APB1ENR |= (1<<30)
#define USART8_PCLK_EN()				RCC->APB1ENR |= (1<<31)

/*To disable clock for different USART peripherals in RCC register*/
#define USART1_PCLK_DIS()				RCC->APB2ENR &=~ (1<<4)
#define USART2_PCLK_DIS()				RCC->APB1ENR &=~ (1<<17)
#define USART3_PCLK_DIS()				RCC->APB1ENR &=~ (1<<18)
#define USART4_PCLK_DIS()				RCC->APB1ENR &=~ (1<<19)
#define USART5_PCLK_DIS()				RCC->APB1ENR &=~ (1<<20)
#define USART6_PCLK_DIS()				RCC->APB2ENR &=~ (1<<5)
#define USART7_PCLK_DIS()				RCC->APB1ENR &=~ (1<<30)
#define USART8_PCLK_DIS()				RCC->APB1ENR &=~ (1<<31)

/*To reset registers of a specific peripheral*/
#define USART1_REG_RESET() \
{\
  RCC->APB2RSTR |= (1<<4); \
  RCC->APB2RSTR &= ~(1<<4);\
 }
#define USART2_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<17); \
  RCC->APB1RSTR &= ~(1<<17); \
  }
#define USART3_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<18); \
  RCC->APB1RSTR &= ~(1<<18); \
  }

#define USART4_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<19); \
  RCC->APB1RSTR &= ~(1<<19);\
 }
#define USART5_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<20); \
  RCC->APB1RSTR &= ~(1<<20); \
  }
#define USART6_REG_RESET() \
{\
  RCC->APB2RSTR |= (1<<5); \
  RCC->APB2RSTR &= ~(1<<5);\
  }
#define USART7_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<30); \
  RCC->APB1RSTR &= ~(1<<30);\
 }
#define USART8_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<31); \
  RCC->APB1RSTR &= ~(1<<31); \
  }



/*********************************************************************************************************************************
 *                    								 Function Prototypes    						                                 *
 ********************************************************************************************************************************/
void USART_Init(USART_PeripheralConfiguration* Config);

void USART_DeInit(USART_TypeDef* USARTx);


void USART_PeriClockControl(USART_TypeDef* USARTx, USART_CLK a_CLK);


void USART_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi);

void USART_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority);

void USART_IRQHandling(USART_PeripheralConfiguration* Config);


/*Blocking*/
void USART_SendData(USART_TypeDef* USARTx, uint16* pTXBuffer, uint32 len);

void USART_ReceiveData(USART_TypeDef* USARTx, uint16* pRXBuffer, uint32 len);


/* non Blocking*/
USART_State USART_SendDataIT(USART_PeripheralConfiguration* Config, uint16* pTXBuffer, uint32 len);

USART_State USART_ReceiveDataIT(USART_PeripheralConfiguration* Config, uint16* pRXBuffer, uint32 len);

//void USART_IRQHandling(I2C_PeripheralConfiguration* Config);

void USART_SetCallback(void (*a_Ptr)(USART_PeripheralConfiguration* Config),
		USART_EventCallBack Event_Set);


#endif /* INC_STM32F429I_UART_DRIVER_H_ */
