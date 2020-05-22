 /********************************************************************************************************************************
 * Module: I2C
 *
 * File Name: stm32f429i_i2c_driver.h
 *
 * Description: header file for I2C module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/



#ifndef STM32F4XX_HAL_DRIVER_INC_STM32F429I_I2C_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_STM32F429I_I2C_DRIVER_H_

/*MCU Specific Header File*/
#include "stm32f429xx.h"

#define I2C_NO_REPEATED_START	0
#define I2C_REPEATED_START		1

#define NO_OF_CALLBACK_EVENTS	10
/*********************************************************************************************************************************
 *                    								 Type Defintions    						                                 *
 ********************************************************************************************************************************/
/*Type definition to enable or disable I2C*/
typedef enum{
	/*Disable I2c*/
	I2C_DIS,
	/*Enable I2c*/
	I2C_EN
}I2C_CLK;




/*Type definition to select I2C or SMBUS mode*/
typedef enum{
	/*Select I2c Mode*/
	I2C_MODE,
	/*Select SMBUS Mode*/
	SMBUS_MODE
}I2C_SMBusMode;

/*Type definition to select  SMBUS Type*/
typedef enum{
	SMBUS_UNUSED = 0, /*SMBUS Mode is I2C*/
	SMBUS_DEVICE = 0,
	SMBUS_HOST = 1
}I2C_SMBusType;

/*Type definition to Enable or disable ARP*/
typedef enum{
	/*SMBUS Mode is I2C*/
	I2C_ARP_UNUSED = 0,
	/*Disable ARP*/
	I2C_ARP_DISABLE = 0,
	/*Enable ARP*/
	I2C_ARP_ENABLE = 1
}I2C_ARPEnable;

/*Type definition to enable or disable PEC*/
typedef enum{
	/*Disable Packet Error Checking*/
	I2C_PEC_DISABLE,
	/*Enable Packet Error checking*/
	I2C_PEC_ENABLE
}I2C_PacketErrorChecking;

/*Type definition to enable or disable General Call*/
typedef enum{
	/*General call disabled. Address 00h is NACKed*/
	I2C_GEN_CALL_DISABLED,
	/*General call enabled. Address 00h is ACKed.*/
	I2C_GEN_CALL_ENABLED
}I2C_GeneralCall;

/*Type defintion to enable or disable clock stretching (Slave mode)
 * This bit is used to disable clock stretching in slave mode when ADDR
 * or BTF flag is set, until it is reset by software*/
typedef enum{
	/*Enable Clock Stretching*/
	I2C_CLOCK_STRETCHING_ENABLED,
	/*Disable Clock Stretching*/
	I2C_CLOCK_STRETCHING_DISABLED
}I2C_ClockStretching;

/*Type Definition for Start generation*/
typedef enum{
	/*No Start Generated*/
	I2C_NO_START_GENERATION,
	/*Start generated, for slave start is generated when bus is free*/
	I2C_START_GENERATION
}I2C_StartGeneration;

/*Type Definition for*/
typedef enum{
	/*No Stop Generated stop generation*/
	I2C_NO_STOP_GENERATION,
	/*For Master: Stop generation
	 * after the current byte transfer
	 * or after the current Start condition is sent.
	 *
	 * For Slave:  Release the SCL and SDA lines
	 * after the current byte transfer. */
	I2C_STOP_GENERATION
}I2C_StopGeneration;

/*Type definition for Acknowledge enable/disbale*/
typedef enum{
	/*No Acknowledge returned*/
	I2C_ACK_DISABLE,
	/*Acknowledge return after matched address or after each byte*/
	I2C_ACK_ENABLE

}I2C_Acknowledge;

/*Type defintion for I2C Acknowledge position for data reception*/
typedef enum{
	/*ACK bit controls the (N)ACK of the current byte being received in the shift register*/
	I2C_POS_CURRENT_BYTE,
	/*ACK bit controls the (N)ACK of the next byte which will be received in the shift regist*/
	I2C_POS_NEXT_BYTE
}I2C_AcknowledgePosition;

/*Type definition for Packet error checking Transfer*/
typedef enum{
	/*No Pec Transfer*/
	I2C_NO_PEC_TRANSFER,
	/*PEC transfer (in Tx or Rx mode)*/
	I2C_PEC_TRANSFER
}I2C_PECTransfer;

/*Type definition for SMBus Alert*/
typedef enum{
	/* Releases SMBA pin high. Alert Response Address Header followed by NACK.*/
	I2C_SMBA_ALERT_NACK,
	/*Drives SMBA pin low. Alert Response Address Header followed by ACK.*/
	I2C_SMBA_ALERT_ACK
}I2C_SMBusAlert;

/*Type definition for Software reset enable or disable
 * When set, the I2C is under reset state.
 * Before resetting this bit, make sure the I2C lines are
released and the bus is free*/
typedef enum{
	/* I2C Peripheral not under reset*/
	I2C_SWRESET_DISABLE,
	/* I2C Peripheral under reset state*/
	I2C_SWRESET_ENABLE
}I2C_SWReset;

/*Type definition for DMA Last Transfer*/
typedef enum{
	/* Next DMA EOT is not the last transfer*/
	I2C_DMA_NOT_LAST_TRANSFER,
	/* Next DMA EOT is the last transfer*/
	I2C_DMA_LAST_TRANSFER

}I2C_DMALast;

/*Type definition to disable/enable DMA Requests*/
typedef enum{
	/*DMA requests disabled*/
	I2C_DMA_REQUEST_DISABLE,
	/*DMA request enabled when TxE=1 or RxNE =1*/
	I2C_DMA_REQUEST_ENABLE
}I2C_DMARequestEnable;

/*Type definition for  Buffer interrupt enable/disable*/
typedef enum{
	/*TxE = 1 or RxNE = 1 does not generate any interrupt*/
	I2C_BUFFER_INTERRUPT_DISABLE,
	/*TxE = 1 or RxNE = 1 generates Event Interrupt (whatever the state of DMAEN)*/
	I2C_BUFFER_INTERRUPT_ENABLE
}I2C_BufferInterrupt;

/*Type definition for Event interrupt enable/disable*/
typedef enum{
	/*Event interrupt disabled*/
	I2C_EVENT_INTERRUPT_DISABLE,
	/*Event interrupt Enabled*/
	I2C_EVENT_INTERRUPT_ENABLE
}I2C_EventInterrupt;

/*Type definition for Error interrupt enable/disable*/
typedef enum{
	/*Error interrupt disabled*/
	I2C_ERROR_INTERRUPT_DISABLE,
	/*Error interrupt Enabled*/
	I2C_ERROR_INTERRUPT_ENABLE
}I2C_ErrorInterrupt;

/*Type definition for I2C Frequency*/
typedef enum{
	I2C_FPCLK_2MHZ = 2,
	I2C_FPCLK_3MHZ = 3,
	I2C_FPCLK_4MHZ = 4,
	I2C_FPCLK_5MHZ = 5,
	I2C_FPCLK_6MHZ = 6,
	I2C_FPCLK_7MHZ = 7,
	I2C_FPCLK_8MHZ = 8,
	I2C_FPCLK_9MHZ = 9,
	I2C_FPCLK_10MHZ = 10,
	I2C_FPCLK_11MHZ = 11,
	I2C_FPCLK_12MHZ = 12,
	I2C_FPCLK_13MHZ = 13,
	I2C_FPCLK_14MHZ = 14,
	I2C_FPCLK_15MHZ = 15,
	I2C_FPCLK_16MHZ = 16,
	I2C_FPCLK_17MHZ = 17,
	I2C_FPCLK_18MHZ = 18,
	I2C_FPCLK_19MHZ = 19,
	I2C_FPCLK_20MHZ = 20,
	I2C_FPCLK_21MHZ = 21,
	I2C_FPCLK_22MHZ = 22,
	I2C_FPCLK_23MHZ = 23,
	I2C_FPCLK_24MHZ = 24,
	I2C_FPCLK_25MHZ = 25,
	I2C_FPCLK_26MHZ = 26,
	I2C_FPCLK_27MHZ = 27,
	I2C_FPCLK_28MHZ = 28,
	I2C_FPCLK_29MHZ = 29,
	I2C_FPCLK_30MHZ = 30,
	I2C_FPCLK_31MHZ = 31,
	I2C_FPCLK_32MHZ = 32,
	I2C_FPCLK_33MHZ = 33,
	I2C_FPCLK_34MHZ = 34,
	I2C_FPCLK_35MHZ = 35,
	I2C_FPCLK_36MHZ = 36,
	I2C_FPCLK_37MHZ = 37,
	I2C_FPCLK_38MHZ = 38,
	I2C_FPCLK_39MHZ = 39,
	I2C_FPCLK_40MHZ = 40,
	I2C_FPCLK_41MHZ = 41,
	I2C_FPCLK_42MHZ = 42,
	I2C_FPCLK_43MHZ = 43,
	I2C_FPCLK_44MHZ = 44,
	I2C_FPCLK_45MHZ = 45,
	I2C_FPCLK_46MHZ = 46,
	I2C_FPCLK_47MHZ = 47,
	I2C_FPCLK_48MHZ = 48,
	I2C_FPCLK_49MHZ = 49,
	I2C_FPCLK_50MHZ = 50,
}I2C_FREQUENCY;


/*Type definition for I2C Address mode (7 or 10 bits)*/
typedef enum{
	/* 7-bit slave address (10-bit address not acknowledged)*/
	I2C_7_BIT,
	/* 10-bit slave address (7-bit address not acknowledged)*/
	I2C_10_BIT
}I2C_AddressMode;

/*Type definition for dual addressing mode enable/disable*/
typedef enum{
	/*Only OAR1 is recognized in 7-bit addressing mode*/
	I2C_DUAL_ADDRESS_ENABLE,
	/*Both OAR1 and OAR2 are recognized in 7-bit addressing mode*/
	I2C_DUAL_ADDRESS_DISABLE
}I2C_DualAddress;

/*Type definition for master mode selection(Standard or Fast Mode)*/
typedef enum{
	/*Sm mode I2C*/
	I2C_STANDARD_MODE,
	/*Fm mode I2C*/
	I2C_FAST_MODE
}I2C_MasterMode;

/*Type definition for I2C fast mode duty cycle*/
typedef enum{
	/*I2C is in standard Mode*/
	I2C_FM_UNUSED = 0,
	/* Fm mode tlow/thigh = 2*/
	I2C_2_to_1 = 0,
	/* Fm mode tlow/thigh = 16/9*/
	I2C_16_to_9 = 1

}I2C_FastModeDuty;

/*Type definition for analog noise filter enable/disable*/
typedef enum{
	/* Analog noise filter enable*/
	I2C_ANALOG_NOISE_FILTER_ENABLE,
	/* Analog noise filter disable*/
	I2C_ANALOG_NOISE_FILTER_DISABLE
}I2C_AnalogNoiseFiler;

/*Type definition for Digital noise filter enable/disable*/
typedef enum{
	/*Digital noise filter disable*/
	I2C_DIGITAL_NOISE_FILTER_DISABLE,
	/*Digital noise filter enabled and filtering capability up to 1* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_1XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 2* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_2XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 3* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_3XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 2* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_4XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 5* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_5XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 6* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_6XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 7* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_7XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 8* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_8XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 9* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_9XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 10* TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_10XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 11 TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_11XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 12 TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_12XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 13 TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_13XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 14 TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_14XTPCLK,
	/*Digital noise filter enabled and filtering capability up to 15 TPCLK1*/
	I2C_DIGITAL_NOISE_FILTER_ENABLE_15XTPCLK,

}I2C_DigitalNoiseFiler;



/*Type defintion to I2C State*/
typedef enum{
	/*I2C initialized and ready to use*/
	I2C_STATE_READY = 0,
	/*Data reception process is ongoing*/
	I2C_STATE_BUSY_RX = 1,
	/*Data transmission process is ongoing*/
	I2C_STATE_BUSY_TX = 2,
}I2C_State;

/*Events for callback function*/

typedef enum{
	/*TX Complete*/
	I2C_TX_COMPLETE = 0,
	/*RX Complete*/
	I2C_RX_COMPLETE = 1,
	/*Bus error invalid start or stop position*/
	I2C_BUS_ERROR = 2,
	/*I2C Lost arbitration*/
	I2C_ARBITRATION_LOSS_ERROR = 3,
	/*No ack returned*/
	I2C_ACK_FAILURE_ERROR = 4,
	/*Overrun or underrun errors*/
	I2C_OVERRUN_ERROR = 5,
	/*Clock stretched for too long*/
	I2C_TIMEOUT_ERROR = 6,
	/*Stop Detected*/
	I2C_STOP_DETECTED = 7,
	/*Master read, slave transmitter, Slave writes data*/
	I2C_DATA_REQUEST = 8,
	/*Master write, slave receiver, slave reads data*/
	I2C_DATA_RECEIVE = 9
}I2C_EventCallBack;

/*Type defintion  for Config Struct*/
typedef struct{
	/*To enable or disable I2C*/
	I2C_CLK I2CEnable;
	/*Select I2C or SMBUS mode*/
	I2C_SMBusMode SMBUSMode;
	/*Select  SMBUS Type*/
	I2C_SMBusType SMBusType;
	/*STo enable or disable ARP*/
	I2C_ARPEnable ARPEnable;
	/*To enable or disable PEC*/
	I2C_PacketErrorChecking PacketErrorChecking;
	/*To enable or disable General Call*/
	I2C_GeneralCall GeneralCall;
	/*To enable or disable clock stretching (Slave mode)*/
	I2C_ClockStretching clockStretching;
	/*For Acknowledge enable/disable*/
	I2C_Acknowledge Acknowledge;
	/*To disable/enable DMA Requests*/
	I2C_DMARequestEnable DMARequestEnable;
	/*For  Buffer interrupt enable/disable*/
	I2C_BufferInterrupt BufferInterrupt;
	/*For Event interrupt enable/disable*/
	I2C_EventInterrupt EventInterrupt;
	/*For Error interrupt enable/disable*/
	I2C_ErrorInterrupt ErrorInterrupt;
	/*For I2C Frequency*/
	I2C_FREQUENCY FREQUENCY;
	/*For I2C Address mode (7 or 10 bits)*/
	I2C_AddressMode AddressMode;
	/*For dual addressing mode enable/disable*/
	I2C_DualAddress DualAddress;
	/*For I2C fast mode duty cycle*/
	I2C_FastModeDuty FastModeDuty;
	/*For analog noise filter enable/disable*/
	I2C_AnalogNoiseFiler AnalogNoiseFiler;
	/*For Digital noise filter enable/disable*/
	I2C_DigitalNoiseFiler DigitalNoiseFiler;
	/*Address 1*/
	uint32 I2C_Address1;
	/*Address 2*/
	uint32 I2C_Address2;
	/*Specify speed of I2c 100khz(Normal Mode) to 400khz(Fast Mode)*/
	uint32 SCLSpeed;
}I2C_InstanceConfigType;
/*removed I2C_MasterMode, I2C_StartGeneration, I2C_StopGeneration, I2C_SMBusAlert
 * I2C_DMALast, I2C_PECTransfer, I2C_AcknowledgePosition, I2C_SWReset but enums still exist*/

/*Struct contains the config struct and varius other configuration,
 * this struct is used to init each I2C peripheral seprately*/

typedef struct{
	/*Instance of I2C mapped to it's base address struct of registers*/
	I2C_TypeDef* Instance;
	/*config struct defined above for that specific instance*/
	I2C_InstanceConfigType Instance_Config;
	/*Pointer to buffer passed by application to send/receive data out of
	/*Pointer to transfer buffer passed by application to send data out of*/
	uint8* Ptr_TXBuffer;
	/*TX Transfer Length*/
	uint32 TX_TransferLen;
	/*Pointer to receive buffer passed by application to receive data in*/
	uint8* Ptr_RXBuffer;
	/*RX Transfer Length*/
	uint32 RX_TransferLen;
	/*Slave Device Address*/
	uint8 devAddress;
	/*State of this I2C Instance*/
	I2C_State Instance_State;
	/*Store Rx Size*/
	uint32 RxSize;
	/*Repeated /Non-Repeated Start config*/
	uint8 Sr;
}I2C_PeripheralConfiguration;

/*********************************************************************************************************************************
 *                    								 Macro Defintions    						                                 *
 ********************************************************************************************************************************/


#define I2C_SCL_SPEEd_SM				100000
#define I2C_SCL_SPEED_FM4K				400000



/*To enable clock for different I2C peripherals in RCC register*/
#define I2C1_PCLK_EN()				RCC->APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()				RCC->APB1ENR |= (1<<22)
#define I2C3_PCLK_EN()				RCC->APB1ENR |= (1<<23)

/*To disable clock for different I2C peripherals in RCC register*/
#define I2C1_PCLK_DIS()				RCC->APB1ENR &=~ (1<<21)
#define I2C2_PCLK_DIS()				RCC->APB1ENR &=~ (1<<22)
#define I2C3_PCLK_DIS()				RCC->APB1ENR &=~ (1<<23)

/*To reset registers of a specific peripheral*/
#define I2C1_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<21); \
  RCC->APB1RSTR &= ~(1<<21);\
 }
#define I2C2_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<22); \
  RCC->APB1RSTR &= ~(1<<22); \
  }
#define I2C3_REG_RESET() \
{\
  RCC->APB1RSTR |= (1<<23); \
  RCC->APB1RSTR &= ~(1<<23); \
  }


/*********************************************************************************************************************************
 *                    								 Function Prototypes    						                                 *
 ********************************************************************************************************************************/

/**************/
void I2C_PeriClockControl(I2C_TypeDef* I2Cx, I2C_CLK a_CLK);

void I2C_Init(I2C_PeripheralConfiguration* Config);

void I2C_DeInit(I2C_TypeDef* I2Cx);


/*********/

///*Blocking*/
void I2C_MasterSendData(I2C_PeripheralConfiguration* Config, uint8* pTXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr);/*Sr is repeated start*/
void I2C_MasterReceiveData(I2C_PeripheralConfiguration* Config, uint8* pRXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr);

void I2C_SlaveSendData(I2C_TypeDef* I2Cx, uint8 val);
uint8 I2C_SlaveReceiveData(I2C_TypeDef* I2Cx);


///* non Blocking*/
I2C_State I2C_MasterSendDataIT(I2C_PeripheralConfiguration* Config, uint8* pTXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr);/*Sr is repeated start*/
I2C_State I2C_MasterReceiveDataIT(I2C_PeripheralConfiguration* Config, uint8* pRXBuffer, uint8 slaveAddr, uint32 len, uint8 Sr);
/*******/

void I2C_IRQConfig(IRQn_Type IRQ_Num, IRQn_EN_DIS EnOrDi);

void I2C_IRQPriority(IRQn_Type IRQ_Num, uint8 IRQ_Priority);


void I2C_EventIRQHandling(I2C_PeripheralConfiguration* Config);

void I2C_ErrorIRQHandling(I2C_PeripheralConfiguration* Config);

//void I2C_IRQHandling(I2C_PeripheralConfiguration* Config);

void I2C_SetCallback(void (*a_Ptr)(I2C_PeripheralConfiguration* Config),
		I2C_EventCallBack Event_Set);

#endif /* STM32F4XX_HAL_DRIVER_INC_STM32F429I_I2C_DRIVER_H_ */
