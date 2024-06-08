/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Apr 16, 2024
 *      Author: IsaJelow
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"
#define _vo volatile
/*
 * Handler structure for SPI*/
typedef struct{
	uint8_t SPI_DeviceMode; /* Master or Slave @DeviceMode*/
	uint8_t SPI_BusConfig; /*full, half or simplex */
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DS; /* Data size */
	uint8_t SPI_CPOL; /*CPOL */
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


typedef struct{
	SPI_Reg_Def_t *pSPIx;
	SPI_Config_t SPI_Config;

	uint8_t *pTxBuffer;
	uint8_t TxLen;
	uint8_t TxState;
	uint8_t *pRxBuffer;
		uint8_t RxLen;
		uint8_t RxState;
}SPI_Handle_t;

/*
 * @DeviceMode
 * */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
 * @BusConfig
 * */
#define SPI_BUS_CONFIG_FD 1 /*FULL DUPLEX*/
#define SPI_BUS_CONFIG_HD 2 /*HALF DUPLEX*/
#define SPI_BUS_CONFIG_S_RXONLY 3 /*SIMPLEX receive only*/

/*
 * @SPI_SclkSpeed
 * */
#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7

/*SPI_DS*/
#define SPI_DS_8_BITS 0
#define SPI_DS_16_BITS 1

/*SPI_CPOL*/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0
/*SPI_CPHA*/
#define SPI_CPHA_SECOND 1
#define SPI_CPHA_FIRST 0
/*SPI_SSM*/
#define SPI_SSM_HW 0
#define SPI_SSM_SW 1
/*BITS POSITIONS*/

#define CPHA_BIT 0
#define CPOL_BIT 1
#define MSTR_BIT 2
#define BR_BIT 3
#define SPE_BIT 6
#define SSI_BIT 8
#define SSM_BIT 9
#define RXONLY_BIT 10
#define DFF_BIT 11
#define BIDIMODE_BIT 15
/*CR2 register*/
#define SSOE_BIT 2
#define TXEIE_BIT 7
#define RXNEIE_BIT 6
#define ERRIE_BIT 5
/*Status register*/
#define RXNE_BIT 0
#define TXE_BIT 1
#define OVR_BIT 6
/*Flag status register*/
#define SPI_RXNE_FLAG (1<<RXNE_BIT)

#define SPI_TXE_FLAG (1<<TXE_BIT)

/*SPI application states*/
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2
/*APIs*/
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Reg_Def_t *pSPIx);

void SPI_SendData(SPI_Reg_Def_t *pSPIx,uint8_t *pTxBuffer,uint8_t Len);
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx,uint8_t *pRxBuffer,uint8_t Len);
/*IRQ configuration and IDR handling*/
void SPI_IRQInterrupConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
/*oTHER APIs*/
uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx,uint32_t FlagName);
void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint8_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint8_t Len);

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
