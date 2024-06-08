/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Apr 16, 2024
 *      Author: IsaJelow
 */

#include "stm32f103xx_spi_driver.h"


void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			if(pSPIx == SPI1){
				SPI1_PERI_CLOCK_ENABLE();
			}
			else if(pSPIx == SPI2){
				SPI2_PERI_CLOCK_ENABLE();
					}
		}
		else{
			if(pSPIx == SPI1){
					SPI1_PERI_CLOCK_DISABLE();
						}
			else if(pSPIx == SPI2){
					SPI2_PERI_CLOCK_DISABLE();
						}
		}
}

void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
				pSPIx->CR1 |= (1 << SPE_BIT);
	}else{
				pSPIx->CR1 &= ~(1 << SPE_BIT);
	}
}

void SPI_SendData(SPI_Reg_Def_t *pSPIx,uint8_t *pTxBuffer,uint8_t Len){
	while((uint8_t)Len > 0){
		while (SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET){}
		if((pSPIx->CR1 >> 11 & 0x1)==SPI_DS_8_BITS){
			//8 Bits data size
			*((_vo uint8_t*)&pSPIx->DR) = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
		else if((pSPIx->CR1 >> 11 & 0x1)==SPI_DS_16_BITS){
			uint16_t Data_swaped;
			Data_swaped=(*pTxBuffer<<8) | (*(pTxBuffer + 1));
			*((_vo uint16_t*)&pSPIx->DR) = Data_swaped;
			Len--;
			Len--;
			pTxBuffer+=2;
		}
	}
}
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*Peripheral clock enable*/
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);
	uint32_t tempreg=0;

	tempreg|=pSPIHandle->SPI_Config.SPI_DeviceMode<<MSTR_BIT;

	if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_FD){
		tempreg &= ~(1<<BIDIMODE_BIT);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_HD){
			tempreg |= (1<<BIDIMODE_BIT);
		}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_S_RXONLY){
		tempreg &= ~(1<<BIDIMODE_BIT);
		tempreg |= (1<<RXONLY_BIT);
			}

	//VELOCIDAD
	tempreg &= ~(3<<BR_BIT);
	tempreg |=  (pSPIHandle->SPI_Config.SPI_SclkSpeed << BR_BIT);

	//CLOCK POLARITY
	tempreg |=  (pSPIHandle->SPI_Config.SPI_CPOL << CPOL_BIT);

	//CLOCK PHASE
	tempreg |=  (pSPIHandle->SPI_Config.SPI_CPOL << CPHA_BIT);
	//SLAVE SELECT SW OR HW
	tempreg |=  (pSPIHandle->SPI_Config.SPI_SSM << SSM_BIT);

	//data size
	tempreg |=  (pSPIHandle->SPI_Config.SPI_DS << DFF_BIT);
	pSPIHandle->pSPIx->CR1 |= tempreg;

}
void SPI_DeInit(SPI_Reg_Def_t *pSPIx){
	if(pSPIx == SPI1)
	{
	SPI1_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR& FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SSI_BIT);
	}else{
		pSPIx->CR1 &= ~(1 << SSI_BIT);
	}
}
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
			pSPIx->CR2 |= (1 << SSOE_BIT);
		}else{
			pSPIx->CR2 &= ~(1 << SSOE_BIT);
		}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t  EnOrDi){
	if(EnOrDi== ENABLE){
		if(IRQNumber<=31){
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}
	}
	else {
		if(IRQNumber<=31){
					*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
					*NVIC_ICER1 |= (1<<(IRQNumber % 32));
				}

	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t iprregister=IRQNumber / 4;
	uint8_t iprsection=IRQNumber % 4;
	uint8_t shift_v=(8 * iprsection)+(8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR+(iprregister))|=(IRQPriority<<shift_v);
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint8_t Len){
	uint8_t state=pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
	pSPIHandle->pTxBuffer=pTxBuffer;
	pSPIHandle->TxLen=Len;
	pSPIHandle->TxState=SPI_BUSY_IN_TX;
	pSPIHandle->pSPIx->CR2 |=(ENABLE<<TXEIE_BIT);
	}
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint8_t Len){
	uint8_t state=pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer=pTxBuffer;
		pSPIHandle->RxLen=Len;
		pSPIHandle->RxState=SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |=(ENABLE<<RXNEIE_BIT);
		}
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;

	temp1=pSPIHandle->pSPIx->SR & (1<<TXE_BIT);
	temp2=pSPIHandle->pSPIx->CR2 & (1<<TXEIE_BIT);
	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);

	}

	temp1=pSPIHandle->pSPIx->SR & (1<<RXNE_BIT);
	temp2=pSPIHandle->pSPIx->CR2 & (1<<RXNEIE_BIT);
	if(temp1 && temp2){
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);

	}

	//check for ovr flag
	temp1=pSPIHandle->pSPIx->SR & (1<<OVR_BIT);
	temp2=pSPIHandle->pSPIx->CR2 & (1<<ERRIE_BIT);
	if(temp1 && temp2){
		//handle TXE
		spi_ovr_interrupt_handle(pSPIHandle);

	}
}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){


			if((pSPIHandle->pSPIx->CR1 >> 11 & 0x1)==SPI_DS_8_BITS){
				*((_vo uint8_t*)&pSPIHandle->pSPIx->DR) = *((uint8_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			else if((pSPIHandle->pSPIx->CR1 >> 11 & 0x1)==SPI_DS_16_BITS){
				uint16_t Data_swaped;
				Data_swaped=((*((_vo uint16_t*)pSPIHandle->pTxBuffer) << 8) | (*((_vo uint16_t*)pSPIHandle->pTxBuffer + 1)));
				*((_vo uint8_t*)&pSPIHandle->pSPIx->DR)  = Data_swaped;
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer+=2;
			}
			if(!pSPIHandle->TxLen){}
			//SPI_CloseTransmission(pSPIHandle);
			//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);


}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if((pSPIHandle->pSPIx->CR1 >> 11 & 0x1)==SPI_DS_8_BITS){
		*((uint8_t*)pSPIHandle->pRxBuffer)=*((_vo uint8_t*)&pSPIHandle->pSPIx->DR);
					pSPIHandle->RxLen--;
					pSPIHandle->pRxBuffer++;
	}else if((pSPIHandle->pSPIx->CR1 >> 11 & 0x1)==SPI_DS_16_BITS){
					uint16_t Data_swaped;
					Data_swaped=*((_vo uint16_t*)&pSPIHandle->pSPIx->DR);
					*((_vo uint16_t*)pSPIHandle->pTxBuffer)=(Data_swaped >> 8);
					*((_vo uint16_t*)pSPIHandle->pTxBuffer + 1)=(Data_swaped & 0xFF);
					pSPIHandle->RxLen--;
					pSPIHandle->RxLen--;
					pSPIHandle->pRxBuffer+=2;
				}
				if(!pSPIHandle->RxLen){
					//SPI_CloseReception(pSPIHandle);
					//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
				}
}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp= pSPIHandle->pSPIx->DR;
		temp= pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1<< TXEIE_BIT);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1<< RXNEIE_BIT);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
