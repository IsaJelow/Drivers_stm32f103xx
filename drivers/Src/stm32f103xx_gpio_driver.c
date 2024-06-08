/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Mar 16, 2024
 *      Author: IsaJelow
 */

#include "stm32f103xx_gpio_driver.h"
/*Peripheral clock setup*/

/* @fn 	*GPIO_PeriClockControl
 *
 * @brief   *enables or disables peripheral clock for the giving GPIO port
 * @pGpiox  Base address of the GPIO
 * @EnOrDi  Enable or Disable MACRO
 *
 * @return none
 * */
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PERI_CLOCK_ENABLE();
				}
		else if(pGPIOx == GPIOC){
			GPIOC_PERI_CLOCK_ENABLE();
						}
		else if(pGPIOx == GPIOD){
			GPIOD_PERI_CLOCK_ENABLE();
						}
		else if(pGPIOx == GPIOE){
			GPIOE_PERI_CLOCK_ENABLE();
						}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PERI_CLOCK_DISABLE();
				}
		else if(pGPIOx == GPIOB){
			GPIOB_PERI_CLOCK_DISABLE();
						}
		else if(pGPIOx == GPIOC){
			GPIOC_PERI_CLOCK_DISABLE();
								}
		else if(pGPIOx == GPIOD){
			GPIOD_PERI_CLOCK_DISABLE();
								}
		else if(pGPIOx == GPIOE){
			GPIOE_PERI_CLOCK_DISABLE();
								}
	}
}

/*Init and deInit*/

/* @fn 	*GPIO_Init
 *
 * @brief   *Initializes the  GPIO pin
 * @pGpioHandle  Pinter to the GPIO handle structure
 *
 *
 * @return none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	uint8_t tempregister=0,tempin=0;
	tempregister=pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber / 8;
	tempin=pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber % 8;
	/*Configure the mode*/
		if(pGPIOHandle->GPIO_PinCnfg.GPIO_PinMode <= GPIO_MODE_OUT_HIGHS){

				temp=(pGPIOHandle->GPIO_PinCnfg.GPIO_PinMode<<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] &=~(0x3 <<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] |= temp;
			}
		else
		{
			if(pGPIOHandle->GPIO_PinCnfg.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				temp=(GPIO_MODE_IN<<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] &=~(0x3 <<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] |= temp;

				EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
				EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
			}

			else if(pGPIOHandle->GPIO_PinCnfg.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				temp=(GPIO_MODE_IN<<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] &=~(0x3 <<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] |= temp;

				EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
				EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinCnfg.GPIO_PinMode == GPIO_MODE_IT_FT_RT)
			{
				temp=(GPIO_MODE_IN<<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] &=~(0x3 <<(4 * tempin));
				pGPIOHandle->pGPIOx->CONFR[tempregister] |= temp;

				EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
				EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);
			}

			uint8_t tempregisterAFIO=0,tempinAFIO=0;
			tempregisterAFIO=pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber / 4;
			tempinAFIO=pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			AFIO_PERI_CLOCK_ENABLE();
			AFIO->AFIO_EXTICR[tempregisterAFIO]=portcode<<(tempinAFIO*4);

			EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinCnfg.GPIO_PinNumber);



		}
	/*Configure the input output mode*/
		temp=0;
		temp=(pGPIOHandle->GPIO_PinCnfg.GPIO_PinInputOutputMode<<((4 * tempin)+2));
		pGPIOHandle->pGPIOx->CONFR[tempregister] &=~(0x3 <<((4 * tempin)+2));
		pGPIOHandle->pGPIOx->CONFR[tempregister] |= temp;




}
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
				GPIOA_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_RESET();
					}
			else if(pGPIOx == GPIOC){
				GPIOC_RESET();
							}
			else if(pGPIOx == GPIOD){
				GPIOD_RESET();
							}
			else if(pGPIOx == GPIOE){
				GPIOE_RESET();
							}

}

/*Read and Write data*/
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber){
	uint8_t value=0;
	value=((pGPIOx->IDR)>>PinNumber) & 0x1;
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	pGPIOx->BSRR |= (Value<<PinNumber);
}
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx,uint8_t Value){

}
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber){
	uint8_t value=0;
	value=((pGPIOx->ODR)>>PinNumber) & 0x1;
	if(value==1){
		pGPIOx->BSRR |=((1)<<(16+PinNumber));
	}else{
		pGPIOx->BSRR |= (1<<PinNumber);
	}


}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t  EnOrDi){
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
void GPIO_IRQPriorityConfig (uint8_t IRQNumber,uint8_t  IRQPriority){

	//ipr register
	uint8_t iprregister=IRQNumber / 4;
	uint8_t iprsection=IRQNumber % 4;
	uint8_t shift_v=(8 * iprsection)+(8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR+(iprregister))|=(IRQPriority<<shift_v);

}
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->EXTI_PR & (1<<PinNumber)){
		EXTI->EXTI_PR |= (1<<PinNumber);
	}
}
