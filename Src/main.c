/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*Programa que togglea el led ubicado en PC13 con el push de un botón*/
#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"
void F_Clock_Init(void);
void F_GPIOx_Init(void);
int main(void)
{

	F_Clock_Init();
	F_GPIOx_Init();

	while(1){
	if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN10) == 1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
	}
	}
}
void F_Clock_Init(void)
{
	HSI_ON();
	SELECT_HSI();
	}
void F_GPIOx_Init(void){

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Handle_t GPIO_Init_struct;
	GPIO_Init_struct.pGPIOx= GPIOA;
	GPIO_Init_struct.GPIO_PinCnfg.GPIO_PinNumber=GPIO_PIN10;
	GPIO_Init_struct.GPIO_PinCnfg.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Init_struct.GPIO_PinCnfg.GPIO_PinInputOutputMode=GPIO_MODE_PU_PD;

	GPIO_Init(&GPIO_Init_struct);

	GPIO_PeriClockControl(GPIOC, ENABLE);
		GPIO_Handle_t GPIOC_Init_struct;
		GPIOC_Init_struct.pGPIOx= GPIOC;
		GPIOC_Init_struct.GPIO_PinCnfg.GPIO_PinNumber=GPIO_PIN13;
		GPIOC_Init_struct.GPIO_PinCnfg.GPIO_PinMode=GPIO_MODE_OUT_LOWS;
		GPIOC_Init_struct.GPIO_PinCnfg.GPIO_PinInputOutputMode=GPIO_MODE_OUT_PP;

		GPIO_Init(&GPIOC_Init_struct);

}
