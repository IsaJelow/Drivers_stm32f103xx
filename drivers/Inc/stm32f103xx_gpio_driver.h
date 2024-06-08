/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Mar 16, 2024
 *      Author: IsaJelow
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_
#include "stm32f103xx.h"
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinInputOutputMode;
	/*uint8_t GPIO_PinOPtype;
	uint8_t GPIO_PinAltFunMode;*/

}GPIO_PinCnfg_t;

typedef struct{
	GPIO_Reg_Def_t *pGPIOx; /*Holds base address of the GPIO*/
	GPIO_PinCnfg_t GPIO_PinCnfg;/*Holds pin configurations settings*/
}GPIO_Handle_t;


/*APIs
 * */
/*Peripheral clock setup*/
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx,uint8_t EnOrDi);

/*Init and deInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx);

/*Read and Write data*/
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber);

/*IRQ configuration and handling*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t  EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber,uint8_t  IRQPriority);


/*
 *
 * @GPIO_PIN:MODES
 * */
#define GPIO_MODE_IN	0 /*: Input mode (reset state)*/
#define GPIO_MODE_OUT_MEDIUMS	1 /* Output mode, max speed 10 MHz.*/
#define GPIO_MODE_OUT_LOWS	2 /*Output mode, max speed 2 MHz.*/
#define GPIO_MODE_OUT_HIGHS	3 /*Output mode, max speed 50 MHz.*/
#define GPIO_MODE_IT_FT	4
#define GPIO_MODE_IT_RT	5
#define GPIO_MODE_IT_FT_RT	6

/*Configuration mode*/
/*in input mode*/
#define GPIO_MODE_ANALOG 0 /*Analog input mode*/
#define GPIO_MODE_FLOATING 1 /*Floating input (reset state)*/
#define GPIO_MODE_PU_PD 2 /*: Input with pull-up / pull-down*/

/*in output mode*/
#define GPIO_MODE_OUT_PP	0 /*General purpose output push-pull*/
#define GPIO_MODE_OUT_OD	1 /*: General purpose output Open-drain*/
#define GPIO_MODE_ALTFN_PP 2 /*Alternate function output Push-pull*/
#define GPIO_MODE_ALTFN_OD 3 /*Alternate function output Open-drain*/



/*
 *
 * @GPIO_PIN_NUMBERS
 * */
#define GPIO_PIN0	0
#define GPIO_PIN1	1
#define GPIO_PIN2	2
#define GPIO_PIN3	3
#define GPIO_PIN4	4
#define GPIO_PIN5	5
#define GPIO_PIN6	6
#define GPIO_PIN7	7
#define GPIO_PIN8	8
#define GPIO_PIN9	9
#define GPIO_PIN10	10
#define GPIO_PIN11	11
#define GPIO_PIN12	12
#define GPIO_PIN13	13
#define GPIO_PIN14	14
#define GPIO_PIN15	15

#define ALTF 0

#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
