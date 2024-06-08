/*
 * stm32f103xx.h
 *
 *  Created on: Mar 12, 2024
 *      Author: IsaJelow
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
#include <stdint.h>
#include <stddef.h>
#define _vo volatile

/*CORTEX M3 */
#define NVIC_ISER0 ((_vo uint32_t*)0xE000E100U)
#define NVIC_ISER1 ((_vo uint32_t*)0xE000E104U)

#define NVIC_ICER0 ((_vo uint32_t*)0XE000E180U)
#define NVIC_ICER1 ((_vo uint32_t*)0XE000E184U)

#define NVIC_IPR_BASE_ADDR ((_vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4
/*Memories*/
#define FLASH_BASEADDR	0x08000000U /*pag 34 datasheet*/
#define SRAM1_BASEADDR	0x20000000U
#define ROM_BASEADDR 0x1FFFF000U/*system memory pag 44 reference m?*/
/*Peripheral Buses*/
#define APB1_BASEADDR	0x40000000U
#define APB2_BASEADDR	0x40010000U
#define AHB1_BASEADDR	0x40018000U

/*peripheral on APB2 PAG 42 Reference manual and 34 datasheet */
#define EXTI_BASEADDR	0x40010400U
#define GPIOA_BASEADDR	0x40010800U
#define GPIOB_BASEADDR	0x40010C00U
#define GPIOC_BASEADDR	0x40011000U
#define GPIOD_BASEADDR	0x40011400U
#define GPIOE_BASEADDR	0x40011800U
#define ADC1_BASEADDR	0x40012400U
#define ADC2_BASEADDR	0x40012800U
#define SPI1_BASEADDR	0x40013000U
#define RCC_BASEADDR	0x40021000U


/*peripheral on APB1 PAG 42 Reference manual and 34 datasheet*/
#define SPI2_BASEADDR	0x40003800U
#define I2C1_BASEADDR	0x40005400U
#define I2C2_BASEADDR	0x40005800U

#define AFIO_BASEADDR	0x40010000U
#define EXTI_BASEADDR	0x40010400U



/*BASE ADDRESSES END*/

/*GPIO REGISTERS PAG 148 REFERENCE MANUAL*/


/*Peripheral register definition structure for GPIO*/
typedef struct
{
	_vo uint32_t CONFR[2]; /*Port configuration register LOW configures mode type and speed od the pins 0-7*/
	/*Port configuration register HIGH configures mode type and speed od the pins 8-15*/
	_vo uint32_t IDR;/*Port input data register contain the input value of the corresponding I/O port*/
	_vo uint32_t ODR;/*Port output data register  */
	_vo uint32_t BSRR;/*Port bit set/reset register modify the ODR bit*/
	_vo uint32_t BRR; /*Port bit reset register reset ODR bit*/
	_vo uint32_t LCKR;
}GPIO_Reg_Def_t;
/*Peripheral definitions*/
#define GPIOA	((GPIO_Reg_Def_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_Reg_Def_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_Reg_Def_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_Reg_Def_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_Reg_Def_t*)GPIOE_BASEADDR)

/**GPIO END**/

/*Peripheral register definition structure for RCC pag 113 reference manual*/
typedef struct
{
	_vo uint32_t CR; /*Clock control register*/
	_vo uint32_t CFGR;/*Clock configuration register*/
	_vo uint32_t CIR;/*Clock interrupt register offset 0x08*/
	_vo uint32_t APB2RSTR;/*APB2 peripheral reset register offset 0x0C*/
	_vo uint32_t APB1RSTR;/*APB1 peripheral reset register offset 0x10*/
	_vo uint32_t AHBENR;/*AHB Peripheral Clock enable register offset 0x14*/
	_vo uint32_t APB2ENR; /*APB2 peripheral clock enable register offset 0x18*/
	_vo uint32_t APB1ENR;/*APB2 peripheral clock enable register offset 0x1C*/
	_vo uint32_t BDCR;/*Backup domain control register offset 0x20*/
	_vo uint32_t CSR; /*Control/status register offset 0x24*/
	_vo uint32_t AHBRSTR; /*AHB peripheral clock reset register offset 0x28*/
	_vo uint32_t CFGR2;/*Clock configuration register2 offset 0x2C*/

}RCC_Reg_Def_t;

/*RCC definitions*/
#define RCC ((RCC_Reg_Def_t*)RCC_BASEADDR)

typedef struct
{
	_vo uint32_t EXTI_IMR;
	_vo uint32_t EXTI_EMR;
	_vo uint32_t EXTI_RTSR;
	_vo uint32_t EXTI_FTSR;
	_vo uint32_t EXTI_SWIER;
	_vo uint32_t EXTI_PR;

}EXTI_Reg_Def_t;

#define EXTI ((EXTI_Reg_Def_t*)EXTI_BASEADDR)


typedef struct
{
	_vo uint32_t AFIO_EVCR;
	_vo uint32_t AFIO_MAPR;
	_vo uint32_t AFIO_EXTICR[4];
} AFIO_Reg_Def_t;
#define AFIO ((AFIO_Reg_Def_t*)AFIO_BASEADDR)

typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;
}SPI_Reg_Def_t;

#define SPI1 ((SPI_Reg_Def_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_Reg_Def_t*)SPI2_BASEADDR)
/*
 * Peripheral Clock Enable Macros
 * */
/*GPIOx*/
#define GPIOA_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<2))
#define GPIOB_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<3))
#define GPIOC_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<4))
#define GPIOD_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<5))
#define GPIOE_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<6))


#define AFIO_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<0))
/*SPI*/
#define SPI1_PERI_CLOCK_ENABLE()	(RCC->APB2ENR |=(1<<12))
#define SPI2_PERI_CLOCK_ENABLE()	(RCC->APB1ENR |=(1<<14))
#define SPI3_PERI_CLOCK_ENABLE()	(RCC->APB1ENR |=(1<<15))
/*I2C*/
#define I2C1_PERI_CLOCK_ENABLE()	(RCC->APB1ENR |=(1<<21))
#define I2C2_PERI_CLOCK_ENABLE()	(RCC->APB1ENR |=(1<<22))


/*
 * Peripheral Clock Reset Macros
 *
 * */
#define GPIOA_RESET()	do{(RCC->APB2RSTR |=(1 << 2));(RCC->APB2RSTR &=~(1 << 2) );}while(0)
#define GPIOB_RESET()	do{(RCC->APB2RSTR |=(1 << 3));(RCC->APB2RSTR &=~(1 << 3) );}while(0)
#define GPIOC_RESET()	do{(RCC->APB2RSTR |=(1 << 4));(RCC->APB2RSTR &=~(1 << 4) );}while(0)
#define GPIOD_RESET()	do{(RCC->APB2RSTR |=(1 << 5));(RCC->APB2RSTR &=~(1 << 5) );}while(0)
#define GPIOE_RESET()	do{(RCC->APB2RSTR |=(1 << 6));(RCC->APB2RSTR &=~(1 << 6) );}while(0)


#define SPI1_RESET()	do{(RCC->APB2RSTR |=(1 << 12));(RCC->APB2RSTR &=~(1 << 12) );}while(0)
#define SPI2_RESET()	do{(RCC->APB1RSTR |=(1 << 14));(RCC->APB1RSTR &=~(1 << 14) );}while(0)
#define SPI3_RESET()	do{(RCC->APB1RSTR |=(1 << 15));(RCC->APB1RSTR &=~(1 << 15) );}while(0)
/*
 * Peripheral Clock Disable Macros
 * */
/*GPIOx*/
#define GPIOA_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<2))
#define GPIOB_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<3))
#define GPIOC_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<4))
#define GPIOD_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<5))
#define GPIOE_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<6))
/*SPI*/
#define SPI1_PERI_CLOCK_DISABLE()	(RCC->APB2ENR &=~(1<<12))
#define SPI2_PERI_CLOCK_DISABLE()	(RCC->APB1ENR &=~(1<<14))
#define SPI3_PERI_CLOCK_DISABLE()	(RCC->APB1ENR &=~(1<<15))
/*I2C*/
#define I2C1_PERI_CLOCK_DISABLE()	(RCC->APB1ENR &=~(1<<21))
#define I2C2_PERI_CLOCK_DISABLE()	(RCC->APB1ENR &=~(1<<22))



/*General Macros */
#define ENABLE	1
#define DISABLE	0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_SET ENABLE
#define FLAG_RESET DISABLE

#define HSI_ON()	(RCC->CR |= (1<<0))
#define SELECT_HSI() do{(RCC->CFGR &= ~(0b111<<24));(RCC->CFGR |= (0b101<<24));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x== GPIOA)? 0 :\
										(x== GPIOB)? 1 :\
										(x== GPIOC)? 2 :\
										(x== GPIOD)? 3 :\
										(x== GPIOE)? 4 : 0)


#define EXTI1_IRQn 7 /*EXTI line1 interrupt*/
#define EXTI15_10_IRQn 40

//SPI Interrupts
#define SPI1_Global_Interrupt 35
#define SPI2_Global_Interrupt 36
#define SPI3_Global_Interrupt 51

#endif /* INC_STM32F103XX_H_ */
