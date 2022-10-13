/*
 * stm32f030xx.h
 *
 *  Created on: Sep 22, 2022
 *      Author: xraid
 */
#include <stdint.h>

#ifndef INC_STM32F030XX_H_
#define INC_STM32F030XX_H_

#define __vo volatile

/*
 * Base addresses of Flash and SRAM
 */

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define ROM_BASEADDR 		0x1FFFEC00U
#define SRAM				SRAM1_BASEADDR

/*
 * AHBx and APBx bus peripherals base addresses
 */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x48000000U

/*
 * AHB1 bus peripherals -- RCC
 */

#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x1000)

/*
 * AHB2 bus peripherals -- GPIOA_F
 */

#define GPIOA_BASEADDR		(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB2PERIPH_BASE + 0x0C00)
#define GPIOF_BASEADDR		(AHB2PERIPH_BASE + 0x1400)

/*
 * APB1 bus peripherals -- I2c1-2 SPI2 USART2-5
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define USART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define USART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/*
 * APB2 bus peripherals -- SPI1 USART1&6 EXTI SYSCFG
 */

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)

#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x0400)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x0000)

/*
 * Peripheral register structure --GPIOx
 */

typedef struct{
	__vo uint32_t MODER;  			/*Offset:0x00 GPIO port mode register */
	__vo uint32_t OTYPER;			/*Offset:0x04 GPIO port output type register */
	__vo uint32_t OSPEEDR;			/*Offset:0x08 GPIO port output speed register */
	__vo uint32_t PUPDR;			/*Offset:0x0C GPIO port pull-up/pull-down register */
	__vo uint32_t IDR;				/*Offset:0x10 GPIO port input data register */
	__vo uint32_t ODR;				/*Offset:0x14 GPIO port output data register */
	__vo uint32_t BSRR;				/*Offset:0x18 GPIO port bit set/reset register */
	__vo uint32_t LCKR;				/*Offset:0x1c GPIO port configuration lock register */
	__vo uint32_t AFR[2];			/*Offset:0x20 GPIO alternate function: AFR[0]:low register / AFR[1]:high register */
	__vo uint32_t BRR;				/*Offset:0x28 GPIO port bit reset register */
}GPIO_RegDef_t;

/*
 * Peripheral register structure -- RCC
 */

typedef struct{
	__vo uint32_t CR;				/*Offset:0x00 Clock control register */
	__vo uint32_t CFGR;				/*Offset:0x04 Clock configuration register */
	__vo uint32_t CIR;				/*Offset:0x08 Clock interrupt register */
	__vo uint32_t APB2RSTR;			/*Offset:0x0C APB peripheral reset register 2 */
	__vo uint32_t APB1RSTR;			/*Offset:0x10 APB peripheral reset register 1 */
	__vo uint32_t AHBENR;			/*Offset:0x14 AHB peripheral clock enable register */
	__vo uint32_t APB2ENR;			/*Offset:0x18 APB peripheral clock enable register 2 */
	__vo uint32_t APB1ENR;			/*Offset:0x1C APB peripheral clock enable register 1 */
	__vo uint32_t BDCR;				/*Offset:0x20 RTC domain control register */
	__vo uint32_t CSR;				/*Offset:0x24 Control/status register */
	__vo uint32_t AHBRSTR;			/*Offset:0x28 AHB peripheral reset register */
	__vo uint32_t CFGR2;			/*Offset:0x2C Clock configuration register 2 */
	__vo uint32_t CFGR3;			/*Offset:0x30 Clock configuration register 3 */
	__vo uint32_t CR2;				/*Offset:0x34 Clock control register 2 */
}RCC_RegDef_t;

/*
 * Peripheral register structure -- I2Cx
 */

typedef struct{
	__vo uint32_t CR1;				/*Offset:0x00 Control register 1 */
	__vo uint32_t CR2;				/*Offset:0x04 Control register 2 */
	__vo uint32_t OAR1;				/*Offset:0x08 Own address 1 register */
	__vo uint32_t OAR2;				/*Offset:0x0C Own address 2 register */
	__vo uint32_t TIMINGR;			/*Offset:0x10 Timing register */
	__vo uint32_t TIMEOUTR;			/*Offset:0x14 Timeout register */
	__vo uint32_t ISR;				/*Offset:0x18 Interrupt and status register */
	__vo uint32_t ICR;				/*Offset:0x1C Interrupt clear register */
	__vo uint32_t PECR;				/*Offset:0x20 PEC register*/
	__vo uint32_t RXDR;				/*Offset:0x24 Receive data register */
	__vo uint32_t TXDR;				/*Offset:0x28 Transmit data register */
}I2C_RegDef_t;

/*
 * Peripheral register structure -- SPIx
 */

typedef struct{
	__vo uint32_t CR1;				/*Offset:0x00 SPI control register 1*/
	__vo uint32_t CR2;				/*Offset:0x04 SPI control register 2 */
	__vo uint32_t SR;				/*Offset:0x08 SPI status register*/
	__vo uint32_t DR;				/*Offset:0x0C SPI data register*/
	__vo uint32_t CRCPR;			/*Offset:0x10 SPI CRC polynomial register*/
	__vo uint32_t RXCRCR;			/*Offset:0x14 SPI Rx CRC register*/
	__vo uint32_t TXCRCR;			/*Offset:0x18 SPI Tx CRC register*/
}SPI_RegDef_t;

/*
 * Peripheral definition(typecasted to xx_RegDef_t) -- GPIOx
 */

#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)

/*
 * Peripheral definition(typecasted to xx_RegDef_t) -- RCC
 */

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Peripheral definition(typecasted to xx_RegDef_t) -- I2Cx
 */

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)

/*
 * Peripheral definition(typecasted to xx_RegDef_t) -- SPIx
 */

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)

/*
 * Clock ENABLE macros for GPIOx
 */

#define GPIOA_PCLK_EN() 	(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN() 	(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN() 	(RCC->AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN() 	(RCC->AHBENR |= (1 << 22))

/*
 * Clock ENABLE macros for I2Cx
 */

#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 22))

/*
 * Clock ENABLE macros for SPIx
 */

#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 14))

/*
 * Clock ENABLE macros for USARTx
 */

#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN() 	(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN() 	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock ENABLE macros for SYSCFG
 */

#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 0))

/*
 * Clock DISABLE macros for GPIOx
 */

#define GPIOA_PCLK_DI() 	(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI() 	(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI() 	(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI() 	(RCC->AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI() 	(RCC->AHBENR &= ~(1 << 22))

/*
 * Clock DISABLE macros for I2Cx
 */

#define I2C1_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock DISABLE macros for SPIx
 */

#define SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 14))

/*
 * Clock DISABLE macros for USARTx
 */

#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock DISABLE macros for SYSCFG
 */

#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 0))

/*
 * Macros to disable GPIOx peripherals
 */

#define GPIOA_REG_RESET()	do{(RCC->AHBRSTR &= ~(1 << 17));	(RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHBRSTR &= ~(1 << 18));	(RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHBRSTR &= ~(1 << 19));	(RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHBRSTR &= ~(1 << 20));	(RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHBRSTR &= ~(1 << 22));	(RCC->AHBRSTR &= ~(1 << 22)); }while(0)

/*
 * Generic Macros
 */

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET

#include "stm32f030xx_gpio_driver.h"

#endif /* INC_STM32F030XX_H_ */
