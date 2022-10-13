/*
 * stm32f030xx_gpio_driver.h
 *
 *  Created on: Sep 23, 2022
 *      Author: xraid
 */

#ifndef INC_STM32F030XX_GPIO_DRIVER_H_
#define INC_STM32F030XX_GPIO_DRIVER_H_

#include "stm32f030xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;				/*possible values for @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;				/*possible values for @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;				/*possible values for @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;		/*possible values for @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;				/*possible values for @GPIO_PIN_OP_TYPE*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;				/*This holds the base address of the GPIO port to witch the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*This holds GPIO pin configuration settings*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin possible number
 */

#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4		/*Input Falling Edge*/
#define GPIO_MODE_IT_RT			5		/*Input Rising Edge*/
#define GPIO_MODE_IT_RFT		6		/*Input Rising Falling Edge*/

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up/pull-down configuration macros
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2
/*
							*********************************
							* 								*
							* APIs supported by this driver *
							*								*
							*********************************
 */

/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and IRS Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32F030XX_GPIO_DRIVER_H_ */
