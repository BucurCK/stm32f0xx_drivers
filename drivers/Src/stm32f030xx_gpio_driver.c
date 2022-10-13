/*
 * stm32f030xx_gpio_driver.c
 *
 *  Created on: Sep 23, 2022
 *      Author: xraid
 */

#include "stm32f030xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/******************************************************************
 * @fn					-	GPIO_PeriClockControl
 *
 * @brief				-	This function enables or disable the peripheral clock for the given GPIO port
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	ENABLE or DISABLE macros
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){

	if(ENorDI == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/******************************************************************
 * @fn					-	GPIO_Init
 *
 * @brief				-	This function initializes the gpio port
 *
 * @param[in]			-	base address of the gpio handler
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; //temporary register
	//1. configure the mode of the GPIO (interrupt or non interrupt)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else{
		//interrupt mode
		//will do later
	}
	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. configure the alternative functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alternative function
		uint8_t temp1, temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/******************************************************************
 * @fn					-	GPIO_DeInit
 *
 * @brief				-	This function de-initializes the gpio port
 *
 * @param[in]			-	base address of the gpio peripheral
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

/*
 * Data Read and Write
 */

/******************************************************************
 * @fn					-	GPIO_ReadFromInputPin
 *
 * @brief				-	This function reads from the Pin
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	the number of the pin
 *
 * @return				-	uint8_t (0 or 1)
 *
 * @Note				-	none

 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR  >> PinNumber) & 0x00000001);
	return value;
}

/******************************************************************
 * @fn					-	GPIO_ReadFromInputPort
 *
 * @brief				-	This function reads from the port
 *
 * @param[in]			-	base address of the gpio peripheral
 *
 * @return				-	uint16_t
 *
 * @Note				-	none

 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/******************************************************************
 * @fn					-	GPIO_WriteToOutPin
 *
 * @brief				-	This function writes to the pin
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	the number of the pin
 * @param[in]			-	the value that will be written
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_RESET){
		//write 1 to the output data register of the corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//write 0 to the output data register of the corresponding pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/******************************************************************
 * @fn					-	GPIO_WriteToOutPort
 *
 * @brief				-	This function writes to the port
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	the value that will be written
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}

/******************************************************************
 * @fn					-	GPIO_ToggleOutputPin
 *
 * @brief				-	This function toggles the pins
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	the pin number
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}
/*
 * IRQ Configuration and IRS Handling
 */

/******************************************************************
 * @fn					-	GPIO_IRQConfig
 *
 * @brief				-	This function configures the IRQ
 *
 * @param[in]			-	the interrupt number
 * @param[in]			-	the interrupt priority
 * @param[in]			-	ENABLE or DISABLE macros
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);

/******************************************************************
 * @fn					-	GPIO_IRQConfig
 *
 * @brief				-	This function configures the IRQ
 *
 * @param[in]			-	the interrupt number
 * @param[in]			-	the interrupt priority
 * @param[in]			-	ENABLE or DISABLE macros
 *
 * @return				-	none
 *
 * @Note				-	none

 */

void GPIO_IRQHandling(uint8_t PinNumber);
