/*
 * led_toggle.c
 *
 *  Created on: 13 Oct 2022
 *      Author: xraid
 */

#include "stm32f030xx.h"

void delay(){
	for(uint32_t i = 0; i < 250000; ++i);
}

int main(void){

	GPIO_Handle_t GpioLed;

 	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){

		GPIO_ToggleOutputPin(GPIOA, 4);
		delay();
	}
	return 0;
}
