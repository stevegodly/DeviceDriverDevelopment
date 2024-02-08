#include <stdint.h>
#include<string.h>
#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{
	GPIO_Handle_t GpioLed,GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GpioLed));



	GpioLed.GPIO = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = 12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	GPIOBtn.GPIO = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);

	GPIO_WriteToOutputPin(GPIOD,12,GPIO_PIN_RESET);
		//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

    while(1);

}


void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD,12);
}
