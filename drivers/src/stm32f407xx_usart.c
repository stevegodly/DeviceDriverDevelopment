/*
 * stm32f407xx_usart.c
 *
 *  Created on: Oct 12, 2023
 *      Author: user
 */
#ifndef INC_STM32F407XX_USART_C_
#define INC_STM32F407XX_USART_C_

#include "stm32f407xx_usart.h"

void USART_PeripheralControl(USART_t *USART,uint32_t x){
	if(x==1) USART->CR1|=1<<13;
	else USART->CR1&=1<<13;
}

uint8_t USART_GetFlagStatus(USART_t *USART, uint8_t x){
    if(USART->SR & x){
    	return SET;
    }

   return RESET;
}

void USART_ClearFlag(USART_t *USART, uint8_t x){
	USART->SR&=1<<x;
}

void USART_ClockControl(USART_t *USART,uint8_t x){
	if(x==1){
		if(USART == USART1){
			USART1_PCCK_EN();
		}else if (USART == USART2){
			USART2_PCCK_EN();
		}else if (USART == USART3){
			USART3_PCCK_EN();
		}else if (USART == UART4){
			UART4_PCCK_EN();
		}else if (USART == UART5){
			UART5_PCCK_EN();
		}else if (USART == USART6){
			USART6_PCCK_EN();
		}
	}
}

void USART_SetBaudRate(USART_t *USART,uint32_t baudRate){
	uint8_t x;
	uint32_t fClk,USARTDIV,temp=0;

	if(USART==USART1 || USART==USART6) fClk=RCC_Pclk1Value();
	else fClk=RCC_Pclk2Value();

	if(USART->CR1 & 1<<15)	x=0;
	else x=1;

	USARTDIV=(fClk*100)/(baudRate*8*(2-x));

	temp|=(USARTDIV/100)<<4;
	temp|=((fClk/1000000)*(USARTDIV%100)+50) & 0x7;

	USART->BRR=temp;
}

void USART_Init(USART_Handle_t *USART_Handle){
	uint32_t temp=0;

	if(USART_Handle->USART_Config.wordLength) temp|=1<<12;

	if(USART_Handle->USART_Config.mode==0) temp|=1<<2;
	else if(USART_Handle->USART_Config.mode==1)	temp|=1<<3;
	else if (USART_Handle->USART_Config.mode == 2){
		temp |= ((1<<2) | (1<<3));
	}

	if(USART_Handle->USART_Config.parityControl==2) temp|=1<<10;

	else if(USART_Handle->USART_Config.parityControl==1){
		temp|=1<<10;
		temp|=1<<9;
	}

	USART_Handle->USART->CR1=temp;
	USART_Handle->USART->CR2|=USART_Handle->USART_Config.stopBits<<12;

	temp=0;
	if(USART_Handle->USART_Config.hwFlowControl==0)	USART_Handle->USART->CR3|=1<<9;

	else if(USART_Handle->USART_Config.hwFlowControl==1) USART_Handle->USART->CR3|=1<<8;

	else if(USART_Handle->USART_Config.hwFlowControl==2){
		USART_Handle->USART->CR3|=1<<9;
		USART_Handle->USART->CR3|=1<<8;
	}
}

void USART_SendData(USART_t *USART,uint8_t *txBuffer,uint8_t len){
	while(len--){
		while(!(USART->SR & 1<<7));
		if(USART->CR1 & 1<<12){
			USART->DR=*((uint16_t*)txBuffer) & 0x01ff;
			if(!(USART->CR1 & 1<<10)){
				txBuffer++;
			}
		}
		else USART->DR=*(txBuffer);
		txBuffer++;

		while(!(USART->SR & 1<<6));
	}
}

void USART_ReceiveData(USART_t *USART, uint8_t *rxBuffer, uint32_t len){
	while(len--){
		while(!(USART->SR &1<<5));
		if(USART->CR1 & 1<<12){
			*((uint16_t*)rxBuffer)=USART->DR & 0x01ff;
			if(USART->CR1 & 1<<10){
				rxBuffer++;
			}
		}
		else *(rxBuffer)=USART->DR;

		rxBuffer++;

		while(!(USART->SR & 1<<6));
	}
}

#endif
