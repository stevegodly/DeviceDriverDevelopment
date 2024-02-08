/*
 * stm32f407xx_usart.h
 *
 *  Created on: Oct 12, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_USART_H_
#define INC_STM32F407XX_USART_H_

#include "stm32f407xx.h"

#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2


typedef struct{
	uint8_t mode;
	uint32_t baud;
	uint8_t stopBits;
	uint8_t wordLength;
	uint8_t parityControl;
	uint8_t hwFlowControl;
}USART_Config_t;

typedef struct{
	USART_t *USART;
	USART_Config_t USART_Config;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}USART_Handle_t;

#define USART1  			((USART_t*)USART1_BASEADDR)
#define USART2  			((USART_t*)USART2_BASEADDR)
#define USART3  			((USART_t*)USART3_BASEADDR)
#define UART4  				((USART_t*)UART4_BASEADDR)
#define UART5  				((USART_t*)UART5_BASEADDR)
#define USART6  			((USART_t*)USART6_BASEADDR)

#endif /* INC_STM32F407XX_USART_H_ */
