/*
 * stm32f407xx_rcc.c
 *
 *  Created on: Oct 13, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_RCC_C_
#define INC_STM32F407XX_RCC_C_

#include "stm32f407xx_rcc.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2, 4 , 8, 16};


uint32_t RCC_Pclk1Value(void){
	uint32_t Pclk1,sysClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 ){
		sysClk = 16000000;
	}
	else if(clksrc == 1){
		sysClk = 8000000;
	}
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[temp-8];
	}
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4){
		apb1p = 1;
	}
	else{
		apb1p = APB1_PreScaler[temp-4];
	}

	Pclk1 =  (sysClk / ahbp) /apb1p;
	return Pclk1;
}

uint32_t RCC_Pclk2Value(void){
	uint32_t Pclk1,sysClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 ){
		sysClk = 16000000;
	}
	else if(clksrc == 1){
		sysClk = 8000000;
	}
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[temp-8];
	}
	temp = ((RCC->CFGR >> 13 ) & 0x7);

	if(temp < 4){
		apb1p = 1;
	}
	else{
		apb1p = APB1_PreScaler[temp-4];
	}

	Pclk1 =  (sysClk / ahbp) /apb1p;
	return Pclk1;
}


#endif
