/*
 * tm32f407xx_I2C_.c
 *
 *  Created on: Oct 6, 2023
 *      Author: user
 */
#include "stm32f407xx_i2c.h"

uint32_t flag;

static void I2C_GenerateStart(I2C_t *I2C){
	I2C->CR1|=1<<8;
}

static void I2C_AddressWrite(I2C_t *I2C,uint8_t slaveAddr){
	I2C->DR=slaveAddr<<1;
}

static void I2C_AddressRead(I2C_t *I2C,uint8_t slaveAddr){
	I2C->DR=(slaveAddr<<1)|1;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *I2C_Handle ){
	uint8_t dummyRead=I2C_Handle->I2C->SR1;
	dummyRead=I2C_Handle->I2C->SR2;
	(void)(dummyRead);
}

static void I2C_StopCondition(I2C_t* I2C){
	I2C->CR1|=1<<9;
}

void I2C_ManageAcking(I2C_t *I2C, uint8_t x){
	if(x == 1){
		I2C->CR1 |=(1<<10);
	}else{
		I2C->CR1 &=~(1<<9);
	}
}

void sendData(I2C_t *I2C,uint8_t *txBuffer,uint32_t flag,uint32_t len){
	while(len > 0)
	{
		while(!I2C_FlagStatus(I2C,flag));
		I2C->DR = *txBuffer;
		txBuffer++;
		len--;
	}
}


void I2C_PeripheralControl(I2C_t *I2C, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		I2C->CR1 |= (1<<0);
	}else
	{
		I2C->CR1 &= ~(1<<0);
	}

}

void I2C_PeriClockControl(I2C_t *I2C, uint8_t x){
	if(x==1){
		if(I2C == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (I2C == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (I2C == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

void I2C_CloseSendData(I2C_Handle_t *I2C_Handle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C_Handle->I2C->CR2 &= ~( 1 << 10);

	//Implement the code to disable ITEVFEN Control Bit
	I2C_Handle->I2C->CR2 &= ~( 1 << 9);


	I2C_Handle->TxRxState = I2C_READY;
	I2C_Handle->TxBuffer = NULL;
	I2C_Handle->TxLen = 0;

	if(I2C_Handle->I2C_Config.ackControl==1) I2C_ManageAcking(I2C_Handle->I2C, 1);
}

void I2C_CloseRecieveData(I2C_Handle_t *I2C_Handle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C_Handle->I2C->CR2 &= ~( 1 << 10);

	//Implement the code to disable ITEVFEN Control Bit
	I2C_Handle->I2C->CR2 &= ~( 1 << 9);


	I2C_Handle->TxRxState = I2C_READY;
	I2C_Handle->RxBuffer= NULL;
	I2C_Handle->RxLen = 0;
}

void I2C_Init(I2C_Handle_t *I2C_Handle){
	uint32_t temp=0;

	I2C_PeriClockControl(I2C_Handle->I2C,1);

	temp |= I2C_Handle->I2C_Config.ackControl << 10;
	I2C_Handle->I2C->CR1 = temp;

	temp = 0;
	temp |= RCC_Pclk1Value() /1000000U ;
	I2C_Handle->I2C->CR2 =  (temp & 0x3F);

	temp = 0;
	temp |= I2C_Handle->I2C_Config.deviceAddress << 1;
	temp |= ( 1 << 14);
	I2C_Handle->I2C->OAR1 = temp;


	uint32_t ccr;
	temp=0;
	if(I2C_Handle->I2C_Config.clkSpeed <= 1000000U){
		ccr = (RCC_Pclk1Value() / ( 2*I2C_Handle->I2C_Config.clkSpeed ));
		temp |= (ccr & 0xFFF);
	}
	else{
		if(I2C_Handle->I2C_Config.fmDutyCycle==0){
			ccr = (RCC_Pclk1Value() / ( 3*I2C_Handle->I2C_Config.clkSpeed ));
		}
		else{
			ccr = (RCC_Pclk1Value() / ( 25*I2C_Handle->I2C_Config.clkSpeed ));
		}
		temp |= (ccr & 0xFFF);
	}
	I2C_Handle->I2C->CCR = temp;

	if(I2C_Handle->I2C_Config.clkSpeed <= 1000000U){
		temp = (RCC_Pclk1Value() /1000000U) + 1 ;
	}
	else{
		temp = ((RCC_Pclk1Value() * 300) / 1000000000U ) + 1;
	}
	I2C_Handle->I2C->TRISE =(temp & 0x3F);

}

uint8_t I2C_FlagStatus(I2C_t *I2C , uint32_t flagName)
{
	if(I2C->SR1 & flagName)
	{
		return 1;
	}
	return 0;
}

void I2C_MasterSendData(I2C_Handle_t *I2C_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x)
{
	I2C_GenerateStart(I2C_Handle->I2C);

	flag=1;
	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));


	I2C_AddressWrite(I2C_Handle->I2C,slaveAddr);

	flag=1<<1;
	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));

	I2C_ClearAddrFlag(I2C_Handle);

	flag=1<<7;
	sendData(I2C_Handle->I2C,txBuffer,flag,len);

	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));

	flag=1<<2;
	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));

	if(x == 0) I2C_StopCondition(I2C_Handle->I2C);
}

void I2C_MasterRecieveData(I2C_Handle_t *I2C_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x){
	I2C_GenerateStart(I2C_Handle->I2C);

	flag=1;
	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));
	I2C_AddressRead(I2C_Handle->I2C,slaveAddr);

	flag=1<<1;
	while(!I2C_FlagStatus(I2C_Handle->I2C,flag));

	I2C_ClearAddrFlag(I2C_Handle);
	if(len==1){
		I2C_Handle->I2C->CR1&=~(1<<10);
		if(x==0) I2C_StopCondition(I2C_Handle->I2C);
	}
	flag=1<<6;
	while(len--){
		while(!I2C_FlagStatus(I2C_Handle->I2C,flag));
		if(len==2){
			I2C_Handle->I2C->CR1&=~(1<<10);
			if(x==0) I2C_StopCondition(I2C_Handle->I2C);
		}
		*txBuffer=I2C_Handle->I2C->DR;
		txBuffer++;
	}
	I2C_ManageAcking(I2C_Handle->I2C,1);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *I2C_Handle, uint8_t *txBuffer,uint32_t len,uint8_t slaveAddr,uint8_t x){
	uint8_t state=I2C_Handle->TxRxState;
	if(state!=I2C_BUSY_IN_TX && state != I2C_BUSY_IN_RX){
		I2C_Handle->TxBuffer=txBuffer;
		I2C_Handle->TxLen=len;
		I2C_Handle->TxRxState=I2C_BUSY_IN_TX;
		I2C_Handle->devAddr=slaveAddr;
		I2C_Handle->Sr=x;

		I2C_GenerateStart(I2C_Handle->I2C);
		I2C_Handle->I2C->CR2|=(1<<9);
		I2C_Handle->I2C->CR2|=(1<<10);

	}
	return state;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *I2C_Handle, uint8_t *rxBuffer, uint32_t len,uint8_t slaveAddr,uint8_t x)
{
	uint8_t state = I2C_Handle->TxRxState;

	if(state != I2C_BUSY_IN_RX && state != I2C_BUSY_IN_RX){
		I2C_Handle->RxBuffer = rxBuffer;
		I2C_Handle->RxLen = len;
		I2C_Handle->TxRxState = I2C_BUSY_IN_RX;
		I2C_Handle->devAddr=slaveAddr;
		I2C_Handle->Sr=x;

		I2C_GenerateStart(I2C_Handle->I2C);
		I2C_Handle->I2C->CR2 |= ( 1 << 9 );
		I2C_Handle->I2C->CR2 |= ( 1 << 10 );

	}
	return state;
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *I2C_Handle )
{
	//We have to do the data reception
	if(I2C_Handle->RxLen == 1)
	{
		*I2C_Handle->RxBuffer = I2C_Handle->I2C->DR;
		I2C_Handle->RxLen--;

	}


	if(I2C_Handle->RxLen > 1)
	{
		if(I2C_Handle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(I2C_Handle->I2C,DISABLE);
		}

			//read DR
			*I2C_Handle->RxBuffer = I2C_Handle->I2C->DR;
			I2C_Handle->RxBuffer++;
			I2C_Handle->RxLen--;
	}

	if(I2C_Handle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(I2C_Handle->Sr == 0)
			I2C_StopCondition(I2C_Handle->I2C);

		//2 . Close the I2C rx
		I2C_CloseRecieveData(I2C_Handle);

		//3. Notify the application
		I2C_ApplicationEventCallback(I2C_Handle,I2C_EV_RX_CMPLT);
	}
}



void I2C_IRQHandling(I2C_Handle_t *I2C_Handle){
	if((I2C_Handle->I2C->SR1 & 1)){

		if(I2C_Handle->TxRxState==I2C_BUSY_IN_TX) I2C_AddressWrite(I2C_Handle->I2C,I2C_Handle->devAddr);

		else if(I2C_Handle->TxRxState==I2C_BUSY_IN_RX) I2C_AddressRead(I2C_Handle->I2C,I2C_Handle->devAddr);
	}

	if((I2C_Handle->I2C->SR1 & 1<<1)){
		I2C_ClearAddrFlag(I2C_Handle);
	}

	if((I2C_Handle->I2C->SR1 & 1<<7)){
		if((I2C_Handle->I2C->SR2 & 1) && I2C_Handle->TxRxState==I2C_BUSY_IN_TX){
			if(I2C_Handle->TxLen > 0)
			{
				I2C_Handle->I2C->DR = *(I2C_Handle->TxBuffer);

				I2C_Handle->TxLen--;

				I2C_Handle->TxBuffer++;
			}
		}
		else{
			if(I2C_Handle->I2C->SR2 & 1<<2) I2C_ApplicationEventCallback(I2C_Handle,I2C_EV_TX_CMPLT);
		}
	}

	if((I2C_Handle->I2C->SR1 & 1<<6))
	{
		if((I2C_Handle->I2C->SR2 & 1)&& I2C_Handle->TxRxState==I2C_BUSY_IN_RX)
		{
			I2C_MasterHandleRXNEInterrupt(I2C_Handle);
		}
		else{
			if(!(I2C_Handle->I2C->SR2 & ( 1 << 2))) I2C_ApplicationEventCallback(I2C_Handle,I2C_EV_STOP);
		}
	}

	if(I2C_Handle->I2C->SR1 & ( 1 << 2)){
		//BTF flag is set
		if(I2C_Handle->TxRxState == I2C_BUSY_IN_TX){
			//make sure that TXE is also set .
			if(I2C_Handle->I2C->SR1 & ( 1 << 7)){
				//BTF, TXE = 1
				if(I2C_Handle->TxLen == 0 ){
					//1. generate the STOP condition
					if(I2C_Handle->Sr == 0)
						I2C_StopCondition(I2C_Handle->I2C);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(I2C_Handle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(I2C_Handle,I2C_EV_TX_CMPLT);

				}
			}

		}
		else if (I2C_Handle->TxRxState == I2C_BUSY_IN_RX ){
			;
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(I2C_Handle->I2C->SR1 & ( 1 << 4))
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		I2C_Handle->I2C->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(I2C_Handle,I2C_EV_STOP);
	}

}

