/*
 * stm32f407xx.h
 *
 *  Created on: Sep 22, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define _vo volatile

#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET 								DISABLE
#define GPIO_PIN_SET        				SET
#define GPIO_PIN_RESET      				RESET
#define FLAG_RESET          				RESET
#define FLAG_SET 							SET

#define IRQ_NO_EXTI0 						6
#define IRQ_NO_EXTI1 						7
#define IRQ_NO_EXTI2 						8
#define IRQ_NO_EXTI3 						9
#define IRQ_NO_EXTI4 						10
#define IRQ_NO_EXTI9_5 						23
#define IRQ_NO_EXTI15_10 					40

#define IRQ_NO_SPI1							35
#define IRQ_NO_SPI2         				36
#define IRQ_NO_SPI3         				51
#define IRQ_NO_SPI4

#define IRQ_NO_I2C1_EV     					31
#define IRQ_NO_I2C1_ER     					32

#define IRQ_NO_USART1	    				37
#define IRQ_NO_USART2	    				38
#define IRQ_NO_USART3	    				39
#define IRQ_NO_UART4	    				52
#define IRQ_NO_UART5	    				53
#define IRQ_NO_USART6	    				71


#define NVIC_ISER0         					((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1          				((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2   						((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3          				((_vo uint32_t*)0xE000E10c)




#define NVIC_ICER0 							((_vo uint32_t*)0XE000E180)
#define NVIC_ICER1							((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2  						((_vo uint32_t*)0XE000E188)
#define NVIC_ICER3							((_vo uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR 					((_vo uint32_t*)0xE000E400)


#define FLASH_BASEADDR						0x08000000
#define SRAM1_BASEADDR						0x20000000
#define SRAM2_BASEADDR 						0x2001C000
#define ROM_BASEADDR						0x1FFF0000

#define APB1_BASEADDR						0x40000000
#define APB2_BASEADDR						0x40010000
#define AHB1_BASEADDR						0x40020000
#define AHB2_BASEADDR						0x50000000

#define GPIOA_BASEADDR						AHB1_BASEADDR
#define GPIOB_BASEADDR						AHB1_BASEADDR + 0x00000400
#define GPIOC_BASEADDR						AHB1_BASEADDR + 0x00000800
#define GPIOD_BASEADDR						AHB1_BASEADDR + 0x00000C00
#define GPIOE_BASEADDR						AHB1_BASEADDR + 0x00001000
#define GPIOF_BASEADDR						AHB1_BASEADDR + 0x00001400
#define GPIOG_BASEADDR		   				AHB1_BASEADDR + 0x00001800
#define GPIOH_BASEADDR						AHB1_BASEADDR + 0x00001C00
#define GPIOI_BASEADDR						AHB1_BASEADDR + 0x00002000

#define I2C1_BASEADDR						APB1_BASEADDR + 0x00005400
#define I2C2_BASEADDR						APB1_BASEADDR + 0x00005800
#define I2C3_BASEADDR						APB1_BASEADDR + 0x00005C00

#define SPI2_BASEADDR						(APB1_BASEADDR + 0x00003800)
#define SPI3_BASEADDR						(APB1_BASEADDR + 0x00003C00)

#define USART2_BASEADDR						(APB1_BASEADDR + 0x00004400)
#define USART3_BASEADDR						(APB1_BASEADDR + 0x00004800)
#define UART4_BASEADDR						(APB1_BASEADDR + 0x00004C00)
#define UART5_BASEADDR						(APB1_BASEADDR + 0x00005000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2_BASEADDR + 0x00003C00)
#define SPI1_BASEADDR						(APB2_BASEADDR + 0x00003000)
#define SYSCFG_BASEADDR        				(APB2_BASEADDR + 0x00003800)
#define USART1_BASEADDR						(APB2_BASEADDR + 0x00001000)
#define USART6_BASEADDR						(APB2_BASEADDR + 0x00001400)

#define RCC_BASEADDR						0x40023800U

typedef struct{
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];
}GPIO_t;

#define GPIOA 								((GPIO_t*)0x40020000)
#define GPIOB 								((GPIO_t*)0x40020400)
#define GPIOC 								((GPIO_t*)GPIOC_BASEADDR)
#define GPIOD 								((GPIO_t*)0x40020C00)
#define GPIOE 								((GPIO_t*)GPIOE_BASEADDR)
#define GPIOF 								((GPIO_t*)GPIOF_BASEADDR)
#define GPIOG 								((GPIO_t*)GPIOG_BASEADDR)
#define GPIOH 								((GPIO_t*)GPIOH_BASEADDR)
#define GPIOI 								((GPIO_t*)GPIOI_BASEADDR)


typedef struct
{
	  _vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  _vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  _vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  _vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  _vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  _vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  _vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  _vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	  _vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  _vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  _vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  _vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  _vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	  _vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  _vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  _vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  _vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  _vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  _vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  _vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  _vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  _vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  _vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  _vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	  _vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	  _vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	  _vo uint32_t DCKCFGR2;
} RCC_t;

#define RCC									((RCC_t*)RCC_BASEADDR)

#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()						(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()						(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()						(RCC->AHB1ENR |= (1 << 8))

static inline void GPIOA_REG_RESET(){
	(RCC->AHB1RSTR |= (1 << 0));
	(RCC->AHB1RSTR &= ~(1 << 0));
}

#define GPIOB_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 1));\
											(RCC->AHB1RSTR &= ~(1 << 1))

#define GPIOC_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 2)); \
											(RCC->AHB1RSTR &= ~(1 << 2))

#define GPIOD_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 3));\
											(RCC->AHB1RSTR &= ~(1 << 3))

#define GPIOE_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 4));\
											(RCC->AHB1RSTR &= ~(1 << 4))

#define GPIOF_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 5));\
											(RCC->AHB1RSTR &= ~(1 << 5))

#define GPIOG_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 6));\
											(RCC->AHB1RSTR &= ~(1 << 6))

#define GPIOH_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 7));\
											(RCC->AHB1RSTR &= ~(1 << 7))

#define GPIOI_REG_RESET()               	(RCC->AHB1RSTR |= (1 << 8));\
											(RCC->AHB1RSTR &= ~(1 << 8))


#define GPIO_BASEADDR_TO_CODE(x)       		((x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:\
											(x == GPIOF)?5:\
											(x == GPIOG)?6:\
											(x == GPIOH)?7:\
											(x == GPIOI)?8:0)


typedef struct
{
	_vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	_vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	_vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	_vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	_vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	_vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_t;

#define EXTI								((EXTI_t*)EXTI_BASEADDR)

typedef struct
{
    _vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	_vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	_vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	_vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	_vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_t;

#define SYSCFG								((SYSCFG_t*)SYSCFG_BASEADDR)

#define SYSCFG_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))


typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	_vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	_vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	_vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	_vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	_vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	_vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_t;

#define SPI1  								((SPI_t*)SPI1_BASEADDR)
#define SPI2  								((SPI_t*)0x40003800)
#define SPI3  								((SPI_t*)SPI3_BASEADDR)


#define SPI1_PCLK_EN() 						(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 						(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 						(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() 						(RCC->APB2ENR |= (1 << 13))


typedef struct{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t OAR1;
	_vo uint32_t OAR2;
	_vo uint32_t SR1;
	_vo uint32_t SR2;
	_vo uint32_t DR;
	_vo uint32_t CCR;
	_vo uint32_t TRISE;
	_vo uint32_t FLTR;
}I2C_t;


#define I2C1 								((I2C_t*)I2C1_BASEADDR)
#define I2C2  								((I2C_t*)I2C2_BASEADDR)
#define I2C3  								((I2C_t*)I2C3_BASEADDR)

#define I2C1_PCLK_EN() 						(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 						(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 						(RCC->APB1ENR |= (1 << 23))


typedef struct{
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t BRR;
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t CR3;
	_vo uint32_t GTPR;
}USART_t;

#define USART1  							((USART_t*)USART1_BASEADDR)
#define USART2  							((USART_t*)USART2_BASEADDR)
#define USART3  							((USART_t*)USART3_BASEADDR)
#define UART4  								((USART_t*)UART4_BASEADDR)
#define UART5  								((USART_t*)UART5_BASEADDR)
#define USART6  							((USART_t*)USART6_BASEADDR)


#define USART1_PCCK_EN() 					(RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() 					(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() 					(RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  					(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  					(RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() 					(RCC->APB1ENR |= (1 << 5))


#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_rcc.h"

#endif
