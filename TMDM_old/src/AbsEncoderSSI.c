/*
 * AbsEncoderSSI.c
 *
 *  Created on: Apr 1, 2014
 *      Author: David Anidjar
 */

//#include <stdio.h>
#include <string.h>
#include "board.h"
#include "stm32f2xx_spi.h"
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include "FreeRTOSConfig.h"
#include "handlers.h"
#include "irqhndl.h"



SPI_InitTypeDef initSSI;





void init_SSI()
{
	GPIO_InitTypeDef      GPIO_InitStructure1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable GPIO clocks */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);

	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure1.GPIO_Pin = SPI1_SCK_PIN;
	GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_InitStructure1.GPIO_Pin = SPI1_MISO_PIN;
	//GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure1.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure1);


	SPI_I2S_DeInit(SPI1);

	initSSI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	initSSI.SPI_DataSize = SPI_DataSize_16b;
	initSSI.SPI_CPOL = SPI_CPOL_High;
	initSSI.SPI_CPHA = SPI_CPHA_1Edge;
	initSSI.SPI_FirstBit = SPI_FirstBit_MSB;
	initSSI.SPI_NSS = SPI_NSS_Soft;
	initSSI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	initSSI.SPI_Mode = SPI_Mode_Master;
	initSSI.SPI_CRCPolynomial = 7; //??????

	SPI_Init(SPI1,&initSSI);
	//SPI1->CR1 |= SPI_CR1_RXONLY;

	installInterruptHandler(SPI1_IRQn,__sPI1_IRQHandler,NULL);
	
	NVIC_Configuration(SPI1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	
	SPI_Cmd(SPI1, ENABLE);
	//GPIO_SetBits(ENCODER_CLK_EN_GPIO_PORT,ENCODER_CLK_EN_PIN);
}




void NVIC_Configuration(uint8_t ch, uint8_t prio, uint8_t subprio)
{
  NVIC_InitTypeDef  NVIC_InitStructure;  

  NVIC_InitStructure.NVIC_IRQChannel = ch;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prio;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = subprio;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

