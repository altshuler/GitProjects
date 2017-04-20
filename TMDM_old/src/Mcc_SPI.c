/*
 * Mcc_SPI.c
 *
 *  Created on: Jan 4, 2015
 *      Author: Evgeny Altshuler
 */

//#include <stdio.h>
#include <string.h>
#include "board.h"
#include "stm32f2xx_spi.h"
#include "Mcc_SPI.h"
#include "sysport.h"
#include "FreeRTOSConfig.h"
#include "handlers.h"
#include "irqhndl.h"
#include "AbsEncoderSSI.h"


SPI_InitTypeDef initSPI;





void Init_MCC_SPI()
{
	GPIO_InitTypeDef      GPIO_InitStructure1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable GPIO clocks */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK, ENABLE);

	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure1.GPIO_Pin = SPI1_SCK_PIN;
	GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_InitStructure1.GPIO_Pin = SPI1_MISO_PIN;
	GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_InitStructure1.GPIO_Pin = SPI1_MOSI_PIN;
	GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

	SPI_I2S_DeInit(SPI1);

	initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	initSPI.SPI_DataSize = SPI_DataSize_16b;
	initSPI.SPI_CPOL = SPI_CPOL_Low;
	initSPI.SPI_CPHA = SPI_CPHA_2Edge;
	initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
	initSPI.SPI_NSS = SPI_NSS_Soft;
	initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	initSPI.SPI_Mode = SPI_Mode_Slave;
	initSPI.SPI_CRCPolynomial = 7; //??????

	SPI_Init(SPI1,&initSPI);
	//SPI1->CR1 |= SPI_CR1_RXONLY;

	installInterruptHandler(SPI1_IRQn,__sPI1_IRQHandler,NULL);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  
	NVIC_Configuration(SPI1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	//SPI_I2S_SendData(SPI1,0xAC53);
}



