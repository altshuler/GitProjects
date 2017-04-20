/*******************************************************************************
 * @file handlers.c
 * @ handlers 
 *
 * @author Evgeny Altshuler
 *
 * @version 0.0.1
 * @date 24.07.2014
 *
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"
//#include "data.h"
#include "handlers.h"
#include "hostcomm.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <string.h>




/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/

uint16_t capture_4 = 0;
uint16_t capture_3 = 0;


extern __IO uint16_t T3_CCR1_Val;
extern __IO uint16_t T4_CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;
extern __IO uint16_t Brake_PWM_Val;

uint8_t Enc_Int_Flag=0;
extern struct sHostInterface 	intHost;
extern struct sDriverStatus DriveStatus;
extern uint8_t 	MccTimeoutFlag;
uint16_t datacnt=0;
extern uint16_t MccDataOut[14];
uint16_t LocalDataOut[14];


int __eXTI15_10_IRQHandler(void * arg)
{

}


int __eXTI9_5_IRQHandler(void * arg)
{

}



int __eTH_IRQHandler(void * arg)
{

}




int __sPI1_IRQHandler (void * arg)
{
	PACKETBUF_HDR *pktBuf=NULL;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	MSG_HDR msg;
	size_t idx;
	size_t rxLen;
	uint8_t localRxBuffer[SPI_MESSAGE_LENGTH];
	uint16_t data;
	int doYield=0,x;



	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
			
			SPI_ClearITPendingBit(SPI1,SPI_I2S_IT_RXNE);


				//if (datacnt==0)
				//		memcpy(LocalDataOut, MccDataOut, sizeof(MccDataOut)); 
			
				
				datacnt++;
				if(datacnt==14)
					datacnt=0;
				/*else if(datacnt==13)
				{
					MccDataOut[13]=calc_checksum(&MccDataOut[0],13);
					SPI1->DR=MccDataOut[datacnt]; //SPI_I2S_SendData(SPI1,LocalDataOut[datacnt]);
				}*/
				else
					SPI1->DR=MccDataOut[datacnt]; //SPI_I2S_SendData(SPI1,LocalDataOut[datacnt]);
					
			
			data=(uint16_t)SPI_I2S_ReceiveData(SPI1);

			localRxBuffer[0]=(uint8_t)(data&0xFF);
			localRxBuffer[1]=(uint8_t)(data>>8);
			rxLen=2;

	
			
			
			for (idx=0;idx<rxLen;idx++)
			{
				pktBuf=handleRxFromHost(localRxBuffer[idx], 0, &intHost.rxPack);
				
				if(pktBuf)
				{
					SPI1->DR=0xAC53;
					datacnt=0;
					msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_SPI, MSG_TYPE_EVENT);
					//msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_HOSTRX, MSG_TYPE_PACKET);
					msg.data=0;
					msg.buf=pktBuf;

					
					xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
					
					if( xHigherPriorityTaskWoken )
					{
						// Actual macro used here is port specific.
						doYield=1;
					}	
				}	
			}
	}

	return doYield;
}



int __tIM4_IRQHandler(void * arg)
{

}




int __tIM3_IRQHandler(void * arg)
{
  MSG_HDR msg;
  portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
  int doYield=0;
  
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

			
/*	if(DriveStatus.Drive1PacketSent==0)
	{	
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_EVENT);
		msg.data=DRV_STATE_POS_CUR;
		xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken )
		{
			// Actual macro used here is port specific.
			doYield=1;
		}
	}
*/
	capture_3 = TIM_GetCapture1(TIM3);
	TIM_SetCompare1(TIM3, capture_3 + T3_CCR1_Val);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
	MccTimeoutFlag=1;
/*	xSemaphoreGiveFromISR(Timer_3_Sem,&xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		doYield=1;
	}
*/
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

	capture_3 = TIM_GetCapture3(TIM3);
	TIM_SetCompare3(TIM3, capture_3 + CCR3_Val);
  }
  else
  {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	
	//SPI_I2S_SendData(SPI1,0x5555);

	capture_3 = TIM_GetCapture4(TIM3);
	TIM_SetCompare4(TIM3, capture_3 + CCR4_Val);
  }

  return doYield;
}





