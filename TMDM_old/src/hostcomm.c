/*******************************************************************************
 * @file Hostcomm.c
 * @ Hostcomm 
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *

**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"
#include "queue.h"
#include "task.h"
/**************************************************************************/
/* Library Includes */
/**************************************************************************/
//#include "stm32f10x_map.h"
/**************************************************************************/
/* Driver includes */
/**************************************************************************/


/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "stm32f2xx.h"
#include "timestamp.h"
#include "packetbuf.h"
#include "hostcomm.h"
#include "msg_type.h"
#include "inthost.h"
#include "hcmdtask.h"
#include "drive_task.h"
//#include "data.h"
/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/

char 		net_addr=48;
uint32_t 	SN_L=0;
uint16_t	SN_H=0;
uint8_t 	MccTimeoutFlag=0;


//static void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS);


/**
* @fn PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
*
* Receive packet statemachine
*
* @author Evgeny Altshuler
*
* @param char rxChar-Receive char
* @param TIMESTAMP rxTS-timestamp
* @param struct sFromHost_packetizer *p-pointer to rx packetizer
*
* @return PACKETBUF_HDR * - Receive packet
*
* @date 30.01.2014
*/
PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
{
	/*volatile*/ PACKETBUF_HDR *fromHostPacket=NULL;
	volatile uint16_t temp;
	
	if (p->buf==NULL)
	{
		switch (p->rxState)
		{
		 case HOST_RX_HEADER:
			// Expecting first sync byte, so still can allocate a fresh buffer
			p->buf=getPacketBuffer(p->pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, HOST_NETWORK, UNDEFINED_FORMAT, 0);
			if (p->buf==NULL)
				p->stat.no_buffers++;
			else	
				ResetHostPacketizer(p);
			
			break;
		}
	}
	if (p->buf)
	{
		if(MccTimeoutFlag)
		{
			ResetHostPacketizer(p);	
			MccTimeoutFlag=0;
		}
		// Do packet reception
		switch (p->rxState)
		{

		case HOST_RX_HEADER:

			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				temp = (((uint16_t)rxChar)<<8)|((uint16_t)(p->prevRxChar));
				if(temp==PACKET_HEADER)//Packet header
				{	
					MccTimeoutFlag=0;
					DriveTimeout(TIM3,300); // 200 us Temeout
					//temp = ((uint16_t)rxChar<<8)|((uint16_t)p->prevRxChar);
					p->chksum+=temp;				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_OPCODE;						
				}
				else
				{
					TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
					ResetHostPacketizer(p);	
				}
			}
		break;

		case HOST_RX_OPCODE:

			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;

			
			if(p->FieldCnt==p->FieldLen)
			{
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));

				switch (temp)
				{
					case PACKET_POWER_ON_CODE:	 	//Power on Opcode
					case PACKET_OPERAT_CODE:		//Operational Opcode
					case PACKET_CALIB_SET_CODE:		//Calibration Opcode
					case PACKET_CONFIG_SET_CODE:	//Configuration Opcode
					case PACKET_CONFIG_REQ_CODE:	//Configuration Request Opcode
					case PACKET_IBIT_SET_CODE:      //IBIT  Request Opcode

						p->chksum+=temp;				
						p->FieldCnt=0;
						p->prevRxState=p->rxState;
						p->rxState=HOST_RX_DATA_1;		

					break;

					default:
						TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
						ResetHostPacketizer(p);	
					break;
						
				}
				
			}


		break;
			
		 case HOST_RX_DATA_1:

				hostPutInBuffer(p, rxChar, rxTS);
				p->FieldCnt++;
				
				if(p->FieldCnt==p->FieldLen)
				{
					temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
					p->chksum+=temp;
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_DATA_2;

				}

			break;
		 case HOST_RX_DATA_2:
		 	
				hostPutInBuffer(p, rxChar, rxTS);
				p->FieldCnt++;
				
				if(p->FieldCnt==p->FieldLen)
				{
					temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
					p->chksum+=temp;
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_DATA_3;

				}
			
			break;

		 case HOST_RX_DATA_3:
		 	
		 		hostPutInBuffer(p, rxChar, rxTS);
				p->FieldCnt++;
				
				if(p->FieldCnt==p->FieldLen)
				{
					temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
					p->chksum+=temp;				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_SPARE;				
				}
			
			break;
			
			
		 case HOST_RX_SPARE:
		
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
				if(temp==0x0)// Byte No 4 always 0x0
				{	
					p->chksum+=temp;				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_CHKSUM;						
				}
				else
				{
					TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
					ResetHostPacketizer(p);	
				}
			}

			break;


		case HOST_RX_CHKSUM:
		
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{			
				p->ReceivedChksum= (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
				
				if(p->ReceivedChksum == p->chksum)
				{
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_1;				
				}
				else
				{
					TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
					ResetHostPacketizer(p);	
				}
			}

			break;	

		case HOST_RESERVED_1:
		
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{	
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
				//if(temp==0x0)// Byte Reserved-1  always 0x0
				//{	
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_2;
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p);	
				//}
			}

			break;

		case HOST_RESERVED_2:
			
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{	
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));			
				//if(temp==0x0)// Byte Reserved-2  always 0xFFFF
				//{				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_3;	
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p);	
				//}
			}
			break;

		case HOST_RESERVED_3:
		
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{	
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));
				//if(temp==0x0)// Byte Reserved-3  always 0x0
				//{				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_4;	
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p);	
				//}			
			}

			break;


		case HOST_RESERVED_4:
		
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));		
				//if(temp==0x0)// Byte Reserved-4  always 0xFFFF
				//{				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_5;	
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p);	
				//}			
			}

			break;


		case HOST_RESERVED_5:
		
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));		
				//if(temp==0x0)// Byte Reserved-5  always 0x0
				//{				
					p->FieldCnt=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RESERVED_6;	
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p);	
				//}

			}

			break;

			
			case HOST_RESERVED_6:
			
				//hostPutInBuffer(p, rxChar, rxTS);
				p->FieldCnt++;
				
				if(p->FieldCnt==p->FieldLen)
				{
					temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));			
					//if(temp==0x0)// Byte Reserved-6  always 0xFFFF
					//{				
						p->FieldCnt=0;
						p->prevRxState=p->rxState;
						p->rxState=HOST_RESERVED_7; 
					//}
					//else
					//{
					//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
					//	ResetHostPacketizer(p); 
					//}

				}
			
				break;



		case HOST_RESERVED_7:
			
			//hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt == p->FieldLen)
			{
				temp = (((uint16_t)rxChar<<8))|((uint16_t)(p->prevRxChar));		
				//if(temp==0x0)// Byte Reserved-7  always 0x0
				//{					
					TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
					fromHostPacket=p->buf;
					ResetHostPacketizer(p);	
				//}
				//else
				//{
				//	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
				//	ResetHostPacketizer(p); 
				//}
			}
			break;	

		}
	}
	else
	{
		// Throw the packet
	}
	p->prevRxChar=rxChar;
	return (PACKETBUF_HDR *)fromHostPacket;
}




void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS)
{
	p->lastPutInBuf=p->buf;
	p->lastPutInBufIdx=p->rxIdx;
	if ((p->rxIdx+1)<p->bufSize)	// leave one space in the end for terminating '\0'
	{
		//p->buf->endTimestamp=rxTS;
		(PACKETBUF_DATA(p->buf))[p->rxIdx]=rxChar;
		p->rxIdx++;
		p->buf->dlen++;
	}

}

void initHostTxStat(struct sToHost_packetizerStat *stat)
{
	memset(stat, 0, sizeof(struct sToHost_packetizerStat));
}


void ResetHostPacketizer(struct sFromHost_packetizer *p)
{
	p->FieldLen=SHORT_LEN;
	p->FieldCnt=0;
	p->chksum=0;
	p->ReceivedChksum=0;
	p->FrameSize=0;
	p->rxIdx=0;
	p->buf->dlen=0;
	p->rxPayloadLength=p->rxPayloadReceived=0;
	p->CommProtType=SYNC_FORMAT;
	p->rxState=HOST_RX_HEADER;
	p->prevRxState=p->rxState;
}

