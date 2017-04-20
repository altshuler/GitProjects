/*******************************************************************************
 * @file Drive_task.c
 * @ Drive Control servers   tasks 
 *
 * @author Evgeni Altshuler
 *
 * @version 0.0.1
 * @date 27.07.2014
 *
*******************************************************************************/


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

//#include <timebase.h>
#include <dev.h>
#include <gio.h>
#include <uart.h>
#include <dma.h>
#include <assert.h>

#include "crc16.h"
//#include "initDrive.h"
#include "drive_task.h"
#include "drive_comm.h"
#include "packetbuf.h"
#include "msg_type.h"
#include "interface.h"
#include "itoa.h"
#include "AbsEncoderSSI.h"
#include "handlers.h"
#include "hostcomm.h"

#define CTL_RX_BUFFER_SIZE	(12+sizeof(PACKETBUF_HDR))
#define N_CTL_RX_BUFFERS	4
#define CTL_RX_LOCAL_BUFFER_SIZE	24
#define CTL_RX_LOCAL_BUFFERS	4

#define CTL_RX_TIMEOUT	100

#define DRIVE_TASK_TX_DELAY ( portTickType ) 100


#define COMM_TYPE_GET	(1<<4)
#define COMM_TYPE_SET	(0<<4)

#define DATA_TYPE_FLOAT	(1<<5)
#define DATA_TYPE_INT	(0<<5)

#define FWD_COLOR (0)
#define CMD_COLOR	(1)
#define SCAN_COLOR (2)

#define FWD_FRAME_COLOR (FWD_COLOR<<6)
#define CMD_FRAME_COLOR	(CMD_COLOR<<6)
#define SCAN_FRAME_COLOR (SCAN_COLOR<<6)

#define FRAME_COLOR (3<<6)


//UART-2
#define CTL1_UART_TX_DMAC_ID 0
#define CTL1_UART_TX_STREAM_ID 6
#define CTL1_UART_TX_CHANNEL_ID 4

#define CTL1_UART_RX_DMAC_ID 0
#define CTL1_UART_RX_STREAM_ID 5
#define CTL1_UART_RX_CHANNEL_ID 4

//UART-3
#define CTL2_UART_TX_DMAC_ID 0
#define CTL2_UART_TX_STREAM_ID 3
#define CTL2_UART_TX_CHANNEL_ID 4

#define CTL2_UART_RX_DMAC_ID 0
#define CTL2_UART_RX_STREAM_ID 1
#define CTL2_UART_RX_CHANNEL_ID 4


const struct sCtlServerParam  ctlServerConfig[N_CTL]= 
{
	/* Controller #1 */
	{
		0,					/* TODO: ctlId */
		1,					/* devId */
		Uart_BaudRate_460_8K,	/* baud */
		Uart_NumStopBits_1,	/* stopBits */
		Uart_CharLen_8,		/* charLen */
		Uart_Parity_NONE	/* parity */
	},

	/* Controller #2 */
	{
		1,					/* TODO: ctlId */
		2,					/* devId */
		Uart_BaudRate_460_8K, /* baud */
		Uart_NumStopBits_1, /* stopBits */
		Uart_CharLen_8, 	/* charLen */
		Uart_Parity_NONE	/* parity */
	}
};

//extern struct sGpsInterface intGps[N_CTL];

/* UART handle for input channel */
GIO_Handle hCtlUart_IN[N_CTL]={NULL,NULL};

/* UART handle for output channel */
GIO_Handle hCtlUart_OUT[N_CTL]={NULL,NULL};

xQueueHandle ctlOutQ[N_CTL]={NULL,NULL};
xQueueHandle ctlInQ[N_CTL]={NULL,NULL};

extern Uart_Params *uartParams[];
extern GIO_Handle  uartInputHandle[];
extern GIO_Handle  uartOutputHandle[];


const struct sDmaResSet ctlUartDma[N_CTL]=
{
	{
		{
			{
				CTL1_UART_TX_DMAC_ID,
				CTL1_UART_TX_STREAM_ID,
				CTL1_UART_TX_CHANNEL_ID
			},
			{
				CTL1_UART_RX_DMAC_ID,
				CTL1_UART_RX_STREAM_ID,
				CTL1_UART_RX_CHANNEL_ID
			}
		}
	},
	{
		{
			{
				CTL2_UART_TX_DMAC_ID,
				CTL2_UART_TX_STREAM_ID,
				CTL2_UART_TX_CHANNEL_ID
			},
			{
				CTL2_UART_RX_DMAC_ID,
				CTL2_UART_RX_STREAM_ID,
				CTL2_UART_RX_CHANNEL_ID
			}
		}
	}
};

__IO uint16_t T3_CCR1_Val = 300;/*300,30000*/
__IO uint16_t T4_CCR1_Val = 30000;/*30000*/
__IO uint16_t CCR2_Val = 1500;
__IO uint16_t CCR3_Val = 800;
__IO uint16_t CCR4_Val = 150;
__IO uint16_t Brake_PWM_Val = 10000;

uint16_t Timer_4_Period = 0;
uint16_t Channel3Pulse = 0;
uint16_t Channel4Pulse = 0;

xSemaphoreHandle Timer_3_Sem ;
xSemaphoreHandle Timer_4_Sem ;

xQueueHandle 					DriveIntQueue;

float CurrentOffset=0;
extern xQueueHandle             hCmdMbx;

__IO uint32_t Timer1Clk=300000;

struct sDriverStatus DriveStatus=DRIVER_INIT_STATUS;
uSSI AbsEncoderData;	


uint8_t ForwardFlag_1=0;
uint8_t ForwardFlag_2=0;


uint32_t interpScanSkipCount=0;

struct sCtlInterface intCtl[N_CTL];

extern MEMBUF_POOL cmdBuffers;

struct sFwdStat 
{
	uint32_t fwd_tx;
	uint32_t fwd_rx;
	uint32_t scan_tx;
	uint32_t scan_rx;
	uint32_t cmd_tx;
	uint32_t cmd_rx;
	uint32_t other_tx;
	uint32_t other_rx;
	
	uint32_t err_len_rx;
	uint32_t err_crc_rx;
	uint32_t scan_skip;
} ;
struct sFwdStat drv_1,drv_2;

uint16_t MccDataIn[14];
uint16_t MccDataOut[14]={0xAC53,0xA5A5,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

static int ctlRxCallback(void *arg, int status, void *addr, size_t size);
void handleCtlIncomingData(char *Buffer, size_t len, struct sFromCtl_packetizer *CtlRxPack, uint16_t fwd, uint16_t chan);
void initCtlDevParams(void);
void initCtlxDevParams(Uart_Params *devp, struct sCtlServerParam *param);
unsigned short calc_checksum (unsigned short  *buffer, size_t len);

//static int ctl1_tx_buffcall(void *buf, void *arg);
//static int ctl2_tx_buffcall(void *buf, void *arg);


Uart_Params ctlDevParams[N_CTL];

void initCtlxDevParams(Uart_Params *devp, struct sCtlServerParam *param)
{
	memcpy(devp, &UART_PARAMS, sizeof(Uart_Params));
	devp->baudRate=param->baud;
	devp->stopBits=param->stopBits;
	devp->charLen=param->charLen;
	devp->parity=param->parity;
}

void initCtl1DevParams(void)
{
	initCtlxDevParams(&ctlDevParams[0], &ctlServerConfig[0]);
	stUsartInit();
}

void initCtl2DevParams(void)
{
	initCtlxDevParams(&ctlDevParams[1], &ctlServerConfig[1]);
	stUsartInit();
}


void ctlRxServerTask(void *para)
{
	GIO_Attrs gioAttrs = GIO_ATTRS;
	GIO_AppCallback gioAppCallback;
	MEMBUF_POOL rxBufPool;
	Usart_ChanParams chanParams;
	QUE_Obj localFreeList;
	MSG_HDR ctl_in_msg;
	int ctlRxTskStatus= 0;
	size_t idx	 =	0;
	size_t len	 =	0;
	int status	 =	0;
	uint16_t chan;
	uint16_t fwd;
	uint32_t key;
	void *p;
	
	char devName[16];
	char devIdName[5];
	
	struct sCtlServerParam *ctlCommParam=NULL;
	
	

	ctlCommParam=(struct sCtlServerParam *)para;

	assert (NULL != ctlCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(ctlCommParam->devId,devIdName,10));
	QUE_new(&localFreeList);

	/*
	** Create rx buffers pool
	*/
	p=pvPortMalloc((CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS);
	initMemBufPool(&rxBufPool,p,(CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS, CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CTL_RX_BUFFERS);

	/*
	** Initialize channel packetizer
	*/
	intCtl[ctlCommParam->ctlId].rxPack.pool= &rxBufPool;
	intCtl[ctlCommParam->ctlId].rxPack.packetType=MT_PACKET_FROM_CTL+ctlCommParam->ctlId;
	//BUF_stat(&HostRxBuffers, &stat);
	intCtl[ctlCommParam->ctlId].rxPack.bufSize=rxBufPool.bufSize-sizeof(PACKETBUF_HDR);
	intCtl[ctlCommParam->ctlId].rxPack.prevRxState=intCtl[ctlCommParam->ctlId].rxPack.rxState=CTL_RX_STATE_SYNC_1;
	intCtl[ctlCommParam->ctlId].rxPack.rxIdx=0;
	intCtl[ctlCommParam->ctlId].rxPack.rxPayloadLength=intCtl[ctlCommParam->ctlId].rxPack.rxPayloadReceived=0;
	memset(&intCtl[ctlCommParam->ctlId].rxPack.stat, 0, sizeof(intCtl[ctlCommParam->ctlId].rxPack.stat));
	intCtl[ctlCommParam->ctlId].rxPack.buf=getPacketBuffer(intCtl[ctlCommParam->ctlId].rxPack.pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT,intCtl[ctlCommParam->ctlId].rxPack.packetType,UNDEFINED_FORMAT,0);
	intCtl[ctlCommParam->ctlId].rxPack.timerId=0;
	
	/*
	if (intHost.rxPack.buf)
		intHost.rxPack.buf->if_id=0;
	*/
	
	/*
	** Create a list of available local buffers
	*/
	p=pvPortMalloc((CTL_RX_LOCAL_BUFFER_SIZE)*CTL_RX_LOCAL_BUFFERS);
	assert(p!=NULL);
	if (p!=NULL)
	{
		for (idx=0;idx<CTL_RX_LOCAL_BUFFERS;idx++)
		{
			QUE_enqueue(&localFreeList,&(((uint8_t *)p)[0]));
			p= &(((uint8_t *)p)[CTL_RX_LOCAL_BUFFER_SIZE]);
		}
	}
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = CTL_RX_LOCAL_BUFFERS+2;

	#if defined(USE_DMA) && defined(USE_RX_DMA_USART234)
	chanParams.hDma = &ctlUartDma[ctlCommParam->ctlId].set[RX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* Initialize UART
	 */
	hCtlUart_IN[ctlCommParam->ctlId] = GIO_create(devName,IODEV_INPUT,&ctlRxTskStatus,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartInputHandle[ctlServerConfig[ctlCommParam->ctlId].devId]=hCtlUart_IN[ctlCommParam->ctlId];
	__restoreInterrupts(key);
	
	if (hCtlUart_IN[ctlCommParam->ctlId])
	{
		// Load the IODDEV_READ commands with buffers into the driver
		while (!QUE_empty(&localFreeList))
		{
			len=CTL_RX_LOCAL_BUFFER_SIZE;
			p=QUE_dequeue(&localFreeList);
			gioAppCallback.fxn=ctlRxCallback;
			gioAppCallback.arg= ctlInQ[ctlCommParam->ctlId];
			status = GIO_submit(hCtlUart_IN[ctlCommParam->ctlId], IODEV_READ, p,&len, &gioAppCallback); // Non blocking Read Gio 
			if (status==IODEV_ENOPACKETS)
				break;
			else if (status==IODEV_COMPLETED)
			{
			}
			else if (status==IODEV_PENDING)
			{
			}
			else
			{
			}
		}
	
	
		for (;;)
		{
			if (xQueueReceive(ctlInQ[ctlCommParam->ctlId], &ctl_in_msg, CTL_RX_TIMEOUT))
			{
			
				if (ctl_in_msg.hdr.bit.type==MSG_TYPE_DATA_BLK)
				{
					if (ctl_in_msg.buf)
					{
					
						chan=intCtl[ctlCommParam->ctlId].rxPack.packetType-MT_PACKET_FROM_CTL+DRIVER_1_ID;
						if (chan==DRIVER_1_ID)
						{
							
							key=__disableInterrupts();
							fwd=ForwardFlag_1;
							__restoreInterrupts(key);
						}
						else if (chan==DRIVER_2_ID)
						{
							key=__disableInterrupts();
							fwd=ForwardFlag_2;
							__restoreInterrupts(key);
						}
						else
							fwd=0;
						// Handle Controller comm reception
						handleCtlIncomingData((char *)ctl_in_msg.buf,ctl_in_msg.data,&intCtl[ctlCommParam->ctlId].rxPack, fwd, chan); // Handle reception on frame level
						// Issue a new driver read command
						len=CTL_RX_LOCAL_BUFFER_SIZE;
						gioAppCallback.fxn=ctlRxCallback;
						gioAppCallback.arg= ctlInQ[ctlCommParam->ctlId];
						status = GIO_submit(hCtlUart_IN[ctlCommParam->ctlId], IODEV_READ, ctl_in_msg.buf,&len, &gioAppCallback); //return buf to the driver
						if (status==IODEV_ENOPACKETS)
						{
						}
						else if (status==IODEV_COMPLETED)
						{
						}
						else if (status==IODEV_PENDING)
						{
						}
						else
						{
						}
					}
				}
			
			}
			else
			{
				// Host reception timeout
				// Reset Host packetizer if not in packet synchronization state
				handleRxTimeoutFromCtl(&intCtl[ctlCommParam->ctlId].rxPack);
			}

		}
	}
	else 
	{
		for (;;)
			vTaskDelay(10);
	}
}




void ctlTxServerTask(void *para)
{
	MSG_HDR ctl_out_msg;
	PACKETBUF_HDR *p=NULL;
	PACKETBUF_HDR *next=NULL;
	//PACKETBUF_HDR hdr;
	uint32_t key;
	
	GIO_Attrs gioAttrs = GIO_ATTRS;
	Usart_ChanParams chanParams;
	int status	 =	0;
	//uint16_t fcs;
	char devName[16];
	char devIdName[5];
	
	struct sCtlServerParam *ctlCommParam=NULL;
	
	ctlCommParam=(struct sCtlServerParam *)para;

	assert (NULL != ctlCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(ctlCommParam->devId,devIdName,10));

	initCtlTxStat(&intCtl[ctlCommParam->ctlId].txPack.stat);
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = 2;
	

	#if defined(USE_DMA) && defined(USE_TX_DMA_USART234)
	chanParams.hDma = &ctlUartDma[ctlCommParam->ctlId].set[TX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* 
	 * Initialize UART
	 */
	hCtlUart_OUT[ctlCommParam->ctlId] = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartOutputHandle[ctlServerConfig[ctlCommParam->ctlId].devId]=hCtlUart_OUT[ctlCommParam->ctlId];
	__restoreInterrupts(key);
	
	for (;;)
	{
		if (xQueueReceive(ctlOutQ[ctlCommParam->ctlId],&ctl_out_msg,portMAX_DELAY)==pdPASS)
		{
	
			if (ctl_out_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{
				if (ctl_out_msg.buf)
				{
					p=(PACKETBUF_HDR *)ctl_out_msg.buf;
					if (p)
					{
						//if (p->h.cbFunc)
						//	(*p->h.cbFunc)(&ctl_out_msg, p->h.cbArg);
						while (p)
						{
							next=p->h.link; // Detach from list
							p->h.link=NULL;
							if (p->dlen)
							{
								
								if (hCtlUart_OUT[ctlCommParam->ctlId])
								{
									// For debugging only
									//localPbuf=(volatile Uint8 *)PACKET_DATA(p);
									//localBufLen=p->Length;
									//
									status = GIO_write(hCtlUart_OUT[ctlCommParam->ctlId], PACKETBUF_DATA(p), &p->dlen);
									if (status==IODEV_COMPLETED)
									{
										updateCtlTxStat(&intCtl[ctlCommParam->ctlId].txPack.stat, p);
									}
									else if (status==IODEV_EBADIO)
									{
										GIO_delete(hCtlUart_OUT[ctlCommParam->ctlId]);

										// For debugging only 
										//vTaskDelay(50);
										//
										
										hCtlUart_OUT[ctlCommParam->ctlId] = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
										key=__disableInterrupts();
										//uartOutputHandle[gpsServerConfig.devId]=hCtlUart_OUT[ctlCommParam->ctlId];
										__restoreInterrupts(key);
									}
									
								}
							}
							retMemBuf(p);
							p=next;
						}
					}
					


					
				}
			}
		
		}
		else if (ctl_out_msg.hdr.bit.type==MSG_TYPE_CMD)
		{
		}
	}
}

void handleCtlIncomingData(char *Buffer, size_t len, struct sFromCtl_packetizer *CtlRxPack, uint16_t fwd, uint16_t chan)
{
	PACKETBUF_HDR *p=NULL;	// packet buffer
    size_t idx   =  0;
	

	for (idx=0;idx<len; idx++)
	{
		p=handleRxFromCtl(Buffer[idx],0,CtlRxPack);
		if (p)
		{	
			if (p->dlen==0)
				retMemBuf(p); // can't forward the buffer due to full dswitch mailbox, return it
			else
			{
				if (sendPacketToDriveInt(p,MSGHDR_CTLRX_PACKET(CtlRxPack->packetType-MT_PACKET_FROM_CTL),portMAX_DELAY,chan)==pdFAIL)
				{
					retMemBuf(p); // can't forward the buffer due to full dswitch mailbox, return it
				}
			}
		}
	}

}



/**************************************************************************/
/* Creating Server test as task for RTOS */
/**************************************************************************/



void DriveInterpTask(void *para)
{
	MSG_HDR drive_in_msg;
	PACKETBUF_HDR *pktBuf=NULL;
	PACKETBUF_HDR *TmpBuf=NULL;
	struct sDriverCmd DriverCmd1;
	struct sDriverCmd DriverCmd2;
	char command[12];
	uint16_t tmp;
	int32_t Current;
	uint32_t key;
	uint32_t interpScanSkipAcc=0;
	uint8_t color;
	uint16_t chksum;
	uMCC_IN DataIn;
	uMCC_IN DataOut;
	
	#ifdef TASK_STACK_CHECK
	unsigned portBASE_TYPE uxHighWaterMark;
	
	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	#endif

	interpScanSkipCount=0;

	#ifdef KUKU
	/* Initialize  interface queue*/
	DriveIntQueue = xQueueCreate( DRIVE_INT_QUEUE_SIZE, sizeof(MSG_HDR));
	if( DriveIntQueue == NULL )
	{
		// Failed to create the queue.
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif

	#ifdef KUKU
	/* synchronize with CTL transmission/reception servers */
	while (1)
	{
		for (tmp=0;tmp<N_CTL;tmp++)
		{
			if ((hCtlUart_OUT[tmp]==NULL) || (hCtlUart_IN[tmp]==NULL))
				break;
		}
		if (tmp<N_CTL)
			vTaskDelay(2);
		else
			break;
	}
	#endif
	
	key=__disableInterrupts();
	/* TIM3 configuration */
	TIM_Config();
	//TIM_ITConfig(TIM3, TIM_IT_CC1 /*| TIM_IT_CC2*/ , ENABLE); //Enable Driver 1 Status and timeout Timers
	__restoreInterrupts(key);

	memset(&drv_1,0,sizeof(drv_1));
	memset(&drv_2,0,sizeof(drv_2));
	while (1)
	{	
		if (xQueueReceive( DriveIntQueue, &drive_in_msg,( portTickType ) portMAX_DELAY ))
		{	
			pktBuf=(PACKETBUF_HDR *)drive_in_msg.buf;
			
			if (drive_in_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{
				
				if (pktBuf)
				{
					chksum=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,pktBuf->dlen-2));
					if(chksum==calcHostFrameCrc((&(PACKETBUF_DATA(pktBuf))[0]),pktBuf->dlen-4))
					{
						if (drive_in_msg.hdr.bit.source==MSG_SRC_HCMD) //Message is coming from network host
						{
							*(PACKETBUF_OFFSET_DATA(pktBuf,4))= (*(PACKETBUF_OFFSET_DATA(pktBuf,4)) & ~FRAME_COLOR) | FWD_FRAME_COLOR; // Force forward color
							if((drive_in_msg.data)==DRIVER_1_ID)
							{
							
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_1_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipCount++;
									drv_1.fwd_tx++;
								}
							}
							if((drive_in_msg.data)==DRIVER_2_ID)
							{	
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_2_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipCount++;
									drv_2.fwd_tx++;
								}
							}
							else if(drive_in_msg.data==DRIVER_1_2_ID)
							{
								memcpy(&command[1], PACKETBUF_DATA(pktBuf), pktBuf->dlen);
								command[0]=(char)pktBuf->dlen;
								TmpBuf=makeSinglePacketResponse(&cmdBuffers, (PAYLOAD_HEADER*)command, RESP_BUFFER_GET_TIMEOUT);
										
								interpScanSkipAcc=0;							
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_1_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipAcc++;
									drv_1.fwd_tx++;
								}
							
								if 	(TmpBuf)
								{
									if (pdFAIL==sendPacketToDrive(TmpBuf, portMAX_DELAY, DRIVER_2_ID))
										retMemBuf(TmpBuf);
									else
									{
										interpScanSkipAcc++;
										drv_2.fwd_tx++;
									}
								}
								interpScanSkipCount+=(interpScanSkipAcc) ? 1 : 0;
							}
						}
						else if (drive_in_msg.hdr.bit.source==MSG_SRC_CTL1RX) //Message is coming from controller 1
						{
							GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
							/*TODO: incoming packet need to be parsed*/
							color= *PACKETBUF_OFFSET_DATA(pktBuf,4) & FRAME_COLOR;
						
							tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,2)); //Calculate Current Offset*/
							DataOut=(uMCC_IN)MccDataOut[11];
							
							if(tmp==DRV_CMD_CURRENT)//A.M.
							{
								tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5));
								if(tmp&0x8000)
									DataOut.bit.AzMotor=1;   	//Motor On
								else
									DataOut.bit.AzMotor=0;		//Motor Off
								MccDataOut[5]=(tmp&0x7FFF)<<4;
							}
							else if (tmp==DRV_CMD_ERROR)
							{
								DataOut.bit.AzMotor=0;			//Motor Off
								MccDataOut[5]=0;
							}

							MccDataOut[11]=(uint16_t)DataOut.all; 

							if(color==FWD_FRAME_COLOR)
							{
								drv_1.fwd_rx++;
								drive_in_msg.hdr.bit.source=MSG_SRC_INTERP;
								drive_in_msg.hdr.bit.type=MSG_TYPE_PACKET;
								drive_in_msg.data=DRIVER_1_ID;
								if (pdFAIL==xQueueSend(hCmdMbx,&drive_in_msg,portMAX_DELAY))
									retMemBuf(pktBuf);
							}
							else if(color==SCAN_FRAME_COLOR)
							{
								drv_1.scan_rx++;
								//if(tmp==3201)
								//{
								//	PrepareFirstCommand(DRV_STATE_SET_TORQUE,&DriverCmd2,SCAN_FRAME_COLOR);
								//	DriveStatus.Status1 = *(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5)); //Calculate Current Offset*/
								//	Current= (*(int32_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5)))>>16;
								//	DriverCmd2.DrvData.data2= _IQ15toF(Current)*50.0-CurrentOffset;// new Rayon FW   full scale -193.548
									
								//	if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
								//		drv_2.scan_tx++;
								//	
								//}
								retMemBuf(pktBuf);
							}
							else if (color==CMD_FRAME_COLOR)
							{
								drv_1.cmd_rx++;
								retMemBuf(pktBuf);
							}
							else
							{
								drv_1.other_rx++;
								retMemBuf(pktBuf);
							}		
						}
						else if (drive_in_msg.hdr.bit.source==MSG_SRC_CTL2RX) //Message is coming from controller 2
						{
							GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
							color= *PACKETBUF_OFFSET_DATA(pktBuf,4) & FRAME_COLOR;
							tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,2)); 
							DataOut=(uMCC_IN)MccDataOut[11];
							
							//if(tmp==284)
							//	GPIO_ResetBits(LED1_GPIO_PORT, LED1_PIN);
							
							if(tmp==DRV_CMD_CURRENT)//A.M.
							{
								tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5));
								if(tmp&0x8000)
									DataOut.bit.ElMotor=1;//Motor On
								else
									DataOut.bit.ElMotor=0;//Motor Off
								MccDataOut[3]=(tmp&0x7FFF)<<4;
							}
							else if (tmp==DRV_CMD_ERROR)
							{
								DataOut.bit.ElMotor=0;//Motor Off
								MccDataOut[3]=0;
							}

							MccDataOut[11]=(uint16_t)DataOut.all; 

							
							MccDataOut[13]=calc_checksum(&MccDataOut[0],13);
							GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
							if(color==FWD_FRAME_COLOR)
							{
								drv_2.fwd_rx++;
								drive_in_msg.hdr.bit.source=MSG_SRC_INTERP;
								drive_in_msg.hdr.bit.type=MSG_TYPE_PACKET;
								drive_in_msg.data=DRIVER_2_ID;
								if (pdFAIL==xQueueSend(hCmdMbx,&drive_in_msg,portMAX_DELAY))
									retMemBuf(pktBuf);
							}
							else if(color==SCAN_FRAME_COLOR)
							{
								drv_2.scan_rx++;
								retMemBuf(pktBuf);
							}
							else if (color==CMD_FRAME_COLOR)
							{
								drv_2.cmd_rx++;
								retMemBuf(pktBuf);
							}
							else
							{
								drv_2.other_rx++;
								retMemBuf(pktBuf);
							}
						}
					}
				}
			}
			else if(drive_in_msg.hdr.bit.type==MSG_TYPE_EVENT)
			{
				if (drive_in_msg.hdr.bit.source==MSG_SRC_ISR_SPI) //Message is coming from SPI ISR
				{
					if (pktBuf)
					{
						memcpy(MccDataIn, PACKETBUF_DATA(pktBuf), sizeof(MccDataIn));

						if((MccDataIn[1]==PACKET_POWER_ON_CODE)||(MccDataIn[1]==PACKET_OPERAT_CODE))
						{
							MccDataOut[1]=MccDataIn[1];
							
							DataIn = (uMCC_IN)MccDataIn[4];
							
							if(DriveStatus.State1==0x1)
							{
								if(DataIn.bit.AzMotor)
								{
									GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
									PrepareFirstCommand(DRV_STATE_SET_CURR,&DriverCmd1,SCAN_FRAME_COLOR);
									DriverCmd1.DrvData.data1= ((int16_t)((MccDataIn[3])<<4))>>4;
									
									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;
								}
								else
								{
									PrepareFirstCommand(DRV_STATE_MOTOR_OFF,&DriverCmd1,SCAN_FRAME_COLOR);
									
									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;

									vTaskDelay(1);
									
									PrepareFirstCommand(DRV_STATE_GET_CURR,&DriverCmd1,SCAN_FRAME_COLOR);
									//DriverCmd1.DrvData.data1= 0;

									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;
								}


								DriveStatus.State1=DataIn.bit.AzMotor;
							}
							else
							{
								if(DataIn.bit.AzMotor)
								{
									PrepareFirstCommand(DRV_STATE_MOTOR_ON,&DriverCmd1,SCAN_FRAME_COLOR);

									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;

									vTaskDelay(1);
									
									PrepareFirstCommand(DRV_STATE_SET_CURR,&DriverCmd1,SCAN_FRAME_COLOR);
									DriverCmd1.DrvData.data1= ((int16_t)((MccDataIn[3])<<4))>>4;

									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;
								}
								else
								{
									PrepareFirstCommand(DRV_STATE_GET_CURR,&DriverCmd1,SCAN_FRAME_COLOR);
									//DriverCmd1.DrvData.data1= 0;
									
									if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
										drv_1.scan_tx++;

								}
									DriveStatus.State1=DataIn.bit.AzMotor;
							}



							if(DriveStatus.State2==0x1)
							{
								if(DataIn.bit.ElMotor)
								{
									GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
									PrepareFirstCommand(DRV_STATE_SET_CURR,&DriverCmd2,SCAN_FRAME_COLOR);
									DriverCmd2.DrvData.data1= ((int16_t)((MccDataIn[2])<<4))>>4;

									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;
								}
								else
								{
									PrepareFirstCommand(DRV_STATE_MOTOR_OFF,&DriverCmd2,SCAN_FRAME_COLOR);

									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;

									vTaskDelay(1); 
									
									PrepareFirstCommand(DRV_STATE_GET_CURR,&DriverCmd2,SCAN_FRAME_COLOR);
									//DriverCmd2.DrvData.data1= 0;

									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;
								}


								DriveStatus.State2=DataIn.bit.ElMotor;
							}
							else
							{
								if(DataIn.bit.ElMotor)
								{
									PrepareFirstCommand(DRV_STATE_MOTOR_ON,&DriverCmd2,SCAN_FRAME_COLOR);

									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;

									vTaskDelay(1); 
									
									PrepareFirstCommand(DRV_STATE_SET_CURR,&DriverCmd2,SCAN_FRAME_COLOR);
									DriverCmd2.DrvData.data1= ((int16_t)((MccDataIn[2])<<4))>>4;

									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;
								}
								else
								{
									PrepareFirstCommand(DRV_STATE_GET_CURR,&DriverCmd2,SCAN_FRAME_COLOR);
									//DriverCmd2.DrvData.data1= 0;
									
									if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
										drv_2.scan_tx++;

								}							
									DriveStatus.State2=DataIn.bit.ElMotor;
							}						
						
					  /*if (interpScanSkipCount==0)
						{
							PrepareFirstCommand(drive_in_msg.data, &DriverCmd1, SCAN_FRAME_COLOR);
							
							if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
								drv_1.scan_tx++;
						}
						else
						{
							interpScanSkipCount--;
							drv_1.scan_skip++;
						}
					*/
						}
						retMemBuf(pktBuf);
					}
				}
			}
			else if(drive_in_msg.hdr.bit.type==MSG_TYPE_CMD)
			{
				if ((drive_in_msg.hdr.bit.source==MSG_SRC_HCMD_1)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ISR_EMERG_1)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ENC))
				{		
					PrepareFirstCommand(drive_in_msg.data, &DriverCmd1, CMD_FRAME_COLOR);
					DriveStatus.Drive1PacketSent=1;
					xSemaphoreTake(Timer_3_Sem,1);
					if(pdPASS==SendCmdToDrive(DRIVER_1_ID, &DriverCmd1))
					{
						interpScanSkipCount++;
						drv_1.cmd_tx++;
						
						xSemaphoreTake(Timer_3_Sem,1);
					}
					DriveStatus.Drive1PacketSent=0;
				}
				else if ((drive_in_msg.hdr.bit.source==MSG_SRC_HCMD_2)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ISR_EMERG_2))
				{		
					PrepareFirstCommand(drive_in_msg.data, &DriverCmd2, CMD_FRAME_COLOR);
					if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
					{
						interpScanSkipCount++;
						drv_2.cmd_tx++;
					}
				}
			}

			#ifdef TASK_STACK_CHECK
			/* Calling the function will have used some stack space, we would therefore now expect
			uxTaskGetStackHighWaterMark() to return a value lower than when it was called on
			entering the task. */
			uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
			#endif
//#endif
		}		
	}
}



int sendPacketToDriveInt(void *packet, uint16_t hdr, uint32_t timeout, uint16_t dst_chan)
{
	MSG_HDR msg;
	
	msg.hdr.all=hdr;
	msg.data=dst_chan;
	msg.buf=packet;
	return xQueueSend(DriveIntQueue,&msg,timeout);
}


int sendPacketToDrive(void *packet, uint32_t timeout, uint16_t dst_chan)
{
	MSG_HDR msg;
	PACKETBUF_HDR *pkt;

	
	pkt = (PACKETBUF_HDR*)packet;	
	msg.data=dst_chan;
	msg.buf=packet;

	if (dst_chan == DRIVER_1_ID)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_INTERP, MSG_TYPE_PACKET); //using len in msghdr as forwarding flag
		if(ctlOutQ[0])
			return xQueueSend(ctlOutQ[0],&msg,timeout);
		else
			return pdFAIL;	
	}
	else if (dst_chan == DRIVER_2_ID)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_INTERP, MSG_TYPE_PACKET);
		if(ctlOutQ[1])
			return xQueueSend(ctlOutQ[1],&msg,timeout);
		else
			return pdFAIL;	
		
	}
	else
		return pdFAIL;

	
}



int SendCmdToDrive(uint16_t channel, struct sDriverCmd *DrvCmd)
{
	char command[12];
	//char parameters;
	PACKETBUF_HDR *DriverPckt=NULL;

	
	command[0] = BuildDrivePckt(&command[1], DrvCmd->cmd, DrvCmd->subcmd, DrvCmd->parameters, DrvCmd->DrvData);
	
	DriverPckt=makeSinglePacketResponse(&cmdBuffers, (PAYLOAD_HEADER*)command, RESP_BUFFER_GET_TIMEOUT);
	if(DriverPckt!=NULL)
	{
		if (sendPacketToDrive(DriverPckt, portMAX_DELAY, channel)==0)
		{
			retMemBuf(DriverPckt);
			return pdFAIL;
		}
	}
	else
		return pdFAIL;
		
	return pdPASS;
}





unsigned char  BuildDrivePckt(char *buf, uint16_t cmd, uint8_t subcmd, unsigned char params, union DriverData data)
{
	uint16_t Checksum;
	char *temp;
	char len;
	
	if (buf!=NULL)
	{
		if (params&COMM_TYPE_GET)
			len=7;
		else
			len=11;
	
		temp=buf;
		*temp=0x49;
		temp++;
		*temp=0x5D;
		temp++;
		*temp= (char)(cmd&0xFF);
		temp++;
		*temp= ((char)((cmd&0x3F00)>>8))|((char)((subcmd&0x3)<<6));
		temp++;
		*temp= (char)(((subcmd&0x3F)>>2)|params);

		if(len==11)
		{
			temp++;
			*temp= (char)(data.data1);
			temp++;
			*temp= (char)(data.data1>>8);
			temp++;
			*temp= (char)(data.data1>>16);
			temp++;
			*temp= (char)(data.data1>>24);
		}

		Checksum= calcHostFrameCrc(buf,len-4); 
		temp++;
		*temp= (char)(Checksum&0xFF); 
		temp++;
		*temp= (char)((Checksum&0xFF00)>>8); 

		return len;
	}
	else 
		return 0;
}



static int ctlRxCallback(void *arg, int status, void *addr, size_t size)
{
	MSG_HDR msg;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	
	msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR_RX,MSG_TYPE_DATA_BLK);
	msg.data=size;
	msg.buf=addr;
	
	if (!xQueueSendFromISR((xQueueHandle)arg,&msg,&xHigherPriorityTaskWoken))
	{
		// Failure to send message with received buffer
	}
	return 0;
}




int  DriveTimeout(TIM_TypeDef* TIMx,uint32_t timeout)
{
	//uint32_t val;
	uint32_t key;
	
	if((timeout<10)||(timeout>1000000))
		return 0;
	else
	{
		CCR2_Val=(uint16_t)((timeout/10)*3);
		key=__disableInterrupts();
		TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE);
		TIMx->CCR2 = TIMx->CNT;
		TIMx->SR = (uint16_t)~TIM_IT_CC2;
		TIMx->CCR2 = TIMx->CNT+CCR2_Val;
		TIM_ITConfig(TIMx, TIM_IT_CC2, ENABLE);
		__restoreInterrupts(key);
		return 1;
	}
}





/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t PrescalerValue = 0;



  vSemaphoreCreateBinary(Timer_3_Sem);
  if (Timer_3_Sem!=NULL)
		vQueueAddToRegistry( Timer_3_Sem, (signed char *)"Timer_3_Sem");
  xSemaphoreTake(Timer_3_Sem,portMAX_DELAY);
  
  vSemaphoreCreateBinary(Timer_4_Sem);
  if (Timer_4_Sem!=NULL)
		vQueueAddToRegistry( Timer_4_Sem, (signed char *)"Timer_4_Sem");
  //xSemaphoreTake(Timer_4_Sem,portMAX_DELAY);


  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  installInterruptHandler(TIM3_IRQn,__tIM3_IRQHandler,NULL);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);




  
  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /6 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
    ==> Toggling frequency = 73.24 Hz
    
    C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
    ==> Toggling frequency = 109.8 Hz
    
    CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
    ==> Toggling frequency = 219.7 Hz
    
    CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
    ==> Toggling frequency = 439.4 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / Timer1Clk) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T3_CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

   /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
 
  /* TIM Interrupts enable */
  //TIM_ITConfig(TIM3, /*TIM_IT_CC1 | TIM_IT_CC2 |*/ TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);




  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = Timer_4_Period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Channel 3, 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel3Pulse*/;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel4Pulse*/;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);


  /* TIM4 counter enable */
  TIM_Cmd(TIM4, ENABLE);

  /* TIM4 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
}






void PrepareFirstCommand(uint16_t data, struct sDriverCmd *DrvCmd, uint8_t frameColor)
{
		if(data==DRV_STATE_MOTOR_ON)
		{
			DrvCmd->cmd=DRV_CMD_MOTOR_EN;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=ENABLE;
		}
		else if(data==DRV_STATE_MOTOR_OFF)
		{
			DrvCmd->cmd=DRV_CMD_MOTOR_EN;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=DISABLE;
		}
		else if(data==DRV_STATE_SET_POS)
		{
			DrvCmd->cmd=DRV_CMD_REF_POS;
			DrvCmd->subcmd=1;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=DriveStatus.TargetPosCmd;
		}
		else if(data==DRV_STATE_SCAN)
		{
			DrvCmd->cmd=DRV_CMD_REF_POS;
			DrvCmd->subcmd=2;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=DriveStatus.TargetPosCmd;
		}
		else if(data==DRV_STATE_POS_CUR)
		{
			DrvCmd->cmd=DRV_CMD_CUR_POS;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=AbsEncoderData.raw32Data>>3;
		}
		else if(data==DRV_STATE_SET_TORQUE)
		{
			DrvCmd->cmd=DRV_CMD_TORQUE;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_FLOAT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data2=0;
		}
		else if(data==DRV_STATE_SET_VEL)
		{
			//GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN);
			DrvCmd->cmd=DRV_CMD_MAX_SPEED;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_FLOAT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data2=DriveStatus.VelocityCmd;
		}
		else if(data==DRV_STATE_SET_CURR)
		{
			//GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN);
			DrvCmd->cmd=DRV_CMD_CURRENT;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_SET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=0;
		}
		else if(data==DRV_STATE_GET_CURR)
		{
			DrvCmd->cmd=DRV_CMD_CURRENT;
			DrvCmd->subcmd=0;
			DrvCmd->parameters=COMM_TYPE_GET|DATA_TYPE_INT|(frameColor & FRAME_COLOR);
			DrvCmd->DrvData.data1=0;
		}
}

unsigned short calc_checksum (unsigned short  *buffer, size_t len)
{
	unsigned short crc=0;

	while (len--)
	{
		crc += *buffer;

		// Move on to the next element
		buffer++;
	}

	// Return the cumulative checksum value
	return crc;
}

