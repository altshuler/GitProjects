
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/

#include <stdio.h>
#include <stdlib.h>

/**************************************************************************/
/* Driver Includes */
/**************************************************************************/

#include "board.h"
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/

#include <freertos.h>
#include <task.h>
#include <dev.h>
#include <gio.h>
#include "drive_task.h"
#include "membuf.h"
#include "packetbuf.h"
#include "root_task.h"
#include "hcmd.h"
#include "main.h"



xTaskHandle LwipDhcpTaskHndl = NULL;
xTaskHandle ToggleLed4TaskHndl = NULL;
xTaskHandle EthRxTaskHndl = NULL;
xTaskHandle EthTxTaskHndl = NULL;
xTaskHandle HcmdTaskHndl = NULL;
xTaskHandle EncTXTaskHndl = NULL;
xTaskHandle ctlRxServerTaskHndl[N_CTL] = {NULL,NULL};
xTaskHandle ctlTxServerTaskHndl[N_CTL] = {NULL,NULL};
xTaskHandle CbitTaskHndl = NULL;
xTaskHandle MotionTaskHndl = NULL;



const char *ctlRxServerTaskName[N_CTL] = {"ctlRx1_task", "ctlRx2_task"}; 
const char *ctlTxServerTaskName[N_CTL] = {"ctlTx1_task", "ctlTx2_task"};
const char *ctlRxServerQueueName[N_CTL] = {"ctlInQ[0]", "ctlInQ[1]"}; 
const char *ctlTxServerQueueName[N_CTL] = {"ctlOutQ[0]", "ctlOutQ[1]"};



char cmdBuffersMemory[(CMD_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*CMD_BUFFERS];
MEMBUF_POOL cmdBuffers;

char cmdResBuffersMemory[(CMD_RES_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*CMD_RES_BUFFERS];
MEMBUF_POOL cmdResBuffers;


extern struct sCtlServerParam  ctlServerConfig[N_CTL];
extern Uart_Params readoutDevParams;
extern Uart_Params ctlDevParams[N_CTL];

extern uint32_t AbsEncOffset;

//extern Spi_Params spiParams2;


const Uart_Params *uartParams[6]={NULL,&ctlDevParams[0],&ctlDevParams[1],NULL,NULL,&readoutDevParams};

GIO_Handle  uartInputHandle[6]={0};

GIO_Handle  uartOutputHandle[6]={0};





void root_task(void *para)
{
	unsigned int i;
    //GIO_Attrs gioAttrs = GIO_ATTRS;
	#if defined (USE_DMA)
	#if defined (USE_DMA_SPI0) || defined (USE_DMA_SPI1) || defined (USE_DMA_SPI2)
    //Spi_ChanParams      spiChanParams		= {NULL};
	#endif
	#endif





		/* Initialize shared memory buffers pools*/
	initMemBufPool(&cmdBuffers,cmdBuffersMemory,sizeof(cmdBuffersMemory),CMD_BUFFER_SIZE+sizeof(PACKETBUF_HDR),CMD_BUFFERS);
	//initMemBufPool(&cmdResBuffers,cmdResBuffersMemory,sizeof(cmdResBuffersMemory),CMD_RES_BUFFER_SIZE+sizeof(PACKETBUF_HDR),CMD_RES_BUFFERS);
	
	   /* Toggle LED4  every 250ms */
	  xTaskCreate(ToggleLed4,( signed char * ) "LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

	 /* Start Host command interpriter task  */
	 xTaskCreate( hCmdTask, ( signed char * ) "HCmd_task", configMINIMAL_STACK_SIZE*2 , NULL, HCMD_TASK_PRIO, NULL );	

	  /* Start DriveInterpTask: Sending commands and Receiving responce from Drivers   */
	   xTaskCreate( DriveInterpTask, ( signed char * ) "DriveInterpTask", configMINIMAL_STACK_SIZE*2 , NULL, DRIVE_INT_TASK_PRIO, NULL );
	
	   /* Start CBIT Task: CBIT Process   */
	   //xTaskCreate( CBITTask, ( signed char * ) "CBITTask", configMINIMAL_STACK_SIZE , NULL, CBIT_TASK_PRIO, NULL );
	
	  /* Start Motion Calc Task: Calculates Next Motion Step  */
	   //xTaskCreate( motion_task, ( signed char * ) "MotionTask", configMINIMAL_STACK_SIZE , NULL, MOTION_TASK_PRIO, NULL );


	for (i=0;i<N_CTL;i++)
	{
		xTaskCreate(ctlRxServerTask, (signed portCHAR *)ctlRxServerTaskName[i], configMINIMAL_STACK_SIZE*3, &ctlServerConfig[i], DRIVE_RX_TASK_PRIO, &ctlRxServerTaskHndl[i]);
		xTaskCreate(ctlTxServerTask, (signed portCHAR *)ctlTxServerTaskName[i], configMINIMAL_STACK_SIZE*2, &ctlServerConfig[i], DRIVE_TX_TASK_PRIO, &ctlTxServerTaskHndl[i]);
	}


	//xTaskCreate(ctlRxServerTask, (signed portCHAR *)ctlRxServerTaskName[0], configMINIMAL_STACK_SIZE*3, &ctlServerConfig[0], DRIVE_RX_TASK_PRIO, &ctlRxServerTaskHndl[0]);
	//xTaskCreate(ctlTxServerTask, (signed portCHAR *)ctlTxServerTaskName[0], configMINIMAL_STACK_SIZE*2, &ctlServerConfig[0], DRIVE_TX_TASK_PRIO, &ctlTxServerTaskHndl[0]);


	//vTaskDelete(NULL);
	while (1)
	{
		vTaskDelay(10000);
	}
}

