/**
* @file inthost.c
* @brief meters internal network services.
*
* @author Andrei Mamtsev
*
* @version 0.0.1
* @date 09.01.2013
*
*/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stddef.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/**************************************************************************/
/* Library Includes */
/**************************************************************************/
//#include "stm32f10x_map.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

/**************************************************************************/
/* Driver includes */
/**************************************************************************/
//#include "uart_dr.h"

/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "buff.h"
#include "IntHost.h"
//#include "board.h"
#include "hostcomm.h"
//#include "msg.h"
//#include "msgbuf.h"
//#include "data.h"
//#include "service_task.h"
#include "Msg_type.h"

/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
xQueueHandle intHostRXQueue = NULL;
xQueueHandle intHostTXQueue = NULL;

//struct sHostInterface intHost;

#define FROM_HOST_RX_BUFFERS				5	/**< Number of buffers */
#define BUF1_RX_SIZE						(30+sizeof(PACKETBUF_HDR))	/**< Size of buffer data area */	

uint8_t MemAreaRXPool1[BUF1_RX_SIZE*FROM_HOST_RX_BUFFERS];
MEMBUF_POOL hostRxPool;
#define HOST_PACKET_TIMEOUT 100





/**
* @fn int initIntNetwork(struct sIntNetwork *net)
*
* This function initializes the internal RS485 network
*
* @author Eli Schneider
*
* @param p pointer to network control data structure
* @param p pointer to network device data structure
*
* @date 10.01.2011
*/
int initHostInterface(struct sHostInterface *iface, void *dev)
{
	int status=0;
	
	iface->dev=dev;
 	iface->rxPacketTimeout=HOST_PACKET_TIMEOUT;
	iface->state=HOST_STATE_IDLE;
	memset(&iface->rxPack, 0, sizeof(iface->rxPack));
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	iface->rxPack.pool= &hostRxPool;
	status=initMemBufPool(iface->rxPack.pool, MemAreaRXPool1, BUF1_RX_SIZE*FROM_HOST_RX_BUFFERS, BUF1_RX_SIZE,FROM_HOST_RX_BUFFERS);
	if(!status)
	{
		// Failed to create the buffer pool.
		status=-1;
	}
	iface->rxPack.bufSize=iface->rxPack.pool->bufSize-sizeof(struct sPacketBufHdr);
	iface->rxPack.rxState=HOST_RX_ESC;
	
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	initHostTxStat(&iface->txPack.stat);
	return status;
}


