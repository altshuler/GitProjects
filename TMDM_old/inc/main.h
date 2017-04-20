/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011 
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "FreeRTOS.h"			// E.A. Line added
#include "Queue.h"				// E.A. Line added
#include "hcmdtask.h"			// E.A. Line added
#include "hcmd.h"				// E.A. Line added
#include "Msg_type.h"			// E.A. Line added
#include "Drive_task.h"			// E.A. Line added
#include "board.h"				// E.A. Line added
#include "irqhndl.h"			// E.A. Line added
#include "root_task.h"			// E.A. Line added





/* Exported types ------------------------------------------------------------*/

/*--------------- Tasks Priority -------------*/     
#define LED_TASK_PRIO		( tskIDLE_PRIORITY + 1 )
#define CBIT_TASK_PRIO  	( tskIDLE_PRIORITY + 2 )		// D.A. Line added
#define HCMD_TASK_PRIO		( tskIDLE_PRIORITY + 3 )		// E.A. Line added
#define DHCP_TASK_PRIO		( tskIDLE_PRIORITY + 6 ) 
#define ETH_TX_TASK_PRIO    ( tskIDLE_PRIORITY + 7 )		// E.A. Line added
#define ETH_RX_TASK_PRIO    ( tskIDLE_PRIORITY + 8 )		// E.A. Line added

//#define SPI_TX_TASK_PRIO     ( tskIDLE_PRIORITY + 6 )		// E.A. Line added
#define DRIVE_TX_TASK_PRIO  ( tskIDLE_PRIORITY + 11 )		// E.A. Line added
#define DRIVE_RX_TASK_PRIO  ( tskIDLE_PRIORITY + 12 )		// E.A. Line added
#define DRIVE_INT_TASK_PRIO ( tskIDLE_PRIORITY + 10 )		// E.A. Line added

#define MOTION_TASK_PRIO    ( tskIDLE_PRIORITY + 6 )		// E.A. Line added
#define READ_TX_TASK_PRIO   ( tskIDLE_PRIORITY + 9 )		// E.A. Line added
#define READ_RX_TASK_PRIO   ( tskIDLE_PRIORITY + 2 )		// E.A. Line added
#define ENC_TASK_PRIO		( tskIDLE_PRIORITY + 8 )		// E.A. Line added

#define ROOT_TASK_PRIO    	( tskIDLE_PRIORITY + 15 )		// E.A. Line added





extern xQueueHandle 		hCmdMbx;
extern xQueueHandle 		intHostTXQueue;
//extern xQueueHandle     	MotionQueue;
extern xQueueHandle			DriveIntQueue;

extern const char *ctlRxServerQueueName[N_CTL]; 
extern const char *ctlTxServerQueueName[N_CTL];

/* Exported constants --------------------------------------------------------*/




/* Exported define ------------------------------------------------------------*/

#define HCMD_QUEUE_SIZE			8
#define CTL_TX_QUEUE_SIZE 		16
#define CTL_RX_QUEUE_SIZE 		8
#define DRIVE_INT_QUEUE_SIZE 	8
#define MOTION_QUEUE_SIZE 		8
#define HOST_TX_QUEUE_SIZE 		5//8
#define READOUT_TX_QUEUE_SIZE	8
#define READOUT_RX_QUEUE_SIZE	8

#ifdef KUKU

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */  

void enc_tx_task(void *para);
void motion_task(void *para);
#endif
void DriveInterpTask(void *para);
void GPIO_Config(void);
void ToggleLed4(void * pvParameters);





#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

