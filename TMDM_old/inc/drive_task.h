/**
* @file drive_task.h
* @brief drive task definitions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 10.03.2014
*/
#ifndef _DRIVETASK_H
#define _DRIVETASK_H

#include <stdint.h>
#include <freertos.h>
//#include <task.h>
//#include <semphr.h>
#include <queue.h>
#include "uart.h"

#define N_CTL			2


struct sCtlServerParam 
{
	int ctlId;								/**< Controller identifier 0..(N_CTL-1)	*/			
	int devId;								/**< UART device identifier		*/								
	Uart_BaudRate		baud;				/**< Baud of Operation			*/
	Uart_NumStopBits	stopBits;			/**< Stopbits of Operation		*/
	Uart_CharLen		charLen;			/**< Character Length			*/
	Uart_Parity			parity;				/**< Parity of Operation		*/
};

extern xQueueHandle ctlOutQ[N_CTL];
extern xQueueHandle ctlInQ[N_CTL];

extern const char *ctlRxServerQueueName[N_CTL]; 
extern const char *ctlTxServerQueueName[N_CTL];






#define DRIVER_1_2_ID		0x01
#define DRIVER_1_ID			0x02
#define DRIVER_2_ID			0x03



/* Drive Command Opcodes*/
#define DRV_CMD_JOG_VEL			204
#define DRV_CMD_REF_POS			205
#define DRV_CMD_CURRENT			209
#define DRV_CMD_CUR_POS			3200
#define DRV_CMD_TORQUE			3202
#define DRV_CMD_MOTOR_EN		284
#define DRV_CMD_UNIT_MODE		289
#define DRV_CMD_STATUS			2223
#define DRV_CMD_MAX_SPEED		3205
#define DRV_CMD_ERROR			100

/* Host Commands*/
#define DRV_STATE_NO_CMD		0x00
#define DRV_STATE_FWD			0x01
#define DRV_STATE_MOTOR_ON		0x02	
#define DRV_STATE_MOTOR_OFF		0x03
#define DRV_STATE_SCAN			0x04
#define DRV_STATE_SET_POS		0x05
#define DRV_STATE_STATUS		0x06
#define DRV_STATE_POS_CUR		0x07	
#define DRV_STATE_SET_TORQUE	0x08
#define DRV_STATE_SET_VEL		0x09
#define DRV_STATE_SET_CURR		0x0A
#define DRV_STATE_GET_CURR		0x0B

#define CURRENT_OFFSET		2.0
#define MOTION_ACTIVE		1
#define MOTION_NOT_ACTIVE	0



//#define   _IQ15toF(A)      (float) (A / (float)32768.0)
#define   _IQ15toF(A)      (float) (A *(float)((double)1.0/(double)32768.0))


#define BRAKE_1_OPEN		GPIO_SetBits(BREAK_M1N_GPIO_PORT,BREAK_M1N_PIN)		// Open Brake 1
#define BRAKE_1_CLOSE		GPIO_ResetBits(BREAK_M1N_GPIO_PORT,BREAK_M1N_PIN)		// Close Brake 1
#define BRAKE_2_OPEN		GPIO_SetBits(BREAK_M2N_GPIO_PORT,BREAK_M2N_PIN)		// Open Brake 2
#define BRAKE_2_CLOSE		GPIO_ResetBits(BREAK_M2N_GPIO_PORT,BREAK_M2N_PIN)		// Close Brake 2

#define DRIVER_INIT_STATUS \
{\
		DRIVER_1_ID,		/* Master Drive  */\
		0,					/* Mode Command */\
		0,					/* Driver 1 Status */\
		0,					/* Driver 2 Status */\
		DRV_STATE_NO_CMD,	/* Driver 1 State */\
		DRV_STATE_NO_CMD,	/* Driver 2 State */\
		DRV_STATE_NO_CMD,	/* Driver 1 Next State */\
		DRV_STATE_NO_CMD,	/* Driver 2 Next State */\
		0,					/* Driver 1 Packet Sent   */\
		0,					/* Driver 2 Packet Sent  */\
		0,					/* Driver 1 Timeout Exp Counter  */\
		0,					/* Driver 2 Timeout Exp Counter */\
		0,					/* Position 1 Command */\
		0,					/* Position 2 Command  */\
		0,					/* Target Position  Command  */\
		0.0,				/* Velocity Command */\
		0,					/* Velocity Update Flag*/\
		0,					/* Motion Status Flag*/\
		0,					/* Current Velocity */\
}

union DriverData{
	uint32_t    data1;
	float 		data2;
};


struct sDriverCmd
{
uint16_t			cmd;
uint8_t				subcmd;
uint8_t				parameters;
union DriverData	DrvData;
};


struct sDriverStatus
{
	uint8_t  MasterDrive;
	uint8_t  ModeCommand;
	uint16_t Status1;
	uint16_t Status2;
	uint16_t State1;
	uint16_t State2;
	uint16_t NextState1;
	uint16_t NextState2;
	uint8_t  Drive1PacketSent;
	uint8_t  Drive2PacketSent;
	uint8_t  Drive1TimeoutCnt;
	uint8_t  Drive2TimeoutCnt;
	uint32_t Position1Cmd;
	uint32_t Position2Cmd;
	uint32_t TargetPosCmd;
	float    VelocityCmd;
	uint8_t  VelUpdFlag;
	uint8_t  MotionStatus;
	int32_t  CurrentVelocity;
};



typedef struct
{
#ifdef KUKU
	uint16_t Spare		:2;
	uint16_t FansPwm	:2;
	uint16_t FanA		:1;
	uint16_t FanB		:1;
	uint16_t FansPwr	:2;
	uint16_t POBIT		:1;
	uint16_t IOBIT		:1;
	uint16_t AzMotor	:1;
	uint16_t ElMotor	:1;
	uint16_t AzEnc		:1;
	uint16_t ElEnc		:1;
	uint16_t AzMtrType	:1;
	uint16_t ElMtrType	:1;
#else
	//uint16_t ElMtrType	:1;	
	//uint16_t AzMtrType	:1;
	//uint16_t ElEnc		:1;
	//uint16_t AzEnc		:1;
	uint16_t ElMotor	:1;
	uint16_t AzMotor	:1;
	//uint16_t IOBIT		:1;
	//uint16_t POBIT		:1;
	uint16_t FansPwr	:2;
	uint16_t FanB		:1;
	uint16_t FanA		:1;
	uint16_t FansPwm	:2;
	uint16_t Spare		:8;
#endif

}sMCC_DATA_IN;


typedef union
{
	uint16_t all;
	sMCC_DATA_IN bit;
}uMCC_IN;



#ifdef __cplusplus
	extern "C" {
#endif
void ctlRxServerTask(void *para);

void ctlTxServerTask(void *para);
unsigned char  BuildDrivePckt(char *buf, uint16_t cmd, uint8_t subcmd, unsigned char params, union DriverData data);
void TIM_Config(void);
int SendCmdToDrive(uint16_t channel, struct sDriverCmd *DrvCmd);
int  DriveTimeout(TIM_TypeDef* TIMx, uint32_t timeout);
void PrepareFirstCommand(uint16_t data, struct sDriverCmd *DrvCmd, uint8_t frameColor);
int sendPacketToDriveInt(void *packet, uint16_t hdr, uint32_t timeout, uint16_t dst_chan);
int sendPacketToDrive(void *packet, uint32_t timeout, uint16_t dst_chan);

#ifdef __cplusplus
}
#endif


#endif

