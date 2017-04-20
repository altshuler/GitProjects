#ifndef _HOSTCOMM_H
#define _HOSTCOMM_H

#include <stdint.h>
#include "packetbuf.h"
#include "timestamp.h"
#include "membuf.h"

#define LOCAL_RX_BUFFER_SIZE 100



struct sFromHost_packetizerStat
{
	uint32_t no_buffers;
	uint32_t timeout;
	uint32_t host_bin;
	uint32_t err_length;
	uint32_t err_frame;
};

struct sToHost_packetizerStat
{
	uint32_t host_ccs;
	uint32_t host_fcs;
	uint32_t other;
};

struct sFromHost_packetizer
{
	uint8_t	 FieldLen;		// field length				E.A line added
	uint8_t	 FieldCnt;		//field bytes counter			E.A line added
	uint16_t chksum;		//calculated checksum		E.A line added
	uint16_t ReceivedChksum;//calculated checksum		E.A line added
	uint32_t FrameSize;		//Frame size				E.A line added
	uint32_t rxState;
	uint32_t prevRxState;
	MEMBUF_POOL *pool;
	uint32_t bufSize; // Length of buffer data area
	uint32_t rxPayloadLength; // Number of bytes to receive in payload reception states of length oriented protocols
	uint32_t rxPayloadReceived; // Number of received payload bytes
	uint32_t rxIdx;				// Index of current character
	char prevRxChar;
	uint8_t CommProtType;
	PACKETBUF_HDR *buf;
	PACKETBUF_HDR *lastPutInBuf;
	uint32_t lastPutInBufIdx;
	uint32_t addr_l;
	uint32_t addr_h;
	uint16_t addr_cnt;
	struct sFromHost_packetizerStat stat;
};

struct sToHost_packetizer
{
	struct sToHost_packetizerStat stat;
};

struct sHostInterface
{
	void *dev;					/**< Pointer to network device driver */
	int rxPacketTimeout;		/**< Packet reception timeout */
	unsigned int state;			/**< media interface state */
	struct sFromHost_packetizer rxPack;	/**< Host Rx packetizer structure */
	struct sToHost_packetizer txPack;	/**< Host Tx packetizer structure */
};

#define HOST_STATE_IDLE					0 /**< Host interface: Idle */
#define HOST_STATE_TX					1 /**< Host interface: Transmitting, driver enabled */
#define HOST_STATE_RX_WAIT_WINDOW		2 /**< Host interface: Response reception window */
#define HOST_STATE_RX					3 /**< Host interface: Receiving */
#define HOST_STATE_TX_DEFER				4 /**< Host interface: Transmission defer window */


//
/*#define HOST_FRAME_SYNC	0x5D493C8BUL
#define HOST_FRAME_SYNC_LO (HOST_FRAME_SYNC&0xff)
#define HOST_FRAME_SYNC_MID_LO ((HOST_FRAME_SYNC>>8)&0xff)
#define HOST_FRAME_SYNC_MID_HI ((HOST_FRAME_SYNC>>16)&0xff)
#define HOST_FRAME_SYNC_HI ((HOST_FRAME_SYNC>>24)&0xff)*/

#define HOST_ESC 		27
#define HOST_STX 		2
#define HOST_ETX 		3

#define HOST_ENQ 		5
#define HOST_SYNC0 		0x15 //5D493C8B
#define HOST_SYNC1 		0x8B
#define HOST_SYNC2 		0x3C
#define HOST_SYNC3 		0x49
#define HOST_SYNC4 		0x5D

#define RES_CR 			13
#define RES_EOF 		4

#define MAX_ADDRESS 		49
#define MIN_ADDRESS 		126

#define MAX_FRAME_PAYLOAD_LENGHT 		27

// HOST Frame reception states
// Reception of first sync character HOST-BIN('\x8b')
#define HOST_RX_STATE_SYNC_1	0
// Reception of second sync character HOST-BIN('\x3C')
#define HOST_RX_STATE_SYNC_2	1
// Reception of third sync character HOST-BIN('\x49')
#define HOST_RX_STATE_SYNC_3	2
// Reception of fourth sync character HOST-BIN('\x5d')
#define HOST_RX_STATE_SYNC_4	3

#define HOST_RX_ESC		0
#define HOST_RX_ADDRESS	1
#define HOST_RX_STX		2
#define HOST_RX_FRAME_PAYLOAD	3
#define HOST_RX_ETX		4
#define HOST_RX_BCC		5

#define HOST_RX_SYNC1		6
#define HOST_RX_SYNC2		7
#define HOST_RX_SYNC3		8
#define HOST_RX_SYNC4		9
#define HOST_RX_FCS_BCC   10



#define HOST_RX_HEADER			0
#define HOST_RX_OPCODE			1
#define HOST_RX_DATA_1			2
#define HOST_RX_DATA_2			3
#define HOST_RX_DATA_3			4
#define HOST_RX_SPARE			5
#define HOST_RX_CHKSUM			6
#define HOST_RESERVED_1			7
#define HOST_RESERVED_2			8
#define HOST_RESERVED_3			9
#define HOST_RESERVED_4			10
#define HOST_RESERVED_5			11
#define HOST_RESERVED_6			12
#define HOST_RESERVED_7			13

#define SHORT_LEN				2
#define LONG_LEN				4

#define PACKET_HEADER					0xAC53
#define PACKET_POWER_ON_CODE			0xA5A5
#define PACKET_OPERAT_CODE				0xA3A3

#define PACKET_CALIB_SET_CODE			0xFCFC

#define PACKET_CONFIG_REQ_CODE			0xF6F6
#define PACKET_CONFIG_SET_CODE			0xF2F2

#define PACKET_IBIT_SET_CODE			0xF1A5
#define PACKET_IBIT_RSP_CODE			0xF1F1



// HOST binary
// Reception of record identifier LOW
#define HOST_RX_FRAME_NUM_LO	4
// Reception of record identifier LOW
#define HOST_RX_FRAME_NUM_HI	5
// Reception of record identifier LOW
#define HOST_RX_FRAME_LEN_LO	6
// Reception of record identifier LOW
#define HOST_RX_FRAME_LEN_HI	7
// Reception of host binary Payload
//#define HOST_RX_FRAME_PAYLOAD	8
// Reception of host binary frame check sequence (FCS)
#define HOST_RX_FRAME_FCS_LO	9
#define HOST_RX_FRAME_FCS_HI	10

extern char net_addr;
extern unsigned long SN_L;
extern unsigned short	SN_H;

#ifdef __cplusplus
extern "C" 
{
#endif

void uart0_HostRxHandler(void);
void uart1_HostRxHandler(void);
int handleRxTimeoutFromHost(struct sFromHost_packetizer *p);
PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p);
void initHostTxStat(struct sToHost_packetizerStat *stat);
void updateHostTxStat(struct sToHost_packetizerStat *stat, PACKETBUF_HDR *p);
int isHostCmdPacket(PACKETBUF_HDR *p);
int HandleRxTimeoutFromHost(struct sFromHost_packetizer *p);
char upper_to_lower(char *pt,char len); 
void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS);
void ResetHostPacketizer(struct sFromHost_packetizer *p);


#ifdef __cplusplus
}
#endif



#endif

