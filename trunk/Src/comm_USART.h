#ifndef __COMM_USART_H__
#define __COMM_USART_H__
//
#define STX	0x02
#define ETX	0x03
#define EOT	0x04
#define ENQ	0x05
#define ACK	0x06
#define _LF	0x0A
#define _CR	0x0D
#define NAK 	0x15


#define COMM_BUFF_SIZE 100
#define COMM_BUFF_SIZE_PC_SEND  700
#define ASCII_TBL_SIZE  	32
#define I_ASCII_TBL_SIZE	39
#define ASCII_BASE	0x30 

extern const unsigned char ASCII_TBL[ASCII_TBL_SIZE],  I_ASCII_TBL[I_ASCII_TBL_SIZE];


//extern	BYTE Uart2_txBuff[COMM_BUFF_SIZE];
extern	BYTE Uart1_txBuffIndex;
extern	BYTE Uart1_txBuffSize;
extern 	BYTE Uart1_txBuff[COMM_BUFF_SIZE];
extern	BYTE Uart1_rxBuffIndex;
extern 	BYTE Uart1_rxBuffSize;
extern	BYTE Uart1_rxBuff[COMM_BUFF_SIZE];

extern	BYTE Uart2_txBuffIndex;
extern	BYTE Uart2_txBuffSize;
extern 	BYTE Uart2_txBuff[COMM_BUFF_SIZE];
extern  BYTE Uart2_rxBuffIndex;
extern	BYTE Uart2_rxBuffSize;
extern	BYTE Uart2_rxBuff[COMM_BUFF_SIZE];

extern	BYTE Uart6_txBuffIndex;
extern 	BYTE Uart6_txBuffSize;
extern	BYTE Uart6_txBuff[COMM_BUFF_SIZE];
extern  BYTE Uart6_rxBuffIndex;
extern	BYTE Uart6_rxBuffSize;
extern	BYTE Uart6_rxBuff[COMM_BUFF_SIZE];

extern	BYTE fUart2Received, fUart6Received;
extern	int  iSensorBoardCommWDOGCnt;

extern BYTE DebugOutIdx;
extern BYTE CommPCMode;

extern	void Uart1_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize);
extern	void Uart2_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize);
extern	void Uart3_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize);
//extern	void Uart6_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize);
//extern	BOOL IsUart1Received(void);
extern	BOOL IsUart2Received(void);
extern	BOOL IsUart6Received(void);
extern	void Isr_Uart2(void);
//extern	void Isr_Uart6(void);
extern void MakeCheckSum ( unsigned char *string, BYTE dataS, BYTE dataE ) ;
extern BYTE DoCheckSum (unsigned char *string, BYTE dataS, BYTE dataE ) ;

#endif /*__COMM_USART__*/