#include "stm32f4xx_hal.h"
#include "mydefs.h"
//#include "option.h"
#include "comm_USART.h"

//#include "mm_ext.h"		// For debug
//#include "ServoControl.h"	// For debug
//#include "Isokinetic.h"
//#include "LoadSensor.h"
//#include "IO_Lib.h"
//#include "DataConv.h"

BYTE Uart1_txBuffIndex;
BYTE Uart1_txBuffSize;
BYTE Uart1_txBuff[COMM_BUFF_SIZE];
BYTE Uart1_rxBuffIndex;
BYTE Uart1_rxBuffSize;
BYTE Uart1_rxBuff[COMM_BUFF_SIZE];

BYTE Uart2_txBuffIndex;
BYTE Uart2_txBuffSize;
BYTE Uart2_txBuff[COMM_BUFF_SIZE];
BYTE Uart2_rxBuffIndex;
BYTE Uart2_rxBuffSize;
BYTE Uart2_rxBuff[COMM_BUFF_SIZE]; 
BYTE Uart2_rxBuff_data[COMM_BUFF_SIZE]; 

BYTE Uart6_txBuffIndex;
BYTE Uart6_txBuffSize;
BYTE Uart6_txBuff[COMM_BUFF_SIZE];
BYTE Uart6_rxBuffIndex;
BYTE Uart6_rxBuffSize;
BYTE Uart6_rxBuff[COMM_BUFF_SIZE]; 
BYTE Uart6_rxBuff_data[COMM_BUFF_SIZE]; 

BYTE fUart2Received, fUart6Received;
int iSensorBoardCommWDOGCnt;

BYTE DebugOutIdx;
BYTE CommPCMode;

WORD wUart0State,wUart1State;

              