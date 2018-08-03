#include "mydefs.h"
#include "stm32f4xx_hal.h"
#include "commModbusRTU.h"
#include "Crc.h"
#include "comm_USART.h"
#include "string.h"

#define MODUBS_ERR_IF	0x01
#define MODUBS_ERR_IA	0x02
#define MODUBS_ERR_ID	0x03
#define MODUBS_ERR_SB	0x06


BYTE bCntEOF;
BYTE modbusRdCnt;
BYTE ModCommDisconTimer;
BOOL fModCommFail;
ModbusCommRcvStateType commRcvState;

BYTE makeModbusReadFrame(BYTE *pRetFrame, BYTE bNum,UWORD wAddr)
{
	BYTE bSize;
	UWORD wCrc;
	
	pRetFrame[0] = MOD_PROT_STATION_ID;
	pRetFrame[1] = MODBUS_FUNC_READ;
	pRetFrame[2] = HiByte(wAddr);
	pRetFrame[3] = LoByte(wAddr);
	pRetFrame[4] = 0;
	pRetFrame[5] = bNum;
	bSize = 6;
	wCrc = CRC16 (Uart1_txBuff, bSize);
	pRetFrame[6] = LoByte(wCrc);
	pRetFrame[7] = HiByte(wCrc);
	bSize = 8;
	return bSize;
}

BYTE makeModbusWriteFrame(BYTE *pRetFrame,UWORD wAddr,UWORD wData)
{
	BYTE bSize;
	UWORD wCrc;
	
	pRetFrame[0] = MOD_PROT_STATION_ID;
	pRetFrame[1] = MODBUS_FUNC_WRITE_M;
	pRetFrame[2] = HiByte(wAddr);
	pRetFrame[3] = LoByte(wAddr);
	pRetFrame[4] = 0;
	pRetFrame[5] = 1;
	pRetFrame[6] = 2;
	pRetFrame[7] = HiByte(wData);
	pRetFrame[8] = LoByte(wData);
	
	bSize = 9;
	
	wCrc = CRC16 (pRetFrame, bSize);
	Uart1_txBuff[bSize++] = LoByte(wCrc);
	Uart1_txBuff[bSize++] = HiByte(wCrc);

	return bSize;
}
BYTE makeModbusWriteOneFrame(BYTE *pRetFrame,UWORD wAddr,UWORD wData)
{
	BYTE bSize;
	UWORD wCrc;
	
	pRetFrame[0] = MOD_PROT_STATION_ID;
	pRetFrame[1] = MODBUS_FUNC_WRITE;
	pRetFrame[2] = HiByte(wAddr);
	pRetFrame[3] = LoByte(wAddr);
	pRetFrame[4] = HiByte(wData);
	pRetFrame[5] = LoByte(wData);
	
	bSize = 6;
	
	wCrc = CRC16 (pRetFrame, bSize);
	Uart1_txBuff[bSize++] = LoByte(wCrc);
	Uart1_txBuff[bSize++] = HiByte(wCrc);

	return bSize;
}

ModbusFrameRcvResultType CommEOFReceived (void)
{
	ModbusFrameRcvResultType bRetValue;
	
	if (bCntEOF >= FRAME_TIMEOUT_CNT) {
		switch(commRcvState)
		{
			case COMM_IDLING:
				bRetValue = COMM_NONE;
				break;
			case COMM_RECEIVING:
				if (CRCok(Uart1_rxBuff, Uart1_rxBuffIndex) == SET) {
					bRetValue = COMM_CRC_OK;
					ModCommDisconTimer =0;
				}
				else {
					bRetValue = COMM_CRC_NOK;
				}

				commRcvState = COMM_IDLING;
				break;
			case COMM_IGNORING:
				commRcvState = COMM_IDLING;
				bRetValue = COMM_NONE;
				break;
			default:
				bRetValue = COMM_NONE;
		}
	}
	else {
		bRetValue = COMM_NONE;
	}
		
	return bRetValue;
}
