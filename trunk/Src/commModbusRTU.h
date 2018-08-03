#ifndef __MODBUSRTU__H__
#define __MODBUSRTU__H__

#define MODBUS_FUNC_READ	0x03
#define MODBUS_FUNC_WRITE	0x06
#define MODBUS_FUNC_WRITE_M	0x10
#define MOD_PROT_STATION_ID	0

#define FRAME_TIMEOUT_CNT	2
#define MOD_COMM_DISCONN_CHK_TIME 60
typedef enum {
	COMM_IDLING,
	COMM_RECEIVING,
	COMM_IGNORING
}ModbusCommRcvStateType;

typedef enum {
	COMM_NONE,
	COMM_CRC_OK,
	COMM_CRC_NOK
}ModbusFrameRcvResultType;

extern ModbusCommRcvStateType commRcvState;
extern BYTE bCntEOF;
extern BYTE ModCommDisconTimer;
extern BOOL fModCommFail;

BYTE makeModbusReadFrame(BYTE *pRetFrame, BYTE bNum,UWORD wAddr);
BYTE makeModbusWriteFrame(BYTE *pRetFrame,UWORD wAddr,UWORD wData);
BYTE makeModbusWriteOneFrame(BYTE *pRetFrame,UWORD wAddr,UWORD wData);
ModbusFrameRcvResultType CommEOFReceived (void);

#endif /*__MODBUSRTU__H__*/
