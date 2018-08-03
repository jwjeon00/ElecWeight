#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "mydefs.h"
#include "IO_Lib.h"
#include "comm_USART.h"
#include "comm2CON.h"
#include "ServoControl.H"
#include "string.h"
#include "Isokinetic.h"
#include "DataConv.h"
//#include "mm_ext.h"


	
WORD	
	systemStatus,		// 0
	curPos,				// 1
	curSpd,				// 2
	curTrq,				// 3
	OriginCmd,			// 4
	GravityCorEn,		// 5
	GravityCor,			// 6
	ROMStatus,			// 7
	ROMStartPos,		// 8
	ROMStopPos,			// 9
	ROMCalPos,			// 10
	awayPos,			// 11
	towardPos,			// 12
	awaySetSpd,			// 13
	towardSetSpd,		// 14
	awayPROM,			// 15
	towardPROM,			// 16
	gotoPosCmd,			// 17
	gotoPosSpd,         // 18
	targetPos,			// 19
	baseLoad,			// 20
	cushionSet,			// 21
	opLoad,				// 22
	ECCTorque,			// 23
	biSyncMode,			// 24
	jointSel,			// 25
	legSel,				// 26
	contractionMode,	// 27
	systemControl,		// 28
	SensorState,		// 29
	SetLEOrg,			// 30
	LEOrgSetState,		// 31
	BiRefSide,			// 32
	jogMode,			// 33
	jogSpeed,           // 34
	jointChangeCmd,		// 35
	isotonicLoad,		// 36
	loadEccLimit,		// 37
	paraWriteCmd,		// 38
	paraWriteNum,		// 39
	paraAddrList[MAX_PARA_WRITE_NUM],
	paraDataList[MAX_PARA_WRITE_NUM];			
	
    
ParameterTableType IsoKineticParameterList[MAX_REG] = {
	{(int*)&(SysStatus.wStatus),READ_ONLY},					// 0
	{(int*)&systemStatus,READ_ONLY},						// 1
	{(int*)&curPos,READ_ONLY},							// 2
	{(int*)&curSpd,READ_ONLY},							// 4
	{(int*)&curTrq,READ_ONLY},							// 6
	{(int*)&OriginCmd,READ_WRITE},						// 8

	{(int*)&GravityCorEn,READ_WRITE},						// 10
	{(int*)&GravityCor,READ_WRITE},						// 12
	{(int*)&ROMStatus,READ_ONLY},						// 14
	{(int*)&ROMStartPos,READ_WRITE},						// 16
	{(int*)&ROMStopPos,READ_WRITE},						// 18

	{(int*)&ROMCalPos,READ_WRITE},						// 20
	{(int*)&awayPos,READ_WRITE},							// 22
	{(int*)&towardPos,READ_WRITE},						// 24
	{(int*)&awaySetSpd,READ_WRITE},						// 26
	{(int*)&towardSetSpd,READ_WRITE},						// 28

	{(int*)&awayPROM,READ_WRITE},							// 30
	{(int*)&towardPROM,READ_WRITE},						// 32
	{(int*)&gotoPosCmd,READ_WRITE},						// 34
	{(int*)&gotoPosSpd,READ_WRITE},						// 36
	{(int*)&targetPos,READ_WRITE},						// 38

	{(int*)&baseLoad,	READ_WRITE},						// 40
	{(int*)&cushionSet,	READ_WRITE},						// 41
	{(int*)&opLoad,		READ_WRITE},						// 42
	{(int*)&ECCTorque,	READ_WRITE},						// 43
	{(int*)&biSyncMode,	READ_WRITE},						// 44
	{(int*)&jointSel,	READ_WRITE},						// 45
	{(int*)&legSel,			READ_WRITE},					// 46
	{(int*)&contractionMode,READ_WRITE},						// 47
	{(int*)&systemControl,	READ_WRITE},					// 48
	{(int*)&SensorState,	READ_ONLY},						// 49

	{(int*)&SetLEOrg,		READ_WRITE},					// 50
	{(int*)&LEOrgSetState,	READ_ONLY},					// 51
	{(int*)&BiRefSide,		READ_WRITE},					// 52
	{(int*)&jogMode,		READ_WRITE},						// 53
	{(int*)&jogSpeed,	READ_WRITE},						// 54
	{(int*)&jointChangeCmd,	READ_WRITE},					// 56
	{(int*)&isotonicLoad,	READ_WRITE},						// 57
	{(int*)&loadEccLimit,	READ_WRITE},						// 58
	{(int*)NULL,			READ_ONLY},						// 59

	{(int*)&servoState,	READ_ONLY},					// 60
	{(int*)NULL,			READ_ONLY},						// 62
	{(int*)NULL,			READ_ONLY},						// 63
	{(int*)NULL,			READ_ONLY},						// 64
	{(int*)NULL,			READ_ONLY},						// 65
	{(int*)&paraWriteCmd,	READ_WRITE},						// 66
	{(int*)&paraWriteNum,	READ_WRITE},						// 68

	{(int*)&paraAddrList[0],	READ_WRITE},						// 59
	{(int*)&paraAddrList[1],	READ_WRITE},						// 59
	{(int*)&paraAddrList[2],	READ_WRITE},						// 59
	{(int*)&paraAddrList[3],	READ_WRITE},						// 59
	{(int*)&paraAddrList[4],	READ_WRITE},						// 59
	{(int*)&paraDataList[0],	READ_WRITE},						// 59
	{(int*)&paraDataList[1],	READ_WRITE},						// 59
	{(int*)&paraDataList[2],	READ_WRITE},						// 59
	{(int*)&paraDataList[3],	READ_WRITE},						// 59
	{(int*)&paraDataList[4],	READ_WRITE}						// 59
 }; 
	
IsoKineticTrainingModeType IsoKineticTrainingMode;
IsoKineticSyncModeType IsoKineticSyncMode;
IsoKineticSensorStateType IsoKineticSensorState;

WORD LoggingPos[MAX_LOGGING_NUM];
WORD LoggingTrq[MAX_LOGGING_NUM];
WORD LoggingSpd[MAX_LOGGING_NUM];
BYTE LogIndex,LogNum;
BYTE fIsotonicMode, fIsotonicModeInit;
int  Cnt10msec;
union SystemStatus  SysStatus;

BYTE bPCCommFailCnt;
BYTE FPCCommFail;

LONG	lStartPosition;

void ParsingCmd(WORD bAddr,WORD wData)  ;
		
void InitIsoKineticVar()
{
	SysStatus.var.Status	= INIT_STATE;
	
	curPos		= 0;		// 6
	curSpd		= 0;		// 7
	curTrq		= 0;		// 8
//	RCurDir		= 0;		// 9
	systemControl	= 0;		// 10
//	InclineHeight= 11;		// 11
//	SavePowerTiming	= 12;	// 12
	GravityCorEn = 0;		// 13
	GravityCor	= 0;		// 14
//	ConfigMode	= 0x1234;		// 15
//	LRomSetDone	= 0;		// 20
//	RRomSetDone	= 0;		// 21
//	LRomStartPos= 0;		// 22
//	LRomStopPos	= 0;		// 23
/*	ROMStartPos	= 2230;		// 24
	ROMStopPos	= 9350;		// 25
	ROMStartPos[LEFT_SIDE]	= 2230;		// 24
	ROMStopPos[LEFT_SIDE]	= 9350;		// 25
	towardPos	= 2400;
	towardPos[LEFT_SIDE]	= 2400;
	awayPos		= 8500;
	awayPos[LEFT_SIDE]		= 8500;
*/
	ROMStartPos	= 400;		// 24
	ROMStopPos	= 2500;		// 25
	towardPos	= 700;
	awayPos		= 2200;
		
//	LRomCalPos	= 0;		// 26
//	RRomCalPos	= 0;		// 27
//	AwayPosLimit= 0;		// 30
//	TowardPosLinit= 0;		// 31
//	AwayDeg			= 0;	// 32
//	TowardDeg		= 0;	// 33
//	AwayLoad		= 0;	// 34
//	TowardLoad		= 0;	// 35
	awaySetSpd	= 6000;	// 36
	towardSetSpd	= 1000;	// 37
//	AwayPRom		= 0;	// 38
//	TowardPRom		= 0;	// 39
//	DirectionSel	= 0;	// 40
//	SafePos		= 0;		// 41
//	MaxLoad		= 0;		// 42
	baseLoad		= 0;	// 43
	cushionSet		= 5 ;	// 44		// 28mm, 2.8deg
	opLoad		= 15;		// 45
	targetPos	= 0;		// 46
//    PrtcNumofSpd	= 0;	// 50
//   PrtcEndofReps	= 0;	// 51
//  PrtcSpdAwy		= 0;	// 52
//    PrtcSpdTwd		= 0;	// 53
//    PrtcTorque		= 0;	// 54
//    PrtcSync		= 0;	// 55
	jogSpeed = 100;
    jointSel	= 0;	// 56
    legSel		= 0;	// 57
    contractionMode= 1;		// 59
    systemControl =0;		//
    SensorState = 0;
    targetPos = HOME_SPEED;
    BiRefSide=RIGHT_SIDE;
    
 //   OriginTorque	= 0;	// 60
 //   OriginSpdAwy	= 100;	// 61
 //   OriginSpdTwd	= 100;	// 62
//    HomeSpd		= 100;		// 63
 //   LGoalPos		= 0;	// 64
 //   RGoalPos		= 0;	// 65
 //   LCmdSpd		= 1000;		// 66
 //   RCmdSpd		= 1000;		// 67
 	isotonicLoad = 2000;

}

void UpdateTrainingParameterLeft(void)
{
	lStartPosition = START_POS_LEG_EXTENSION;
			
}

WORD wTT;
void ConsoleProcess()
{
	int 	i;
	WORD 	wAddr;
	WORD	wData, checkSum;
	BYTE	bNoOfData;
	unsigned short int iTemp; 
	
	if (IsUart2Received()) {
		
		bPCCommFailCnt = RESET ;
		FPCCommFail = RESET;
		
		wAddr = 	(Uart2_rxBuff[2]-'0')*10 + (Uart2_rxBuff[3]-'0');
		wData = 0;
		switch(Uart2_rxBuff[1])
		{
			case 'R':
			{
//				wData = *(WORD *)(ParameterList.[wAddr]);
				wData = *(WORD *)(IsoKineticParameterList[wAddr].ParameterVariable);
				/*if (wAddr == R_CUR_POS) {
					wData = wTT;
					wTT += 0x10;
				}*/
				Uart2_txBuffSize = 0;
				Uart2_txBuff[Uart2_txBuffSize++] = ACK;
				Uart2_txBuff[Uart2_txBuffSize++] = 'R';
				// 2010/07/13 JJW
//				Uart2_txBuffSize = 1;
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+1),"R%02d%04X",wAddr,wData&0xFFFF);
//BYTE Data2DecString	( char *string, UINT data, BYTE digit )

				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,wAddr,2,0);
				Uart2_txBuffSize += Data2String((BYTE *)(Uart2_txBuff+Uart2_txBuffSize),wData);
				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;
			}
			case 'W':
			{
				if (IsoKineticParameterList[wAddr].ParameterAttr == READ_WRITE) {
					wData = String2Data(Uart2_rxBuff+4);
					*(WORD *)(IsoKineticParameterList[wAddr].ParameterVariable) = wData;
				}
				
				Uart2_txBuff[0] = ACK;
				
// 2010/07/13 JJW
//				Uart2_txBuffSize = 1;
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+1),"W%02d",wAddr);
				Uart2_txBuff[1] = 'W';
				Uart2_txBuffSize = 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,wAddr,2,0);

				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				
				ParsingCmd(wAddr,wData);
				
				break;
			}
			
			case 'Q':
			{
				wAddr = 	(Uart2_rxBuff[2]-'0')*10 + (Uart2_rxBuff[3]-'0');
				bNoOfData = (Uart2_rxBuff[4]-'0');
				
				for (i=0;i<bNoOfData;i++) 
				{
					wData = String2Data(Uart2_rxBuff+5+i*4);
					*(WORD *)(IsoKineticParameterList[wAddr+i].ParameterVariable) = wData;
					ParsingCmd(wAddr,wData);
				}
				
				Uart2_txBuff[0] = ACK;
// 2010/07/13 JJW
//				Uart2_txBuffSize = 1;
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+1),"Q%02d%1d",wAddr,bNoOfData);
				Uart2_txBuff[1] = 'Q';
				Uart2_txBuffSize = 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,wAddr,2,0);
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,bNoOfData,1,0);

				for(i=0;i<bNoOfData*4;i++)
				{
					Uart2_txBuff[Uart2_txBuffSize+i]=Uart2_rxBuff[5+i];
				}
				Uart2_txBuffSize += bNoOfData*4;
				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;
			}
				
			case 'G':
				Uart2_txBuff[0] = ACK;
				Uart2_txBuffSize = 1;

				Uart2_txBuff[Uart2_txBuffSize++] = 'G';
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SysStatus.var.Status);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curPos);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curSpd);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curTrq);
				
				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;

			case 'g':
				Uart2_txBuff[0] = ACK;
				Uart2_txBuffSize = 1;
				
				if (Uart2_rxBuff[2] == '1') {
					SysStatus.var.ExerCntTick = RESET;
				}

				Uart2_txBuff[Uart2_txBuffSize++] = 'g';
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SysStatus.wStatus);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curPos);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curSpd);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,curTrq);
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SysStatus.var.Status);


				checkSum = 0;
				for (i=1;i<Uart2_txBuffSize;i++)
				{
					checkSum += Uart2_txBuff[i];
				}
				checkSum &= 0xFFFF;				
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,checkSum);

				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;

			case 'L':
				LogNum = Cnt10msec;
				Cnt10msec = 0;
				Uart2_txBuff[0] = ACK;
				Uart2_txBuffSize = 1;
				
				if (Uart2_rxBuff[2] == '1') {
					SysStatus.var.ExerCntTick = RESET;
					LogIndex = 0;
				}

				if (SysStatus.var.ExerCntTick == SET )
					iTemp = Tick100mTimerCnt + SysStatus.wStatus&0xFF00 ;
				else
					iTemp = SysStatus.wStatus&0xFFFF ;

				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+1),"L%04X%02X%02X",iTemp, LogIndex,LogNum);

				Uart2_txBuff[Uart2_txBuffSize++] = 'L';
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,iTemp&0xffff);
				Uart2_txBuffSize += ByteData2String(Uart2_txBuff+Uart2_txBuffSize,LogIndex&0xff);
				Uart2_txBuffSize += ByteData2String(Uart2_txBuff+Uart2_txBuffSize,LogNum&0xff);


				for (i=0;i<LogNum;i++)
				{
					Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,LoggingPos[i]);
					Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,LoggingSpd[i]);
					Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,LoggingTrq[i]);
				}		
				checkSum = 0;
				for (i=1;i<Uart2_txBuffSize;i++)
				{
					checkSum += Uart2_txBuff[i];
				}
				checkSum &= 0xFFFF;				
				ShortData2String(Uart2_txBuff+Uart2_txBuffSize,checkSum);
				Uart2_txBuffSize += 4;

				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;
			
			case 'S':	//add 090928, TWD/AWY 위치, 속도 정보를 PC로 전달
				Uart2_txBuff[0] = ACK;
				Uart2_txBuffSize = 1;
				
			
// 2010/07/13 JJW
				Uart2_txBuff[1] = 'S';
				Uart2_txBuffSize = 2;
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,SysStatus.wStatus);
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,towardPos);
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,awayPos);
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,awaySetSpd);
				
				checkSum = 0;
				for (i=1;i<Uart2_txBuffSize;i++)
				{
					checkSum += Uart2_txBuff[i];
				}
				checkSum &= 0xFFFF;				
				ShortData2String(Uart2_txBuff+Uart2_txBuffSize,checkSum);
				Uart2_txBuffSize += 4;

				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;
			case 'D':	//ROM 일때, 위치정보와 상태를 전송
				Uart2_txBuff[0] = ACK;
				Uart2_txBuff[1] = 'D';
				Uart2_txBuffSize = 2;
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,SysStatus.wStatus);
				Uart2_txBuffSize += ShortData2String(Uart2_txBuff+Uart2_txBuffSize,curPos);

				checkSum = 0;
				for (i=1;i<Uart2_txBuffSize;i++)
				{
					checkSum += Uart2_txBuff[i];
				}
				checkSum &= 0xFFFF;				
				ShortData2String(Uart2_txBuff+Uart2_txBuffSize,checkSum);
				Uart2_txBuffSize += 4;

				Uart2_txBuff[Uart2_txBuffSize++] = ETX;
				Uart2_TransmitStart(Uart2_txBuff,Uart2_txBuffSize);
				break;
		}
	}
}

void ParsingCmd(WORD wAddr,WORD wData)
{
	switch(wAddr)
	{
		case R_ORIGIN_CMD:
			SetServoIDLE(CONSOLE_ORIGIN);
			break;

		case SYSTEM_CONTROL:

			if ((systemControl & 0x100) == 0x100 ) {
				fIsotonicMode = SET;
			}
			else {
				fIsotonicMode = RESET;
				fIsotonicModeInit = RESET;
			}
			
			if (systemControl == 1 )
			{
					
				SetServoIDLE(CONSOLE_ROM);
						
			}
			else if (systemControl == 2 ) {

				SetServoIDLE( CONSOLE_EXERCISE);
			}
			else if (systemControl == 3) {
				ConsoleCmd = CONSOLE_END;
				CmdOpPause = RESET ;
				
			}
			else if (systemControl == 4) {
				CmdOpPause =  SET;
			}
			else if (systemControl == 5) {
				CmdOpPause = RESET ;
			}
			else if (systemControl == 6) {
				ServoSVOff();
			}
			break;

		case 	R_ROM_START_POS:
			ROMStatus |= 0x01;
			break;

		case 	R_ROM_STOP_POS:
			ROMStatus |= 0x02;
			break;


		case R_GOTO_POS_CMD :
			SetServoIDLE(CONSOLE_START_POSITION);
			break;
				

		case SET_LEG_ORG :
		
			break;

		case R_PARA_WRITE_CMD:
			ConsoleCmd = CONSOLE_PARA_WRITE;
			break;
			
		case JOINT_CHANGE_CMD :
			jointChangeCmd = SET;
			JointChangeSeqState = JOINT_CHANGE_SEQ_IDLE;
			break;
	}

}

void UpdateLoggingData(BYTE bOffset)
{
	LoggingPos[bOffset] = curPos;
	LoggingTrq[bOffset] = curTrq;
	LoggingSpd[bOffset] = curSpd;
}

void SetServoState(ServoStateType sType)
{
	if (sType == SERVO_EXERCISE ) {
		fInitExercise = SET; }
	else if (sType == SERVO_ROM ) {	
		fInitROM = SET;
	}
	
	servoState = sType;
	
}

void SetServoIDLE(ConsolCmdType sCmd)
{
	
	ConsoleCmd = sCmd;
	servoState = SERVO_IDLE;
	
}

