#include "stm32f4xx_hal.h"

#include "stdio.h"
#include "stdlib.h"
#include "mydefs.h"
#include "IO_Lib.h"
#include "ServoControl.H"
#include "comm_USART.h"
#include "comm2CON.h"
#include "string.h"
#include "LoadSensor.h"
#include "DataConv.h"
#include "Isokinetic.h"
#include "math.h"
//#include "DCMotorControl.h"
#include "commModbusRTU.h"

#define HOME_SPEED				600
#define MAX_LOAD
#define ORIGIN_TORQUE			200		// jjw test
#define ORIGIN_AWAY_SPEED		600

#define PC_PROGRAM			0
#define HYPER_TERMINAL		1

#define ADDR_CMD_POS_PULSE	3
#define ADDR_FBK_POS_PULSE	4
#define ADDR_CURR_TORQUE	22

#define ADDR_SPEED_CMD1	280
#define SPD_DIR_FWD		0
#define SPD_DIR_REV		1

#define ADDR_OP_MODE	76
#define OP_MODE_SPD		1
#define OP_MODE_POS		2

//BYTE FPCCommFail;

BYTE FlSensorCal;
//BYTE FlFilterInit,FlInitSensorCal, 
BYTE FlInitSensorCalSet;
BYTE bOpJointSel;

WORD DebugVal1,DebugVal2,DebugVal3;
int makeServoIncreement ( int iSpeed, int iGoalPos, int iCurPos, BYTE CtrlMode );
int checkCushion( int iDiffPos, int iRomCushion, int iSetMsSpd, int iMinSpeed  );
void setMotorDir (signed char bDir);
void ServoOperationControl(void);
BYTE checkPosLimit( int iPosCnt, signed char cMDir );
void jointChangeProc ( void );
int checkDCMotorTimer (WORD wTimerCnt, WORD wRefCnt );
void checkSafety( int iSpeed);
void syncOpSide(BYTE bMainSide  );
void makeSpeedEachSide(void);
void initRomModeVariable(void);
void initExerciseModeVariable(void);
void opVarInit(void);
void isokineticControl(BYTE fMode);
void initExerciseModeVariable(void);
void changeCoordinate(BYTE bCrossMode);
void isotonicControl(void);
void checkExerciseCount(void);

const BYTE strReadDataIOState[] = "00RSS0107%MD0808";
const BYTE strReadData[]  		= "00RSS0107%MD";
const BYTE strWriteData[]  		= "00WSS0107%MD";
const BYTE strPointConData[]  	= "00WSS0106%MX";
const BYTE strNormalSpeedCmd[]	= "00CJR%1d%05d%04d%04d%04d";
const BYTE strOrgLimitSpeedCmd0[]= "00CJR000200010001009999";
const BYTE strOrgLimitSpeedCmd1[]= "00CJR100200010001009999";
const BYTE strOrgLimitSpeedCmd[2][24]={ "00CJR000200010001009999","00CJR100200010001009999" };
const BYTE strOrgSpeedCmd[]  	= "00CJR100100010001000001";
const BYTE strMonPosLoadCmd[]  	= "00RCS01100000";

#define SERVO_RESP_TIMEOUT		50
#define	CNT_5MS		(200.)		//  count/s, 1초간 5ms 수량

//#define ORG_DETECT_LOAD_RATE	7

int	AbsPosLmtTWD, AbsPosLmtAWAY;
int iAccSpd;
int iCurJointState;
int iJointChgSeq;
BYTE FDCMotorRun;
BYTE FECCLoadLimit;


//extern LONG	lStartPosition;

ConsolCmdType ConsoleCmd; //,ConsoleCmd,ConsoleCmd;
ServoStateType servoState; //,servoState,servoState;
ServoOriginStateType 	servoOrgState; //, servoOrgState, servoOrgState;
ServoStartPosStateType	servoStartPosState;//, servoStartPosState,servoStartPosState;
ServoROMStateType		servoROMState; //,servoROMState,servoROMState;
ServoExerciseStateType	servoExerciseState; //,servoExerciseState,servoExerciseState;
ServoParaWriteType		servoParaWriteState;

WORD wParaWriteAddr;
WORD wParaWriteData;
BYTE bTotalParaWriteNum,bParaWriteIdx;

LONG	lTotalPositionCount,lTotalPositionCountOut;
LONG	lServoPulseAddCnt, lServoPulseAddCntPrev;
//BYTE	ORG_SIDE[2] = {  LEFT_SIDE }, OP_SIDE[2] = {  RIGHT_SIDE };

WORD	Uart1TimeoutCnt;
int		imaxSpeed = 5000;


BYTE	bRetryCnt;

BOOL fJointChangeFin;
BOOL fInitExercise,fInitROM;
BOOL FIsJointDefined;
//BYTE fIsotonicMode, fIsotonicModeInit;

int iServoPos,iServoCmdPos,iServoLoad,iServoPeakLoad;
int CurPosRel;
//int iServoPos,iServoCmdPos,iServoLoad,iServoPeakLoad;
//int iServoPos,iServoCmdPos,iServoLoad,iServoPeakLoad;

BYTE fServoOn, fServoOff; //,fServo0On, fServoOff, fServo1On, fServoOff;
BYTE fStopOn, fStopOff;
BYTE fServoStart;

signed char fMotorDir;

BYTE fConModeAway, fConModeToward, fMeasureDir, fEccEn, fBiRefSide;
BYTE CmdOpPause;

WORD wMaxSpeed;
WORD wBaseLoad, wMaxLoad;
WORD AccelGain;

BYTE ExerCntTickDir;

int jointChangeSpeed;

WORD DCMotorRedrawRunTimeoutCnt,DCMotorPushRunTimeoutCnt;

UWORD ServoStateTimeoutCnt;

int TickInterVal,Tick100mTimerCnt;

//BYTE LogIndex,LogNum;

float  AngLPArm;

UINT servoErrCnt;

BYTE ServoTxBuff[40];

double dt = 0.005;
double Kp, Ki, Kd;

double load_diff;
double err, prev_err, pprev_err;  
double I_err, D_err;
double Kp_term, Ki_term, Kd_term;
double control;
double control_incr;
double control_prev;
double load_value;

int load_thr;

int iServoIncr;
int iServoIncrOld;

//void ServoNormalSpeedRunCmd( BYTE fDir, int iSpeed, UWORD wTime )
void ServoNormalSpeedRunCmd( BYTE fDir, int iSpeed)
{
	BYTE bFrameSize;
	
	if (fDir == SPD_DIR_REV) {
		iSpeed *= -1;
	}
	bFrameSize = makeModbusWriteFrame(Uart1_txBuff,ADDR_SPEED_CMD1,iSpeed);
	Uart1_TransmitStart(Uart1_txBuff, bFrameSize);
}



void ServoIOControl(WORD wParaNum , BYTE bPoint, BYTE fOn)
{
	int i;
	BYTE bFrameSize;
	WORD wCommCode;
	
	Uart1_txBuff[0] = ENQ;
	for (i=0;i<(sizeof(strPointConData)-1);i++)
	{
		Uart1_txBuff[i+1] = strPointConData[i];
	}
	i++;
	
	wCommCode = wParaNum*32+bPoint;
	Uart1_txBuff[i++] = wCommCode/100+'0';
	Uart1_txBuff[i++] = (wCommCode%100)/10+'0';
	Uart1_txBuff[i++] = (wCommCode%10)+'0';
	Uart1_txBuff[i++] = '0';
	Uart1_txBuff[i++] = fOn+'0';
	
	Uart1_txBuff[i++] = EOT;
	bFrameSize = i;
	Uart1_TransmitStart(Uart1_txBuff,bFrameSize);
}

void WriteServoParaData(WORD wParaAddr, WORD wParaData)
{
	BYTE bFrameSize;
	
	bFrameSize = makeModbusWriteFrame(Uart1_txBuff,wParaAddr,wParaData);
	Uart1_TransmitStart(Uart1_txBuff,bFrameSize);
}

void ReadServoParaData(WORD wPara)
{
	BYTE bFrameSize;
	
	bFrameSize = makeModbusReadFrame(Uart1_txBuff,1,wPara);
	Uart1_TransmitStart(Uart1_txBuff,bFrameSize);
}


int 	invControlState;
UWORD	invCurrData;
BYTE 	checkMIInvStateFrame(void);
UINT 	checkTOInvStateFrame(void);
BYTE 	GetMIInvCurr(void);
UWORD 	GetTOInvCurr(void);

void 	MakeRefFrame(UINT refSpeed);

BYTE CommEnable;

#define MAX_RETRY_CNT	6

#define DETECT_LOAD_RATE		130		// 5.00 %
#define ROM_SPD					1
#define ROM_SPD_MAX				190

typedef enum {
	SERVO_COMM_SEQ_IDLE,
	SERVO_COMM_SEQ_GET_LOAD_CMD,
	SERVO_COMM_SEQ_WAIT_LOAD_CMD,
	SERVO_COMM_SEQ_GET_POS_CMD,
	SERVO_COMM_SEQ_WAIT_POS_CMD,
	SERVO_COMM_SEQ_CMD,
	SERVO_COMM_SEQ_WAIT_CMD,
	SERVO_COMM_SEQ_WRITE_PARA_CMD,
	SERVO_COMM_SEQ_WRITE_PARA_WAIT_CMD
}ServoCommSeqStateType;

typedef enum {
	SERVO_COMM_IDLE,
	SERVO_COMM_WAIT,
	SERVO_COMM_RECEIVED,
	SERVO_COMM_TIME_OUT
}ServoCommStateType;

typedef enum {
	SERVO_COMM_CMD_NONE,
	SERVO_COMM_CMD_SPEED,
	SERVO_COMM_CMD_WRITE_PARAMETER,
	SERVO_COMM_CMD_SPEED_FIN,
	SERVO_COMM_CMD_WRITE_PARAMETER_FIN
}ServoCommCmdType;


typedef struct {
	BYTE bDir;
	UWORD wSpeed;
	UWORD wTime;
}ServoCommSpeedCmdType;

ServoCommSeqStateType 	ServoCommSeqState;
ServoCommStateType 		ServoCommState;
ServoCommCmdType 		ServoCommCmd;

ServoCommSpeedCmdType 	ServoCommSpeedCmd;
BYTE	bSVCommFailCnt;


#define SERVO_COMM_FAIL_LIMIT	10
#define SERVO_COMM_WAIT_TIME 30	// 서보의 응답지연이 최대 25msec  까지 바뀜
void CommServo()
{
	BYTE bFrameSize;
	BYTE bRetValue;
	switch(ServoCommSeqState)
	{
	case SERVO_COMM_SEQ_IDLE:
		
		if (ServoCommCmd == SERVO_COMM_CMD_SPEED) {
			ServoCommSeqState = SERVO_COMM_SEQ_CMD;
		}
		else if (ServoCommCmd == SERVO_COMM_CMD_WRITE_PARAMETER) {
			ServoCommSeqState = SERVO_COMM_SEQ_WRITE_PARA_CMD;
		}
		else {
			ServoCommSeqState = SERVO_COMM_SEQ_GET_LOAD_CMD;
		}
		break;
	case SERVO_COMM_SEQ_GET_LOAD_CMD:
		ReadServoParaData(ADDR_CURR_TORQUE);
		Uart1TimeoutCnt = 0;			
		
		ServoCommSeqState = SERVO_COMM_SEQ_WAIT_LOAD_CMD;
		ServoCommState = SERVO_COMM_WAIT;
		break;
	case SERVO_COMM_SEQ_WAIT_LOAD_CMD:
		if (CommEOFReceived()) {
			bSVCommFailCnt = 0;
			iServoLoadRate = MakeWord(Uart1_rxBuff[3],Uart1_rxBuff[4]);
			//				sscanf((char *)(Uart1_rxBuff+10),"%08x",&(iServoLoadRate));
			
			ServoCommState = SERVO_COMM_RECEIVED;
			ServoCommSeqState = SERVO_COMM_SEQ_GET_POS_CMD;
		}
		else if (Uart1TimeoutCnt > SERVO_COMM_WAIT_TIME) {
			if (bSVCommFailCnt < SERVO_COMM_FAIL_LIMIT) {
				bSVCommFailCnt++;
			}
			ServoCommState = SERVO_COMM_TIME_OUT;
			ServoCommSeqState = SERVO_COMM_SEQ_GET_POS_CMD;
		}
		else {
			ServoCommState = SERVO_COMM_WAIT;
		}
		break;
	case SERVO_COMM_SEQ_GET_POS_CMD:
		//			ReadServoParaData(4);
		bFrameSize = makeModbusReadFrame(Uart1_txBuff,2,ADDR_CMD_POS_PULSE);
		Uart1_TransmitStart(Uart1_txBuff,bFrameSize);
		
		Uart1TimeoutCnt = 0;			
		
		ServoCommState = SERVO_COMM_WAIT;
		ServoCommSeqState = SERVO_COMM_SEQ_WAIT_POS_CMD;
		break;
		
	case SERVO_COMM_SEQ_WAIT_POS_CMD:
		if (CommEOFReceived()) {
			bSVCommFailCnt = 0;
			//				sscanf((char *)(Uart1_rxBuff+10),"%08x",&(iServoCmdPos));
			iServoCmdPos = MakeWord(Uart1_rxBuff[3],Uart1_rxBuff[4]);
			
			ServoCommState = SERVO_COMM_RECEIVED;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
		}
		else if (Uart1TimeoutCnt > SERVO_COMM_WAIT_TIME) {
			if (bSVCommFailCnt < SERVO_COMM_FAIL_LIMIT) {
				bSVCommFailCnt++;
			}
			ServoCommState = SERVO_COMM_TIME_OUT;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
		}
		else {
			ServoCommState = SERVO_COMM_WAIT;
		}
		break;
		
		
	case SERVO_COMM_SEQ_CMD:
		switch(ServoCommCmd)
		{
		case SERVO_COMM_CMD_SPEED:
			ServoNormalSpeedRunCmd(ServoCommSpeedCmd.bDir, ServoCommSpeedCmd.wSpeed);
			Uart1TimeoutCnt = 0;			
			
			ServoCommState = SERVO_COMM_WAIT;
			ServoCommSeqState = SERVO_COMM_SEQ_WAIT_CMD;
			break;
		default:
			ServoCommState = SERVO_COMM_IDLE;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
			break;
		}
		ServoCommCmd = SERVO_COMM_CMD_NONE;
		break;
	case SERVO_COMM_SEQ_WAIT_CMD:
		if (CommEOFReceived()) {
			bSVCommFailCnt = 0;
			ServoCommState = SERVO_COMM_RECEIVED;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
			ServoCommCmd = SERVO_COMM_CMD_SPEED_FIN;
		}
		else if (Uart1TimeoutCnt > SERVO_COMM_WAIT_TIME) {
			if (bSVCommFailCnt < SERVO_COMM_FAIL_LIMIT) {
				bSVCommFailCnt++;
			}
			ServoCommState = SERVO_COMM_TIME_OUT;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
		}
		else {
			ServoCommState = SERVO_COMM_WAIT;
		}
		break;
		
		
	case SERVO_COMM_SEQ_WRITE_PARA_CMD:
		WriteServoParaData(wParaWriteAddr, wParaWriteData);
		Uart1TimeoutCnt = 0;			
		
		ServoCommState = SERVO_COMM_WAIT;
		ServoCommSeqState = SERVO_COMM_SEQ_WRITE_PARA_WAIT_CMD;
		ServoCommCmd = SERVO_COMM_CMD_NONE;
		break;
		
	case SERVO_COMM_SEQ_WRITE_PARA_WAIT_CMD:
		if (bRetValue = CommEOFReceived()) {
			if (bRetValue == COMM_CRC_OK) {
				bSVCommFailCnt = 0;
			}
			ServoCommState = SERVO_COMM_RECEIVED;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
			ServoCommCmd= SERVO_COMM_CMD_WRITE_PARAMETER_FIN;
		}
		else if (Uart1TimeoutCnt > SERVO_COMM_WAIT_TIME) {
			if (bSVCommFailCnt < SERVO_COMM_FAIL_LIMIT) {
				bSVCommFailCnt++;
			}
			ServoCommState = SERVO_COMM_TIME_OUT;
			ServoCommSeqState = SERVO_COMM_SEQ_IDLE;
		}
		else {
			ServoCommState = SERVO_COMM_WAIT;
		}
		break;
		
		
	}
	
	if (bSVCommFailCnt == SERVO_COMM_FAIL_LIMIT) {
		SysStatus.var.Status = SYS_ERROR;
	}
	
}			


WORD 	iServoLoadRate; //,iServoLoadRate,iServoLoadRate;
WORD		iServoRestPos; //,iServo0RestPos,iServo1RestPos;
LONG	lCurrentPosition;
WORD 	accSpd;
WORD 	iSysStatus;

void ServoControl(void)
{
	//	int iTempData;
	WORD	wDiffPos;
	
	iSysStatus = SysStatus.var.Status;
	
	if (fServoOn == TRUE) {
		fServoOn= FALSE;
		fServoStart = TRUE;
		ServoSVOn();	
		//		ServoSpdModeOn();
	}
	
	if (fServoOff == TRUE) {
		fServoOff= FALSE;
		fServoStart = FALSE;
		ServoSVOff();
		//		ServoSpdModeOff();
	}
	if (fStopOn == TRUE) {
		fStopOn= FALSE;
		ServoSVStopOn();	
	}
	if (fStopOff == TRUE) {
		fStopOff= FALSE;
		ServoSVStopOff();
	}
	setMotorDir (fMotorDir);
	
	switch(servoState)
	{
	case SERVO_INIT:
		SetServoState(SERVO_IDLE);
		break;
	case SERVO_IDLE:
		{
			if (ConsoleCmd == CONSOLE_START_POSITION) {
				ConsoleCmd = CONSOLE_NULL;
				SetServoState(SERVO_START_POS);
				servoStartPosState = SERVO_START_POS_CMD;
				iSysStatus=TO_START_POS;
			}
			else if (ConsoleCmd == CONSOLE_ORIGIN) {
				ConsoleCmd = CONSOLE_NULL;
				SetServoState(SERVO_ORIGIN);
				servoOrgState  = SERVO_ORG_CMD_INIT;
				iSysStatus=TO_ORIGIN;
			}
			else if (ConsoleCmd == CONSOLE_ROM) {
				ConsoleCmd = CONSOLE_NULL;
				SetServoState(SERVO_ROM);
				servoROMState = SERVO_ROM_CMD;
				wMaxSpeed = ROM_SPD_MAX;
				wBaseLoad = DETECT_LOAD_RATE;
				iSysStatus=ROM_SETTING;
			}
			else if (ConsoleCmd == CONSOLE_EXERCISE) {
				ConsoleCmd = CONSOLE_NULL;
				SetServoState(SERVO_EXERCISE);
				servoExerciseState = SERVO_EXERCISE_CMD;
				iSysStatus=MEASURING;
				SysStatus.var.ExerCntTick  = 0;
				ExerCntTickDir = 2;
			}
			else if (ConsoleCmd == CONSOLE_PARA_WRITE) {
				ConsoleCmd = CONSOLE_NULL;
				//				if ((bTotalParaWriteNum > 0) && (bTotalParaWriteNum <= MAX_PARA_WRITE_NUM)){
				SetServoState(SERVO_PARA_WRITE);
				servoParaWriteState = SERVO_PARA_WRITE_CMD;
				bParaWriteIdx = 0;
			}
			else {
				if ((iSysStatus!=TO_ORIGIN) && (iSysStatus!=INIT_STATE)){
					iSysStatus=STANDBY;
				}
			}
			break;
		}
	case SERVO_ORIGIN:
		{
			iSysStatus=TO_ORIGIN;
			switch(servoOrgState )
			{
			case SERVO_ORG_CMD_INIT:
				ServoSpdModeOn();
				ServoSVStopOff();	
				servoOrgState  = SERVO_ORG_GET_LOAD_CMD;
				break;
			case SERVO_ORG_GET_LOAD_CMD:
				if (abs(iServoLoadRate) > ORIGIN_TORQUE) {
					ServoStateTimeoutCnt=0;
					servoOrgState  = SERVO_ORG_END;
					ServoSVStopOn();	
				}
				break;
			case SERVO_ORG_END:
				if (ServoStateTimeoutCnt > 1000)
				{						
					targetPos = LP_OFF_LENGTH;
					gotoPosSpd = HOME_SPEED;
					ServoSpdModeOff();
					SetServoIDLE ( CONSOLE_START_POSITION);
					FlInitSensorCalSet=SET;
					
				}
				break;
			}
			break;
		}
		
	case SERVO_START_POS:
		{
			iSysStatus=TO_START_POS;
			switch(servoStartPosState)
			{
			case SERVO_START_POS_CMD:
				servoStartPosState = SERVO_START_POS_IDLE;
				ServoSVStopOn();	
				break;
			case SERVO_START_POS_IDLE:
				wDiffPos = abs(curPos - targetPos);
				if (ConsoleCmd == CONSOLE_END) {
					ConsoleCmd = CONSOLE_NULL;
					servoStartPosState = SERVO_START_POS_END;
				}
				else if (wDiffPos < 10 ) {
					servoStartPosState =  SERVO_START_POS_END;
				}
				break;
				
			case SERVO_START_POS_END:
				SetServoIDLE (CONSOLE_NULL);
				
				if (FlInitSensorCalSet )
				{
					FlInitSensorCal = SET;
					FlInitSensorCalSet = 0;
				}
				break;
			}
			break;
		}
		
	case SERVO_ROM:
		{
			iSysStatus=ROM_SETTING;
			
			switch(servoROMState)
			{
			case SERVO_ROM_CMD:
				servoROMState = SERVO_ROM_IDLE;
				break;
			case SERVO_ROM_IDLE:
				if (ConsoleCmd == CONSOLE_END) {
					ConsoleCmd = CONSOLE_NULL;
					servoROMState = SERVO_ROM_END;
				}
				else {
					servoROMState = SERVO_ROM_CMD;
				}
				break;
			case SERVO_ROM_END:
				//					servoROMState = SERVO_ROM_CMD;
				//					SetServoState(SERVO_IDLE);
				SetServoIDLE ( CONSOLE_NULL);
				iSysStatus=STANDBY;
				break;
			}
			break;
		}
		
	case SERVO_EXERCISE:
		{
			iSysStatus=MEASURING;
			switch(servoExerciseState)
			{
			case SERVO_EXERCISE_CMD:
				iSysStatus = MEASURING;
				//					fMeasureDir = AWAY_DIR;
				fEccEn = RESET;
				fConModeAway = contractionMode/2;
				fConModeToward = contractionMode%2;
				
				servoExerciseState = SERVO_EXERCISE_IDLE;
				break;
			case SERVO_EXERCISE_IDLE:
				if (ConsoleCmd == CONSOLE_END) {
					ConsoleCmd = CONSOLE_NULL;
					servoExerciseState = SERVO_EXERCISE_END;
				}
				else {
					servoExerciseState = SERVO_EXERCISE_CMD;
				}
				break;
			case SERVO_EXERCISE_END:
				//					servoExerciseState = SERVO_EXERCISE_CMD;
				SetServoIDLE (  CONSOLE_NULL);
				iSysStatus=STANDBY;
				break;
			}
			break;
		}
	case SERVO_PARA_WRITE:
		{
			//			iSysStatus=TO_ORIGIN;
			switch(servoParaWriteState )
			{
			case SERVO_PARA_WRITE_CMD:
				ServoCommCmd = SERVO_COMM_CMD_WRITE_PARAMETER;
				//					wParaWriteAddr = paraAddrList[bParaWriteIdx];
				//					wParaWriteData = paraDataList[bParaWriteIdx];
				//					bParaWriteIdx++;
				servoParaWriteState  = SERVO_WRITE_ACK_WAIT;
				bRetryCnt = 0;
				break;
			case SERVO_WRITE_ACK_WAIT:
				if (ServoCommCmd == SERVO_COMM_CMD_WRITE_PARAMETER_FIN) {
					servoParaWriteState  = SERVO_PARA_WRITE_END;
					//						ServoCommState = SERVO_COMM_IDLE;
				}
				else if (ServoCommState == SERVO_COMM_TIME_OUT) {
					if (bRetryCnt < 3) {
						ServoCommCmd = SERVO_COMM_CMD_WRITE_PARAMETER;			
						bRetryCnt++;
						//							Uart0TimeoutCnt = 0;
					}
					else {
						servoState = SERVO_IDLE;
						if (servoErrCnt < 0xFFFFFFFF){
							servoErrCnt++;
						}
						
					}
					//						ServoCommState = SERVO_COMM_IDLE;
				}
				break;
				
			case SERVO_PARA_WRITE_END:
				if (bParaWriteIdx < bTotalParaWriteNum) {
					servoParaWriteState  = SERVO_PARA_WRITE_CMD;
				}
				else {
					SetServoIDLE ( CONSOLE_NULL);
					bParaWriteIdx = 0;
					bTotalParaWriteNum = 0;
				}
				break;
			}
			break;
		}
	}
	
	SysStatus.var.Status = iSysStatus; 
}


//const BYTE DtctLdRt[2] = { 15, 10 };
const BYTE DtctLdRt = 15;

#define VIRTUAL_EXT_ARM_LENGTH	3000.					// 0.1 mm,  가속규정용 가상의 팔길이  
#define ISOTONIC_MIN_SPD 		500
#define LOAD2ACC_RATE 			(.1*98000./CNT_5MS)		// 10kg 질량 기준 , 5ms , 1G
#define MIN_SPD_DEV				490
#define NORM_ACC_PRESS			(.1*98000./CNT_5MS)			//  1G , 5ms
#define NORM_ACC_EXTENSION		(1.*(98000./VIRTUAL_EXT_ARM_LENGTH)*(1800./3.141592)/CNT_5MS ) 		// 0.1deg/s^2, 5ms, 1G기준,  가상 팔길이에 가속도 적용기준
#define FOLLOW_SPD_ADD			20					// 동기 모드시 종동부 추종량
#define PRESS_CUSHION_UNIT		20					// 0.1mm
#define	EXTENSION_CUSHION_UNIT	100					// 0.01deg

signed char fMotorPositiveDir;
int iCurLoad,iCurLoadRel, iAbsCurLoad, iCurPos, iCurPosRel,iRomCushion, iRomCushionAway, iRomCushionTwd, iDiffPos, iAccSpdOld;
int iCurTonicSpd;
signed char fMotorDirRel, fTempMotorDir,fTempMotorDirRel;
int iNormAcc,iNormAccROM, iHalfNormAcc; 
WORD wAccelGain, wAccSpdLimit;
int iFollowBand,iFollowSpdAdd;
int	towardSetSpdRel, awaySetSpdRel, ROMLength[2], awayPosRel,towardPosRel, iTemp, iTemp2;
BYTE bMeasureDir;
BYTE fLoadApplied;
BYTE fDoCheckCushion;
int iCushionBaseLenth;
BYTE fCrossMode, bOpSide, bRefSide;

void ServoOperationControl(void)
{
	//	int DetectLoadRate;
	//	int iTempSpd,iTempSpd1, iTempSpd2, iFollowBand,iFollowSpdAdd, iSpdAdd, iSpdAdd2;
	//	BYTE bSide;
	
	//	static BYTE bIsOutROM;
	
	opVarInit();	
	
	
	// 100501 jjw
	/*******************************************************************************
	Isotonic 운동 동작 사양
	
	LEG_PRESS모드
	1. iCurLoad 가 baseLoad 보다 작으면 저속으로 Toward방향으로 이동
	2. iCurLoad가 baseLoad보다 크고 isotonicLoad보다 작으면 반대방향으로 그 차만큼의 가속시간을 가진다.
	3. iCurLoad가 isotonicLoad보다 크면 그 방향으로 가속시간을 가진다.
	*******************************************************************************/
	if (fIsotonicMode == SET ){
		
		//		cur5msecSpd = (int)(fCurSpd/200.);
		//		cur5msecSpd = (int)(fCurSpd/200.);
		
		if (fIsotonicModeInit == RESET)
		{
			ExerCntTickDir = 2;
			fIsotonicModeInit = SET;
			iCurTonicSpd = curSpd*fMotorDir;;
			iCurTonicSpd = curSpd*fMotorDir;
		}
		
		isotonicControl();
		
		if (((fMeasureDir == AWAY_DIR) && ( ExerCntTickDir == TWD_DIR)) 							// 방향전환시
			|| ( (iCurPosRel <= (towardPosRel /*- iRomCushionTwd*/ + iFollowBand*2 )) && (ExerCntTickDir ==2))	)	{// 초기에 출발점 이전에 위치할 경우
				SysStatus.var.ExerCntTick = SET;
				LogIndex = Cnt10msec;
				
				TickInterVal = ( Tick100mTimerCnt >=  0xff) ? 0 : Tick100mTimerCnt;
				Tick100mTimerCnt = 0;
				
			}	
		
	}
	
	else if (jogMode )
	{
		
		if (CommPCMode == HYPER_TERMINAL )
		{
			jogSpeed =150;
		}
		
		
		/*		if ((jogMode &0x10) ) 
		{	
		jogSpeed[0] =  -jogSpeed[0];
		jogSpeed[1] = -jogSpeed[1];
	}
		*/		
		
		iAccSpd = jogSpeed;
		fMotorDir =  (jogMode &0x01) ? SERVO_FWD : SERVO_BWD;
		
		makeServoIncreement (  iAccSpd, 0, 0, SERVO_SPEED_MODE );
		
	}
	else if ( servoState == SERVO_ROM ) {
		
		
		if (fInitROM == SET)
		{
			iAccSpdOld = 0;
			fInitROM = RESET;
		}	
		isokineticControl(ISOKINETIC_ROM);
		
	}
	else if (servoState == SERVO_EXERCISE )		// Exercise Mode
	{
		
		
		if (fInitExercise == SET)
		{
			iAccSpdOld= 0;
			fMeasureDir = AWAY_DIR;
			fInitExercise = RESET;
		}
		
		if (legSel == BOTH_SIDE)
		{	
			if (!fCrossMode && ( (( bMeasureDir  == TWD_DIR ) && (fConModeToward== ECC_MODE) )
								 || (( bMeasureDir == AWAY_DIR ) && (fConModeAway == ECC_MODE) )) )
				isotonicControl();
			else 
				isokineticControl(ISOKINETIC_EXE);
		}
		else {
			if ( (( bMeasureDir  == TWD_DIR ) && (fConModeToward == ECC_MODE) )
				|| (( bMeasureDir  == AWAY_DIR ) && (fConModeAway == ECC_MODE) ) )
				isotonicControl();
			else 
				isokineticControl(ISOKINETIC_EXE);
		}	
		
		checkExerciseCount();	
		
		/***** Check Exercise count  *****/
		/*		{
		//				bSide = ( ( legSel == LEFT_SIDE ) || (( legSel == BOTH_SIDE ) && (fBiRefSide == LEFT_SIDE))) ? LEFT_SIDE : RIGHT_SIDE;
		
		// 왕복 Tick 기준용
		bRefSide = ( ( legSel == LEFT_SIDE ) || (( legSel == BOTH_SIDE ) && (fBiRefSide == LEFT_SIDE))) ? LEFT_SIDE : RIGHT_SIDE;
		
		if (((fMeasureDir[bRefSide] == AWAY_DIR) && ( ExerCntTickDir == TWD_DIR)) 							// 방향전환시
		|| ( (iCurPosRel[bRefSide] <= (towardPosRel[bRefSide] - iRomCushionTwd + 20 )) && (ExerCntTickDir ==2))	)	{// 초기에 출발점 이전에 위치할 경우
		SysStatus.var.ExerCntTick = SET;
		LogIndex = Cnt10msec;
		
		TickInterVal = ( Tick100mTimerCnt >=  0xff) ? 0 : Tick100mTimerCnt;
		Tick100mTimerCnt = 0;
		
		
		//					LogIndex = 0;
		//					Cnt10msec = 0;
	}
		
		
		ExerCntTickDir = fMeasureDir[bRefSide];
	}
		*/
		
		//	else if (
		
		
		
	}
	else if (servoState == SERVO_START_POS )
	{
		DebugVal1++;
		if (DebugVal1> 100) DebugVal1=0;
		if (servoState == SERVO_START_POS)	{
			ServoGotoPos();
		}
		
	}	
	
	
}


/***** Check Exercise count  *****/
void checkExerciseCount(void)	
{
	//				bSide = ( ( legSel == LEFT_SIDE ) || (( legSel == BOTH_SIDE ) && (fBiRefSide == LEFT_SIDE))) ? LEFT_SIDE : RIGHT_SIDE;
	
	// 왕복 Tick 기준용
	bRefSide = ( ( legSel == LEFT_SIDE ) || (( legSel == BOTH_SIDE ) && (fBiRefSide == LEFT_SIDE))) ? LEFT_SIDE : RIGHT_SIDE;
	
	if (((fMeasureDir == AWAY_DIR) && ( ExerCntTickDir == TWD_DIR)) 							// 방향전환시
		|| ( (iCurPosRel <= (towardPosRel /*- iRomCushionTwd*/ + iFollowBand*2 )) && (ExerCntTickDir ==2))	)	{// 초기에 출발점 이전에 위치할 경우
			SysStatus.var.ExerCntTick = SET;
			LogIndex = Cnt10msec;
			
			TickInterVal = ( Tick100mTimerCnt >=  0xff) ? 0 : Tick100mTimerCnt;
			Tick100mTimerCnt = 0;
			
			
			//					LogIndex = 0;
			//					Cnt10msec = 0;
		}
	
	
	ExerCntTickDir = fMeasureDir;
}



JointChangeSeqStateType JointChangeSeqState;

//BYTE PhotoSenSEQ[2];

#define ANG_BAND	20			// unit:0.0225deg,  .6deg


//#JOINT_CHG_SPEED_LE		

void ServoGotoPos()
{
	fMotorDir = ( targetPos > curPos )	? SERVO_FWD : SERVO_BWD;
	makeServoIncreement ( gotoPosSpd, targetPos, curPos, SERVO_POSITION_MODE );
}


//#define 	ACC_LIMIT	7
#define 	ACC_LIMIT	10
#define 	MAX_PULSE_OUT

int makeServoIncreement (int iSpeed, int iGoalPos, int iCurPos, BYTE CtrlMode)
{
//	float fTempAng, fCosVal;
//	float fPosIncrement, fDiff_5ms, fDiffPos;
	
	iServoIncr = 0;
	
	// 제한 범위를 넘어섰거나 일시정지가 된 경우에 속도 0으로 설정
	//	if (jointChangeCmd !=SET)
	//	{	
	//		if ((!checkPosLimit(lTotalPositionCount, fMotorDir)) || (CmdOpPause == SET))
	//			iSpeed = 0;
	//	}
	
	//	checkSafety(iSpeed);
	
	if (( SysStatus.var.Status == SYS_ERROR) || FPCCommFail )
		iSpeed = 0;
	
	if (iSpeed  )
	{
		iServoIncr = iSpeed;				// 0.01deg/s
		
		//		fDiff_5ms = (float)iSpeed / CNT_5MS;		// 0.1mm/5ms   , 0.1deg/5ms
		//		fDiff_5ms = (float)iSpeed;		// 0.1mm/5ms   , 0.1deg/5ms
		//		fDiffPos =( CtrlMode == SERVO_POSITION_MODE ) ?   (float)(abs ( iCurPos - iGoalPos )) : fDiff_5ms ;		
		//		fPosIncrement = ( fDiffPos >= fDiff_5ms ) ? fDiff_5ms : fDiffPos;
		
		//		if (CtrlMode == SERVO_ANG_SPEED_MODE )   // machine control
		//		{
		//			iServoIncr = (int)(fPosIncrement * PULSE_SCALER_MAIN_REDUCER*10 +0.5);				// 0.01deg/s
		//		}
		//		else if (jointSel == LEG_PRESS_MODE )
		//		{
		//			fTempAng = AngLPArm * 1.745329e-4;     //   1.745329e-4 = pi / 18000. 
		//			fCosVal = cos( fTempAng );
		////			DebugVal =3;
		//			
		//			iServoIncr = (int)(fPosIncrement* PULSE_SCALER_LP /(fLPAngScale*fCosVal * LP_ARM_LENGTH) *(5729.578)+.5);      // 5729.578 = 18000./pi   ,   pulse count / 5ms
		//		}
		//		else if (jointSel == LEG_EXTENSION_MODE )
		//		{
		//	//		DebugVal =4;
		//			iServoIncr = (int)(fPosIncrement *PULSE_SCALER_LE * 10.+.5);					// 속도는 0.1deg/s,  위치는 0.01deg이므로 10곱해줌
		//		}
		
		//		iServoIncr = ( iServoIncr >390) ? 390 : iServoIncr;
		//		iServoIncr = ( iServoIncr > 5000) ? 5000 : iServoIncr;
		
		if ((iServoIncr - iServoIncrOld) >= ACC_LIMIT )
			iServoIncr = iServoIncrOld + ACC_LIMIT;
		
		lServoPulseAddCnt = iServoIncr;
		
		lTotalPositionCount += fMotorDir*iServoIncr;
		
		setMotorDir (fMotorDir);
		
		Uart1TimeoutCnt = 0;
	}
	
	iServoIncrOld = iServoIncr;		
	
	return  iServoIncr;
}



int checkCushion( int iDiffPos, int iRomCushion, int iSetMsSpd, int iMinSpeed )
{
	int iTempSpd;
	
	
	if (iDiffPos <=  0 )
		iTempSpd = 0;
	else if (iDiffPos >= iRomCushion)
		iTempSpd = iSetMsSpd;
	else
		iTempSpd = iSetMsSpd * iDiffPos / iRomCushion + iMinSpeed;
	
	return  iTempSpd;
}

void setMotorDir (signed char bDir)
{
	
	if (bDir == SERVO_FWD ) ServoDirFwd();			
	else ServoDirRev();
}

BYTE checkPosLimit(int iPosCnt, signed char cMDir)
{
	BYTE ret;
	//	int iMaxCnt, iMinCnt;
	
	//	iMaxCnt = ( jointSel ==LEG_PRESS_MODE) ? AbsPosLmtAWAY[jointSel] : AbsPosLmtTWD[jointSel];
	//	iMinCnt = ( jointSel ==LEG_PRESS_MODE) ? AbsPosLmtTWD[jointSel] : AbsPosLmtAWAY[jointSel];
	
	if (((cMDir > 0) && (iPosCnt > AbsPosLmtAWAY)) || ((cMDir < 0) && (iPosCnt <  AbsPosLmtTWD)))
	{
		ret = 0;
	}
	else
	{
		ret = 1;
	}
	
	return ret;
}


int reduceSpeed ( int iSpeed, int idecrement )
{
	int ret;
	
	if ( iSpeed > 0  )		// 부하가 없는데 속도가 존재하는 경우
	{
		if (iSpeed >= idecrement)
			ret = iSpeed - idecrement;
		else
			ret = 0;
	}
	else
		ret = 0;
	
	return  ret;
}

int SpeedLimit ( int iSpeed, int iUpperSpeedLimit, int iLowerSpeedLimit)
{
	int ret;
	
	if (iSpeed > iUpperSpeedLimit)
		ret = iUpperSpeedLimit;
	else if (iSpeed < iLowerSpeedLimit )
		ret = iLowerSpeedLimit;
	else
		ret = iSpeed;
	
	return ret;
}


#define LE_SAFETY_TORQUE	200			// 0.1Nm

void checkSafety( int iSpeed)
{
	int iCurTqLE;
	
	iCurTqLE = abs(CurTqLE);  
	
	// 조그 운전시
	// 위치 이동시	
	// 모드 전환시
	
	if (SensorState == 0x1111 )
	{
		if (iSpeed || (CmdOpPause ==RESET) )
		{	
			if (jogMode || (jointChangeCmd == SET) || (servoState == SERVO_START_POS) )
			{
				if (iCurTqLE > LE_SAFETY_TORQUE )
				{	
					SysStatus.var.Status = SYS_ERROR;
				}			
			}		
		}		
	}
}



void makeSpeedEachSide(void)
{
	BYTE bSide;
	int iTempSpd1, iTempSpd2;	
	
	DebugVal3 = 0;
	
	for ( bSide = RIGHT_SIDE; bSide <= LEFT_SIDE; bSide++)
	{			
		fTempMotorDirRel = fMotorDirRel;
		
		if (( bMeasureDir  != AWAY_DIR ) && (iCurLoadRel < -baseLoad) )
		{
			if (fMotorDirRel == SERVO_FWD )		// 부하방향은 바뀌었는데 속도는 반대로 움직이는경우
			{
				iAccSpd = reduceSpeed (iAccSpdOld, iNormAcc);
				fTempMotorDirRel = (iAccSpd) ? SERVO_FWD : SERVO_STOP;
				if (bSide == fBiRefSide) DebugVal3 = 1;
			}
			else if ((  fMotorDirRel == SERVO_BWD ) || ( iCurLoadRel <  -opLoad) ) 	// 부하방향으로 제어
			{
				iTempSpd1 = iAccSpdOld + iNormAcc;
				if (iTempSpd1 > towardSetSpdRel ) iTempSpd1 = towardSetSpdRel;
				
				iTempSpd2 = iTempSpd1;
				
				if (fDoCheckCushion )			// cushion을 적용할경우만 실행 
				{
					iTempSpd2 = checkCushion( iCurPosRel-towardPosRel, iRomCushionTwd, towardSetSpdRel,FOLLOW_SPD_ADD*3 );
					if (( iTempSpd2+iNormAcc) < iAccSpdOld)		// 쿠션에 의한 감속시에 정규감속보다 감속폭이 크면 정규감속적용
						iTempSpd2  = iAccSpdOld - iNormAcc;
				}	
				
				iAccSpd = ( iTempSpd1 < iTempSpd2 ) ? iTempSpd1 : iTempSpd2;				
				
				if (iAccSpd )	{			
					fTempMotorDirRel = SERVO_BWD;
					fLoadApplied = SET;
				}						
				else						
					fTempMotorDirRel = SERVO_STOP;				// 개념적인 방향
				
				if (bSide == fBiRefSide) DebugVal3 = 2;
				
			}				
		}
		else if (( bMeasureDir != TWD_DIR )&&(iCurLoadRel > baseLoad) )
		{	
			if ( fMotorDirRel == SERVO_BWD )		// 부하방향은 바뀌었는데 속도는 반대로 움직이는경우
			{
				iAccSpd = reduceSpeed (iAccSpdOld, iNormAcc);
				if (bSide == fBiRefSide) DebugVal3 = 3;
			}
			else  if (( fMotorDirRel == SERVO_FWD ) || (iCurLoadRel > opLoad) )  // 부하방향으로 제어
			{
				iTempSpd1 = iAccSpdOld + iNormAcc;
				if (iTempSpd1 > awaySetSpdRel ) iTempSpd1 = awaySetSpdRel;
				
				iTempSpd2 = iTempSpd1;
				
				if ( fDoCheckCushion )			// cushion을 적용할경우만 실행 
				{
					iTempSpd2 = checkCushion( awayPosRel-iCurPosRel , iRomCushionAway, awaySetSpdRel,FOLLOW_SPD_ADD*3 );
					if (( iTempSpd2+iNormAcc) < iAccSpdOld)		// 쿠션에 의한 감속시에 정규감속보다 감속폭이 크면 정규감속적용
						iTempSpd2  = iAccSpdOld - iNormAcc;
				}
				
				iAccSpd = ( iTempSpd1 < iTempSpd2 ) ? iTempSpd1 : iTempSpd2;
				
				if (iAccSpd )	{			
					fTempMotorDirRel = SERVO_FWD;
					fLoadApplied = SET;
				}						
				else						
					fTempMotorDirRel = SERVO_STOP;				// 개념적인 방향
				
				if (bSide == fBiRefSide) DebugVal3 = 4;	
				
			}
		}
		else
		{
			if ( iAccSpdOld > 0  )		// 부하가 없는데 속도가 존재하는 경우
			{
				iAccSpd = reduceSpeed (iAccSpdOld, iNormAcc);
				if (iAccSpd == 0 ) 
					fTempMotorDirRel = SERVO_STOP;
			}
		}				
	}
}


void isokineticControl(BYTE fMode)
{
	if (fMode== ISOKINETIC_EXE)
		initExerciseModeVariable();		
	else
		initRomModeVariable();
	
	makeSpeedEachSide();
	
	
	fMeasureDir = bMeasureDir;
	
	iAccSpdOld= iAccSpd;
	
	
	makeServoIncreement (  iAccSpd, 0, 0, SERVO_SPEED_MODE );
	SysStatus.var.Status = ( fMeasureDir  == TWD_DIR) ? TO_TWD_DIR : TO_AWAY_DIR;	
}

void initExerciseModeVariable(void)
{
	
	if (CommPCMode == HYPER_TERMINAL ) {
		awaySetSpd	= 2000;	// 36
		towardSetSpd= 1000;	// 37
		towardPos	= 4700;
		awayPos		= 800;
		loadEccLimit= 450;
	}
	
	loadEccLimit= 1800;
	
	baseLoad *= 3;
	opLoad *= 3;	 
	
	changeCoordinate(fCrossMode);
	
	if (jointSel== LEG_PRESS_MODE) {
		iNormAcc =  (int)NORM_ACC_PRESS*4;
		iCushionBaseLenth = cushionSet*PRESS_CUSHION_UNIT;
	}
	else {
		iNormAcc = (int)NORM_ACC_EXTENSION;	
		iCushionBaseLenth = cushionSet*EXTENSION_CUSHION_UNIT;		
	}
	
	fDoCheckCushion = SET;
	
	iRomCushionAway =  iCushionBaseLenth + iCushionBaseLenth*6*awaySetSpd/ imaxSpeed;				
	iRomCushionTwd =  iCushionBaseLenth + iCushionBaseLenth*6*towardSetSpd/ imaxSpeed;				
	
	
	// ecc모드에서 부하체크	
	
	// ECC등속모드에 따른 부하표시 방향전환
	if ((( bMeasureDir  == TWD_DIR ) && (fConModeToward == ECC_MODE) )
		|| (( bMeasureDir  == AWAY_DIR ) && (fConModeAway == ECC_MODE) ))
		iCurLoadRel = -iCurLoadRel;
	
	
	
}		

void initRomModeVariable(void)
{
	
	awaySetSpd	=  200;	// 36
	towardSetSpd=  200;	// 37
	/*towardPos	= towardPos		= 7700;
	awayPos		= awayPos		= -2200;*/
	iNormAcc = (int)NORM_ACC_EXTENSION/2;	
	
	
	fCrossMode = 0;
	changeCoordinate(fCrossMode);
	
	/*	for ( bSide = RIGHT_SIDE; bSide <= LEFT_SIDE; bSide++)
	{
	towardSetSpdRel = towardSetSpd;
	awaySetSpdRel = awaySetSpd;
	
	iCurPosRel = iCurPos;		
	iCurLoadRel = iCurLoad;	
}*/
	
	bMeasureDir = NO_DIR;
	
	iFollowBand = iFollowBand*4;
	
	fDoCheckCushion = RESET;
}

void opVarInit(void)
{	
	int ROMEndMargin;
	
	fLoadApplied = 0;	
	
	fBiRefSide = BiRefSide;
	//	bOpSide = OP_SIDE[fBiRefSide];
	
	iCurPos = curPos;
	
	//	curJointState();
	
	//	iCurLoad = CurForceLP;
	iCurLoad = LoadDataAcquired;
	iFollowBand = 50;		
	baseLoad = DtctLdRt;
	opLoad = 30;	 
	
	iAbsCurLoad = abs(iCurLoad);	
	
	iAccSpd	= 0;	
	
	ROMEndMargin = 0;	// 400
	ROMStartPos	= towardPos + ROMEndMargin;		// 24
	ROMStopPos	= awayPos - ROMEndMargin;		// 25
	ROMStartPos	= towardPos + ROMEndMargin;		// 24
	ROMStopPos	= awayPos - ROMEndMargin;		// 25
	
	
	fCrossMode = (( legSel == BOTH_SIDE ) && ( biSyncMode == CROSS_MODE )) ? 1 : 0;
	
}

void pid_calc()
{
	
	if (!fServoStart || FPCCommFail || getLoLimitState() || getHiLimitState())
	{
		Kp = 0.06;
		Ki = 0.03;
		load_thr = 10;

		isotonicLoad = 11000;
		
		load_value = (double)LoadDataAcquired;
		
		prev_err = 0;
		control = 0;
		control_prev = 0;
		I_err = 0;
		return;
	}

	load_diff = (double)LoadDataAcquired - (double)isotonicLoad;

	if (load_diff > load_thr)
	{
		err = load_diff - load_thr;
	}
	else if (load_diff < -load_thr)
	{
		err = load_diff + load_thr;
	}
	else
	{
		err = 0;
		// 측정값과 기준값이 안정구간 범위 내에 있는 경우 I_err를 50%씩 줄임
		if (fabs(I_err) > (1.0/Ki))
		{ 
			I_err *= 0.5;
		}
		else
		{
			I_err = 0;
		}
	}
		
	// P term
	Kp_term = Kp*err;

	// I term
	I_err += err*dt;
	Ki_term = Ki*I_err;

	// D term
	D_err = (err - prev_err)/dt;
	Kd_term = Kd*D_err;

	// Output
	control = Kp_term + Ki_term + Kd_term;
	prev_err = err;

	if (control > SERVO_MAX_PULSE)
	{
		if (Ki == 0) I_err = 0;
		else I_err = ((float)SERVO_MAX_PULSE - Kp_term - Kd_term) / Ki;
		control = SERVO_MAX_PULSE;
	}
	else if (control < -SERVO_MAX_PULSE)
	{
		if (Ki == 0) I_err = 0;
		else I_err = (-(float)SERVO_MAX_PULSE - Kp_term - Kd_term) / Ki;
		control = -SERVO_MAX_PULSE;
	}
	
	lServoPulseAddCnt = (LONG)control;
	control_prev = control;		
}

void pid_v_calc()
{
	if (!fServoStart || FPCCommFail || getLoLimitState() || getHiLimitState())
	{
		Kp = 0.1;
		Ki = 4.0;	
		Kd = 0.0;

		isotonicLoad = 11000;

		prev_err = 0;
		pprev_err = 0;
		control = 0;
		lServoPulseAddCnt = 0;
		return;
	}

	load_diff = LoadDataAcquired - (double)isotonicLoad;
	
	if (load_diff > 0)
	{
		err = pow(fabs(load_diff), 0.5);
	}
	else if (load_diff < 0)
	{
		err = -pow(fabs(load_diff), 0.5);		
	}
	
	Kp_term = Kp * (err - prev_err);
	Ki_term = Ki * err * dt;
	Kd_term = Kd * (err - 2.0*prev_err + pprev_err) / dt;
	
	pprev_err = prev_err;
	prev_err = err;
	
	control_incr = Kp_term + Ki_term + Kd_term;
	
//	if (control_incr > 100.0)
//	{
//		control_incr = 100.0;
//	}
//	else if (control_incr < -100.0)
//	{
//		control_incr = -100.0;
//	}
	
	control += control_incr;

	if (control > SERVO_MAX_PULSE)
	{
		control = SERVO_MAX_PULSE;
	}
	else if (control < -SERVO_MAX_PULSE)
	{
		control = -SERVO_MAX_PULSE;
	}
	
	lServoPulseAddCnt = (LONG)control;
}

void isotonicControl(void)
{	
	if (CommPCMode == HYPER_TERMINAL ) {
		awaySetSpd	= 2000;	// 36
		towardSetSpd= 1000;	// 37
		towardPos	= 4700;
		awayPos		= 800;
	}

//	pid_calc();
////	pid_v_calc();
}

void changeCoordinate(BYTE bCrossMode)
{
	BYTE bSide;
	
	//좌표계 전환	:  오른쪽 고정에 왼쪽만 뒤집기
	for ( bSide = RIGHT_SIDE; bSide <= LEFT_SIDE; bSide++)
	{
		towardSetSpdRel = towardSetSpd;
		awaySetSpdRel = awaySetSpd;
		
		if (jointSel ==LEG_PRESS_MODE)
		{
			//				ROMLength = ROMStopPos - ROMStartPos;
			
			awayPosRel = (int)(awayPos - ROMStartPos);
			towardPosRel = (int)(towardPos - ROMStartPos);
			iCurPosRel = iCurPos - ROMStartPos;
			
			iCurLoadRel = iCurLoad;
		}
		else //  jointSel == LEG_EXTENSION_MODE
		{
			//				ROMLength = ROMStartPos - ROMStopPos;
			
			awayPosRel = (int)( ROMStartPos - awayPos );
			towardPosRel = (int)( ROMStartPos- towardPos );
			iCurPosRel = ROMStartPos - iCurPos;
			
			iCurLoadRel = -iCurLoad;
		}		
	}			
	
	CurPosRel = iCurPosRel;
	
	
	// 측정방향 체크 및 규정	
	if ( iCurPosRel <= (towardPosRel  + iFollowBand*10 ) )
	{	
		if (iAccSpdOld ==0)
			fMeasureDir = AWAY_DIR;
	}
	else if (iCurPosRel >= (awayPosRel  - iFollowBand*10 ) )
	{				
		if (iAccSpdOld ==0)
			fMeasureDir = TWD_DIR;
	}
	
	bMeasureDir = fMeasureDir;
	
	
}

void Isr_System_HyperTerm(BYTE tmpChar)
{	
	//	static int Uart2_rxBuffIndex;
	static BYTE pcBuffIndex;
	
	if (tmpChar=='1')
	{
		lServoPulseAddCntPrev = 0;
		fServoOff = TRUE;
	}
	else if (tmpChar == '2')
	{
		lServoPulseAddCntPrev = 0;
		fServoOn = TRUE;
	}
	else if (tmpChar=='3')
	{
		fStopOff = TRUE;
	}
	else if (tmpChar == '4')
	{
		fStopOn = TRUE;
	}
	else if (tmpChar == 'P' )
	{
		ConsoleCmd = CONSOLE_PARA_WRITE;
		wParaWriteAddr = ADDR_OP_MODE;
		wParaWriteData = OP_MODE_POS;
		bParaWriteIdx = 1;		
		//		WriteServoParaData(76, 3);
	}
	else if (tmpChar == 'S') {
		ConsoleCmd = CONSOLE_PARA_WRITE;
		wParaWriteAddr = ADDR_OP_MODE;
		wParaWriteData = OP_MODE_SPD;
		bParaWriteIdx = 1;		
		
		//		WriteServoParaData(76, 2);
	}
	
	else if (tmpChar == '6')
	{
		lServoPulseAddCntPrev = 800; //cnt;
		lServoPulseAddCnt = lServoPulseAddCntPrev;
		
	}
	else if (tmpChar == '7')
	{
		lServoPulseAddCntPrev = -100; //cnt;
		lServoPulseAddCnt = lServoPulseAddCntPrev;
	}
	else if (tmpChar == '8')
	{
		GravityCorEn =1;
		GravityCor = RealCurTqLE;
	}
	else if (tmpChar == '9')
	{
		OffsetPos = -(int)lTotalPositionCount;
		
		
		//			lPulseCnt = 900;
		//			Uart0_Init(19200);
	}
	else if (tmpChar == 'r')
	{
		ReadServoParaData(60);
	}
	else if (tmpChar == 'R')
	{
		ReadServoParaData(60);
	}
	else if (tmpChar == 'w')
	{
		WriteServoParaData(60, 20);
	}
	else if (tmpChar == 'p')
	{
		ServoSpdModeOff();
	}
	else if (tmpChar == 's')
	{
		ServoSpdModeOn();
	}
	else if (tmpChar == 'o') {
		ConsoleCmd = CONSOLE_ORIGIN;
	}
	else if (tmpChar == 'O') {
	}
	else if (tmpChar == 'm') {
		if (IsoKineticTrainingMode == LEG_PRESS_MODE) {
			IsoKineticTrainingMode = LEG_EXTENSION_MODE;
		}
		else {
			IsoKineticTrainingMode = LEG_PRESS_MODE;
		}
		UpdateTrainingParameterLeft();
	}
	else if (tmpChar == 'x') {
		if (IsoKineticSyncMode == CROSS_MODE) {
			IsoKineticSyncMode = SYNC_MODE;
		}
		else {
			IsoKineticSyncMode = CROSS_MODE;
		}
		UpdateTrainingParameterLeft();
	}
	else if (tmpChar == '`') {
		targetPos = 1000;
		gotoPosSpd = 6000;
		ConsoleCmd = CONSOLE_START_POSITION;
	}
	else if (tmpChar == '~') {
		targetPos = 0;
		gotoPosSpd = 6000;
		ConsoleCmd = CONSOLE_START_POSITION;
	}
	else if (tmpChar == '%') {
		SetServoIDLE(CONSOLE_ROM);
	}
	else if (tmpChar == '^') {
		ConsoleCmd = CONSOLE_END;			
		
	}
	else if (tmpChar == '&') {
		SetServoState(SERVO_IDLE);
		ConsoleCmd = CONSOLE_EXERCISE;
		
	}
	else if (tmpChar == '*') {
		ConsoleCmd = CONSOLE_END;
	}
	else if (tmpChar == '?') {
		DebugOutIdx++;
		if (DebugOutIdx >=7 ) DebugOutIdx =0;
	}
	else if (tmpChar == '<') {
		jogMode =  jogMode ? 0 : 0x10;
	}
	else if (tmpChar == '[')
	{
		fIsotonicMode = SET;
	}
	else if (tmpChar == '{' )
	{
		fIsotonicMode = RESET;
		fIsotonicModeInit = RESET;
	}
	else if (tmpChar == 'z') {
		contractionMode++;
		if (contractionMode >=4 ) contractionMode =0;
	}
	else if (tmpChar == 'P' )
	{
		fIsotonicMode = RESET;
		fIsotonicModeInit = RESET;
	}
	else if (tmpChar == 'S') {
		contractionMode++;
		if (contractionMode >=4 ) contractionMode =0;
	}
	
	if (pcBuffIndex) {
		pcBuffIndex = RESET;
		if (tmpChar == 'c' )
			CommPCMode = PC_PROGRAM;
	}
	else if (tmpChar == 'p' )
		pcBuffIndex = SET;
	
	
	//	if (Uart2_rxBuffIndex < COMM_BUFF_SIZE) {	// 수신 버퍼 overflow check
	//		if ((tmpChar == '(') || (tmpChar == ENQ) || (tmpChar == ACK) || (tmpChar == NAK)) {		// 수신 프레임의 시작 데이터인 경우				Uart2_rxBuff[0] = tmpChar;	// 수신 데이터를 버퍼에 저장
	//			Uart2_rxBuffIndex = 1;
	//		}
	//		else if ((tmpChar == EOT)|| (tmpChar == ETX)) {	// 현재 수신 프레임의 마지막 수신 데이터인 경우 
	//			Uart2_rxBuffSize = Uart2_rxBuffIndex;
	//			Uart2_rxBuffIndex = 0;
	//			fUart2Received = 1;
	//		}
	//		else {
	//			Uart2_rxBuff[Uart2_rxBuffIndex] = tmpChar;	// 수신 데이터를 버퍼에 저장
	//			Uart2_rxBuffIndex++;
	//		}
	//			
	//	}
}

