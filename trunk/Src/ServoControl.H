//#include "head.h"

#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#define PC_PROGRAM			0
#define HYPER_TERMINAL		1

#define SERVO_MAX_PULSE		800

typedef enum {
	CONSOLE_NULL,
	CONSOLE_START_POSITION,
	CONSOLE_ORIGIN,
	CONSOLE_ROM,
	CONSOLE_EXERCISE,
	CONSOLE_PARA_WRITE,
	CONSOLE_END
}ConsolCmdType;

typedef enum {
	SERVO_INIT,
	SERVO_IDLE,
	SERVO_START_POS,
	SERVO_ORIGIN,
	SERVO_ROM,
	SERVO_EXERCISE,
	SERVO_POSITION,
	SERVO_PARA_WRITE
}ServoStateType;

typedef enum {
	SERVO_POSITION_MODE,
	SERVO_SPEED_MODE,
	SERVO_ANG_SPEED_MODE	// 1차 감속기 속도
}ServoControlMode;
	

typedef enum {
	SERVO_ORG_CMD_INIT,
	SERVO_ORG_CMD,
	SERVO_ORG_WAIT_RESP,
	SERVO_ORG_GET_LOAD_CMD,
	SERVO_ORG_WAIT_LOAD_RESP,
	SERVO_ORG_STOP,
	SERVO_ORG_END
}ServoOriginStateType;

typedef enum {
	SERVO_START_POS_CMD,
	SERVO_START_POS_WAIT,
	SERVO_START_POS_IDLE,
	SERVO_START_POS_END
}ServoStartPosStateType;

typedef enum {
	SERVO_ROM_CMD,
	SERVO_ROM_WAIT,
	SERVO_ROM_IDLE,
	SERVO_ROM_END
}ServoROMStateType;

typedef enum {
	SERVO_EXERCISE_CMD,
	SERVO_EXERCISE_WAIT,
	SERVO_EXERCISE_IDLE,
	SERVO_EXERCISE_END
}ServoExerciseStateType;

typedef enum {
	SERVO_PARA_WRITE_CMD,
	SERVO_WRITE_ACK_WAIT,
	SERVO_PARA_WRITE_END
}ServoParaWriteType;

typedef enum {
	JOINT_CHANGE_SEQ_IDLE,
	JOINT_CHANGE_SEQ_GOTO_PARK_CMD,
	JOINT_CHANGE_SEQ_REDRAW_CLUTCH_CMD,
	JOINT_CHANGE_SEQ_REDRAW_CLUTCH_WAIT_CMD,
	JOINT_CHANGE_SEQ_GOTO_CHANGE_CMD,
	JOINT_CHANGE_SEQ_PUSH_CLUTCH_CMD,
	JOINT_CHANGE_SEQ_PUSH_CLUTCH_WAIT_CMD,
	JOINT_CHANGE_END_CMD
}JointChangeSeqStateType;

#define SERVO_FWD	(1)
#define SERVO_STOP	(0)
#define	SERVO_BWD	(-1)

#define MAX_PARA_WRITE_NUM		5
#define MAX_LOGGING_NUM 26//8,mod 091016

extern ConsolCmdType ConsoleCmd;//,ConsoleCmd[RIGHT_SIDE],ConsoleCmd[LEFT_SIDE];
extern BYTE CommEnable;
extern ServoStateType servoState; //,servoState[RIGHT_SIDE],servoState[LEFT_SIDE];
extern ServoOriginStateType 	servoOrgState; //, servoOrgState[RIGHT_SIDE], servoOrgState[LEFT_SIDE];
extern ServoStartPosStateType 	servoStartPosState; //,servoStartPosState[RIGHT_SIDE], servoStartPosState[LEFT_SIDE];
extern ServoROMStateType 		servoROMState; //,servoROMState[RIGHT_SIDE], servoROMState[LEFT_SIDE];
extern ServoExerciseStateType	servoExerciseState; //,servoExerciseState[RIGHT_SIDE],servoExerciseState[LEFT_SIDE];
extern int iServoPos,iServoCmdPos,iServoLoad,iServoPeakLoad;
extern JointChangeSeqStateType JointChangeSeqState;

//extern int iServoPos[RIGHT_SIDE],iServoCmdPos[RIGHT_SIDE],iServoLoad[RIGHT_SIDE],iServoPeakLoad[RIGHT_SIDE];
//extern int iServoPos[LEFT_SIDE],iServoCmdPos[LEFT_SIDE],iServoLoad[LEFT_SIDE],iServoPeakLoad[LEFT_SIDE];

extern BYTE fServoOn, fServoOff; //,fServo0On, fServoOff[RIGHT_SIDE], fServo1On, fServoOff[LEFT_SIDE];
extern BYTE fStopOn, fStopOff;
extern BYTE fServoStart;

extern signed char fMotorDir;
extern BYTE fConModeAway, fConModeToward, fMeasureDir, fEccEn ;
extern BYTE CmdOpPause;

extern WORD	Uart0TimeoutCnt;
extern WORD	Uart1TimeoutCnt;
extern WORD iServoLoadRate;
extern BOOL fInitExercise,fInitROM;
extern BOOL FIsJointDefined;

extern int AbsPosLmtTWD, AbsPosLmtAWAY ;
extern int iServoIncr;
extern LONG	lServoPulseAddCnt, lServoPulseAddCntPrev;
extern LONG	lTotalPositionCount,lTotalPositionCountOut;
extern WORD wMaxSpeed;
extern WORD AccelGain ;
extern int iAccSpd;
extern int iCurJointState;
extern int iJointChgSeq;
extern BYTE FDCMotorRun;

extern WORD DCMotorRedrawRunTimeoutCnt,DCMotorPushRunTimeoutCnt;
extern UWORD ServoStateTimeoutCnt;
extern int jointChangeSpeed;

extern BYTE CommPCMode;

extern int TickInterVal,Tick100mTimerCnt ;
extern void SetServoState(ServoStateType sType);
extern void SetServoIDLE(ConsolCmdType sCmd);
extern  int CurPosRel;

//extern BYTE Cnt10msec;




extern  void CommServo(void);
extern	void ServoControl(void);
extern	void ReadServoIOState(void);
extern	void ReadServoParaData(WORD wPara);
extern	void ServoIOControl(WORD wParaNum , BYTE bPoint, BYTE fOn);
extern	void WriteServoParaData(WORD wParaNum, WORD wParaData);

extern int makeServoIncreement ( int iSpeed, int iGoalPos, int iCurPos, BYTE CtrlMode ) ;
extern int checkCushion( int iDiffPos, int iRomCushion, int iSetMsSpd, int iMinSpeed  );
extern void setMotorDir ( signed char bDir) ;
extern void ServoOperationControl(void);
extern void ServoGotoPos(void);
extern int reduceSpeed ( int iSpeed, int idecrement );
extern int SpeedLimit ( int iSpeed, int iUpperSpeedLimit, int iLowerSpeedLimit);
extern int curJointState(void);
extern void opVarInit(void);


#endif /*__SERVO_CONTROL_H__*/