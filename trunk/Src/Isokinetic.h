/***********************************************
 * NAME    : ISOKINETIC.h      	                    *
 * Version : 30.May.2006                        *
 ***********************************************/

#ifndef __ISOKINETIC_H__
#define __ISOKINETIC_H__


//#define START_POS_LEG_PRESS_TOWARD	1000L
//#define	START_POS_LEG_PRESS_AWAY	100000L
//#define START_POS_LEG_EXTENSION		50000L

#define START_POS_LEG_PRESS_TOWARD	-170000L
#define	START_POS_LEG_PRESS_AWAY	-99999L
#define START_POS_LEG_EXTENSION		50000L

#define MAX_LOGGING_NUM 26//8,mod 091016
#define MAX_PARA_WRITE_NUM		5

#define PC_COMM_FAIL_LIMIT 		100    // 1s unit: 10ms
	
typedef enum {
	LEG_PRESS_MODE,
	LEG_EXTENSION_MODE
}IsoKineticTrainingModeType;

typedef enum {
	SYNC_MODE,
	CROSS_MODE
}IsoKineticSyncModeType;

typedef enum {
	AWAY_DIR,
	TWD_DIR,
	NO_DIR
}IsoKineticDirType;

typedef enum {
	RIGHT_SIDE,
	LEFT_SIDE,
	BOTH_SIDE
}IsoKineticSideType;
	
typedef enum {
	CON_MODE,
	ECC_MODE
}IsoKineticConModeType;

typedef enum {
	STANDBY,
	TO_START_POS,
	TO_AWAY_DIR,
	TO_TWD_DIR,
	MEASURING,
	ROM_SETTING,
	TO_ORIGIN,
	SYS_ERROR,
	INIT_STATE = 0x0F
}IsoKineticSysStatusType;

typedef enum {
	SENSOR_IDLE,
	SENSOR_COMM_FAIL
}IsoKineticSensorStateType;

enum {
	R_SYSTEM_STATUS,
	R_CUR_POS,
	R_CUR_SPD,
	R_CUR_TRQ,
	R_ORIGIN_CMD,

	R_GRAVITY_COR_EN,		// 5
	R_GRAVITY_COR,			
	R_ROM_SET_STATUS,
	R_ROM_START_POS,
	R_ROM_STOP_POS,

	R_ROM_CAL_POS,			// 10
	R_AWAY_POS,
	R_TOWARD_POS,
	R_AWAY_SET_SPD,
	R_TOWARD_SET_SPD,

	R_AWAY_PROM,			//15
	R_TOWARD_PROM,
	R_GOTO_POS_CMD,
	R_GOTO_POS_SPD,
	R_TARGET_POS,

	BASE_LOAD,				//20
	CUSHION_SET,		
	OP_LOAD,			
	ECC_TORQUE,		
	BI_SYNC_MODE,		
	JOINT_SEL,		
	LEG_SEL,			
	CONTRACTION_MODE,
	SYSTEM_CONTROL,
	SENSOR_STATE,

	SET_LEG_ORG,			//30
	LEG_ORG_SET_STATE,
	BI_LAT_REF_SIDE,
	RESERVED53,
	RESERVED54,
	RESERVED55,
	JOINT_CHANGE_CMD,
	ISOTONIC_LOAD,
	ECC_LOAD_LIMIT,
	RESERVED59,

	R_SERVO_STATUS,			//40
	RESERVED61,
	RESERVED62,
	RESERVED63,
	RESERVED64,
	RESERVED65,
	R_PARA_WRITE_CMD,
	L_PARA_WRITE_CMD,
	R_PARA_WRITE_NUM,
	L_PARA_WRITE_NUM,

	R_PARA_WRITE_ADDR0,		// 50
	R_PARA_WRITE_ADDR1,
	R_PARA_WRITE_ADDR2,
	R_PARA_WRITE_ADDR3,
	R_PARA_WRITE_ADDR4,
	R_PARA_WRITE_DATA0,
	R_PARA_WRITE_DATA1,
	R_PARA_WRITE_DATA2,
	R_PARA_WRITE_DATA3,
	R_PARA_WRITE_DATA4,
	
	L_PARA_WRITE_ADDR0,		//70
	L_PARA_WRITE_ADDR1,
	L_PARA_WRITE_ADDR2,
	L_PARA_WRITE_ADDR3,
	L_PARA_WRITE_ADDR4,
	L_PARA_WRITE_DATA0,
	L_PARA_WRITE_DATA1,
	L_PARA_WRITE_DATA2,
	L_PARA_WRITE_DATA3,
	L_PARA_WRITE_DATA4,
	MAX_REG
};

typedef enum {
	READ_ONLY,
	READ_WRITE
}ParameterAttrType;

enum {
	ISOKINETIC_ROM,
	ISOKINETIC_EXE
};

extern IsoKineticTrainingModeType IsoKineticTrainingMode;
extern IsoKineticSyncModeType IsoKineticSyncMode;
extern BYTE LogIndex,LogNum;

extern LONG	lStartPosition;

extern int  Cnt10msec;
extern BYTE fIsotonicMode, fIsotonicModeInit;

extern BYTE bPCCommFailCnt;
extern BYTE FPCCommFail;

extern void UpdateTrainingParameterLeft(void);
extern void UpdateTrainingParameterRight(void);
extern void ConsoleProcess(void);
extern void UpdateLoggingData(BYTE bOffset);

void InitIsoKineticVar(void);

union SystemStatus
{
	UWORD wStatus;
	struct
	{
	  UWORD Status 		: 4;
	  UWORD ExerCntTick	: 1;
	  UWORD SensorStatus	: 4;
	  UWORD Estop: 1;
	}var;
};

typedef struct 
{
	int *ParameterVariable;
	int ParameterAttr;
}ParameterTableType;


extern union SystemStatus  SysStatus;	   
extern WORD	
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


 
#define HOME_SPEED				600
#define MAX_LOAD
//#define ORIGIN_TORQUE			20		// jjw test
#define ORIGIN_AWAY_SPEED		600


#endif /* __ISOKINETIC_H__ */