/*******************************************************************

	 TRXL_100.c            : Program for MM5050
	 Ver. 1.00
	 date : 2005.10.9

********************************************************************

*/


#include <string.h>
#include <stdio.h>
#include "mydefs.h"
//#include "option.h"
//#include "AT91SAM7S.h"

//#include "mct_ext.h"
//#include "mm_ext.h"

//#include "head.h"
//#include "main_ext.h"
//#include "run_ext.h"
//#include "comm2CON.h"
//#include "SpdControl.h"
#include "TIK_CON.h"
#include "ServoControl.H"
#include "comm_USART.h"
#include "IO_Lib.h"
#include "LoadSensor.h"
//#include "TWI_Lib.h"
//#include "string.h"
//#include "stdio.h"
#include "DataConv.h"
#include "Isokinetic.h"


unsigned char OnOff;

void debugOut (void);

//TimerTick contains all rutines which depend on time
extern int invControlState;
extern BYTE	invCurrData;
extern WORD wUart0State;
extern LONG lPulseCnt[2];

extern double Kp_term, Ki_term, Kd_term, control, control_incr, err, load_value;

void Scheduler(void)
{

//	static BYTE HalfSecCyc, OneSecCyc, HunMillSecs=0, k;
//	static BYTE fl, tFl, tcnt, Interval_mil;
//		const unsigned char  tString[] = "1234";
//	unsigned char	*pString; 
//	pString = Uart2_txBuff ;
	
	CommonVarInit() ;
//	TimerScheduler() ;
	CommServo() ;
	
	ServoControl();
}
void CommonVarInit(void)
{
	// 반응속도 때문에 LoadDataAcquired를 사용해야 함
	LoadData = LoadDataAcquired/10; // by james
}


//BYTE DebugOutIdx;

void debugOut (void)
{
	Uart2_txBuffSize = sprintf((char*)(Uart2_txBuff),"%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f\r\n",
		LoadDataAcquired, Kp_term, Ki_term, Kd_term, control, control_incr, lTotalPositionCount, isotonicLoad, err, load_value);

	Uart2_TransmitStart(Uart2_txBuff, Uart2_txBuffSize);

/*
	static BYTE outIdx=FALSE;
	extern UINT wLoadSensorData[4];

	//BYTE Sensor_ID;
	switch ( DebugOutIdx )
	{
		case 0 :		
		{
			if (outIdx ) {
				outIdx = FALSE;	
			
				strcpy((char*)Uart2_txBuff,"R : ");
				Uart2_txBuffSize = 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurAngLE[RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurSpdLE[RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurTqLE[RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurPosLP[RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurSpdLP[RIGHT_SIDE],4,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurForceLP[RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,lTotalPositionCount[RIGHT_SIDE],6,' ');
			}
			else {
				outIdx = TRUE;
// 2010/07/13 JJW
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,lTotalPositionCountOut[0],6,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoCmdPos[RIGHT_SIDE],7,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoLoadRate[RIGHT_SIDE],6,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,jointSel,2,':');
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,(SysStatus.var.StatusR<<12)|(SysStatus.var.StatusL<<8)|(servoState[RIGHT_SIDE]<<4)|(servoState[LEFT_SIDE]<<0));
				Uart2_txBuff[Uart2_txBuffSize++] = ':';
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SensorState);
				Uart2_txBuff[Uart2_txBuffSize++] = CR;

				Uart2_TransmitStart();
			}
			break;
		}
		case 1 :
		{
			if (outIdx ) {
				outIdx = FALSE;
// 2010/07/13 JJW				

				strcpy((char*)Uart2_txBuff,"L : ");
				Uart2_txBuffSize = 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurAngLE[LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurSpdLE[LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurTqLE[LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurPosLP[LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurSpdLP[LEFT_SIDE],4,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurForceLP[LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,lTotalPositionCount[LEFT_SIDE],6,' ');
			}
			else {
				outIdx = TRUE;

				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,lTotalPositionCountOut[1],6,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoCmdPos[LEFT_SIDE],6,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoLoadRate[LEFT_SIDE],7,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,jointSel,2,':');
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,(SysStatus.var.StatusR<<12)|(SysStatus.var.StatusL<<8)|(servoState[RIGHT_SIDE]<<4)|(servoState[LEFT_SIDE]<<0));
				Uart2_txBuff[Uart2_txBuffSize++] = ':';
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SensorState);
				Uart2_txBuff[Uart2_txBuffSize++] = CR;

				Uart2_TransmitStart();	
			}
			break;
		}

		case 2:
		{
			if (outIdx ) {
				outIdx = FALSE;
// 2010/07/13 JJW

				strcpy((char*)Uart2_txBuff,"Load Sensor : ");
				Uart2_txBuffSize = 14;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,LoadVal[LEG_EXTENSION_MODE][RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,LoadVal[LEG_PRESS_MODE][RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,LoadVal[LEG_EXTENSION_MODE][LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,LoadVal[LEG_PRESS_MODE][LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,MidPointLoadVal[LEG_EXTENSION_MODE][RIGHT_SIDE],5,' ');
			}
			else {
				outIdx = TRUE;
// 2010/07/13 JJW

				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,MidPointLoadVal[LEG_PRESS_MODE][RIGHT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,MidPointLoadVal[LEG_EXTENSION_MODE][LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,MidPointLoadVal[LEG_PRESS_MODE][LEFT_SIDE],5,' ');
				Uart2_txBuffSize += Data2String(Uart2_txBuff+Uart2_txBuffSize,SensorState);
				Uart2_txBuff[Uart2_txBuffSize++] = ' ';
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,DebugVal4,4,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,DebugVal3,4,':');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,contractionMode,1,' ');
				Uart2_txBuff[Uart2_txBuffSize++] = CR;

				Uart2_TransmitStart(); 	
			}
			break;
		}
		 
		case 3:
		{
			if (outIdx ) {
				outIdx = FALSE;
// 2010/07/13 JJW
//				Uart2_txBuffSize = sprintf((char*)(Uart2_txBuff),"SVLoad %04d %04d AngSensor : %05d %05d %05d ",
//					iServoLoadRate[RIGHT_SIDE],iServoLoadRate[LEFT_SIDE],AngSensorVal[0],AngSensorVal[1],AngSensorVal[2]);
				strcpy((char*)Uart2_txBuff,"SVLoad ");
				Uart2_txBuffSize = 7;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoLoadRate[RIGHT_SIDE],4,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iServoLoadRate[LEFT_SIDE],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"AngSensor : ");
				Uart2_txBuffSize += 12;
				
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[0],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[1],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[2],5,' ');
			}
			else {
				outIdx = TRUE;
	
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+Uart2_txBuffSize),"%05d %05d %05d %05d %1d %1d\r\n",
//					AngSensorVal[3],AngSensorVal[4],AngSensorVal[5],AngLPArm[0]
//					, DC_SEN_R_STS ,DC_SEN_L_STS);

				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[3],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[4],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,AngSensorVal[5],5,' ');
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,(int)AngLPArm[0],5,':');
				Uart2_txBuff[Uart2_txBuffSize++] = CR;

				Uart2_TransmitStart(); 	
			}
			break;
		}
		 
		case 4:
			break;
			 
		case 5:
		{
			if (outIdx ) {
				outIdx = FALSE;
//				Uart2_txBuffSize = sprintf((char*)(Uart2_txBuff),"SR:%04d SL:%04d PR:%04d PL:%04d FR:%04d ",
//					iAccSpd[0],iAccSpd[1],CurPosLP[0],CurPosLP[1],CurForceLP[0]);		
				strcpy((char*)Uart2_txBuff,"SR:");
				Uart2_txBuffSize = 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iAccSpd[0],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iAccSpd[1],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"PR:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,(jointSel == 0) ? CurPosLP[0]: CurAngLE[0],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,(jointSel == 0) ? CurPosLP[1]: CurAngLE[1],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"PRR:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurPosRel[0],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurPosRel[1],4,' ');
			}
			else {
				outIdx = TRUE;
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+Uart2_txBuffSize),"FL:%04d MdR:%02d MdL:%02d Leg:%1d DrR:%01d DrL:%01d D:%05d\r\n",
//					CurForceLP[1],fMotorDir[0],fMotorDir[1],legSel,fMeasureDir[RIGHT_SIDE],fMeasureDir[LEFT_SIDE],DebugVal);		
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"FR:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,(jointSel == 0) ? CurForceLP[0]:CurTqLE[0],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,(jointSel == 0) ? CurForceLP[1]:CurTqLE[1],4,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"MdR:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,fMotorDir[0],2,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,fMotorDir[1],2,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"Leg:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,legSel,1,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"DiR:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,fMeasureDir[RIGHT_SIDE],1,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,fMeasureDir[LEFT_SIDE],1,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"D:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,DebugVal3,5,' ');

				Uart2_txBuff[Uart2_txBuffSize++] = CR;
				Uart2_TransmitStart(); 	
			}
			break;
		}
			
		case 6:
		{
			
			if (outIdx ) {
				outIdx = FALSE;
//				Uart2_txBuffSize = sprintf((char*)(Uart2_txBuff),"SR:%04d SL:%04d PR:%04d PL:%04d FR:%04d ",
//					iAccSpd[0],iAccSpd[1],CurPosLP[0],CurPosLP[1],CurForceLP[0]);		
				strcpy((char*)Uart2_txBuff,"Jnt:");
				Uart2_txBuffSize = 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,jointSel,1,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"OJt:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,bOpJointSel,1,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"JD:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,10*FIsJointDefined[RIGHT_SIDE]+FIsJointDefined[LEFT_SIDE],4,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"AR:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurServoSenAng[RIGHT_SIDE],5,' ');
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"AL:");
				Uart2_txBuffSize += 3;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,CurServoSenAng[LEFT_SIDE],5,' ');
			}
			else {
				outIdx = TRUE;
//				Uart2_txBuffSize += sprintf((char*)(Uart2_txBuff+Uart2_txBuffSize),"FL:%04d MdR:%02d MdL:%02d Leg:%1d DrR:%01d DrL:%01d D:%05d\r\n",
//					CurForceLP[1],fMotorDir[0],fMotorDir[1],legSel,fMeasureDir[RIGHT_SIDE],fMeasureDir[LEFT_SIDE],DebugVal);		
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"PAR:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,ParkAng[jointSel][RIGHT_SIDE],5,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"L:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,ParkAng[jointSel][LEFT_SIDE],5,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"CJS:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iCurJointState[0]*10+iCurJointState[1],2,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"S:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,iJointChgSeq[0]*10+iJointChgSeq[1],2,' ');

				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"Cmd:");
				Uart2_txBuffSize += 4;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,jointChangeCmd,1,' ');
							
				strcpy((char*)Uart2_txBuff+Uart2_txBuffSize,"S:");
				Uart2_txBuffSize += 2;
				Uart2_txBuffSize += Data2DecString(Uart2_txBuff+Uart2_txBuffSize,FDCMotorRun[0]+FDCMotorRun[1]*2,1,0);
		
				Uart2_txBuff[Uart2_txBuffSize++] = CR;
				Uart2_TransmitStart(); 	
			}
			break;		
		}	 
		default :
		 	break;
	}
	*/
	
}