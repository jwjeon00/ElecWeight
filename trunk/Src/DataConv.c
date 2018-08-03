#include "stm32f4xx.h"
#include "mydefs.h"
#include "IO_LIB.h"
#include <stdlib.h>
#include "ServoControl.H"
#include "math.h"
#include "Isokinetic.h"
#include "DataConv.h"
#include "LoadSensor.h"

void conv_angle_Force_Torque (void);
int checkHysteresis ( int Data, int *UpperOld, int *LowerOld, int *DataOld, int MovingBand );
int CurSpdLP, CurSpdLE;


int/* AngScaleLP_MUL, AngScaleLP_DIV,*/ LPForceScale_MUL, LPForceScale_DIV,	OffsetPos ;

int CurPosLP, CurForceLP, LPArmMoment, LEArmMoment;
int CurAngLE, CurTqLE, RealCurTqLE; 

BYTE LETqScale_MUL,	LETqScale_DIV ;
BYTE OperMode;
int TqLftLE, TqRgtLE;

//float  AngLPArm;

float AngScaleLE;
float fLPForceScale, fLETqScale,fLPAngScale, fLEAngScale ;

//int	iServoIncrement[4];
	
//int iServoCmdPos;
//UINT InitMidPointLoadVal ;

#define LEG_PRESS 0 

#define LP_MOV_BAND		6			// .1kg
#define LE_MOV_BAND		4
//#define GRVTY_CORR_ANG	(1.4142)		//  1/cos(ang)값, 45인경우 수평으로부터의 각도
#define GRVTY_CORR_ANG	(1.)		//  1/cos(ang)값, 90인경우 수평으로부터의 각도






/**********************************/

void conv_angle_Force_Torque (void)
{
	static int ForceLPOld, ForceLPUpperOld, ForceLPLowerOld ;
	static float fPosLPOld, fPosLPOlder, fPosLP3rdOlder, fPosLP4thOlder,fCurPosLP;
	int  tempForceTorq ;

	float fTempAng;
	float fSinVal;

	int iServoCmdPulse;
	
	
	/********** LEG Press *************/
	iServoCmdPulse  = lTotalPositionCount ;		// 자체 연산한 값

	AngLPArm = (float)( -iServoCmdPulse - OffsetPos)*ANGLE_SCALER_LP*fLPAngScale;
	
	fTempAng = (float)AngLPArm * 1.745329e-4;     //   1.745329e-4 = pi / 18000. 
	fSinVal = sin(fTempAng);
	 
	fCurPosLP =LP_OFF_LENGTH - LP_ARM_LENGTH * fSinVal ;
	CurPosLP =(int)fCurPosLP;

	if ((SensorState & 0x1010) == 0x1010) 	
	{	
		tempForceTorq = (int)((LoadData + LPArmMoment*fSinVal
						-(float)MidPointLoadVal)*(float)fLPForceScale) ;      // 1N
		CurForceLP = checkHysteresis ( tempForceTorq, &(ForceLPUpperOld), &(ForceLPLowerOld), &(ForceLPOld), LP_MOV_BAND );
	}				
	else
	CurForceLP = 0;
	
	CurSpdLP = (int)((fCurPosLP - fPosLP4thOlder) * 50.) ;			// 10ms간격이므로  100배 함
	fPosLP4thOlder = fPosLP3rdOlder;
	fPosLP3rdOlder = fPosLPOlder;
	fPosLPOlder = fPosLPOld;
	fPosLPOld = fCurPosLP;
	
	curPos = CurPosLP ;
	curSpd = CurSpdLP;
	curTrq = CurForceLP;  

}


int checkHysteresis ( int Data, int *UpperOld, int *LowerOld, int *DataOld, int MovingBand )
{
	int iTemp;
	
	iTemp = MovingBand/2;

	*UpperOld = *LowerOld + MovingBand ;
	
	if (Data > *UpperOld)
	{
		*DataOld = *UpperOld = Data;
		*LowerOld = *UpperOld - MovingBand ;
	}
	else if (Data < *LowerOld)
	{
		*DataOld = *LowerOld = Data;
		*UpperOld = *LowerOld + MovingBand ;
	}
	else if (( Data < iTemp) && ( Data > -iTemp) ) 
		Data = 0;
	else
		Data = *DataOld ;		
		
	return Data ;

}


