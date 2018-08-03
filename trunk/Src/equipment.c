#include "stm32f4xx_hal.h"

//#include "head.h"
#include "mydefs.h"
//#include "mm_ext.h"
#include "comm_USART.h"
#include "IO_LIB.h"
//#include "SpdControl.h"
#include <stdlib.h>
#include "ServoControl.H"
#include "math.h"
#include "Isokinetic.h"
#include "DataConv.h"

void initEquipCommonVar(void);

int ParkAng;

void initEquipCommonVar(void)
{
//	AbsPosLmtTWD = (int)(-1900.*PULSE_SCALER_LP/fLPAngScale - (float)OffsetPos);    // ���� 19.2�� ����, 
//	AbsPosLmtAWAY = (int)(1900.*PULSE_SCALER_LP/fLPAngScale - (float)OffsetPos) ;		// �Ĺ� 19.2�� ����
	AbsPosLmtTWD = -400000;    		// ���� 19.2�� ����, 
	AbsPosLmtAWAY = 400000 ;		// �Ĺ� 19.2�� ����

	LEOrgSetState = 3;
	CmdOpPause = RESET ;
	
//	fServoOn = TRUE ;
	fIsotonicMode = SET; // for degug
	fInitExercise = fInitROM = RESET;

}


/**********************************/
/* 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 */


void initEquipVar (void)
{
	OffsetPos =(int)(24199)  ; //58212;	//47290;

	ParkAng = 11174;

	fLPForceScale =  1.3434; //0.7141; //0.7719; //0.7495;     //1.499;
	fLPAngScale = 0.989;
	LPArmMoment = 130 ;

	initEquipCommonVar();

}
