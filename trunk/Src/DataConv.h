/***********************************************
 * NAME    : DataConv.h      	                    *
 * Version : 06.Jul.2009                        *
 ***********************************************/

#ifndef __DATA_CONV_H__
#define __DATA_CONV_H__

#include "mydefs.h"

#define LP_OFF_LENGTH		3000   // 0.1mm, 3550      // 수직 중심선 위치 규정 
#define LP_ARM_LENGTH		8200.  // 0.1mm

/*****************  장비 사양  ****/

#define MAIN_REDUCTION_RATIO		( 100 )								// 감속기 감속비
#define PRESS_REDUCTION_RATIO		( 4.121 )		// 감속기 * 체인 시스템
#define EXTENSION_REDUCTION_RATIO   ( 0.750 )		// 감속기 * 체인 시스템
#define SERVO_ELECTRIC_GEAR_RATIO	( 8750./1000.)			// PE-702 설정값 / PE-703설정		---  enc수가 3000인 경우 10500
#define SERVO_ENC_SIZE				( 2500.*4. )			// 서보 사양 , 4체배
#define PULSE_SCALER_MAIN_REDUCER   ( SERVO_ENC_SIZE * MAIN_REDUCTION_RATIO / SERVO_ELECTRIC_GEAR_RATIO/360./100. ) 		//	pulse / 0.01deg , 3.1746
#define PULSE_SCALER_LP				( PULSE_SCALER_MAIN_REDUCER * PRESS_REDUCTION_RATIO )   		//	pulse / 0.01deg ,   13.083
#define PULSE_SCALER_LE				( PULSE_SCALER_MAIN_REDUCER * EXTENSION_REDUCTION_RATIO )		//	pulse / 0.01deg ,	2.381
#define ANGLE_SCALER_LP				( 1./PULSE_SCALER_LP )											//  0.01deg / pulse	,	.076438	
#define ANGLE_SCALER_LE				( 1./PULSE_SCALER_LE )											//  0.01deg / pulse ,	.42


extern void conv_angle_Force_Torque (void);
extern int checkHysteresis ( int Data, int *UpperOld, int *LowerOld, int *DataOld, int MovingBand );
extern	int CurSpdLP, CurSpdLE;

extern int  LPForceScale_MUL, LPForceScale_DIV, OffsetPos ;
extern float AngLPArm;

extern int CurPosLP, CurForceLP, LPArmMoment, LEArmMoment;
extern int CurAngLE, CurTqLE, RealCurTqLE; 

extern BYTE LETqScale_MUL,	LETqScale_DIV ;
extern BYTE OperMode;
extern int TqLftLE, TqRgtLE;

extern float AngScaleLE;
extern float fLPForceScale, fLETqScale,fLPAngScale, fLEAngScale ;
extern UINT MidPointLoadVal,InitMidPointLoadVal ;



#endif /* __DATA_CONV_H__ */