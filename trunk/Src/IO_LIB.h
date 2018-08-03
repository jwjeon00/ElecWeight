/***********************************************
 * NAME    : IO_LIB.h      	                    *
 * Version : 30.OCT.2005                        *
 ***********************************************/

#ifndef __IO_LIB_H__
#define __IO_LIB_H__

#include <stdint.h>

#define LPF_SHIFT				16
#define MAX_ADC 	0x0fff  // 12bit
#define TIM_100MSEC				100
#define TIM_500MSEC				500
#define TIM_1000MSEC			1000
#define TIM_2000MSEC			2000
	 
extern uint16_t ADC_Value; //ADC Readings
extern BYTE FlFilterInit;
//extern unsigned int LoadValAcquired,LoadValAcquiredOrg, LoadDataFiltered, LoadDataFiltered_10x;
extern unsigned int LoadDataAcquired, LoadDataFiltered, LoadDataFiltered_10x ;

extern void getLoadData( void );
extern void LPF(int *out, int in, int *prev, int gain) ;
extern void getPosData(void);
extern void CPU_LEDToggle(void);
extern void CPU_LEDOff(void);
extern void CPU_LEDOn(void);
extern void ServoSpdModeOn(void);
extern void ServoSpdModeOff(void);
extern void ServoSVOn(void);
extern void ServoSVOff(void);
extern void ServoDirFwd(void);
extern void ServoDirRev(void);
extern void ServoSVStopOn(void);
extern void ServoSVStopOff(void);
extern void ServoAlmLedOn(void);
extern BOOL getEstopState(void);
extern BOOL getLoLimitState(void);
extern BOOL getHiLimitState(void);
#endif /*__IO_LIB_H__*/

