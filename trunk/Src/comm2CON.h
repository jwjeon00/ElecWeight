//#include "head.h"
#include "mydefs.h"

#ifndef __COMM2CON_H__
#define __COMM2CON_H__

#define DFSPCSpeedUp 1
#define DFSPCSpeedDn 2

extern void comm2CON (void);
extern void CheckConsoleData(void);


#define		Df485Terminator		Df485CCodeCR
#define		Df485ErrRCVTime		0x01		//수신대기시간 초과
#define		Df485ErrRCVStrOver		0x02		//수신 문자수 초과
#define		Df485_RCV_OverTime		7			//수신한계시간, unit : 10ms



#define	DfCommNull			'0'

// Group ID
#define DfCommIncCtl		'0'
#define DfCommSpeedCtl		'1'
#define DfCommMon			'2'


// Incline control
#define DfCommTgtGrade		'0' 
#define DfCommCurGrade		'1'
#define DfCommUpLimit		'2'
#define DfCommDnLimit		'3'
#define DfCommNoIncPulse	'4'
#define DfCommIncZeroSet	'5'

// Speed Control
#define	DfCommTgtSpeed		'0'
#define DfCommCurSpeed		'1'
#define DfCommAutoSpeed		'2'

// Monitoring
#define DfCommADValue		'0'
#define DfCommLoadZero		'1'

#define DfMonActHeat 		0x01
#define DfMonUpLimit 		0x02
#define DfMonDnLimit 		0x04



#define DfCommRFlagIdx			1
#define DfCommRTgtSpeedIdx		5
#define DfCommRTgtGradeIdx		9
#define DfRCheckSumStart		1
#define DfRCheckSumEnd			12
#define DfCommRStringLenth		16
#define DfCommRStringEnd		(DfCommRStringLenth-1)

#define DfCommTFlag1Idx			1
#define DfCommTFlag2Idx			5
#define DfCommTCurSpeedIdx		9
#define DfCommTCurGradeIdx		13
#define DfcommTUserWeight		17
#define DfTCheckSumStart		1
#define DfTCheckSumEnd			20
#define DfCommTStringLenth		24
#define DfCommTStringEnd		(DfCommTStringLenth-1)


#define DfCheckSumStart		1
#define DfDataStart			3
#define DfCheckSumEnd		6
#define DfCommStringLenth	10
#define DfCommStringEnd		(DfCommStringLenth-1)


extern int ShortData2String	( unsigned char *string, unsigned short data );
extern int ByteData2String	( unsigned char *string, unsigned short data );
extern BYTE	Hex2ToByte(BYTE *);
extern void 	ByteToHex2 ( BYTE *string,BYTE data );
extern int	Data2String(BYTE *,UINT refSpeed);
extern UINT String2Data 	( unsigned char *string );
extern	void Int2String	( unsigned char *string, int data );
extern	int String2Int 	( unsigned char *string );
extern BYTE Data2DecString	( BYTE *string, int data, BYTE digit, BYTE chData );
#endif /*__COMM2CON_H__*/


