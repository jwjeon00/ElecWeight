#include "stm32f4xx.h"
#include "LoadSensor.h"
//#include "TWI_LIB.h"
//#include "head.h"
#include "mydefs.h"
//#include "mm_ext.h"
//#include "comm_USART.h"
#include "IO_LIB.h"
//#include "SpdControl.h"
#include <stdlib.h>
#include "Isokinetic.h"

LONGField  lMidPointLoadVal;
BYTE cmdReadSensorZeroData;
UINT MidPointLoadVal = 548 /* V기준*/, InitMidPointLoadVal;
BYTE aa[2];
BYTE fSensEEPWrt,fSensEEPWrtStart;
UINT AMP_Vref;
BYTE AMP_PGA_Gain, AMP_Out_Gain;

unsigned short AmpRegData_Old[9];

unsigned short LoadDataArr[LOAD_DATA_ARRY_NUM];
UINT CntSensorCalTerm;

union PGA309_REG    AMP_REG[2];
BYTE 	CmdForcedSensorCal = SET;

unsigned short CalibResetCnt;
unsigned short LoadDataArrDiff;
BOOL	FlSensorCalEnable;
BOOL	FlInitSensorCal;
UINT wLoadSensorData;
unsigned int LoadValAcquired,LoadValAcquiredOrg;
void ResetLoadDataArr(void);
void MaxMinAveDataArr (unsigned short *data, unsigned short *max, unsigned short *min, unsigned short *ave, unsigned short DataNo);

void SensorProc()
{
	int iTemp;
	
	wLoadSensorData = LoadValAcquired;


//	if (FlInitSensorCal ) {
//		FlInitSensorCal =RESET ;
//		SensorState |= 0x0010;
//		MidPointLoadVal = wLoadSensorData;
//		wLoadSensorData = LoadVal;				
//	}
	
	#define PRESS_ZERO_LOAD_BAND		7
	#define EXTENSION_ZERO_LOAD_BAND		7
	
	iTemp = wLoadSensorData-MidPointLoadVal;
	if (iTemp > PRESS_ZERO_LOAD_BAND )
		LoadData = wLoadSensorData -PRESS_ZERO_LOAD_BAND;
	else if (iTemp < -PRESS_ZERO_LOAD_BAND )
		LoadData = wLoadSensorData +PRESS_ZERO_LOAD_BAND;
	else
		LoadData = MidPointLoadVal;				

}

void ResetLoadDataArr(void)
{
	int i;
	
	for (i=0; i<LOAD_DATA_ARRY_NUM; i++)
	{
		LoadDataArr[i] = LoadData;
	}
}



void InitSensorRegisterData ( void)
{
	
	AMP_REG[1].Var.Zero_DAC = 0x051F;  // 0.02Vref, 292당 출력값100증가
	AMP_REG[1].Var.Gain_DAC = 0xFFFF;  // 1
	AMP_REG[1].Var.Ref_En = SET;	// Internal Reference  사용 
	AMP_REG[1].Var.Ref_Sel = RESET;	// Internal Reference 전압 , Vref = 4.096V
	AMP_Vref = 4096;					// mV
	AMP_REG[1].Var.Vex_En = SET;		// Vexe사용
	
	AMP_REG[1].Var.Coarse_Offset = 3;	// 3*Vref*0.85E-3 ,  입력값이 음수인경우를 감당하기 위해 초기 일정값 띄움, 게이지가 351과 349의 조합인경우가 최악
	
	AMP_REG[1].Var.PGA_Gain = 6;		// Front End PGA Gain  : 64
	AMP_PGA_Gain = 64;
	
	AMP_REG[1].Var.Out_Gain = 0;		// Out amp Gain  : 2 
	AMP_Out_Gain = 2;
	
	AMP_REG[1].Var.Low_Limit = 2;		// 0.144V, Under-scale Threshold  : 0.03516 Vref
	AMP_REG[1].Var.High_Limit = 5;		// 3V,  Over-scale Threshold  : 0.7324 Vref
	AMP_REG[1].Var.OU_En = SET;			// Over/Under Scale Limit enable
	
	AMP_REG[1].Var.Ext_En = SET; 		// Enable External Fault Comparator Group
	AMP_REG[1].Var.Ext_Pol = RESET;		// Force VOUT low when any comparator in the External Fault Comparator Group detects a fault
	AMP_REG[1].Var.Int_En = SET; 		// Enable Internal Fault Comparator Group
	AMP_REG[1].Var.Int_Pol = RESET;		// Force VOUT low when any comparator in the Internal Fault Comparator Group detects a fault
	
	AMP_REG[1].Var.Clk_Cfg = 1;		// No Chopping ,   온도의 영향이 적을 때 사용, 레돈이외의 경우 잘 파악해야함
	
	AMP_REG[1].Var.TempADC_Res_Sel = 3; 		//Temp ADC Resolution (Conversion time) Select
	AMP_REG[1].Var.InTemp_Mode_En = SET;			// Enable Internal Temperature Mode
	AMP_REG[1].Var.TempADC_ConCov_En = SET;			// Enable Temp ADC Continuous Conversion Mode
	
	AMP_REG[1].Var.Out_Enable_Counter = 0xff;   // VOUT Enable Timeout: 2550ms
	
	

}


void SensorAmpControl (BYTE *txBuff)
{
	static unsigned short AmpREGAddr;
	unsigned short utemp;

	do {
		AmpREGAddr =  (AmpREGAddr >= 7) ? 1 : AmpREGAddr+1 ;
		utemp = AMP_REG[1].AmpRegData[AmpREGAddr];
	} while ( (utemp == AmpRegData_Old[AmpREGAddr]) && ( AmpREGAddr != 7) );
	
			
//	if (utemp != AmpRegData_Old[AmpREGAddr] )
	AmpRegData_Old[AmpREGAddr] = utemp;
		
	txBuff[0] = 0x55;				// Initialization Byte (55h)
	txBuff[1] = 0x01;				// Register Address Command (01h)
	txBuff[2] = (BYTE)AmpREGAddr;		//	Register Address Pointer
	
	txBuff[3] = 0x055;			// Initialization Byte (55h)
	txBuff[4] = 0x04;				// Register Write Command (04h)
	txBuff[5] = (BYTE)(AmpRegData_Old[AmpREGAddr]& 0x00FF);// Register Data (8 LSBs)
	txBuff[6] = (BYTE)(AmpRegData_Old[AmpREGAddr]>>8);	// Register Data (8 MSBs)
}


#define SENSOR_CAL_TERM_COUNT		360000	// 60분, unit : 10ms
#define CAL_LOAD_DIFF_LIMIT			2		// data편차 한도
#define CAL_LOAD_AVE_DIFF_LIMIT		5		// 평균값 변동 한도
#define AMP_OFFSET_CHANGE_TERM		200		// 2초, Amp에 offset을 가했을 경우 다음 Offset설정시까지 최소 시간
#define MID_POINT_LOAD_VAL			(900)
#define AMP_MID_V_OUT_TGT			(1458)  // mV, 평형상태에서의 AMP의 목표 출력 전압


void SensorCal(void)
{
	unsigned short max, min, ave, diff;
	static unsigned short bResetCnt;
	static BYTE fChangeOffset;
	//unsigned int Vout, Coarse_Offset_Unit, iTemp,Zero_DAC_ADD;
	float Vout, Coarse_Offset_Unit, Vout_Adj; 
	unsigned int Zero_DAC_ADD,iTemp;
//	int Vout_Adj;
	BYTE Coarse_Offset_Level, AMP_Out_Gain_Old;

	//1. data적합성 check
	MaxMinAveDataArr(LoadDataArr,&max,&min,&ave, LOAD_DATA_ARRY_NUM);
	diff = max - min;

	if (FlSensorCalEnable || FlInitSensorCal )
	{
		if (bResetCnt )
		{
			bResetCnt--;
			ResetLoadDataArr();
			MaxMinAveDataArr(LoadDataArr, &max, &min, &ave, LOAD_DATA_ARRY_NUM);
			diff = max - min;
			MidPointLoadVal = ave;
		}			

		//2. 적합하면 정상적 설정 및 종료
		if ((diff <= CAL_LOAD_DIFF_LIMIT) &&  max  ) // max가 0이 아니어야 함
		{
			if (!InitMidPointLoadVal ) // Amp보정 전 원래 값
				InitMidPointLoadVal = ave;
						
			if (!fChangeOffset )
			{
			
				if (FlInitSensorCal )
				{
					if (( ave >= 150 ) && ( ave <= 2020 ))
					{
										
						Vout =(float)ave * 1621.1- (float)(AMP_REG[1].Var.Zero_DAC)*(float)(AMP_Out_Gain)*(float)(AMP_Vref)/65.536;     // uV, ave*3320000/2048,  3.32V A/D ref,  2048
						
						// step 1 : Coarse Offset 조정
		
						Coarse_Offset_Unit =  (float)(AMP_PGA_Gain * AMP_Out_Gain *  AMP_Vref) * .85;		// uV
						Coarse_Offset_Level = (BYTE)(Vout/ Coarse_Offset_Unit )+1;
						
						if (Coarse_Offset_Level <=3 )
							AMP_REG[1].Var.Coarse_Offset -= Coarse_Offset_Level;
						else
							AMP_REG[1].Var.Coarse_Offset = Coarse_Offset_Level -3 + 16;
								
						
						// step 2  : Zero DAC 설정
						
						AMP_Out_Gain_Old = AMP_Out_Gain;
						AMP_REG[1].Var.Out_Gain = 6;		// Out amp Gain  : 9 
						AMP_Out_Gain = 9;
						
						Vout_Adj = (float)AMP_MID_V_OUT_TGT - ( (Vout - Coarse_Offset_Unit*(float)Coarse_Offset_Level) )*(float)AMP_Out_Gain /(float)AMP_Out_Gain_Old/1000.;       // mV, 목표 전압까지의 조정량
						
						Zero_DAC_ADD = (unsigned short) (Vout_Adj * 65536. / (float)(AMP_Out_Gain * AMP_Vref)); 
							
						//AMP_REG[1].Var.Zero_DAC += Zero_DAC_ADD;
						AMP_REG[1].Var.Zero_DAC = Zero_DAC_ADD;
											
						bResetCnt = AMP_OFFSET_CHANGE_TERM;
						fChangeOffset = SET;
					}
		
				}
				else
				{
					Vout = (float)ave * 1.6211; 
					
					if (( ave > (MID_POINT_LOAD_VAL+50) ) && ( ave <= 2020 ) )
					{
						Vout_Adj = Vout - AMP_MID_V_OUT_TGT;
						iTemp = Zero_DAC_ADD = (unsigned short)(Vout_Adj * 65536. / (float)(AMP_Out_Gain * AMP_Vref)); 
	
//						iTemp = (ave-1000)*292/100;
						if (AMP_REG[1].Var.Zero_DAC > iTemp )
							AMP_REG[1].Var.Zero_DAC -= iTemp;  // Internal register 1, 292당 출력값100증가
	
						bResetCnt = AMP_OFFSET_CHANGE_TERM;
						fChangeOffset = SET;
//						CmdForcedSensorCal = SET;
					}
					else if ((ave < (MID_POINT_LOAD_VAL - 50) ) && ( ave >= 150 ) )
					{
						Vout_Adj = AMP_MID_V_OUT_TGT-Vout ;
						iTemp = Zero_DAC_ADD = (unsigned short)(Vout_Adj * 65536. / (float)(AMP_Out_Gain * AMP_Vref)); 
	
//						iTemp =	(1000-ave)*292/100;
						if (AMP_REG[1].Var.Zero_DAC < (65536-iTemp) )
							AMP_REG[1].Var.Zero_DAC += iTemp;  // Internal register 1, 292당 출력값100증가
						bResetCnt = AMP_OFFSET_CHANGE_TERM;
						fChangeOffset = SET;
//						CmdForcedSensorCal = SET;
					}
					else 
					{
						ResetLoadDataArr();
						MaxMinAveDataArr(LoadDataArr, &max, &min, &ave, LOAD_DATA_ARRY_NUM);
						diff = max - min;
						MidPointLoadVal = ave;
						CntSensorCalTerm = 0;
						fChangeOffset = RESET;
						FlInitSensorCal = RESET;
						FlSensorCalEnable = RESET;
						CmdForcedSensorCal = RESET;
					}						
						
					// EEPROM쓰기
				}
			}
			else
			{
				if (!bResetCnt )
				{
					ResetLoadDataArr();
					MaxMinAveDataArr(LoadDataArr, &max, &min, &ave, LOAD_DATA_ARRY_NUM);
					diff = max - min;
					MidPointLoadVal = ave;
					CntSensorCalTerm = 0;
					fChangeOffset = RESET;
					FlInitSensorCal = RESET;
					FlSensorCalEnable = RESET;
					CmdForcedSensorCal = RESET;
					// checkSensorExistance(MidPointLoadVal );
					// startWriteSensorZeroData();
				}
			}
		}
	}
	else if (CmdForcedSensorCal)	// 콘솔 SETUP Menu에 의한 강제설정
	{
		FlSensorCalEnable = SET;
	}
	else
	{
		CntSensorCalTerm++;		

		// 일정시간 동안 측정값이 유지되는 경우
		if ((diff <= CAL_LOAD_DIFF_LIMIT) &&  max  )  // max가 0이 아니어야 함
		{
			// 벨트위에 물체가 올라간 상태로 전원이 켜진 후 물체가 없어지면 다시 캘리브레이션
			if (abs(MidPointLoadVal-ave) > CAL_LOAD_AVE_DIFF_LIMIT)
			{
				CntSensorCalTerm = 0;
				FlSensorCalEnable = SET;
			}
			// calibration 설정 후 일정시간 경과후 다시 캘리브레이션
			if (CntSensorCalTerm > SENSOR_CAL_TERM_COUNT )
			{
				CntSensorCalTerm = 0;
				FlSensorCalEnable = SET;
			}
		}
	}

	// Monitoring Calibration Process	
	LoadDataArrDiff = diff;
	CalibResetCnt = bResetCnt;
}

UINT LdDataIdxIntervalTimer, LdDataIdxIntervalTime;
unsigned short LdDataIdx=0 ;
void LoadDataCollect(void)
{
	
	LdDataIdxIntervalTime = CmdForcedSensorCal ? 10 : 1800;

	if (LdDataIdxIntervalTimer >= LdDataIdxIntervalTime)
	{
		LoadDataArr[LdDataIdx] = LoadData;
		if (LdDataIdx < LOAD_DATA_ARRY_NUM) {
			LdDataIdx++;
		}
		else {
			LdDataIdx = 0;
		}
		LdDataIdxIntervalTimer = 0;
	}
	else
	{
		LdDataIdxIntervalTimer++;
	}	


}

void MaxMinAveDataArr (unsigned short *data, unsigned short *max, unsigned short *min, unsigned short *ave, unsigned short DataNo)
{
	UINT temp;
	BYTE i;

	*max = 0;
	*min = 65535;
	temp = 0;

	for (i=0; i<DataNo; i++)
	{
		if(data[i]<*min)
			*min =data[i];
		if(data[i]>*max)
			*max =data[i];
		temp += data[i];
	}

	*ave = temp/DataNo;

}