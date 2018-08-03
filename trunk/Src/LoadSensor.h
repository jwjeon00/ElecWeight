/***********************************************
 * NAME    : LoadSensor.h      	                    *
 * Version : 30.May.2006                        *
 ***********************************************/

#ifndef __LOAD_SENSOR_H__
#define __LOAD_SENSOR_H__

#include "mydefs.h"

#define LOAD_DATA_ARRY_NUM	50
union PGA309_REG
{
	unsigned short AmpRegData[9];
	struct
	{
		unsigned short Temp_ADC_Out;		// Register 0: Temp ADC Output Register (Read Only, Address Pointer = 00000)
		unsigned short Zero_DAC;			// Register 1: Fine Offset Adjust (Zero DAC) Register (Read/Write, Address Pointer = 00001)
		unsigned short Gain_DAC;			// Register 2: Fine Gain Adjust (Gain DAC) Register (Read/Write, Address Pointer = 00010)
		unsigned int Linear_DAC_Set : 8;	// Register 3: Linearization DAC setting, 7-bit + sign
		unsigned int Ref_En : 1;			// Register 3: Enable/Disable Internal VREF,     
											//				0 = External Reference (disable internal reference)
											// 				1 = Internal Reference (enable internal reference)
		unsigned int Ref_Sel : 1;			// Register 3: Internal VREF Select (2.5V or 4.096V)
											//				0 = 4.096V
											//				1 = 2.5V
		unsigned int Vex_En : 1;			// Register 3: VEXC Enable
											//				1 = Enable VEXC
											//				0 = Disable VEXC
		unsigned int Vex_Sel : 1;			// Register 3: Linearization Adjust and Excitation Voltage (VEXC) Gain Select (Range1 or Range2)
											//				0 = Range 1 (-0.166VFB < Linearization DAC Range < +0.166VFB, VEXC Gain = 0.83VREF)
											//				1 = Range 2 (-0.124VFB < Linearization DAC Range < +0.124VFB, VEXC Gain = 0.52VREF)
		unsigned int :4;					// Register 3: Reserved Factory Bit: Set to zero for proper operation
		unsigned int Coarse_Offset : 5;	// Register 4: Coarse Offset Adjust on Front-End PGA, 4-bit + sign,   1LSB = (VREF)(0.85E-3)
		unsigned int :3;					// Register 4: Reserved Factory Bit: Set to zero for proper operation
		unsigned int PGA_Gain : 4;		// Register 4: Front-End PGA Gain Select, 1-of-8, and Input Mux Control
											//				GI[3] = Input Mux Control
											//				GI[2:0] = Gain Select
		unsigned int Out_Gain : 3;		// Register 4: Output Amplifier Gain Select, 1-of-7 plus internal feedback disable
		unsigned int One_Wire_Dis : 1;	// Register 4: One-Wire Disable (only valid while VOUT is enabled, for use when PRG is connected to VOUT)
											//				1 = Disable
											//				0 = Enable
		unsigned int Low_Limit : 3;		// Register 5: Under-Scale Threshold Select
		unsigned int High_Limit : 3;		// Register 5: Over-Scale Threshold Select
		unsigned int OU_En : 1;			// Register 5: Over/Under-Scale Limit Enable.
											//				1 = Enable Over/Under-Scale limits
											//				0 = Disable Over/Under-Scale limits			
		unsigned int  : 1;				// Register 5: (Reserved Factory Bit): Set to zero for proper operation
		unsigned int Int_Pol : 1;			// Register 5: Selects VOUT output polarity when Internal Fault Comparator Group detects a fault, if INTEN = 1
											//				1 = Force VOUT high when any comparator in the Internal Fault Comparator Group detects a fault
											//				0 = Force VOUT low when any comparator in the Internal Fault Comparator Group detects a fault
		unsigned int Ext_Pol : 1;			// Register 5: Selects VOUT output polarity when External Fault Comparator Group detects a fault, if INTEN = 1
											//				1 = Force VOUT high when any comparator in the External Fault Comparator Group detects a fault
											//				0 = Force VOUT low when any comparator in the External Fault Comparator Group detects a fault
		unsigned int Int_En : 1;			// Register 5: Enable Internal Fault Comparator Group (A2SAT_LO, A2SAT_HI, A1SAT_LO, A1SAT_HI, A3_VCM)
											//				1 = Enable Internal Fault Comparator Group
											//				0 = Disable Internal Fault Comparator Group
		unsigned int Ext_En : 1;			// Register 5: Enable External Fault Comparator Group (A2SAT_LO, A2SAT_HI, A1SAT_LO, A1SAT_HI, A3_VCM)
											//				1 = Enable External Fault Comparator Group
											//				0 = Disable External Fault Comparator Group
		unsigned int Clk_Cfg : 2;			// Register 5: Clocking scheme for Front-End PGA auto-zero and Coarse Offset DAC Chopping
		unsigned int  : 2;				// Register 5: (Reserved Factory Bit): Set to zero for proper operation
		unsigned int TempADC_Res_Sel : 2;	// Register 6: Temp ADC Resolution (Conversion time) Select
		unsigned int TempADC_PGA_Gain : 2;	// Register 6: Temp ADC PGA Gain Select (x1, 2, 4, or 8)
		unsigned int TempADC_InMux_Sel : 2;	// Register 6: Temp ADC Input Mux Select
		unsigned int TempADC_Ext_Ref : 2;		// Register 6: Temp ADC External Reference Select (VSA, VEXC, VREF)
		unsigned int TempADC_Int_Ref_En : 1;	// Register 6: Temp ADC internal reference enable
												//			1 = Enable Temp ADC internal reference (internal reference is 2.048V typical)
												//			0 = Disable Temp ADC internal reference (use external ADC reference; see RV[1:0])
		unsigned int InTemp_Mode_En : 1;	// Register 6: Internal Temperature Mode Enable 
											// 				1 = Enable Internal Temperature Mode (기타 설정은 datasheet 참조)
											//				0 = External Signal Mode
		unsigned int TempADC_ConCov_En : 1;	// Register 6: Enable Temp ADC Continuous Conversion Mode
											// 				1 = Continuous Conversion mode
											//				0 = Noncontinuous Conversion mode
		unsigned int TempIN_Cur_Source_En : 1;	// Register 6: TEMPIN Current source (ITEMP) Enable
											// 				1 = Enable 7μA current source, ITEMP
											//				0 = Disable 7μA current source, ITEMP
		unsigned int TempADC_START : 1;	// Register 6: Start (restart) the Temp ADC (single conversion control if CEN = 0)
											//				0 = No Start/Restart Temp ADC
											//				1 = Start/Restart Temp ADC (each write of a ‘1’ causes single conversion; 
											//						when conversion is completed ADCS = ‘0’)
		unsigned int TempADC_2X : 1;		// Register 6: Temp ADC runs 2x faster (not for internal Temp Sense Mode)
											//				0 = 1x conversion speed (6ms typical, R1, R0 = ‘00’, TEN = ‘0’, AREN = ‘0’)
											//				1 = 2x conversion speed (3ms typical, R1, R0 = ‘00’, TEN = ‘0’, AREN = ‘0’)
		unsigned int  : 2;				// Register 6: (Reserved Factory Bit): Set to zero for proper operation
		unsigned int Out_Enable_Counter : 8;	// Register 7: Output Enable Counter for One-Wire Interface/VOUT Multiplexed Mode.
		unsigned int TempADC_Delay : 4;	// Register 7: OTemp ADC Delay
		unsigned int  : 4;				// Register 7: (Reserved Factory Bit): Set to zero for proper operation
		unsigned int Alarm_Status : 16 ;		// Register 8: Over-Scale Threshold Select
	} Var;
}  ;

extern union PGA309_REG    AMP_REG[2] ;
extern BYTE 	CmdForcedSensorCal,FlSensorCal,FlFilterInit,FlInitSensorCalSet;
extern BOOL FlInitSensorCal;
extern BYTE cmdReadSensorZeroData ;

extern UINT MidPointLoadVal,InitMidPointLoadVal ;

extern BYTE fSensEEPWrt,fSensEEPWrtStart;
//extern BYTE AmpRegData[7][2];
//extern unsigned short AmpRegData[9];

extern unsigned short LoadDataArr[LOAD_DATA_ARRY_NUM] ;

extern unsigned short AmpRegData_Old[9];


extern void startReadSensorZeroData(void);
extern void startWriteSensorZeroData(void);
extern void checkReadSensorZeroDataFinish (void);
extern void checkSensorExistance(void );
extern void InitSensorRegisterData ( void);
extern void SensorAmpControl (BYTE *txBuff);
extern void SensorCal(void) ;
extern void LoadDataCollect(void);

extern unsigned short CalibResetCnt;
extern unsigned short LoadDataArrDiff;
extern unsigned int LoadData;

#endif /* __LOAD_SENSOR_H__ */