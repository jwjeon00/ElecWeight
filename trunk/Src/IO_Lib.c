//#include "head.h"
#include "mydefs.h"
#include "stm32f4xx_hal.h"
#include "IO_LIB.h"
//#include "mm_ext.h"
//#include "SpdControl.h"
#include "LoadSensor.h"

#define LOADDATA_DIV 	10

//extern SPI_HandleTypeDef hspi1;

uint16_t ADC_Value=0; //ADC Readings
unsigned int LoadDataAcquired, LoadDataFiltered, LoadDataFiltered_10x ;
unsigned int LoadData;
BYTE FlFilterInit;
void getLoadData( void )
{
	int gain,gain1 ;
//	unsigned int ADC_Value;
	unsigned int uiTemp;
	static int  out,out1;
	static int  prev,prev1;
	static unsigned int LoadDataFiltered_10x_Old;

	ADC_Value *= 5;
//****  Input Data	****
	if (FlFilterInit )			// 전원 켤 때 고속 수렴 처리
	{
		gain1 =  -809;   // -65536/(1+2*40) 
		gain = gain1 ;

		gain= -21;		// -65536/(1+2*1600) 
		gain1 = -9; //8;	// -65536/(1+2*2200) 		// 50us	
	}
	else			// 정규처리과정
	{
		gain= -21;		// -65536/(1+2*1600) 
		gain1 = -9; //8;	// -65536/(1+2*2200) 		// 50us	
	}


	LPF (&out, (int)ADC_Value, &prev, gain);
	uiTemp = (unsigned int)((UINT)out >> LPF_SHIFT) ;
	LoadDataAcquired = uiTemp ;
	
	LPF (&out1, (int)LoadDataAcquired, &prev1, gain1);
	uiTemp = (unsigned int)((UINT)out1 >> LPF_SHIFT) ;

	LoadDataFiltered_10x = uiTemp ;			// Final Filtered load data

	if (LoadDataFiltered_10x > ( LoadDataFiltered_10x_Old+4 ) 
		||  ( LoadDataFiltered_10x < ( LoadDataFiltered_10x_Old-4 )))
	{
		LoadDataFiltered = LoadDataFiltered_10x/10;
		LoadDataFiltered_10x_Old = LoadDataFiltered_10x;
	}
}


void LPF (int *out, int in, int *prev, int gain )
{
    int outH;
    int lTemp;
    unsigned int itemp ;
    
    outH = ((unsigned int)(*out) >> LPF_SHIFT);

    lTemp = (outH - *prev + outH - in);
    *out += gain*lTemp;
   
    itemp = 20*((unsigned int)MAX_ADC << (LPF_SHIFT)) ; // 입력값 20배
    
    if (*out > itemp) {
        *out = itemp;
    }
    else if (*out < 0) {
        *out = 0;
    }
 
    *prev = in;
}

/*
void getPosData(void)
{
	uint8_t bSpiData[2]
	switch(bPosSensorCommIndex)
	{
		case 0:
			PosSensorDisable();
		case 1:
			// 메인에서 데이터 처리시간 확보
			break;
		case 2:
			bPosSensorChannelIndex++;
			bPosSensorChannelIndex %= 6;
			PosSensorEnable(bPosSensorChannelIndex);

			break;
		case 3:
			HAL_SPI_Transmit(hspi2, uint8_t *pData, uint16_t Size, uint32_t Timeout)
			rSPI_TDR = 0xAA;
			break;
		case 4:
			rSPI_TDR = 0xFF;
			break;
		case 5:
			rSPI_TDR = 0xFF;
			break;
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			rSPI_TDR = 0xFF;
//			if (bPosSensorChannelIndex == 0) {
				bRcvData[bPosSensorCommIndex-6] = rSPI_RDR;
//			}
			break;
		case 13:
//			if (bPosSensorChannelIndex == 0) {
				bRcvData[bPosSensorCommIndex-6] = rSPI_RDR;
//			}

			fPosSensorReceived = SET;
			break;
	}
	bPosSensorCommIndex++;
	bPosSensorCommIndex %= 14;	
}

*/
void CPU_LEDToggle(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
}
void CPU_LEDOff(void)
{
	HAL_GPIO_WritePin(LED_CPU_GPIO_Port, LED_CPU_Pin,GPIO_PIN_SET);
}
void CPU_LEDOn(void)
{
	HAL_GPIO_WritePin(LED_CPU_GPIO_Port, LED_CPU_Pin,GPIO_PIN_RESET);
}

void ServoSpdModeOn(void)
{
	HAL_GPIO_WritePin(SPD_POS_GPIO_Port, SPD_POS_Pin,GPIO_PIN_RESET);
}

void ServoSpdModeOff(void)
{
	HAL_GPIO_WritePin(SPD_POS_GPIO_Port, SPD_POS_Pin,GPIO_PIN_SET);
}
void ServoSVOn(void)
{
	HAL_GPIO_WritePin(SERVO_SVON_GPIO_Port, SERVO_SVON_Pin,GPIO_PIN_SET);
}

void ServoSVOff(void)
{
	HAL_GPIO_WritePin(SERVO_SVON_GPIO_Port, SERVO_SVON_Pin,GPIO_PIN_RESET);
}
void ServoDirFwd(void)
{
	HAL_GPIO_WritePin(SERVO_PULSE_DIR_GPIO_Port, SERVO_PULSE_DIR_Pin,GPIO_PIN_RESET);
}
void ServoDirRev(void)
{
	HAL_GPIO_WritePin(SERVO_PULSE_DIR_GPIO_Port, SERVO_PULSE_DIR_Pin,GPIO_PIN_SET);
}
//void ServoSVAlarmRstOn(void)
//{
//	HAL_GPIO_WritePin(SERVO_ALMRST_GPIO_Port, SERVO_ALMRST_Pin,GPIO_PIN_SET);
//}
//void ServoSVAlarmRstOff(void)
//{
//	HAL_GPIO_WritePin(SERVO_ALMRST_GPIO_Port, SERVO_ALMRST_Pin,GPIO_PIN_RESET);
//}
void ServoSVStopOn(void)
{
	HAL_GPIO_WritePin(SERVO_STOP_GPIO_Port, SERVO_STOP_Pin,GPIO_PIN_SET);
}
void ServoSVStopOff(void)
{
	HAL_GPIO_WritePin(SERVO_STOP_GPIO_Port, SERVO_STOP_Pin,GPIO_PIN_RESET);
}


void ServoAlmLedOn(void)
{
	HAL_GPIO_WritePin(SERVO_ALM_GPIO_Port, SERVO_ALM_Pin,GPIO_PIN_RESET);
}
void ServoAlmLedOff(void)
{
	HAL_GPIO_WritePin(SERVO_ALM_GPIO_Port, SERVO_ALM_Pin,GPIO_PIN_SET);
}

BOOL getEstopState(void)
{
	BOOL fRetValue;
	
	if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) {
		fRetValue = SET;
	}
	else {
		fRetValue = RESET;
	}
	return fRetValue;
}

BOOL getLoLimitState(void)
{
	BOOL fRetValue;
	
	if (HAL_GPIO_ReadPin(LO_LIMIT_GPIO_Port, LO_LIMIT_Pin) == GPIO_PIN_SET) {
		fRetValue = SET;
	}
	else {
		fRetValue = RESET;
	}
	return fRetValue;
}
BOOL getHiLimitState(void)
{
	BOOL fRetValue;
	
	if (HAL_GPIO_ReadPin(HI_LIMIT_GPIO_Port, HI_LIMIT_Pin) == GPIO_PIN_SET) {
		fRetValue = SET;
	}
	else {
		fRetValue = RESET;
	}
	return fRetValue;
}