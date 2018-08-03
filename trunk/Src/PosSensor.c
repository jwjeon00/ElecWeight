// Sensor Error Frame
/*	Pos Sensor(MLX90316 Rotary Position Sensor) Datasheet
	Data Frame Structure
	A data frame consists of 10 bytes:
	* 2 start bytes (AAh followed by FFh)
	* 2 data bytes (DATA16 ? most significant byte first)
	* 2 inverted data bytes (/DATA16 - most significant byte first)
	* 4 all-Hi bytes

	The Master should send AAh (55h in case of inverting transistor) followed by 9 bytes FFh. The Slave will
	answer with two bytes FFh followed by 4 data bytes and 4 bytes FFh.

	Data16 Structure
	Normal Data16
	Angle A[13:0] with (Angle Span)/2
	Most Significant Byte 		Least Significant Byte
	MSB 					    LSB MSB							LSB
	A13 A12 A11 A10 A09 A08 A07 A06 A05 A04 A03 A02 A01 A00   0   1

	Error Data16
	Most Significant Byte 		Least Significant Byte
	MSB                         LSB MSB                         LSB
	E15 E14 E13 E12 E11 E10 E09 E08 E07 E06 E05 E04 E03 E02 E01 E00

	BIT NAME Description
	E00 0
	E01 1
	E02 F_ADCMONITOR ADC Failure
	E03 F_ADCSATURA ADC Saturation (Electrical failure or field too strong)
	E04 F_RGTOOLOW Analog Gain Below Trimmed Threshold (Likely reason: field too weak)
	E05 F_MAGTOOLOW Magnetic Field Too Weak
	E06 F_MAGTOOHIGH Magnetic Field Too Strong
	E07 F_RGTOOHIGH Analog Gain Above Trimmed Threshold (Likely reason: field too strong)
	E08 F_FGCLAMP Never occurring in serial protocol
	E09 F_ROCLAMP Analog Chain Rough Offset Compensation: Clipping
	E10 F_MT7V Device Supply VDD Greater than 7V
	E11 -
	E12 -
	E13 -
	E14 F_DACMONITOR Never occurring in serial protocol
	E15 -
*/

#include "mydefs.h"
#include "PosSensor.h"

#define START_BYTE		0xAA

#define	ANGLE_DATA_LSB_IDX	2
#define DATA16_MSB_IDX		2
#define DATA16_LSB_IDX		3
#define NDATA16_MSB_IDX		4
#define NDATA16_LSB_IDX		5

UWORD 	wPosSensorData;
UWORD 	wPosSensorErr;
int		wPosSensorDataErrCnt;
int		wPosSensorFrameErrCnt;
static BOOL CheckPosSensorDataFrame(BYTE *pPosSensorData);

void PosSensorProc(BYTE *pPosSensorData)
{
	UWORD wTemp;
	BOOL fCheckPosSensorDataFrame;
	
	fCheckPosSensorDataFrame = CheckPosSensorDataFrame(pPosSensorData);
	if (fCheckPosSensorDataFrame) {
		wTemp = MakeWord(pPosSensorData[DATA16_MSB_IDX],pPosSensorData[DATA16_LSB_IDX]);
		if (wTemp & 1) {
			wPosSensorData = wTemp >> ANGLE_DATA_LSB_IDX;
		}
		else {
			wPosSensorErr = wTemp;
			wPosSensorDataErrCnt++;
		}
	}
}

static BOOL CheckPosSensorDataFrame(BYTE *pPosSensorData)
{
	BOOL fReturnValue;
	
	fReturnValue = FALSE;
	if ((pPosSensorData[0] == BYTE_FF) && (pPosSensorData[1] == BYTE_FF) &&
		(pPosSensorData[6] == BYTE_FF) && (pPosSensorData[7] == BYTE_FF) &&
		(pPosSensorData[8] == BYTE_FF) && (pPosSensorData[9] == BYTE_FF)) {
		if ((pPosSensorData[DATA16_MSB_IDX] == ~pPosSensorData[NDATA16_MSB_IDX]) && (pPosSensorData[DATA16_LSB_IDX] == ~pPosSensorData[NDATA16_LSB_IDX])) {
			fReturnValue = TRUE;
		}
		else {
			wPosSensorFrameErrCnt++;
		}
	}
	else {
		wPosSensorFrameErrCnt++;
	}
	
	return fReturnValue;
}