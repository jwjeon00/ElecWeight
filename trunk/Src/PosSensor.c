#include "mydefs.h"
#include "PosSensor.h"

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
		wTemp = MakeWord(pPosSensorData[2],pPosSensorData[3]);
		if (wTemp & 0x0001) {
			wPosSensorData = wTemp >> 2;
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
	if ((pPosSensorData[0] == 0xFF) && (pPosSensorData[1] == 0xFF) &&
		(pPosSensorData[6] == 0xFF) && (pPosSensorData[7] == 0xFF) &&
		(pPosSensorData[8] == 0xFF) && (pPosSensorData[9] == 0xFF)) {
		if ((pPosSensorData[2] == ~pPosSensorData[4]) && (pPosSensorData[3] == ~pPosSensorData[5])) {
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