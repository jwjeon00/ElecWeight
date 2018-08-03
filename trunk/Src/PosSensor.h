/***********************************************
 * NAME    : PosSensor.h      	                    *
 * Version : 02.Aug.2018                        *
 ***********************************************/

#ifndef __POS_SENSOR_H__
#define __POS_SENSOR_H__

#include "mydefs.h"

extern UWORD wPosSensorData;
extern UWORD wPosSensorErr;

extern void PosSensorProc(BYTE *pPosSensorData);

#endif /* __POS_SENSOR_H__ */