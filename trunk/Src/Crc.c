#include "mydefs.h"
#include "stm32f4xx_hal.h"
//#include "head.h"
#include "limits.h"

UWORD CRC16 (BYTE *buf, BYTE numbytes);

BYTE CRCok(BYTE *msg, BYTE len)
{
	UWORD chk;
	UWORD rcvCRC;
	BYTE retValue;
	BYTE *msgbuf = msg;

	retValue = RESET;
	if(len > 2)	{
		chk = CRC16(msg,len-2);
	 	rcvCRC = ((UWORD)*(msgbuf+(len-1)) << 8) + *(msgbuf+(len-2));
		if (chk == rcvCRC) {
			retValue = SET;
		}
	}
	return retValue;
}

#define		M16 	0xA001		// crc-16 mask

UWORD CRC16 (BYTE *buf, BYTE numbytes)
{
	UWORD crc;
	UWORD i,carrybit;
	BYTE *ptr = buf;

	crc = __UNSIGNED_SHORT_MAX__;
	do
	{
		crc = crc ^ ((UWORD)*ptr++);
		i = 8;
		do
		{
			carrybit = (crc & 0x0001) ? 1 : 0;
			crc >>= 1;
			if (carrybit) crc ^= M16;
			} while(--i);
		} while (--numbytes);

	return (crc);
}
