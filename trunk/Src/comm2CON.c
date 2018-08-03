 /***********************************************
 * NAME    : comm485.c      	                    *
 * Version : 17.AUG.2003                        *
 ***********************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include "mydefs.h"
#include "comm_USART.h"

UINT String2UINTData ( unsigned char *string ) ;
BYTE String2ByteData ( unsigned char *string ) ;
int ShortData2String ( unsigned char *string, unsigned short data ) ;
int ByteData2String	( unsigned char *string, unsigned short data );




#define DfASCIIBase	0x30 
/*------------------------ ASCII Code of Hex code ----------------------*/
/* 0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F*/
const unsigned char ASCII_TBL[ASCII_TBL_SIZE] =
   					{ 0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46,
   						0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56} ;
/* char code : DfCharBase = 0x30À» »©¾ßÇÔ */   					
const unsigned char I_ASCII_TBL[I_ASCII_TBL_SIZE] = { 0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,10,11,12,13,14,15,
									16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 } ;

BYTE String2ByteData ( unsigned char *string )
{
	BYTE data ;
	BYTE a,b ;
	
	a = I_ASCII_TBL[string[0]-DfASCIIBase];
	b = I_ASCII_TBL[string[1]-DfASCIIBase];
	data = (BYTE)(a<<4) + (BYTE)(b) ;
	
	return data ;
}		

UINT String2UINTData ( unsigned char *string )
{
	UINT data ;
	BYTE a,b,c,d ;
	
	a = I_ASCII_TBL[string[0]-DfASCIIBase];
	b = I_ASCII_TBL[string[1]-DfASCIIBase];
	c = I_ASCII_TBL[string[2]-DfASCIIBase];
	d = I_ASCII_TBL[string[3]-DfASCIIBase];
	data = (UINT)(a <<12) + (UINT)(b<<8) + (UINT)(c<<4) + (UINT)(d) ;
	
	return data ;
}		

int ShortData2String	( unsigned char *string, unsigned short data )
{
	string[0] = ASCII_TBL[(unsigned char)( (data>>12 ) & 0x0f)] ;
	string[1] = ASCII_TBL[(unsigned char)( (data>>8 ) & 0x0f)] ;
	string[2] = ASCII_TBL[(unsigned char)( (data>>4 ) & 0x0f)] ;
	string[3] = ASCII_TBL[(unsigned char)( data & 0x0f)] ;
	
	return 4;
}

int ByteData2String	( unsigned char *string, unsigned short data )
{
	string[0] = ASCII_TBL[(unsigned char)( (data>>4 ) & 0x0f)] ;
	string[1] = ASCII_TBL[(unsigned char)( data & 0x0f)] ;
	
	return 2;
}

BYTE Hex2ToByte ( BYTE *string )
{
	BYTE data ;
	BYTE a,b;
	
	a = I_ASCII_TBL[string[0]-ASCII_BASE];
	b = I_ASCII_TBL[string[1]-ASCII_BASE];
	data = (BYTE)(a<<4) + (BYTE)(b) ;
	
	return data ;
}		

void ByteToHex2 ( BYTE *string,BYTE data )
{
	string[0] = ASCII_TBL[data>>4];
	string[1] = ASCII_TBL[data&0x0F];
}		

void Int2String	( unsigned char *string, int data )
{
	string[0] = ASCII_TBL[(unsigned char)( (data>>28) & 0x0f)] ;
	string[1] = ASCII_TBL[(unsigned char)( (data>>24) & 0x0f)] ;
	string[2] = ASCII_TBL[(unsigned char)( (data>>20) & 0x0f)] ;
	string[3] = ASCII_TBL[(unsigned char)( (data>>16) & 0x0f)] ;
	string[4] = ASCII_TBL[(unsigned char)( (data>>12) & 0x0f)] ;
	string[5] = ASCII_TBL[(unsigned char)( (data>>8 ) & 0x0f)] ;
	string[6] = ASCII_TBL[(unsigned char)( (data>>4 ) & 0x0f)] ;
	string[7] = ASCII_TBL[(unsigned char)(  data      & 0x0f)] ;
}
int String2Int ( unsigned char *string )
{
	UINT data ;
	BYTE a[8] ;
	int i;
	
	for (i=0;i<8;i++)
	{
		if (string[i] < '0') {
			a[i] = 0;
		}
		else if (string[i] <= '9') {
			a[i] = string[i] - '0';
		}
		else if (string[i] < 'A') {
			a[i] = 0;
		}
		else if (string[i] < 'F') {
			a[i] = 10 + string[i]-'A';
		}
		else {
			a[i] = 0;
		}
	}
	data = (UINT)(a[0] <<28) + (UINT)(a[1]<<24) + (UINT)(a[2]<<20) + (UINT)(a[3]<<16) + (UINT)(a[4] <<12) + (UINT)(a[5]<<8) + (UINT)(a[6]<<4) + (UINT)(a[7]) ;
	
	
	return (int)data ;
}		
int Data2String	( unsigned char *string, UINT data )
{
	string[0] = ASCII_TBL[(unsigned char)( (data>>12 ) & 0x0f)] ;
	string[1] = ASCII_TBL[(unsigned char)( (data>>8 ) & 0x0f)] ;
	string[2] = ASCII_TBL[(unsigned char)( (data>>4 ) & 0x0f)] ;
	string[3] = ASCII_TBL[(unsigned char)( data & 0x0f)] ;
	return 4;
}
UINT String2Data ( unsigned char *string )
{
	UINT data ;
	BYTE a,b,c,d ;
	
	a = I_ASCII_TBL[string[0]-ASCII_BASE];
	b = I_ASCII_TBL[string[1]-ASCII_BASE];
	c = I_ASCII_TBL[string[2]-ASCII_BASE];
	d = I_ASCII_TBL[string[3]-ASCII_BASE];
	data = (UINT)(a <<12) + (UINT)(b<<8) + (UINT)(c<<4) + (UINT)(d) ;
	
	return data ;
}		

UINT String2Data_5bit ( unsigned char *string )
{
	UINT data ;
	BYTE a,b,c ;
	
	a = I_ASCII_TBL[string[0]-ASCII_BASE];
	b = I_ASCII_TBL[string[1]-ASCII_BASE];
	c = I_ASCII_TBL[string[2]-ASCII_BASE];
	data = (UINT)(a <<10) + (UINT)(b<<5) + (UINT)(c) ;
	
	return data ;
}		            
BYTE Data2DecString	( BYTE *string, int data, BYTE digit, BYTE chData )
{
	int i;
	BYTE bSign = RESET;
	
	if (data <0)
	{	
		data = -data;
		bSign =SET;
	}
	
	for (i=1;i<=digit;i++)
	{
		string[digit-i] = data%10+'0';
		data /= 10;		
	}
	
	if (bSign== SET )
		string[0] = '-';
	
	if (chData) {
		//digit++;
		string[digit++] = chData;
	}
	return digit;
}
    
void CheckConsoleData(void)
{
//	BYTE bSpeed ;
	


}	
void MakeCheckSum ( unsigned char *string, BYTE dataS, BYTE dataE )
{                                                                  
	UINT sum ;                                                     
	BYTE i ;                                                       
	                                                               
	sum = 0 ;                                                      
	for( i=dataS ; i<= dataE ; i++)                                
	{                                                              
		sum += (UINT)string[i] ;                                   
	}                                                              
	                                                               
	string[dataE+2]= ASCII_TBL[ sum & 0x0f] ;                      
	string[dataE+1] = ASCII_TBL[( sum>>4) & 0x0f] ;                
}                                                                  
	                                                               
	                                                               
BYTE DoCheckSum (unsigned char *string, BYTE dataS, BYTE dataE )   
{                                                                  
	UINT sum ;                                                     
	BYTE i ;                                                       
	                                                               
	sum = 0 ;                                                      
	for( i=dataS ; i<= dataE ; i++)                                
	{                                                              
		sum += (UINT)string[i] ;                                   
	}                                                              
	                                                               
	if ((string[dataE+2] == ASCII_TBL[ sum & 0x0f] )              
		&& (string[dataE+1] == ASCII_TBL[( sum>>4) & 0x0f] ))      
	{                                                              
		return 0 ;					// Checksum OK.                
	}	                                                           
	else                                                           
		return 2 ;                                                 
}                              

