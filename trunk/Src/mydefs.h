/***********************************************
 * NAME    : mydefs.h      	                    *
 * Version : 17.AUG.2003                        *
 ***********************************************/


//************************************************************************
//
//	Type Definitions for Variable Declarations			 
//
//************************************************************************

#ifndef __MYDEF__H__
#define __MYDEF__H__

typedef char                CHAR;
typedef unsigned char		BYTE;
typedef signed char			SBYTE;
typedef unsigned int		UINT;
typedef unsigned long		ULONG;
typedef unsigned long int	ULINT;
typedef long int			LONG;
typedef unsigned short 		UWORD ;
typedef signed short 		WORD ;
typedef char		 		BOOL ;
/*typedef unsigned char		U8;
typedef unsigned short		U16;
typedef unsigned long		U32;*/
typedef char			S8;
typedef short			S16;
typedef long			S32;
typedef int             INT;
typedef float			FLOAT;


//************************************************************************
//
//	INT Data Bit Field Define
//
//************************************************************************

typedef union
{
	UINT	value;
	struct
	{
		UINT	b0   : 1;
		UINT	b1   : 1;
		UINT	b2   : 1;
		UINT	b3   : 1;
		UINT	b4   : 1;
		UINT	b5   : 1;
		UINT	b6   : 1;
		UINT	b7   : 1;
		UINT	b8   : 1;
		UINT	b9   : 1;
		UINT	b10  : 1;
		UINT	b11  : 1;
		UINT	b12  : 1;
		UINT	b13  : 1;
		UINT	b14  : 1;
		UINT	b15  : 1;
	} Bit;
} INTField;

//************************************************************************
//
//	INT Data Bit Field Define
//
//************************************************************************

typedef union
{
	UINT	value;
	struct
	{
		UINT	b0   : 1;
		UINT	b1   : 1;
		UINT	b2   : 1;
		UINT	b3   : 1;
		UINT	b4   : 1;
		UINT	b5   : 1;
		UINT	b6   : 1;
		UINT	b7   : 1;
	} Bit;
} BYTEField;


//************************************************************************
//
//	LONG Data Bit Field Define
//
//************************************************************************
typedef	union
{
	ULONG	Value;
	BYTE	Byte[4];
} LONGField;

#define LoByte(data)	((BYTE)(data))
#define HiByte(data)	((BYTE)((data)>>8))

#define	MakeWord(hi, lo) (UWORD)(((UWORD)(LoByte(hi)) << 8) + ((lo) & 0xff))

#define min(x1,x2) (((x1)<(x2))? (x1):(x2))
#define max(x1,x2) (((x1)>(x2))? (x1):(x2))
#define absolute(x)	((x)>0 ? (x) : -(x)) 
	
#define TRUE 	(1)
#define FALSE 	(0)
#endif /*__MYDEF__H__*/