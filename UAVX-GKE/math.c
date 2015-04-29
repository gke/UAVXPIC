// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "uavx.h"

int16 SRS16(int16, uint8);
int32 SRS32(int32, uint8);
int16 Make2Pi(int16);
int16 MakePi(int16);
int16 Table16(int16, const int16 *);
int16 int16sin(int16);
int16 int16cos(int16);
int16 int32atan2(int32, int32);
int16 int16sqrt(int16);
int32 int32sqrt(int32);

int16 SRS16(int16 x, uint8 s)
{
	static i16u Temp;

	if ( s == (uint8)8 ) {
		Temp.i16 = x;
		return( (int16) Temp.i1 );
	} else
		return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS16

int32 SRS32(int32 x, uint8 s)
{
	static i32u Temp;

	if ( s == (uint8)8 ) {
		Temp.i32 = x;
		return( (int32)Temp.i3_1 );
	} else
		return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS32

int16 Make2Pi(int16 A) {
	while ( A < 0 ) A += TWOMILLIPI;
	while ( A >= TWOMILLIPI ) A -= TWOMILLIPI;
	return( A );
} // Make2Pi

int16 MakePi(int16 A) {
	while ( A < -MILLIPI ) A += TWOMILLIPI;
	while ( A >= MILLIPI ) A -= TWOMILLIPI;
	return( A );
} // MakePi

#pragma idata sintable
const int16 SineTable[17]={ 
	0, 50, 98, 142, 180, 212, 236, 250, 255,
	250, 236, 212, 180, 142, 98, 50, 0
   };
#pragma idata

int16 Table16(int16 Val, const int16 *T) {
	static uint8 Index,Offset;
	static int16 Temp, Low, High;

	Index = (uint8) (Val >> 4);
	Offset = (uint8) (Val & 0x0f);
	Low = T[Index];
	High = T[++Index];
	Temp = (High-Low) * Offset;

	return( Low + SRS16(Temp, 4) );
} // Table16

int16 int16sin(int16 A) {
	// A is in milliradian 0 to 2000Pi, result is -255 to 255
	static int16 	v;
	static boolean	Negate;

	A = Make2Pi(A);

	Negate = A >= MILLIPI;
	if ( Negate )
		A -= MILLIPI;

	v = Table16(((int24)A * 256 + HALFMILLIPI)/MILLIPI, SineTable);

	if ( Negate )
		v = -v;

	return(v);
} // int16sin


int16 int16cos(int16 A) {	
	// A is in milliradian 0 to 2000Pi, result is -255 to 255
	return(int16sin(A + HALFMILLIPI));
} // int16cos


#pragma idata arctan
const int16 ArctanTable[17]={
	0, 464, 785, 983, 1107, 1190, 1249, 1292, 1326,
	1352, 1373, 1391, 1406, 1418, 1429, 1438, 1446
   };
#pragma idata

int16 int32atan2(int32 y, int32 x) {	
	// Result is in milliradian
	// Caution - this routine is intended to be acceptably accurate for 
	// angles less Pi/4 within a quadrant. Larger angles are directly interpolated
	// to Pi/2. 
 
	static int32 Absx, Absy, TL;
	static int16 A;

	Absy = Abs(y);
	Absx = Abs(x);

	if ( x == 0 )
		if ( y < 0 )
			A = -HALFMILLIPI;
		else
			A = HALFMILLIPI;
	else
		if (y == 0)
			if ( x < 0 )
				A=MILLIPI;
			else
				A = 0;
		else {
			TL = (Absy * 32)/Absx;
			if ( TL < 256 )
				A = Table16(TL, ArctanTable);
			else {  // extrapolate outside table
				TL -= 256;
				A =  ArctanTable[16] + (TL >> 2);
				A = Limit(A, 0, HALFMILLIPI);
			}

			if ( x < 0 )
				if ( y > 0 ) // 2nd Quadrant 
					A = MILLIPI - A;
				else // 3rd Quadrant 
					A = MILLIPI + A;
			else
				if ( y < 0 ) // 4th Quadrant 
					A = TWOMILLIPI - A;
	}
	return(A);
} // int32atan2

int16 int16sqrt(int16 n) {
	// 16 bit numbers 
  static int16 r, b;

  r = 0;
  b = 256;
  while ( b > 0 ) {
    if ( r*r > n )
      r -= b;
    b >>= 1;
    r += b;
    }
  return(r);
} // int16sqrt

int32 int32sqrt(int32 n) {
	// 32 bit numbers 
  static int32 r, b;

  r = 0;
  b = 65536;
  while (b > 0) {
    if ( r*r > n )
      r -= b;
    b >>= 1;
    r += b;
    }
  return(r);
} // int32sqrt

