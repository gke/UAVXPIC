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

// Temperature Burr-Brown TMP101

#include "uavx.h"


void GetTemperature(void);
void InitTemperature(void);

int16 AmbientTemperature;

#ifdef INC_TEMPERATURE

/* NOT IN PIC VERSION

#define TMP100_MAX_ADC 	4095 		// 12 bits
		
#define TMP100_WR		0x96 		// Write
#define TMP100_RD		0x97 		// Read	
#define TMP100_TMP		0x00		// Temperature
#define TMP100_CMD		0x01 	
#define TMP100_LOW		0x02 		// Alarm low limit
#define TMP100_HI		0x03 		// Alarm high limit
#define TMP100_CFG		0b00000000	// 0.5 deg resolution continuous

void GetTemperature(void)
{

  ReadI2Ci16v(TMP100_RD, &AmbientTemperature, 1, true); // b1, b0

  // Top 9 bits 0.5C res. scale to 0.1C
  AmbientTemperature = SRS16(AmbientTemperature, 7) * 5;	 
  if ( AmbientTemperature > Stats[MaxTempS])
    Stats[MaxTempS] = AmbientTemperature;
  else
    if ( AmbientTemperature < Stats[MinTempS] )
      Stats[MinTempS] = AmbientTemperature;

} // GetTemperature

void InitTemperature(void)
{
  WriteI2CAtAddr(TMP100_ID, TMP100_CMD, TMP100_CFG);
  WriteI2CByte(TMP100_ID, TMP100_TMP);  // Select temperature

  GetTemperature();

} // InitTemperature

*/

#else

void InitTemperature(void)
{
	AmbientTemperature = 0;
} // InitTemperature

#endif // INC_TEMPERATURE
