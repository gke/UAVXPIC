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

int16 ADC(uint8);
void InitADC(void);

int16 ADC(uint8 ADCChannel)
{ // all ADC reads use 5V reference
	static i16u v;

	ADCON0 = ((ADCChannel << 2) & 0b00111100) | (ADCON0 & 0b11000011);
	ADCON0bits.GO = true;

	while ( ADCON0bits.GO){};

	v.b1 = ADRESH;
	v.b0 = ADRESL;

	return ( v.i16 );
} // ADC

void InitADC()
{
	static uint8 i, c;

	OpenADC(ADC_FOSC_32 & 
          ADC_RIGHT_JUST &
          ADC_20_TAD,		// 12 TAD?
		  ADC_CH0 &
		  ADC_INT_OFF &
          ADC_VREFPLUS_VDD &
          ADC_VREFMINUS_VSS,	  
          ADCPORTCONFIG);

	SetChanADC(	0 );					// using automatic acq
	ConvertADC();

} // InitADC


