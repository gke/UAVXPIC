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

int8 ReadEE(uint16);
int16 Read16EE(uint16);
int32 Read32EE(uint16);
void WriteEE(uint16, int8);
void Write16EE(uint16, int16);
void Write32EE(uint16, int32);

int8 ReadEE(uint16 a) {
	static int8 b;

	EEADR = a;
	EEADRH = a>>8;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE

int16 Read16EE(uint16 a) {
	static i16u Temp16;

	Temp16.b0 = ReadEE(a);
	Temp16.b1 = ReadEE(a + 1);

	return ( Temp16.i16 );
} // Read16EE

int32 Read32EE(uint16 a) {
	static i32u Temp32;

	Temp32.b0 = ReadEE(a);
	Temp32.b1 = ReadEE(a + 1);
	Temp32.b2 = ReadEE(a + 2);
	Temp32.b3 = ReadEE(a + 3);

	return ( Temp32.i32 );
} // Read32EE

void WriteEE(uint16 a, int8 d) {
	static int8 rd;
	static uint8 IntsWereEnabled;
	
	rd = ReadEE(a);
	if ( rd != d ) { // avoid redundant writes
		EEDATA = d;				
		EEADR = a;
		EEADRH = a>>8;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;
		
		IntsWereEnabled = InterruptsEnabled;
		DisableInterrupts;
		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);
		if ( IntsWereEnabled )
			EnableInterrupts;

		EECON1bits.WREN = false;
	}

} // WriteEE

void Write16EE(uint16 a, int16 d) {
	static i16u Temp16;

	Temp16.i16 = d;
	WriteEE(a, Temp16.b0);
	WriteEE(a + 1, Temp16.b1);

} // Write16EE

void Write32EE(uint16 a, int32 d) {
	static i32u Temp32;

	Temp32.i32 = d;
	WriteEE(a, Temp32.b0);
	WriteEE(a + 1, Temp32.b1);
	WriteEE(a + 2, Temp32.b2);
	WriteEE(a + 3, Temp32.b3); 

} // Write16EE
