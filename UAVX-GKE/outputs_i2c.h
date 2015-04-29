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

void DoI2CESCs(void);
void WriteT580ESC(uint8, uint8, uint8);
void WriteT580ESCs(int8,  uint8, uint8, uint8, uint8);
void T580ESCs(uint8, uint8, uint8, uint8);

void DoI2CESCs(void)
{
	static uint8 m;
	static uint8 r;

	// in X3D and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL.
	// ACK (r) not checked as no recovery is possible. 
	// Octocopters may have ESCs paired with common address so ACK is meaningless.
	// All motors driven with fourth motor ignored for Tricopter.

	switch ( P[ESCType] ) {
	case ESCX3D:
		#if ( defined QUADROCOPTER | defined VTCOPTER )
		ESCI2CStart();
			r = WriteESCI2CByte(0x10); // one command, 4 data bytes - may not support 6
			for ( m = 0 ; m < (uint8)NO_OF_I2C_ESCS ; m++ )
				r += WriteESCI2CByte( I2CESCLimit(PW[m]) ); 
		ESCI2CStop();
		ESCI2CFail[0] = r;
		#endif // QUADROCOPTER | VTCOPTER
		break;
	case ESCYGEI2C:
		for ( m = 0 ; m < (uint8)NO_OF_I2C_ESCS ; m++ ) {
			ESCI2CStart();
				r = WriteESCI2CByte(0x62 + ( m*2 ) ); // one cmd, one data byte per motor
				r += WriteESCI2CByte( I2CESCLimit(PW[m]) >> 1 );
		  	ESCI2CStop();
			ESCI2CFail[m] = r;
		}
		break;
 	case ESCHolger:
		for ( m = 0 ; m < (uint8)NO_OF_I2C_ESCS ; m++ ) {
			ESCI2CStart();
				r = WriteESCI2CByte(0x52 + ( m*2 )); // one command, one data byte per motor
				r += WriteESCI2CByte( I2CESCLimit(PW[m]) );
		  	ESCI2CStop();
			ESCI2CFail[m] = r;
		}
		break;
	default:
		break;
	} // switch

} // DoI2CESCs





