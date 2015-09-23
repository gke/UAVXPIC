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
	case ESCLRCI2C:
		#ifdef QUADROCOPTER
			T580ESCs(I2CESCLimit(PW[FrontC]), I2CESCLimit(PW[LeftC]), I2CESCLimit(PW[RightC]), I2CESCLimit(PW[BackC]));
		#endif // QUADROCOPTER
		break;
	case ESCYGEI2C:
		for ( m = 0 ; m < NO_OF_I2C_ESCS ; m++ )
		{
			ESCI2CStart();
				r = WriteESCI2CByte(0x62 + ( m*2 ) ); // one cmd, one data byte per motor
				r += WriteESCI2CByte( I2CESCLimit(PW[m]) >> 1 );
		  	ESCI2CStop();
			ESCI2CFail[m] = r;
		}
		break;
 	case ESCHolger:
		for ( m = 0 ; m < NO_OF_I2C_ESCS ; m++ )
		{
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

// LotusRC T580 I2C ESCs

enum T580States { T580Starting = 0, T580Stopping = 1, T580Constant = 2 };

boolean T580Running = false;
uint32 T580Available = 0;

void WriteT580ESC(uint8 a, uint8 s, uint8 d2) {

	ESCI2CStart();
		WriteESCI2CByte(a);
		WriteESCI2CByte(0xa0 | s );
		WriteESCI2CByte(d2);
	ESCI2CStop();

} // WriteT580ESC

void WriteT580ESCs(int8 s, uint8 f, uint8 l, uint8 r, uint8 b) {

    if ( ( s == T580Starting ) || ( s == T580Stopping ) ) {
        WriteT580ESC(0xd0, s, 0);
        WriteT580ESC(0xd2, s, 0);
        WriteT580ESC(0xd4, s, 0);
        WriteT580ESC(0xd6, s, 0);
    } else {
        WriteT580ESC(0xd0, s, l);
        WriteT580ESC(0xd2, s, f);
        WriteT580ESC(0xd4, s, b);
        WriteT580ESC(0xd6, s, r);
    }

} // WriteT580ESCs

void T580ESCs(uint8 f, uint8 l, uint8 r, uint8 b) {

    static boolean Run;
    static uint8 i;

    Run = ((int16)f+b+r+l) > 0;

    if ( T580Running )
        if ( Run )
            WriteT580ESCs(T580Constant, f, l, r, b);
        else 
		{
            WriteT580ESCs(T580Stopping, 0, 0, 0, 0);
			Delay1mS(2);
            for ( i = 0; i < (uint8)50; i++ ) 
			{
				WriteT580ESCs(T580Constant, 0, 0, 0, 0);
				Delay1mS(2);
			}
        }
    else
        if ( Run ) 
		{
        	for ( i = 0; i < (uint8)50; i++ )
			{ 
				WriteT580ESCs(T580Constant, 0, 0, 0, 0);
				Delay1mS(2);
			}
            WriteT580ESCs(T580Starting, 0, 0, 0, 0);
        }
        else
            WriteT580ESCs(T580Constant, f, l, r, b);

    T580Running = Run;

} // T580ESCs





