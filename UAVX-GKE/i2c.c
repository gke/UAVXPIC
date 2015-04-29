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

// I2C Routines - Compass is the limiting device at 100KHz!

#include "uavx.h"

void InitI2C(void);
void I2CStart(void);
void I2CStop(void);
boolean I2CWaitClkHi(void); // has timeout
uint8 ReadI2CByte(uint8);
uint8 WriteI2CByte(uint8);
void ShowI2CDeviceName(uint8);
uint8 ScanI2CBus(void);
uint8 ReadI2CByteAtAddr(uint8, uint8);
void WriteI2CByteAtAddr(uint8, uint8, uint8);
boolean ReadI2Ci16vAtAddr(uint8, uint8, int16 *, uint8, boolean);
boolean ReadI2Ci16v(uint8, int16 *, uint8, boolean);
boolean I2CResponse(uint8);

#define I2C_IN			1
#define I2C_OUT			0

boolean UseI2C100KHz;

// These routine need much better tailoring to the I2C bus spec.
// 40MHz 0.1uS/Cycle

#if defined(UAVX_HW)
	#define I2C_SDA_SW			PORTCbits.RC4
	#define I2C_DIO_SW			TRISCbits.TRISC4
	#define I2C_SCL_SW			PORTCbits.RC3
	#define I2C_CIO_SW			TRISCbits.TRISC3
#else
	#define I2C_SDA_SW			PORTBbits.RB6
	#define I2C_DIO_SW			TRISBbits.TRISB6
	#define I2C_SCL_SW			PORTBbits.RB7
	#define I2C_CIO_SW			TRISBbits.TRISB7
#endif // UAVX_HW

#define I2C_DATA_LOW	{I2C_SDA_SW=0;I2C_DIO_SW=I2C_OUT;}
#define I2C_DATA_FLOAT	{I2C_DIO_SW=I2C_IN;}
#define I2C_CLK_LOW		{I2C_SCL_SW=0;I2C_CIO_SW=I2C_OUT;}
#define I2C_CLK_FLOAT	{I2C_CIO_SW=I2C_IN;} 

void InitI2C(void) {
	UseI2C100KHz = false;
} // InitI2C

#if defined(I2C100KHZ)  // No 100KHz I2C devices (HMC6352)

#define T_LOW_STA		if(UseI2C100KHz){Delay10TCYx(2);}	
#define T_HD_STA		if(UseI2C100KHz){Delay10TCYx(2);}else{Delay1TCY();Delay1TCY();Delay1TCY();}	// 4.0/0.6uS
#define T_HD_DAT		if(UseI2C100KHz)Delay10TCYx(5)	// 5.0/0.0uS
#define T_SU_DAT									// 250/100nS

#define T_HIGH_R		if(UseI2C100KHz){Delay10TCYx(2);Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}	// 4.0/0.6uS	
#define T_LOW_R			if(UseI2C100KHz){Delay10TCYx(3);Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}	// 4.7/1.3uS
#define T_HIGH_W		if(UseI2C100KHz){Delay10TCYx(3);Delay1TCY();Delay1TCY();}	// 4.0/0.6uS
#define T_LOW_W			if(UseI2C100KHz){Delay10TCYx(2);Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}	// 4.7/1.3uS

#define T_HIGH_ACK_R	if(UseI2C100KHz){Delay10TCYx(3);Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}		
#define T_HIGH_ACK_W	if(UseI2C100KHz){Delay10TCYx(2);Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}
#define T_LOW_STP		if(UseI2C100KHz){Delay10TCYx(2);}	
#define T_SU_STO		if(UseI2C100KHz){Delay10TCYx(3);}else{Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}
#define T_BUF			Delay10TCYx(5)

boolean I2CWaitClkHi(void) {
	static uint8 s;

	Delay1TCY(); // setup
	Delay1TCY();
	Delay1TCY();

	I2C_CLK_FLOAT;		// set SCL to input, output a high
	s = 1;
	while( !I2C_SCL_SW )	// timeout wraparound through 255 to 0 0.5mS @ 40MHz
		if( ++s == (uint8)0 ) {
			Stats[I2CFailS]++;
			return (false);
		}
	return( true );
} // I2CWaitClkHi

void I2CStart(void) {
	static boolean r;
	
	I2C_DATA_FLOAT;
	r = I2CWaitClkHi();
	I2C_DATA_LOW;
	T_HD_STA;
	I2C_CLK_LOW;
	T_LOW_STA;

} // I2CStart

void I2CStop(void) {
	static boolean r;

	T_LOW_STP;
	I2C_DATA_LOW;
	r = I2CWaitClkHi();
	T_SU_STO;
	I2C_DATA_FLOAT;

	T_BUF;

} // I2CStop 

uint8 ReadI2CByte(uint8 r) {
	static uint8 s, d;

	I2C_DATA_FLOAT;
	d = 0;
	s = 8;
	do {
		if( I2CWaitClkHi() ) { 
			d <<= 1;
			if( I2C_SDA_SW ) d |= 1;
			T_HIGH_R;
			I2C_CLK_LOW;
			T_LOW_R;
 		} else {
			Stats[I2CFailS]++;
			return(false);
		}
	} while ( --s );

	I2C_SDA_SW = r;
	I2C_DIO_SW = I2C_OUT;
										
	if( I2CWaitClkHi() ) {
		T_HIGH_ACK_R;
		I2C_CLK_LOW;
		return(d);
	} else {
		Stats[I2CFailS]++;
		return(false);
	}
	
} // ReadI2CByte

uint8 WriteI2CByte(uint8 d) {
	static uint8 s, dd;

	dd = d;  // a little faster
	s = 8;
	do {
		if( dd & 0x80 )
			I2C_DATA_FLOAT
		else
			I2C_DATA_LOW
	
		if( I2CWaitClkHi() ) { 	
			T_HIGH_W;
			I2C_CLK_LOW;
			T_LOW_W;
			dd <<= 1;
		} else {
			Stats[I2CFailS]++;
			return(I2C_NACK);
		}
	} while ( --s );

	I2C_DATA_FLOAT;
	if( I2CWaitClkHi() )
		s = I2C_SDA_SW;
	else {
		Stats[I2CFailS]++;
		return(I2C_NACK);
	}	
	T_HIGH_ACK_W;
	I2C_CLK_LOW;

	return(s);
} // WriteI2CByte

#else

// Squeek out to 333KHz if there are no 100KHz devices installed

boolean I2CWaitClkHi(void) {
	static uint8 s;

	Delay1TCY();Delay1TCY();//Delay1TCY();Delay1TCY(); // tSU:DAT + call
	I2C_CLK_FLOAT;		// set SCL to input, output a high
	s = 0;
	while( !I2C_SCL_SW )	// timeout wraparound through 255 to 0 0.5mS @ 16MHz
		if( ++s == (uint8)0 ) {
			Stats[I2CFailS]++;
			return (false);
		}
	return( true );
} // I2CWaitClkHi

void I2CStart(void) {
	static boolean r;

	I2C_DATA_FLOAT; // should be floating after previous TBuf
	r = I2CWaitClkHi();
	I2C_DATA_LOW;
	Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY(); // tHD:STA
	I2C_CLK_LOW;
} // I2CStart

void I2CStop(void) {
	static boolean r;

	I2C_DATA_LOW;
	r = I2CWaitClkHi();
	Delay1TCY();Delay1TCY();Delay1TCY(); // tsu:STO
	I2C_DATA_FLOAT;

	Delay10TCYx(2); // TBuf

} // I2CStop 

uint8 ReadI2CByte(uint8 r) {
	static uint8 s, d;

	I2C_DATA_FLOAT;
	d = 0;
	s = 8;
	do {
		if( I2CWaitClkHi() ) { 
			d <<= 1;
			if( I2C_SDA_SW ) d |= 1;
			I2C_CLK_LOW;
 		} else {
			Stats[I2CFailS]++;
			return(false);
		}
	} while ( --s );

	I2C_SDA_SW = r;
	I2C_DIO_SW = I2C_OUT;
									
	if( I2CWaitClkHi() ) {
		I2C_CLK_LOW;
		return(d);
	} else {
		Stats[I2CFailS]++;
		return(false);
	}
	
} // ReadI2CByte

uint8 WriteI2CByte(uint8 d) {
	static uint8 s, dd;

	dd = d;  // a little faster
	s = 8;
	do {
		if( dd & 0x80 )
			I2C_DATA_FLOAT
		else
			I2C_DATA_LOW
	
		if( I2CWaitClkHi() ) { 	
			I2C_CLK_LOW;
			dd <<= 1;
		} else {
			Stats[I2CFailS]++;
			return(I2C_NACK);
		}
	} while ( --s );

	I2C_DATA_FLOAT;
	if( I2CWaitClkHi() )
		s = I2C_SDA_SW;
	else {
		Stats[I2CFailS]++;
		return(I2C_NACK);
	}	

	I2C_CLK_LOW;

	return(s);
} // WriteI2CByte

#endif // I2C100KHZ

uint8 ReadI2CByteAtAddr(uint8 d, uint8 address) {
	static uint8 data;
		
	I2CStart();
		if( WriteI2CByte(d) != I2C_ACK ) goto IRerror;
		if( WriteI2CByte(address) != I2C_ACK ) goto IRerror;
	I2CStart();
		if( WriteI2CByte(d | 1) != I2C_ACK ) goto IRerror;	
		data = ReadI2CByte(I2C_NACK);
	I2CStop();
	
	return ( data );

IRerror:
	I2CStop();
	Stats[I2CFailS]++;
	return (0);
} // ReadI2CByteAtAddr

void WriteI2CByteAtAddr(uint8 d, uint8 address, uint8 v) {
	I2CStart();	// restart
		if( WriteI2CByte(d) != I2C_ACK ) goto IWerror;
		if( WriteI2CByte(address) != I2C_ACK ) goto IWerror;
		if(WriteI2CByte(v) != I2C_ACK ) goto IWerror;
	I2CStop();
	return;

IWerror:
	I2CStop();
	Stats[I2CFailS]++;
	return;
} // WriteI2CByteAtAddr

boolean ReadI2Ci16vAtAddr(uint8 d, uint8 cmd, int16 *v, uint8 l, boolean h) {
	static uint8 b, c;
	static uint8 S[16];

	I2CStart();
		if( WriteI2CByte(d) != I2C_ACK ) goto IRSerror;
		if( WriteI2CByte(cmd) != I2C_ACK ) goto IRSerror;
	I2CStart();	
		if( WriteI2CByte(d | 1) != I2C_ACK ) goto IRSerror;
		for (b = 0; b < l*2; b++) {
			if ( b < (l*2-1) )
				S[b] = ReadI2CByte(I2C_ACK);
			else
				S[b] = ReadI2CByte(I2C_NACK);
		}
	I2CStop();

	// fix endian!
	c = 0;
	for ( b = 0; b < l; b++) {
		if ( h )
			v[b] = ((int16)(S[c])<<8) | S[c+1];
		else
			v[b] = ((int16)(S[c+1])<<8) | S[c];
		c += 2;
	}
	
	return( true );

IRSerror:
	I2CStop();

	for (b = 0; b < l; b++)
		v[b] = 0;

	return(false);

} // ReadI2Ci16vAtAddr

boolean I2CResponse(uint8 d) {
	static boolean r;

    I2CStart();
 	   r = WriteI2CByte(d) == I2C_ACK;
    I2CStop();

	return (r);
} // I2CResponse

// -----------------------------------------------------------

// SW I2C Routines for ESCs

boolean ESCWaitClkHi(void);
void ESCI2CStart(void);
void ESCI2CStop(void);

uint8 WriteESCI2CByte(uint8);
void ProgramSlaveAddress(uint8);
void ConfigureESCs(void);

// Constants

#define	 ESC_SDA		PORTBbits.RB1
#define	 ESC_SCL		PORTBbits.RB2
#define	 ESC_DIO		TRISBbits.TRISB1
#define	 ESC_CIO		TRISBbits.TRISB2

#define ESC_DATA_LOW	{ESC_SDA=0;ESC_DIO=I2C_OUT;}
#define ESC_DATA_FLOAT	{ESC_DIO=I2C_IN;}
#define ESC_CLK_LOW		{ESC_SCL=0;ESC_CIO=I2C_OUT;}
#define ESC_CLK_FLOAT	{ESC_CIO=I2C_IN;}

boolean ESCWaitClkHi(void) {
	static uint8 s;

	Delay1TCY();Delay1TCY();Delay1TCY(); // TSU:DAT + Call
	ESC_CLK_FLOAT;
	s = 1;						
	while( !ESC_SCL )
		if( ++s == (uint8)0 ) return (false);					

	return ( true );
} // ESCWaitClkHi

void ESCI2CStart(void) {
	static uint8 r;

	ESC_DATA_FLOAT;
	r = ESCWaitClkHi();
	ESC_DATA_LOW;
	Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY(); // tHD:STA		
	ESC_CLK_LOW;				
} // ESCI2CStart

void ESCI2CStop(void) {
	ESC_DATA_LOW;
	ESCWaitClkHi();
	Delay1TCY();Delay1TCY();Delay1TCY(); // tsu:STO
	ESC_DATA_FLOAT;

    Delay10TCYx(2); // TBuf

} // ESCI2CStop

uint8 WriteESCI2CByte(uint8 d) { // ~320KHz @ 40MHz
	static uint8 s, dd;

	dd = d; // a little faster
	s = 8;
	do {
		if( dd & 0x80 )
			ESC_DATA_FLOAT
		else
			ESC_DATA_LOW
	
		if( ESCWaitClkHi() ) { 	
			ESC_CLK_LOW;
			dd <<= 1;
		} else
			return(I2C_NACK);	
	} while ( --s );

	ESC_DATA_FLOAT;
	if( ESCWaitClkHi() )
		s = ESC_SDA;
	else
		return(I2C_NACK);	
	ESC_CLK_LOW;

	return( s);

} // WriteESCI2CByte



