// =======================================================================
// =                                 UAVX                                =
// =                         Quadrocopter Control                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://www.uavp.org                        =
// =======================================================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

// SPI routines for LEDs and LISL

#include "UAVX.h"

#define SPI_CS		PORTCbits.RC5
#define SPI_SDA		PORTCbits.RC4
#define SPI_SCL		PORTCbits.RC3
#define SPI_IO		TRISCbits.TRISC4

#define	RD_SPI	1
#define WR_SPI	0

#define DSEL_LISL  1
#define SEL_LISL  0

// LISL-Register mapping
#define	LISL_WHOAMI		(0x0f)
#define	LISL_OFFSET_X	(0x16)
#define	LISL_OFFSET_Y	(0x17)
#define	LISL_OFFSET_Z	(0x18)
#define	LISL_GAIN_X		(0x19)
#define	LISL_GAIN_Y		(0x1A)
#define	LISL_GAIN_Z		(0x1B)
#define	LISL_CTRLREG_1	(0x20)
#define	LISL_CTRLREG_2	(0x21)
#define	LISL_CTRLREG_3	(0x22)
#define	LISL_STATUS		(0x27)
#define LISL_OUTX_L		(0x28)
#define LISL_OUTX_H		(0x29)
#define LISL_OUTY_L		(0x2A)
#define LISL_OUTY_H		(0x2B)
#define LISL_OUTZ_L		(0x2C)
#define LISL_OUTZ_H		(0x2D)
#define LISL_FF_CFG		(0x30)
#define LISL_FF_SRC		(0x31)
#define LISL_FF_ACK		(0x32)
#define LISL_FF_THS_L	(0x34)
#define LISL_FF_THS_H	(0x35)
#define LISL_FF_DUR		(0x36)
#define LISL_DD_CFG		(0x38)
#define LISL_INCR_ADDR	(0x40)
#define LISL_READ		(0x80)

void WaitLISLReady(void);
void SendCommand(int8);
void IsLISLactive(void);
void WriteLISLByte(uint8);
void WriteLISL(uint8, uint8);
uint8 ReadLISL(uint8);
uint8 ReadLISLNext(void);
void ReadAccelerations(void);

// LIS3LV02DQ Inertial Sensor (Accelerometer)

#define SPI_HI_DELAY Delay10TCY()
#define SPI_LO_DELAY Delay10TCY()


void WaitLISLReady(void)
{
	while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == 0 );
} // WaitLISLReady

void SendCommand(int8 c)
{
	int8 s;

	SPI_IO = WR_SPI;	
	SPI_CS = SEL_LISL;	
	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		if( c & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		c <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
} // SendCommand

uint8 ReadLISL(uint8 c)
{
	uint8 d;

//	SPI_SDA = 1;	// very important!! really!! LIS3L likes it
	SendCommand(c);
	SPI_IO = RD_SPI;	// SDA is input
	d=ReadLISLNext();
	
	if( (c & LISL_INCR_ADDR) == 0 )
		SPI_CS = DSEL_LISL;
	return(d);
} // ReadLISL

uint8 ReadLISLNext(void)
{
	int8 s;
	uint8 d;

	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		SPI_LO_DELAY;
		d <<= 1;
		if( SPI_SDA == 1 )
			d |= 1;	
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	return(d);
} // ReadLISLNext

void WriteLISL(uint8 d, uint8 c)
{
	int8 s;

	SendCommand(c);

	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	SPI_CS = DSEL_LISL;
	SPI_IO = RD_SPI;	// IO is input (to allow RS232 reception)
} // WriteLISL

void IsLISLactive(void)
{
	uint8 s;

	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	s = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( s == 0x3A )	// a LIS03L sensor is there!
	{
		WriteLISL(0b11000111, LISL_CTRLREG_1); // startup, enable all axis
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01001000, LISL_FF_CFG); // Y-axis is height
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		_UseLISL = true;
	}
} // IsLISLactive

void ReadAccelerations()
{
	uint8 r;

	r = ReadLISL(LISL_STATUS + LISL_READ);
	Ax  = (int16)ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
	Ax |= (int16)ReadLISLNext()*256;
	Ay  = (int16)ReadLISLNext();
	Ay |= (int16)ReadLISLNext()*256;
	Az  = (int16)ReadLISLNext();
	Az |= (int16)ReadLISLNext()*256;
	SPI_CS = DSEL_LISL;	// end transmission

} // ReadAccelerations

//-----------------------------------------------------------------------------

//send LedShadow byte to TPIC6B595N
void SendLeds(void)
{
	uint8	LEDs;
	uint8	s;

	LEDs = LedShadow;

    SPI_IO = WR_SPI;
	SPI_CS = DSEL_LISL;							// select TPIC	
	for(s=8; s ; s--)
	{
		SPI_SCL = 0;
		if ( LEDs & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		LEDs <<= 1;
		SPI_SCL = 1;
	}

	PORTCbits.RC1=1;
	PORTCbits.RC1=0;						// latch into drivers
} // SendLeds
