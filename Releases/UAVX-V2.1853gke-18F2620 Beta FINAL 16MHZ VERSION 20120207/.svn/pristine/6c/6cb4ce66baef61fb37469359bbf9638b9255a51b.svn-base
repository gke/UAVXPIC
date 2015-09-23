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

// Accelerator 400KHz I2C or SPI

#include "uavx.h"

#include "MPU6050.h"

void ShowAccType(void);
void AccFailure(void);
void ReadAccelerations(void);
void GetNeutralAccelerations(void);
void AccelerometerTest(void);
void InitAccelerometers(void);

void ReadADXL345Acc(void);
void InitADXL345Acc(void);
boolean ADXL345AccActive(void);

void ReadMPU6050Acc(void);
void InitMPU6050Acc(void);
boolean MPU6050AccActive(void);

void ReadBMA180Acc(void);
void InitBMA180Acc(void);
boolean BMA180AccActive(void);
void ShowBMA180State(void);

void SendCommand(int8);
uint8 ReadLISL(uint8);
uint8 ReadLISLNext(void);
void WriteLISL(uint8, uint8);
void InitLISLAcc(void);
boolean LISLAccActive(void);
void ReadLISLAcc(void);

int16 RawAcc[3];
uint8 AccType;
uint8 BMA180_ID;

const rom char * AccName[AccUnknown+1] = 
		{"LIS3L","ADXL345","BMA180","MPU6050","Fail or not supported @ 16MHz"};

void ShowAccType(void)
{
	TxString(AccName[AccType]);
} // ShowAccType

void AccFailure(void)
{
	if ( State == InFlight )
	{
		Stats[AccFailS]++;	
		F.AccFailure = true;
	}
} // AccFailure

void ReadAccelerations(void)
{
	// X/Forward FB Acc sense to simplify gyro comp code
	switch ( P[SensorHint] ) {
	#ifdef INC_ADXL345
	case SFDOF6:
		ReadADXL345Acc();
		A[Roll].AccADC = RawAcc[Y] * 5; 			     	
		A[Pitch].AccADC = RawAcc[X] * 5; 
		A[Yaw].AccADC = RawAcc[Z] * 5;
		break;
	case SFDOF9: 
		ReadADXL345Acc();
		A[Roll].AccADC = -RawAcc[X] * 5; 		
	  	A[Pitch].AccADC = RawAcc[Y] * 5;
		A[Yaw].AccADC = RawAcc[Z] * 5; 
		break;
	#endif // INC_ADXL345
	#ifdef INC_BMA180
	case FreeIMU:
		ReadBMA180Acc();
		A[Roll].AccADC = -SRS16(RawAcc[X], 4); 
		A[Pitch].AccADC = SRS16(RawAcc[Y], 4);
		A[Yaw].AccADC = SRS16(RawAcc[Z], 4);
	case Drotek:
		ReadBMA180Acc();
		A[Roll].AccADC = SRS16(RawAcc[Y], 4); 
		A[Pitch].AccADC = SRS16(RawAcc[X], 4);
		A[Yaw].AccADC = SRS16(RawAcc[Z], 4);
		break;
	#endif // INC_BMA180
	#ifdef INC_MPU6050
	case MPU6050:
		ReadMPU6050Acc();// QuadroUFO 	
		A[Roll].AccADC = -SRS16(RawAcc[X], 3); 
		A[Yaw].AccADC = SRS16(RawAcc[Z], 3);
		A[Pitch].AccADC = SRS16(RawAcc[Y], 3);
		break;
	#endif // INC_MPU6050
	case ITG3200Gyro: // Use LISL
	default:
		ReadLISLAcc();
		#ifdef FLAT_LISL_ACC
			A[Roll].AccADC = RawAcc[X];
			A[Pitch].AccADC = -RawAcc[Y];
			A[Yaw].AccADC = RawAcc[Z];
		#else
			A[Roll].AccADC = RawAcc[X];
			A[Pitch].AccADC = RawAcc[Z];
			A[Yaw].AccADC = RawAcc[Y];
		#endif //FLAT_LISL_ACC

		break;
	} // switch

} // ReadAccelerations

void GetNeutralAccelerations(void)
{
	// this routine is called ONLY ONCE while booting
	// and averages accelerations over 16 samples.
	// Puts values in Neutralxxx registers.
	static uint8 i, a;
	static int16 Temp[3], b;

	// already done in caller program
	Temp[Roll] = Temp[Pitch] = Temp[Yaw] = 0;
	if ( F.AccelerationsValid )
	{
		for ( i = 16; i; i--)
		{
			ReadAccelerations();
			for ( a = LR; a<=(uint8)DU; a++ )
				Temp[a] += A[a].AccADC;

			Delay1mS(10);
		}	
	
		Temp[Yaw] -= 16*1024;

		for ( a = LR; a<=(uint8)DU; a++ )
		{
			b = Temp[a];
			b = SRS32(b, 4);	
			A[a].AccBias = Limit1(b, 99);
		}
	}
	else
		A[Roll].AccBias = A[Pitch].AccBias = A[Yaw].AccBias = 0;

} // GetNeutralAccelerations

void InitAccelerometers(void)
{
	static uint8 a;

	for ( a = Roll; a<=(uint8)Yaw; a++)
		A[a].AccBias = A[a].AccADC = 0;
	A[Yaw].AccADC = GRAVITY;

	AccType = AccUnknown;
	F.AccelerationsValid = false;

	switch ( P[SensorHint]){
	#ifdef CLOCK_16MHZ
		case SFDOF6: // ITG3200
		case SFDOF9:
		case FreeIMU:
		case Drotek:
		case MPU6050:
			AccType = AccUnknown;
			break;
		case ITG3200Gyro:
		default:	
		if ( LISLAccActive() )
		{
			AccType = LISLAcc;
			InitLISLAcc();
		}
		break;
	#else
		#ifdef INC_ADXL345
		case SFDOF6: // ITG3200
		case SFDOF9:
			if ( ADXL345AccActive() )
			{
				AccType = ADXL345Acc;
				InitADXL345Acc();
			}
			break;
		#endif // ADXL345
		#ifdef INC_BMA180
		case FreeIMU:
		case Drotek:
			if ( BMA180AccActive() )
			{
				AccType = BMA180Acc;
				InitBMA180Acc();
			}
			break;
		#endif // INC_BMA_180
		#ifdef INC_MPU6050
		case MPU6050:
			INV_ID = INV_ID_MPU6050;
			if ( MPU6050AccActive() )
			{
				AccType = MPU6050Acc;
				InitMPU6050Acc();
			}
			break;
		#endif // INC_MPU6050
		case ITG3200Gyro:
		default:	
			if ( LISLAccActive() )
			{
				AccType = LISLAcc;
				InitLISLAcc();
			}
			break;
	#endif // CLOCK_16MHZ
	} // switch

	if( F.AccelerationsValid )
		GetNeutralAccelerations();
	else
		F.AccFailure = true;

} // InitAccelerometers

#ifdef TESTING

void AccelerometerTest(void)
{
	static int16 Mag;

	TxString("\r\nAccelerometer test - ");
	ShowAccType();
	TxString("\r\n\r\n");

	if( F.AccelerationsValid )
	{
		TxString("Read once - no averaging (1G ~= 1024)\r\n");
		#ifdef INC_BMA180
		if ( AccType == BMA180Acc )
			ShowBMA180State();
		#endif // INC_BMA180
	
		ReadAccelerations();
	
		TxString("\tL->R: \t");
		TxVal32((int32)A[Roll].AccADC, 0, 0);
		if ( Abs((A[Roll].AccADC)) > 128 )
			TxString(" fault?");
		TxNextLine();

		TxString("\tF->B: \t");	
		TxVal32((int32)A[Pitch].AccADC, 0, 0);
		if ( Abs((A[Pitch].AccADC)) > 128 )
			TxString(" fault?");	
		TxNextLine();

		TxString("\tD->U:    \t");
	
		TxVal32((int32)A[Yaw].AccADC, 0, 0);
		if ( ( A[Yaw].AccADC < 896 ) || ( A[Yaw].AccADC > 1152 ) )
			TxString(" fault?");	
		TxNextLine();

		TxString("\t|Mag|:   \t");
		Mag = int32sqrt(Sqr((int24)A[Roll].AccADC)+Sqr((int24)A[Pitch].AccADC)+Sqr((int24)A[Yaw].AccADC));
		TxVal32((int32)Mag, 0, 0);
		TxNextLine();
	}

} // AccelerometerTest

#endif // TESTING

//________________________________________________________________________________________________

#ifdef INC_ADXL345

boolean ADXL345AccActive(void);

void ReadADXL345Acc(void) 
{
	if ( !ReadI2Ci16vAtAddr(ADXL345_ID, 0x32, RawAcc, 3, false) ) 
		AccFailure();

} // ReadADXL345Acc

void InitADXL345Acc() {

	WriteI2CByteAtAddr(ADXL345_ID, 0x2D, 0x08);  // measurement mode
    Delay1mS(5);
	WriteI2CByteAtAddr(ADXL345_ID, 0x31, 0x0B);  // full resolution, 2g
    Delay1mS(5);
	WriteI2CByteAtAddr(ADXL345_ID, 0x2C,
	    //0x0C);  	// 400Hz
		//0x0b);	// 200Hz 
	  	//0x0a); 	// 100Hz 
	  	0x09); 	// 50Hz
    Delay1mS(5);

} // InitADXL345Acc

boolean ADXL345AccActive(void) 
{
    F.AccelerationsValid = I2CResponse(ADXL345_ID);
    return( F.AccelerationsValid );
} // ADXL345AccActive

#endif // INC_ADXL345

//________________________________________________________________________________________________

#ifdef INC_MPU6050

boolean MPU6050AccActive(void);

void ReadMPU6050Acc(void) 
{
	if ( !ReadI2Ci16vAtAddr(MPU6050_ID, MPU6050_ACC_XOUT_H, RawAcc, 3, true) ) 
		AccFailure();

} // ReadMPU6050Acc

void InitMPU6050Acc() {

	WriteI2CByteAtAddr(MPU6050_ID,MPU6050_PWR_MGMT_1, 0xc0); // Reset to defaults
	Delay1mS(50);
	WriteI2CByteAtAddr(MPU6050_ID,MPU6050_SMPLRT_DIV, 0x00); // continuous update
	WriteI2CByteAtAddr(MPU6050_ID,MPU6050_GYRO_CONFIG, 0b00011001);	// 188Hz, 2000deg/S
	WriteI2CByteAtAddr(MPU6050_ID,MPU6050_INT_ENABLE, 0b00000000);	// no interrupts
	WriteI2CByteAtAddr(MPU6050_ID,MPU6050_PWR_MGMT_1, 0b00000001);	// X Gyro as Clock Ref.

	WriteI2CByteAtAddr(MPU6050_ID, MPU6050_ACC_CONFIG, 
				0 // 2G
				//1 << 3 // 4G
				//2 << 3 // 8G
				//3 << 3 // 16G 
				//| 1 // 2.5Hz
				//| 2 // 2.5Hz
				| 3 // 1.25Hz	//zzz
				//| 4 // 0.63Hz
				//| 7 // 0.63Hz
				);

} // InitMPU6050Acc

boolean MPU6050AccActive(void) 
{
    F.AccelerationsValid = I2CResponse(MPU6050_ID);
    return( F.AccelerationsValid );
} // MPU6050AccActive

#endif // INC_MPU6050

//________________________________________________________________________________________________

#ifdef INC_BMA180

// Bosch BMA180 Acc

// 0 1g, 1 1.5g, 2 2g, 3 3g, 4 4g, 5 8g, 6 16g
// 0 19Hz, 1 20, 2 40, 3 75, 4 150, 5 300, 6 600, 7 1200Hz 

#define BMA180_RANGE	2
#define BMA180_BW 		1 // 4

#define BMA180_Version 0x01
#define BMA180_ACCXLSB 0x02
#define BMA180_TEMPERATURE 0x08

#define BMA180_RESET 	0x10
#define BMA180_STATREG1 0x09
#define BMA180_STATREG2 0x0A
#define BMA180_STATREG3 0x0B
#define BMA180_STATREG4 0x0C
#define BMA180_CTRLREG0 0x0D
#define BMA180_CTRLREG1 0x0E
#define BMA180_CTRLREG2 0x0F

#define BMA180_BWTCS 0x20		// bandwidth
#define BMA180_CTRLREG3 0x21

#define BMA180_HILOWNFO 0x25
#define BMA180_LOWDUR 0x26
#define BMA180_LOWTH 0x29

#define BMA180_tco_y 0x2F
#define BMA180_tco_z 0x30

#define BMA180_OLSB1 0x35		// setting range

boolean BMA180AccActive(void);

void ReadBMA180Acc(void) 
{
	if ( !ReadI2Ci16vAtAddr(BMA180_ID, BMA180_ACCXLSB, RawAcc, 3, false) )
		AccFailure();

} // ReadBMA180Acc

void InitBMA180Acc() {
	
	uint8 i, bw, range, ee_w;

	// if connected correctly, ID register should be 3
//	if(read(ID) != 3)
//		return -1;

	WriteI2CByteAtAddr(BMA180_ID, BMA180_RESET, 0); 

	ee_w = ReadI2CByteAtAddr(BMA180_ID, BMA180_CTRLREG0);
	ee_w |= 0x10;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_CTRLREG0, ee_w); // write enable registers

	bw = ReadI2CByteAtAddr(BMA180_ID, BMA180_BWTCS);
	bw &= ~0xF0;
	bw |= BMA180_BW << 4;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_BWTCS, bw); // set BW	
		
	range = ReadI2CByteAtAddr(BMA180_ID, BMA180_OLSB1);
	range &= ~0x0E;
	range |= BMA180_RANGE<<1;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_OLSB1, range); // set range

} // InitBMA180Acc

boolean BMA180AccActive(void) {

	BMA180_ID = BMA180_ID_0x80;
    if ( I2CResponse(BMA180_ID) )
		F.AccelerationsValid = true;
	else
	{
		BMA180_ID = BMA180_ID_0x82;
		if ( I2CResponse(BMA180_ID) )
			F.AccelerationsValid = true;
		else
			F.AccelerationsValid = false;
	}

    return( F.AccelerationsValid );

} // BMA180AccActive

#ifdef TESTING

void ShowBMA180State(void)
{
	#ifdef FULL_ACC_TEST

	static uint8 bw, range;
	bw = ReadI2CByteAtAddr(BMA180_ID, BMA180_BWTCS);		
	range = ReadI2CByteAtAddr(BMA180_ID, BMA180_OLSB1);
	TxString("\r\n\tBW:\t");
	TxVal32((bw>>4) & 0x0f,0,HT);
	TxString("Range:\t");
	TxVal32((range>>1) & 0x07,0,0);
	TxNextLine();

	#endif // FULL_ACC_TEST
} // ShowBMA180State

#endif // TESTING

#endif // INC_BMA180

//________________________________________________________________________________________________

// LISL Acc

#ifdef CLOCK_16MHZ
	#define SPI_HI_DELAY Delay10TCY()
	#define SPI_LO_DELAY Delay10TCY()
#else // CLOCK_40MHZ
	#define SPI_HI_DELAY Delay10TCYx(2)
	#define SPI_LO_DELAY Delay10TCYx(2)
#endif // CLOCK_16MHZ

// LISL-Register mapping

#define	LISL_WHOAMI		(uint8)(0x0f)
#define	LISL_OFFSET_X	(uint8)(0x16)
#define	LISL_OFFSET_Y	(uint8)(0x17)
#define	LISL_OFFSET_Z	(uint8)(0x18)
#define	LISL_GAIN_X		(uint8)(0x19)
#define	LISL_GAIN_Y		(uint8)(0x1A)
#define	LISL_GAIN_Z		(uint8)(0x1B)
#define	LISL_CTRLREG_1	(uint8)(0x20)
#define	LISL_CTRLREG_2	(uint8)(0x21)
#define	LISL_CTRLREG_3	(uint8)(0x22)
#define	LISL_STATUS		(uint8)(0x27)
#define LISL_OUTX_L		(uint8)(0x28)
#define LISL_OUTX_H		(uint8)(0x29)
#define LISL_OUTY_L		(uint8)(0x2A)
#define LISL_OUTY_H		(uint8)(0x2B)
#define LISL_OUTZ_L		(uint8)(0x2C)
#define LISL_OUTZ_H		(uint8)(0x2D)
#define LISL_FF_CFG		(uint8)(0x30)
#define LISL_FF_SRC		(uint8)(0x31)
#define LISL_FF_ACK		(uint8)(0x32)
#define LISL_FF_THS_L	(uint8)(0x34)
#define LISL_FF_THS_H	(uint8)(0x35)
#define LISL_FF_DUR		(uint8)(0x36)
#define LISL_DD_CFG		(uint8)(0x38)
#define LISL_INCR_ADDR	(uint8)(0x40)
#define LISL_READ		(uint8)(0x80)

void SendCommand(int8 c)
{
	static uint8 s;

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
	static uint8 d;

	SPI_SDA = 1; // very important!! really!! LIS3L likes it
	SendCommand(c);
	SPI_IO = RD_SPI;	// SDA is input
	d = ReadLISLNext();
	
	if( (c & LISL_INCR_ADDR) == (uint8)0 )
		SPI_CS = DSEL_LISL;
	return(d);
} // ReadLISL

uint8 ReadLISLNext(void)
{
	static uint8 s;
	static uint8 d;

	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		SPI_LO_DELAY;
		d <<= 1;
		if( SPI_SDA == (uint8)1 )
			d |= 1;	
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	return(d);
} // ReadLISLNext

void WriteLISL(uint8 d, uint8 c)
{
	static uint8 s;

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

void InitLISLAcc(void)
{
	static int8 r;

	F.AccelerationsValid = false;

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( r == 0x3A )	// a LIS03L sensor is there!
	{
		WriteLISL(0b11000111, LISL_CTRLREG_1); // on always, 40Hz sampling rate,  10Hz LP cutoff, enable all axes
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01000000, LISL_FF_CFG); // latch, no interrupts; 
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		F.AccelerationsValid = true;
	}
	else
		AccFailure();
} // InitLISLAcc

boolean LISLAccActive(void)
{
	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	F.AccelerationsValid = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a;

	return ( F.AccelerationsValid );
} // LISLAccActive

void ReadLISLAcc()
{
	static charint16x4u L;

//	while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == (uint8)0 );

	F.AccelerationsValid = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a; // Acc still there?
	if ( F.AccelerationsValid ) 
	{
		L.c[0] = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
		L.c[1] = ReadLISLNext();
		L.c[2] = ReadLISLNext();
		L.c[3] = ReadLISLNext();
		L.c[4] = ReadLISLNext();
		L.c[5] = ReadLISLNext();
		SPI_CS = DSEL_LISL;	// end transmission

		RawAcc[X] = L.i16[X];
		RawAcc[Y] = L.i16[Y];
		RawAcc[Z] = L.i16[Z];
	}
	else
		AccFailure();
} // ReadLISLAcc


