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

void CompensateRollPitchGyros(void);
void GetAttitude(void);
void DoAttitudeAngles(void);
void GyrosAndAccsTest(void);

void ShowGyroType(uint8);
void GetGyroValues(void);
void CalculateGyroRates(void);
void ErectGyros(uint16);

#pragma udata gyronames
uint8 GyroType;
real32 KpAcc;
real32 PIDdT;
#pragma udata

#pragma idata gyroinames
const rom uint8 * GyroName[GyroUnknown+1] ={
		 "MLX90609", "ADXRS613/150",
		"ST-AY530", "ADXRS610/300", "MPU6050", "MPU6050", 
		"Unknown" };

//MLX90609B OK
//ADXRS613/150 ?
//ST-AY530 OK
//ADXRS610/300 ?
const rom int8 GyroScale[(GyroUnknown+1)] = {13, 7, 26, 18, 1, 1, 1};
#pragma idata

void ShowGyroType(uint8 G) {
	TxString(GyroName[G]);
} // ShowGyroType

void CalculateGyroRates(void) {
	static uint8 a;
	static AxisStruct *C;

	for (a = Roll; a <= Yaw; a++) {
		C = &A[a];
		C->Rate = (C->GyroADC - C->GyroBias) * GyroScale[GyroType];
	}

} // CalculateGyroRates

void ErectGyros(uint16 s) {
	static uint8 a;
	static AxisStruct *C;
	static uint16 i;
	static int32 Av[3];

	for ( a = Roll; a <=(uint8)Yaw; a++ )	
		Av[a] = 0;

    for ( i = 0; i < s; i++ ) {
		LEDRed_TOG;
		Delay1mS(20);
		OutSignals();

		GetRatesAndAccelerations();

		for ( a = Roll; a <= (uint8)Yaw; a++ )
			Av[a] += A[a].GyroADC;
	}
	
	for ( a = Roll; a <= (uint8)Yaw; a++ ) {
		C = &A[a];
		C->GyroBias = Av[a]/s; // InvenSense is signed
		C->Rate = C->Ratep = C->Angle = 0;
		C->AngleHR = 0.0f;
	}

	LEDRed_OFF;

} // ErectGyros

#pragma idata acc_names
const rom uint8 MPUDLPFMask[] = { MPU_RA_DLPF_BW_256, MPU_RA_DLPF_BW_188,
		MPU_RA_DLPF_BW_98, MPU_RA_DLPF_BW_42 };

#if defined(TESTING)
const rom uint16 InertialLPFHz[] = { 256, 188, 98, 42 };
const rom uint8 * DHPFName[] = { "Reset/0Hz", "5Hz", "2.5Hz", "1.25Hz", "0.63Hz",
		"?", "?", "Hold" };
#endif
#pragma idata

void ShowAccType(void);
void GetRatesAndAccelerations(void);
void GetNeutralAccelerations(void);
void InitAccelerations(void);

void GetMPU6050Values(void);
void InitMPU6050(void);
boolean MPU6050Active(void);

void SendCommand(int8);
uint8 ReadLISL(uint8);
uint8 ReadLISLNext(void);
void WriteLISL(uint8, uint8);
void InitLISLAcc(void);
boolean LISLAccActive(void);
void ReadLISLAcc(void);

#pragma udata accnames
uint8 AccType;
uint8 MPU6050_ID;
real32 AccConfidence;
uint8 MPU6050DLPF, MPU6050DHPF;
#pragma udata

#pragma idata accinames
const rom uint8 * AccName[AccUnknown+1] = 
		{"LIS3L", "MPU6050", "No response"};
#pragma idata

void ShowAccType(void) {
	TxString(AccName[AccType]);
} // ShowAccType

void InitAccelerations(void) {
	static uint8 a;

	for ( a = Roll; a<=(uint8)Yaw; a++)
		A[a].AccBias = A[a].AccADC = 0;
	A[DU].AccADC = GRAVITY;

	ReadAccCalEE();

} // InitAccelerations

//________________________________________________________________________________________________

real32 CalculateAccConfidence(int accMag) {
	static int16 m = GRAVITY;
	static real32 c;

	m = HardFilter(m, accMag);

	c = 1.0f - (Abs(m - GRAVITY)) * (GRAVITYR * 2.0f); // 2.0 should be AccConfSD
	if (c < 0.0f) c = 0.0f;

	return(c);
} // CalculateAccConfidence

void DoAttitudeAngles(void) {	
	// Angles and rates are normal aircraft coordinate conventions
	// X/Forward axis reversed for Acc to simplify compensation

	static uint8 a;
	static AxisStruct *C;
	static int32 m;
	static int16 accMag;
	static real32 NormR;
	static real32 Ka;

	m = 0;
	for ( a = Roll; a <= (uint8)Yaw; a++ ) {
		C = &A[a];
		C->Acc = C->AccADC - C->AccBias;
		m += Sqr((int24)C->Acc);
	}
	
	accMag = int32sqrt(m); // ~1024
	AccConfidence = CalculateAccConfidence(accMag);

	NormR = GRAVITY / (real32)accMag;

	Ka = KpAcc *  AccConfidence;

	// RESORTING TO FLOATS!!
	for ( a = Roll; a <= (uint8)Pitch; a++ ) {
		C = &A[a];
	
		C->AngleHR += (real32)C->Rate * PIDdT;
			
		// Forget small angle check as Ka should be "small" so gyro 
		// integration  will dominate. Strictly we should use asin(C->Acc)	
		C->AngleHR = C->AngleHR * (1.0f - Ka) - (C->Acc * NormR) * Ka;

		C->Angle = (int16)C->AngleHR; 
	}

} // DoAttitudeAngles

void GetRatesAndAccelerations(void) {
	static uint32 LastUpdateuS = 0;
	static uint32 PIDdTuS;
	static uint32 NowuS;

	NowuS = uSClock();
	PIDdTuS = NowuS - LastUpdateuS;

#if defined(USE_PID_DT_CLIP)
	PIDdTuS = Limit(PIDdTuS, PID_CYCLE_US-400, PID_CYCLE_US+400);
#else
	if ((State == InFlight) && ((PIDdTuS < (PID_CYCLE_US-400)) || (PIDdTuS > (PID_CYCLE_US+400)))) {
		PIDdT = PID_CYCLE_US * 0.000001f;
		Stats[BadS]++;
		Stats[BadNumS] = PIDdTuS;
	}
	else
#endif
		PIDdT = (real32) PIDdTuS * 0.000001f;

	LastUpdateuS = NowuS;

	if (UsingMPU6050)
		GetMPU6050Values();
	else {
		GetAnalogGyroValues();
		GetLISLValues();
	}

} // GetRatesAndAccelerations

	
void GetAttitude(void) {

	GetRatesAndAccelerations();
	CalculateGyroRates(); 
	DoAttitudeAngles();
} // GetAttitude


//________________________________________________________________________________________________


boolean MPU6050Active(void);

void GetMPU6050Values(void) {
	static int16 b[7];

	ReadI2Ci16vAtAddr(MPU6050_ID, MPU6050_ACC_XOUT_H, b, 7, true);

	A[FB].AccADC = SRS16(b[1], 1); 
	A[LR].AccADC = -SRS16(b[0], 1);
	A[DU].AccADC = SRS16(b[2], 1) + GRAVITY;

	// b[3] temperature

	A[Roll].GyroADC = -b[5];
	A[Pitch].GyroADC = -b[4];
	A[Yaw].GyroADC = -b[6];

} // GetMPU6050Values


void InitMPU6050(void) {
	uint8 v;

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_PWR_MGMT_1, 1<< MPU_RA_PWR1_DEVICE_RESET_BIT);
	Delay1mS(5);

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_FIFO_EN, 0); // disable FIFOs

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_SMPLRT_DIV, 0); 
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_PWR_MGMT_1, MPU_RA_CLOCK_PLL_XGYRO); // No sleeping, temperature sensor On, Z ref.

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_ACC_CONFIG, (MPU_RA_ACC_FS_16 << 3)| MPU6050_DHPF_1P25); // DHPF only used by free-fall alarms

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_GYRO_CONFIG, MPU_RA_GYRO_FS_2000 << 3);

	// Enable I2C master mode
	v = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_USER_CTRL);
	Clear(v, MPU_RA_USERCTRL_I2C_MST_EN_BIT);
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_USER_CTRL,v);

	// Allow bypass access to slave I2C devices (Magnetometer)
	v = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_INT_PIN_CFG);
	Set(v, MPU_RA_INTCFG_I2C_BYPASS_EN_BIT);
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_INT_PIN_CFG, v);

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_CONFIG, MPUDLPFMask[P[GyroLPF]]);

	Delay1mS(10);

#if defined(TESTING)
	MPU6050DLPF = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_CONFIG) & 0x07;
	MPU6050DHPF = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_ACC_CONFIG) & 0x07;
#endif // TESTING

//	DoMPU6050AccScale(); // determine acc scaling

} // InitMPU6050

boolean MPU6050Active(void) {

	MPU6050_ID = MPU6050_0xD0_ID;
    F.IMUActive = I2CResponse(MPU6050_ID);
	if (!F.IMUActive) {
		MPU6050_ID = MPU6050_0xD2_ID;
		F.IMUActive = I2CResponse(MPU6050_ID);
	}
    return( F.IMUActive );
} // MPU6050Active


//________________________________________________________________________________________________

// LISL Acc

#define SPI_HI_DELAY Delay10TCYx(2)
#define SPI_LO_DELAY Delay10TCYx(2)

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

void SendCommand(int8 c) {
	static uint8 s;

	SPI_IO = WR_SPI;	
	SPI_CS = SEL_LISL;	
	for( s = 8; s; s-- ) {
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

uint8 ReadLISL(uint8 c) {
	static uint8 d;

	SPI_SDA = 1; // very important!! really!! LIS3L likes it
	SendCommand(c);
	SPI_IO = RD_SPI;	// SDA is input
	d = ReadLISLNext();
	
	if( (c & LISL_INCR_ADDR) == (uint8)0 )
		SPI_CS = DSEL_LISL;
	return(d);
} // ReadLISL

uint8 ReadLISLNext(void) {
	static uint8 s;
	static uint8 d;

	for( s = 8; s; s-- ) {
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

void WriteLISL(uint8 d, uint8 c) {
	static uint8 s;

	SendCommand(c);
	for( s = 8; s; s-- ) {
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

void InitLISLAcc(void) {
	static int8 r;

	Delay1mS(500);

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	F.IMUActive = r == 0x3A;

	WriteLISL(0b11000111, LISL_CTRLREG_1); // on always, 40Hz sampling rate,  10Hz LP cutoff, enable all axes
	WriteLISL(0b00000000, LISL_CTRLREG_3);
	WriteLISL(0b01000000, LISL_FF_CFG); // latch, no interrupts; 
	WriteLISL(0b00000000, LISL_FF_THS_L);
	WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
	WriteLISL(255, LISL_FF_DUR);
	WriteLISL(0b00000000, LISL_DD_CFG);

} // InitLISLAcc

boolean LISLAccActive(void) {
	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	F.IMUActive = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a;

	return ( F.IMUActive );
} // LISLAccActive

void GetLISLValues(void) {
	static charint16x4u L;

//	while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == (uint8)0 );

	F.IMUActive = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a; // Acc still there?

	L.c[0] = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
	L.c[1] = ReadLISLNext();
	L.c[2] = ReadLISLNext();
	L.c[3] = ReadLISLNext();
	L.c[4] = ReadLISLNext();
	L.c[5] = ReadLISLNext();
	SPI_CS = DSEL_LISL;	// end transmission

	A[LR].AccADC = L.i16[X];
	A[FB].AccADC = L.i16[Z];
	A[DU].AccADC = L.i16[Y];

} // GetLISLValues


void WriteAccCalEE(void) {
	static uint8 a;

	for ( a = LR; a<=(uint8)DU; a++ ) 
		Write16EE(ACC_BIAS_ADDR_EE + (a<<1), A[a].AccBias);

} // WriteAccCalEE

void ReadAccCalEE(void) {
	static uint8 a;

	for ( a = LR; a<=(uint8)DU; a++ ) 
		A[a].AccBias = Read16EE(ACC_BIAS_ADDR_EE + (a<<1));

} // ReadAccCalEE


//________________________________________________________________________________________________


// Analog Gyros

void GetAnalogGyroValues(void) {
	// change of sign to get to normal aircraft sense 
	A[Roll].GyroADC = -ADC(ADCRollChan);
	A[Pitch].GyroADC = -ADC(ADCPitchChan);
	A[Yaw].GyroADC = ADC(ADCYawChan);

} // GetAnalogGyroValues

void InitAnalogGyros(void) {
	GyroType = P[SensorHint];
	F.UsingAnalogGyros = true;
} // InitAnalogGyros



