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

// Gyros

#include "uavx.h"

void ShowGyroType(uint8);
void GetGyroValues(void);
void CalculateGyroRates(void);
void CheckGyroFault(uint8, uint8, uint8);
void ErectGyros(void);

void GyroTest(void);
void InitGyros(void);

uint8 GyroType;
int16 RawGyro[3];
int32 NoAccCorr;
int16x16Q YawF;

#include "MPU6050.h"

#include "gyro_i2c.h"
#include "gyro_analog.h"

#pragma idata gyronames
const rom char * GyroName[GyroUnknown+1] ={
		"MLX90609","ADXRS613/150","IDG300 UNSUPPORTED","ST-AY530","ADXRS610/300",
		"ITG3200","SF-DOF6","SF-9DOF","MPU6050","FreeIMU","Drotek","IR Sensors",
		"Fail or not supported @ 16MHZ"
		};
#pragma idata

void ShowGyroType(uint8 G)
{
	TxString(GyroName[G]);
} // ShowGyroType

void CalculateGyroRates(void)
{
	static i24u RollT, PitchT, YawT;

	A[Roll].Rate = A[Roll].GyroADC - A[Roll].GyroBias;
	A[Pitch].Rate = A[Pitch].GyroADC - A[Pitch].GyroBias;
	A[Yaw].Rate = A[Yaw].GyroADC - A[Yaw].GyroBias;

	switch ( GyroType ) {
	case IDG300Gyro:// 500 Deg/Sec 
		RollT.i24 = 0;
		PitchT.i24 = 0;
		YawT.i24 = 0;
		break;
 	case LY530Gyro:// REFERENCE generically 300deg/S 3.3V
		RollT.i24 = (int24)A[Roll].Rate * 256;
		PitchT.i24 = (int24)A[Pitch].Rate * 256;
		YawT.i24 = (int24)A[Yaw].Rate * 128;
		break;
	case MLX90609Gyro:// generically 300deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 127;
		PitchT.i24 = (int24)A[Pitch].Rate * 127;
		YawT.i24 = (int24)A[Yaw].Rate * 63;
		break;
	case ADXRS300Gyro:// ADXRS610/300 300deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 169;
		PitchT.i24 = (int24)A[Pitch].Rate * 169;
		YawT.i24 = (int24)A[Yaw].Rate * 84;
		break;
	case MPU6050:
	case ITG3200Gyro: // Gyro alone or 6&9DOF SF Sensor Stick 73/45
		RollT.i24 = (int24)A[Roll].Rate * 11; // 18
		PitchT.i24 = (int24)A[Pitch].Rate * 11;
		YawT.i24 = (int24)A[Yaw].Rate * 5;
		break;
	case IRSensors:// IR Sensors - NOT IMPLEMENTED IN PIC VERSION
		RollT.i24 = PitchT.i24 = YawT.i24 = 0;
		break;
	case ADXRS150Gyro:// ADXRS613/150 or generically 150deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 68;
		PitchT.i24 = (int24)A[Pitch].Rate * 68;
		YawT.i24 = (int24)A[Yaw].Rate * 34;
		break;
	default:;
	} // GyroType

	A[Roll].Rate = RollT.i2_1;
	A[Pitch].Rate = PitchT.i2_1;
	A[Yaw].Rate = YawT.i2_1;

} // CalculateGyroRates

void ErectGyros(void)
{
	static uint8 i, g;
	static int32 Av[3];
	static AxisStruct *C;

	for ( g = Roll; g <=(uint8)Yaw; g++ )	
		Av[g] = 0;

    for ( i = 32; i ; i-- )
	{
		LEDRed_TOG;
		Delay100mSWithOutput(1);

		GetGyroValues();

		for ( g = Roll; g <= (uint8)Yaw; g++ )
			Av[g] += A[g].GyroADC;
	}
	
	for ( g = Roll; g <= (uint8)Yaw; g++ )
	{
		C = &A[g];
		C->GyroBias = (int16)SRS32( Av[g], 5); // InvenSense is signed
		C->Rate = C->Ratep = C->Angle = C->RawAngle = 0;
	}

	LEDRed_OFF;

} // ErectGyros

void GetGyroValues(void)
{
	switch ( P[SensorHint] ) {
	case ITG3200Gyro:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[X];
		A[Pitch].GyroADC = RawGyro[Y];
		A[Yaw].GyroADC = -RawGyro[Z];
		break;	
	case SFDOF6:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[X];
		A[Pitch].GyroADC = RawGyro[Y];
		A[Yaw].GyroADC = -RawGyro[Z];
		break;
	case SFDOF9: 
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = RawGyro[X];
		A[Pitch].GyroADC = -RawGyro[Y];
		A[Yaw].GyroADC = -RawGyro[Z];
		break;
	case FreeIMU:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[Y]; // not done yet
		A[Pitch].GyroADC = -RawGyro[X];
		A[Yaw].GyroADC = -RawGyro[Z];
		break;
	case Drotek:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[X];
		A[Pitch].GyroADC = RawGyro[Y];
		A[Yaw].GyroADC = -RawGyro[Z];
		break;
	#ifdef INC_MPU6050
	case MPU6050:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[Y];
		A[Pitch].GyroADC = -RawGyro[X];
		A[Yaw].GyroADC = RawGyro[Z];
		break;
	#endif // INC_MPU6050	
	default:
		GetAnalogGyroValues();
		break;
	} // switch
} // GetGyroValues

void InitGyros(void)
{
	InitSmooth16x16(&YawF);

	F.UsingAnalogGyros = false;
	switch ( P[SensorHint]){
	#ifdef CLOCK_16MHZ
		case ITG3200Gyro:
		case SFDOF6: // ITG3200
		case SFDOF9:
		case FreeIMU:
		case Drotek:
		case MPU6050:
			GyroType = GyroUnknown;
			break;
	#else
		case ITG3200Gyro:
		case SFDOF6: // ITG3200
		case SFDOF9:
		case FreeIMU:
		case Drotek:
			INVGyroAddress = INV_GX_H;
			if (InvenSenseGyroActive())
			{
				GyroType = ITG3200Gyro;
				InitInvenSenseGyro();
			}
			break;
		#ifdef INC_MPU6050
		case MPU6050:
			INVGyroAddress = MPU6050_GYRO_XOUT_H;
			if (InvenSenseGyroActive())
			{
				GyroType = MPU6050;
				InitInvenSenseGyro();
			}
			else
				GyroType = GyroUnknown;
			break;
		#else
		case MPU6050:
			GyroType = GyroUnknown;
			break;
		#endif // INC_MPU6050
	#endif // CLOCK_16MHZ
	default:
		InitAnalogGyros();
		GyroType = P[SensorHint];
		F.UsingAnalogGyros = true;
		break;
	} // switch

} // InitGyros

#ifdef TESTING
void GyroTest(void)
{
	TxString("\r\nGyro test - ");
	ShowGyroType(GyroType);
	TxNextLine();

	if ( F.UsingAnalogGyros )
		GyroAnalogTest();
	else
	{

		GetGyroValues();
	
		if ( !F.GyroFailure )
		{
			TxString("\tRoll:     \t");TxVal32(A[Roll].GyroADC,0,0);
			TxString("\r\n\tPitch:\t");TxVal32(A[Pitch].GyroADC,0,0);
			TxString("\r\n\tYaw:  \t");TxVal32(A[Yaw].GyroADC,0,0);
			TxNextLine();
		}
	}	
} // GyroTest
#endif // TESTING





