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


// Barometers Freescale TI ADC and Bosch BMP085 3.8MHz, Bosch SMD500 400KHz  

#include "uavx.h"

int24 AltitudeCF(int24); 
void GetBaroAltitude(void);
void InitBarometer(void);

void BaroTest(void);

int24 BaroToCm(void);
void InitI2CBarometer(void);
void StartI2CBaroADC(boolean);
void GetI2CBaroAltitude(void);


#define MS5611_TEMP_TIME_MS			11	// 10 
#define MS5611_PRESS_TIME_MS		(ALT_UPDATE_MS - MS5611_TEMP_TIME_MS)	// 10 
 
#define MS5611_PROM 	0xA0
#define MS5611_PRESS    0x40
#define MS5611_TEMP 	0x50
#define MS5611_RESET    0x1E

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256  0x00	//0.065 mBar
#define MS5611_OSR_512  0x02	//0.042
#define MS5611_OSR_1024 0x04	//0.027
#define MS5611_OSR_2048 0x06	//0.018
#define MS5611_OSR_4096 0x08	//0.012

#define OSR MS5611_OSR_4096

void ReadMS5611Baro(void);
boolean IsMS5611BaroActive(void);

#define BOSCH_ID_BMP085		((uint8)(0x55))
#define BOSCH_TEMP_BMP085	0x2e
#define BOSCH_TEMP_SMD500	0x6e
#define BOSCH_PRESS			0xb4				// OSRS=2 for BMP085 13.5mS	
#define BOSCH_CTL			0xf4					
#define BOSCH_ADC_MSB		0xf6
#define BOSCH_ADC_LSB		0xf7
#define BOSCH_ADC_XLSB		0xf8				// BMP085
#define BOSCH_TYPE			0xd0
#define BOSCH_BMP085_OSS	3					// 0 4.5mS, 1 7.5mS, 2 13.5mS, 3 25.5mS

// BMP085 4.5mS (T) 25.5mS (P) OSRS=3
#define BOSCH_TEMP_TIME_MS			5		// 10 increase to make P+T acq time ~50mS		
#define BOSCH_PRESS_TIME_MS			(ALT_UPDATE_MS - BOSCH_TEMP_TIME_MS)	// 
		
void ReadBoschBaro(void);
boolean IsBoschBaroActive(void);

uint32	BaroPressure, BaroTemperature;
boolean AcquiringPressure;
int32	OriginBaroTemperature, OriginBaroPressure;
int24	OriginAltitude, BaroAltitude;
int16 	BaroROC;
i32u	BaroVal;
uint8	BaroType;
int8 	SimulateCycles;
boolean PrimeROC = true;

real32 AltitudeFilter(int32 Alt) {
	static int24 Altp;
	static int16 BaroROCp;
	static int24 NewAlt;

	if (PrimeROC) {
		Altp = NewAlt = Alt;
		AltFiltComp = BaroROCp = 0;
		PrimeROC = false;
	} else {

		AltFiltComp = Alt;
		NewAlt = SlewLimit(Altp, Alt, ALT_SANITY_CHECK_CM);
		NewAlt = AltFilter32(Altp, NewAlt);
		AltFiltComp -= NewAlt;

		BaroROC = (NewAlt - Altp) * ALT_UPDATE_HZ; // simple filter
		BaroROC = SlewLimit(BaroROCp, BaroROC, ALT_SLEW_LIMIT_CM);
		BaroROC = MediumFilter32(BaroROCp, BaroROC);
		BaroROCp = BaroROC;

		Altp = NewAlt;

		if (State == InFlight)
			if (Abs( Alt - Altp ) > (ALT_SANITY_CHECK_CM))
				Stats[BaroFailS]++;

		F.NewBaroValue = true;
	}

	return (NewAlt);
} // AltitudeFilter

const rom uint8 * BaroName[BaroUnknown+1] = {
		"BMP085","MS5611", "No response"
		};		

void ShowBaroType(void) {
	TxString(BaroName[BaroType]);
} // ShowBaroType


void GetBaroAltitude(void) {
	if (F.BaroActive) {
		GetI2CBaroAltitude();	
		if ( F.NewBaroValue ) {
			BaroAltitude = BaroToCm();
			BaroAltitude = AltitudeFilter(BaroAltitude);	
			if ( State == InFlight ) {
				StatsMax(BaroAltitude, BaroAltitudeS);	
				StatsMinMax(BaroROC, MinROCS, MaxROCS);
			} else
				OriginAltitude = BaroAltitude;
		}
	} else
		BaroAltitude = BaroROC = 0;	
} // GetBaroAltitude

void InitBarometer(void) {
	static uint8 i;
	static int32 Alt;
	static uint24 Timeout;
	static uint16 B[7];

	BaroAltitude = OriginBaroPressure = SimulateCycles = BaroROC = Altitude = ROC = 0;
	BaroType = BaroUnknown;

	F.BaroActive = true; // optimistic

	if ( IsBoschBaroActive() ) {
		BaroType = BaroBMP085;
		InitI2CBarometer();
	} else
		if ( IsMS5611BaroActive() ) {
			BaroType = BaroMS5611;
			Delay1mS(3);
			I2CStart();
				WriteI2CByte(MS5611_ID);
				WriteI2CByte(0);
				WriteI2CByte(MS5611_RESET);
			I2CStop();
			Delay1mS(5);

		//	ReadI2Ci16vAtAddr(MS5611_ID, MS5611_PROM + 2, MS5611Constants, 6, false);

			InitI2CBarometer();
		}
		else	
		{
			F.BaroActive = F.HoldingAlt = false;
			Stats[BaroFailS]++;
		}

	if (F.BaroActive) {
		for ( i = 0; i < 50; i++) // attempts to make the "MS5611 warmup"
			F.NewBaroValue = false;
			while (!F.NewBaroValue) { 
				GetI2CBaroAltitude();
				Delay1mS(10);
			}

		OriginBaroPressure = BaroPressure;
		OriginBaroTemperature = BaroTemperature;
		OriginAltitude = BaroToCm();
		BaroAltitude = AltitudeFilter(BaroAltitude);
		SetDesiredAltitude(0);
	}
} // InitBarometer

// -----------------------------------------------------------

// Generic I2C Baro

void BaroFail(void) {
	F.HoldingAlt = false;
	if ( State == InFlight ) {
		Stats[BaroFailS]++; 
		F.BaroFailure = true;
	}
} // BaroFail

void StartI2CBaroADC(boolean ReadPressure){
	static uint8 TempOrPress;

	switch ( BaroType ) {
	case BaroMS5611:
		if ( ReadPressure ) {
			mSTimer(BaroUpdate, MS5611_PRESS_TIME_MS); 
			I2CStart();
				WriteI2CByte(MS5611_ID);
				WriteI2CByte(MS5611_PRESS | OSR);
			I2CStop();
		} else {
			mSTimer(BaroUpdate, MS5611_TEMP_TIME_MS);	
			I2CStart();
				WriteI2CByte(MS5611_ID);
				WriteI2CByte(MS5611_TEMP | OSR);
			I2CStop();
		}
		break;
	case BaroBMP085:
		if ( ReadPressure ) {
			TempOrPress = BOSCH_PRESS;
			mSTimer(BaroUpdate, BOSCH_PRESS_TIME_MS);
		} else {
			mSTimer(BaroUpdate, BOSCH_TEMP_TIME_MS);	
			TempOrPress = BOSCH_TEMP_BMP085;	
		}
	
		WriteI2CByteAtAddr(BOSCH_ID, BOSCH_CTL, TempOrPress); // select 32kHz input
		break;
	default:
		break;
	} // switch	

} // StartI2CBaroADC

void ReadI2CBaro(void) {
	switch ( BaroType ) {
	case BaroMS5611:
		ReadMS5611Baro();
		break;
	case BaroBMP085:
		ReadBoschBaro();
		break;
	default:
		break;
	} // switch

} // ReadI2CBaro

void GetI2CBaroAltitude(void) {
	static int24 Temp;

	F.NewBaroValue = false;
	if ( mSClock() >= mS[BaroUpdate] ) {
 		ReadI2CBaro();
		if ( F.BaroActive )
			if ( AcquiringPressure ) {
				BaroPressure = (int24)BaroVal.u32;
				AcquiringPressure = false;
				F.NewBaroValue = true;
			} else {
				BaroTemperature = (int24)BaroVal.u32;			
				AcquiringPressure = true;			
			}
		else
		{
			AcquiringPressure = true;
			BaroFail();
		}
		StartI2CBaroADC(AcquiringPressure);
	}			
} // GetI2CBaroAltitude

int24 BaroToCm(void) {
	// These calculations are intended to produce approximate altitudes
	// within the code and performance limits of the processor  
	static int32 A, Td, Pd;

	Td = (int32)BaroTemperature - (int32)OriginBaroTemperature;
	Pd = (int32)BaroPressure - (int32)OriginBaroPressure;

	switch ( BaroType ) {
	case BaroBMP085:
		A = Td * (-14); // 14.07  
		A += Pd * (-25); // 24.95 
		break;
	case BaroMS5611:
		//	A = Td * (-0.05435); 
		//	A += Pd * (-0.15966); 
		//A = Td / (-18); // 18.399
		A = -(Td / 3 + Pd) / 6; // 6.263
		break;
	default:
		break;
	} // switch
	
	return ( A );

} // BaroToCm


void InitI2CBarometer(void) {

	F.NewBaroValue = false;

	AcquiringPressure = false; // temperature must be first
	StartI2CBaroADC(AcquiringPressure);

} // InitI2CBarometer

// -----------------------------------------------------------

// Measurement Specialities MS5611 Barometer

void ReadMS5611Baro(void) {
	I2CStart();
		WriteI2CByte(MS5611_ID);
		WriteI2CByte(0);
	I2CStart();
		WriteI2CByte(MS5611_ID | 1);
		BaroVal.b3 = 0;	
		BaroVal.b2 = ReadI2CByte(I2C_ACK);
		BaroVal.b1 = ReadI2CByte(I2C_ACK);
		BaroVal.b0 = ReadI2CByte(I2C_NACK);
	I2CStop();

} // ReadMS5611Baro

boolean IsMS5611BaroActive(void) { // check for MS Barometer

	F.BaroActive = I2CResponse(MS5611_ID);
	return (F.BaroActive);

} // IsMS5611BaroActive

// -----------------------------------------------------------

// Bosch BMP085 Barometer

void ReadBoschBaro(void) {


	// Possible I2C protocol error - split read of ADC
	BaroVal.b3 = 0;
	BaroVal.b2 = 0;

	BaroVal.b0 = ReadI2CByteAtAddr(BOSCH_ID, BOSCH_ADC_LSB);
	BaroVal.b1 = ReadI2CByteAtAddr(BOSCH_ID, BOSCH_ADC_MSB);

} // ReadBoschBaro


boolean IsBoschBaroActive(void) { 
	F.BaroActive = ReadI2CByteAtAddr(BOSCH_ID, BOSCH_TYPE)
		== BOSCH_ID_BMP085;
	return (F.BaroActive);

} // IsBoschBaroActive











