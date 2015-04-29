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

// Compass 100KHz I2C

#include "uavx.h"

void ShowCompassType(void);
int16 GetCompass(void);
void GetHeading(void);
int16 MinimumTurn(int16);
void GetCompassParameters(void);
void DoCompassTest(void);
void CalibrateCompass(void);
void InitHeading(void);
void InitCompass(void);

int16 GetHMC5883LMagnetometer(void);
void DoTestHMC5883LMagnetometer(void);
void CalibrateHMC5883LMagnetometer(void);
void InitHMC5883LMagnetometer(void);
boolean HMC5883LCompassActive(void);

int16 GetHMC6352Compass(void);
void DoTestHMC6352Compass(void);
void CalibrateHMC6352Compass(void);
void InitHMC6352Compass(void);
boolean HMC6352CompassActive(void);

i24u 	Compass;
int16 	MagHeading, Heading, DesiredHeading, CompassOffset;
uint8 	CompassType;
uint8 	MagRetries;

#pragma idata compass_strings
const rom uint8 * CompassName[CompassUnknown+1] = {
		"HMC6352", "HMC5883L", "No Response"
		};
#pragma idata

void ShowCompassType(void) {
	TxString(CompassName[CompassType]);
} // ShowCompassType		

int16 GetCompass(void) {

	if ( CompassType == HMC5883LMagnetometer )
		return (GetHMC5883LMagnetometer());
	else
		if ( CompassType == HMC6352Compass )
			return (GetHMC6352Compass());
		else
			return(0);	
} // GetCompass
 
void GetHeading(void) {
	static int16 NewHeading, HeadingChange, HeadingP, Temp;

	if ( SpareSlotTime && ( mSClock() >= mS[CompassUpdate] )) {
		mSTimer(CompassUpdate, COMPASS_TIME_MS);
		F.NewMagValues = true;
		SpareSlotTime = false;

		if( F.MagnetometerActive ) // continuous mode but Compass only updates avery 50mS
			MagHeading = GetCompass();
		else
			MagHeading = 0;

		HeadingP = Heading;
		Heading = Make2Pi(MagHeading - CompassOffset);

		HeadingChange = Abs( Heading - HeadingP );
		if ( HeadingChange < MILLIPI )
			HeadingP = Heading;

		Heading = SoftFilter(Heading, HeadingP);
		HeadingP = Heading;

		if (( HeadingChange < MILLIPI ) && ( HeadingChange > COMPASS_MAX_SLEW ))  
			Stats[CompassFailS]++;
	}	
} // GetHeading

int16 MinimumTurn(int16 A ) {
    static int16 AbsA;

    AbsA = Abs(A);
    if ( AbsA > MILLIPI )
        A = ( AbsA - TWOMILLIPI ) * Sign(A);

    return ( A );

} // MinimumTurn

void InitHeading(void) {
	MagHeading = GetCompass();
	Heading = Make2Pi( MagHeading - CompassOffset );
	DesiredHeading = Heading;
} // InitHeading

void InitCompass(void) {

	if ( HMC5883LMagnetometerActive() )
	{	
		CompassType = HMC5883LMagnetometer;
		InitHMC5883LMagnetometer();
	}
	else
		if ( HMC6352CompassActive() )
		{
			CompassType = HMC6352Compass;
			InitHMC6352Compass();
		}
		else
			CompassType = CompassUnknown;

} // InitCompass

#if defined(TESTING) 

void CalibrateCompass(void) {

	if ( CompassType == HMC5883LMagnetometer )
		CalibrateHMC5883LMagnetometer();
	else
		if ( CompassType == HMC6352Compass )
			CalibrateHMC6352Compass();
} // CalibrateCompass

#endif // TESTING

//________________________________________________________________________________________


// HMC5883L 3 Axis Magnetometer

MagStruct Mag[3];

#define HMC5883L_CONFIG_A	0x00
#define HMC5883L_CONFIG_B	0x01
#define HMC5883L_MODE		0x02
#define HMC5883L_DATA		0x03
#define HMC5883L_STATUS		0x09

int16 GetHMC5883LMagnetometer(void) {
	static int16 b[3];
	static int32 Temp;
    static uint8 a;
    static int16 xh, yh;
    static int16 Theta, Phi, CosTheta, SinTheta, CosPhi, SinPhi;
	static int16 CompassVal;
	static MagStruct * M;

	F.MagnetometerActive = ReadI2Ci16vAtAddr(HMC5883L_ID, HMC5883L_DATA, b, 3, true);

	if( F.MagnetometerActive && !((b[MX]==-4096)||(b[MY]==-4096)||(b[MZ]==-4096))) {
		switch ( P[SensorHint] ) {
		default:
			Mag[X].G = -b[MY];
			Mag[Y].G = -b[MX];
			Mag[Z].G = -b[MZ];
			break;
		} // switch
	
		for ( a = X; a<=(uint8)Z; a++ ) {
			M = &Mag[a];

			Temp = Min(M->Min,  M->G);
			M->Min = SlewLimit(M->Min, Temp, 1);
			Temp = Max(M->Max,  M->G);
			M->Max = SlewLimit(M->Max, Temp, 1);

		    M->G -= SRS16(M->Max + M->Min, 1);
		}

		if ( Armed() ) {
			// various sources including:
			// "Applications of Magnetoresistive Sensors in Navigation Systems", 
			// M.J. Caruso, Honeywell Inc.	
			// Aircraft: Pitch +up, Roll +right
			// Magnetometer: Theta away from Z around X, Phi towards Z around Y			
			Theta = A[Roll].Angle; 
			Phi = A[Pitch].Angle; 
	
			Theta = -Limit1(Theta, HALFMILLIPI);
			Phi = Limit1(Phi, HALFMILLIPI);

			CosTheta = int16cos(Theta);
			SinTheta = int16sin(Theta);
			CosPhi = int16cos(Phi);
			SinPhi = int16sin(Phi);	

			Temp = SinPhi * ((int32)Mag[Z].G*CosTheta - (int32)Mag[Y].G*SinTheta);
			xh = SRS32(SRS32(Temp, 8) + (int32)Mag[X].G*CosPhi, 8);
	
			yh = SRS32((int32)Mag[Y].G*CosTheta + (int32)Mag[Z].G*SinTheta, 8);

			CompassVal = -int32atan2( yh, xh );
		} else 
			CompassVal = -int32atan2( Mag[Y].G, Mag[X].G );
	} else
		Stats[CompassFailS]++;

	return ( CompassVal );

} // GetHMC5883LMagnetometer

void InitHMC5883LMagnetometer(void) {
	static uint8 a;
	static int16 C[3];
	static boolean r;
	
	MagRetries = 0;

	do {

		Delay1mS(100);
	
		WriteI2CByteAtAddr(HMC5883L_ID, HMC5883L_CONFIG_A, 0b00010001); // 10Hz normal mode
		WriteI2CByteAtAddr(HMC5883L_ID, HMC5883L_MODE, 0b00010101); 
	
		Delay1mS(10);
	
		r = ReadI2Ci16vAtAddr(HMC5883L_ID, HMC5883L_DATA, C, 3, true);
	
		for ( a = X; a<=(uint8)Z; a++)
			Mag[a].Scale = C[a];

		Delay1mS(10);
	
		WriteI2CByteAtAddr(HMC5883L_ID, HMC5883L_CONFIG_A, 0b00010100); // 20Hz normal mode
		WriteI2CByteAtAddr(HMC5883L_ID, HMC5883L_CONFIG_B, 0b00100000); // default gain
		WriteI2CByteAtAddr(HMC5883L_ID, HMC5883L_MODE, 0x00); // Set continuous mode (default to 10Hz)

		Delay1mS(100);

	} while ( (++MagRetries < MAG_INIT_RETRIES) && (C[X] == C[Y]) && (C[Y] == C[Z]) );

	F.MagnetometerActive = MagRetries < (uint8)MAG_INIT_RETRIES;

	mS[CompassUpdate] = mSClock();

	ReadMagCalEE();

} // InitHMC5883LMagnetometer

boolean HMC5883LMagnetometerActive(void) {
	F.MagnetometerActive = I2CResponse(HMC5883L_ID);

	return (F.MagnetometerActive);
} //  HMC5883LMagnetometerActive


//________________________________________________________________________________________


// HMC6352 Bosch Compass

void WriteHMC6352Command(uint8 c) {
	UseI2C100KHz = true;
	I2CStart(); // Do Bridge Offset Set/Reset now
		WriteI2CByte(HMC6352_ID);
		WriteI2CByte(c);
	I2CStop();
	UseI2C100KHz = false;
} // WriteHMC6352Command

int16 GetHMC6352Compass(void) {
	static i16u CompassVal;

	UseI2C100KHz = true;
	I2CStart();
		WriteI2CByte(HMC6352_ID+1) != I2C_ACK; 
		CompassVal.b1 = ReadI2CByte(I2C_ACK);
		CompassVal.b0 = ReadI2CByte(I2C_NACK);
	I2CStop();
	UseI2C100KHz = false;

	return ( ConvertDDegToMPi( CompassVal.i16 ) );
} // GetHMC6352Compass

#if defined(TESTING)

void CalibrateHMC6352Compass(void) {	
	// calibrate the compass by rotating the ufo through 720 deg smoothly
	static uint8 ch;

	TxString("\r\nCalibrate compass - ");
	TxString("Click CONTINUE to confirm you wish to calibrate or CANCEL\r\n");

	do {
		ch = PollRxChar();
	} while ((ch != 'x') && (ch != 'z'));

	if (ch == 'x') {
		
		WriteHMC6352Command('O'); // Do Set/Reset now	
		Delay1mS(7);	 
		WriteHMC6352Command('C'); // set Compass device to Calibration mode
	
		TxString("\r\nRotate horizontally 720 deg in ~30 sec. - Click CONTINUE to FINISH\r\n");
		while( PollRxChar() != 'x' );
	
		WriteHMC6352Command('E'); // set Compass device to End-Calibration mode 
		TxString("\r\nCalibration complete\r\n");
		Delay1mS(COMPASS_TIME_MS);
	
		InitCompass();
	} else
		TxString("\r\nCancelled");

} // CalibrateHMC6352Compass

#endif
void InitHMC6352Compass(void) {
	// 20Hz continuous read with periodic reset.
	#if defined(SUPPRESS_COMPASS_SR)
		#define COMP_OPMODE 0b01100010
	#else
		#define COMP_OPMODE 0b01110010
	#endif // SUPPRESS_COMPASS_SR

	UseI2C100KHz = true;
	// Set device to Compass mode 
	I2CStart();
		WriteI2CByte(HMC6352_ID);
		WriteI2CByte('G');
		WriteI2CByte(0x74);
		WriteI2CByte(COMP_OPMODE);
	I2CStop();
	UseI2C100KHz = false;

	Delay1mS(1);

	WriteHMC6352Command('L'); // save operation mode in EEPROM
	Delay1mS(1); 
	WriteHMC6352Command('O'); // Do Bridge Offset Set/Reset now
	Delay1mS(COMPASS_TIME_MS);

	// use default heading mode (1/10th degrees)

} // InitHMC6352Compass

boolean HMC6352CompassActive(void) {
	UseI2C100KHz = true;
	F.MagnetometerActive = I2CResponse(HMC6352_ID);
	UseI2C100KHz = false;
	return(F.MagnetometerActive);

} // HMC6352CompassActive

//________________________________________________________________________________________

void WriteMagCalEE(void) {
	static uint8 a;
	static MagStruct * M;

	if ( CompassType == HMC5883LMagnetometer ) 
		for ( a = X; a<=(uint8)Z; a++) {
			M = &Mag[a];
			Write16EE(MAG_BIAS_ADDR_EE + (a*4), M->Min);
			Write16EE(MAG_BIAS_ADDR_EE + (a*4+2), M->Max);
		}
} // WriteMagCalEE

void ReadMagCalEE(void) {
	static uint8 a;
	static MagStruct * M;

	if ( CompassType == HMC5883LMagnetometer) 
		for ( a = X; a<=(uint8)Z; a++) {
			M = &Mag[a];
			M->Min = Read16EE(MAG_BIAS_ADDR_EE + (a*4));
			M->Max = Read16EE(MAG_BIAS_ADDR_EE + (a*4+2));
		}
} // ReadMagCalEE



