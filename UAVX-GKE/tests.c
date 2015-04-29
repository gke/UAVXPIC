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

#if defined(TESTING)

void DoLEDs(void);
void ReceiverTest(void);
void PowerOutput(int8);
void BatteryTest(void);

void DoLEDs(void) {
	if( F.Signal ) {
		LEDRed_OFF;
		LEDGreen_ON;
	} else {
		LEDGreen_OFF;
		LEDRed_ON;
	}
} // DoLEDs

//________________________________________________________________________________________

void ReceiverTest(void) {
	static uint8 s;
	static uint16 v;

	TxString("\r\nRx: ");
	ShowRxSetup();
	TxString("\r\n");
	
	TxString("\tRAW RC - neutrals NOT applied\r\n");

	TxString("\tChannel order is: ");
	for ( s = 0; s < NoOfRCChannels; s++)
		TxChar(RxChMnem[RMap[s]]);

	if ( F.Signal )
		TxString("\r\nSignal OK ");
	else
		TxString("\r\nSignal FAIL ");	
	TxVal32(mSClock() - mS[LastValidRx], 0, 0);
	TxString(" mS ago\r\n");
	
	// Be wary as new RC frames are being received as this
	// is being displayed so data may be from overlapping frames

	for ( s = 0; s < NoOfRCChannels; s++ ) {
		TxChar(RxChMnem[RMap[s]]);
		TxString(":\t");
		TxVal32(( PPM[s].i16 * 8L + 5L ) / 10L, 3, 0);
		TxChar(HT);
		TxVal32(((int32)PPM[s].i16*100L + 625L ) / 1250L, 0, '%');
		if ( ( PPM[s].i16 < 0 ) || ( PPM[s].i16 > 1250 ) ) 
			TxString(" FAIL");
		TxNextLine();
	}

	// show pause time
	TxString("Gap:\t");
	v = ( PauseTime * 8L + 5 )/10L;
	TxVal32( v, 3, 0);		
	TxString("mS\r\n");
	TxString("Glitches:\t");
	TxVal32(RCGlitches,0,0);
	TxNextLine();

} // ReceiverTest

//________________________________________________________________________________________

void GetNeutralAccelerations(void) {
	static uint8 i, a, ch;
	static int16 Temp[3], b;

	TxString("\r\nCalibrate accelerometers - ");
	TxString("Click CONTINUE to confirm you wish to calibrate or CANCEL\r\n");

	do {
		ch = PollRxChar();
	} while ((ch != 'x') && (ch != 'z'));

	if (ch == 'x') {
		Temp[LR] = Temp[FB] = Temp[DU] = 0;
		if ( F.IMUActive` ) {
			for ( i = 16; i; i--) {
				
				GetRatesAndAccelerations();

				for ( a = LR; a<=(uint8)DU; a++ )
					Temp[a] += A[a].AccADC;
				Delay1mS(10);
			}	
					
			for ( a = LR; a<=(uint8)DU; a++ ) 
				A[a].AccBias = SRS32(Temp[a], 4);
			A[DU].AccBias -= GRAVITY;

			TxNextLine();
			TxString("\tL->R: \t");TxVal32(A[LR].AccBias,0,0);
			if ( Abs(A[LR].AccBias) > 100 ) 
				TxString(" >100 - not level?");
			TxNextLine();
			TxString("\tF->B: \t");TxVal32(A[FB].AccBias,0,0); 
			if ( Abs(A[FB].AccBias) > 100 ) 
				TxString(" >100 - not level?");
			TxNextLine();
			TxString("\tD->U: \t");TxVal32(A[DU].AccBias,0,0); 
			TxNextLine();

			WriteAccCalEE();	
		} else {
			TxString("\r\nAccelerometer read failure\r\n");
			A[LR].AccBias = A[FB].AccBias = A[DU].AccBias = 0;
		}
	} else
		TxString("\r\nCancelled");

} // GetNeutralAccelerations

//________________________________________________________________________________________


void GyrosAndAccsTest(void) {
	static int16 Mag;

	GetRatesAndAccelerations();

	TxString("\r\nGyros & Accs test\r\n\r\nAccs: ");
	ShowAccType();
	if (F.IMUActive) 
		TxString(" (OK)"); 
	else 
		TxString(" (FAIL)");
	TxString("\r\nGyros: ");
	ShowGyroType(GyroType);
	TxNextLine();

	if ( AccType == MPU6050Acc ) {
		TxString("\r\nDLPF: ");
		if ((MPU6050DLPF >= MPU_RA_DLPF_BW_256) && (MPU6050DLPF
						<= MPU_RA_DLPF_BW_42)) {
			TxVal32(InertialLPFHz[MPU6050DLPF],0,0);
			TxString("Hz");
		} else
			TxString("unknown");
		
		TxString(" DHPF: ");
		if ((MPU6050DHPF >= MPU_RA_DHPF_RESET) && (MPU6050DLPF
						<= MPU_RA_DHPF_HOLD)) 
			TxString(DHPFName[MPU6050DHPF]);
		else
			TxString("unknown\r\n");
		}
	
	TxString("\r\nAccMag:\t");
	Mag = int32sqrt(Sqr((int24)A[Roll].AccADC)+Sqr((int24)A[Pitch].AccADC)+Sqr((int24)A[Yaw].AccADC));
	TxVal32((int32)Mag, 0, 0);
	
	TxString("\r\n\r\n\tRoll: \t");
	TxVal32(A[Roll].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Roll].GyroBias,0, ')');
	TxString("\tL->R: \t");
	TxVal32((int32)A[Roll].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Roll].AccBias, 0, ')');

	TxString("\r\n\tPitch:\t");
	TxVal32(A[Pitch].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Pitch].GyroBias,0, ')');
	TxString("\tF->B: \t");	
	TxVal32((int32)A[Pitch].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Pitch].AccBias, 0, ')');	

	TxString("\r\n\tYaw:  \t");
	TxVal32(A[Yaw].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Yaw].GyroBias,0, ')');
	TxString("\tD->U:    \t");
	
	TxVal32((int32)A[Yaw].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Yaw].AccBias, 0, ')');	

} // GyrosAndAccsTest

//________________________________________________________________________________________


void DoCompassTest(void) {
	TxString("\r\nCompass test - ");
	
	if ( CompassType == HMC5883LMagnetometer )
		DoTestHMC5883LMagnetometer();
	else
		if ( CompassType == HMC6352Compass )
			DoTestHMC6352Compass();
		else
			TxString("not installed?\r\n");
} // DoCompassTest

void InitialHMC5883LMagnetometerBias(void) {

	// Nominal x/y 766, z 660 @1Ga
	Mag[FB].Min = Mag[LR].Min = -500;
	Mag[FB].Max = Mag[LR].Max = 500;
	Mag[DU].Min = -400;
	Mag[DU].Max = 400;

} // InitialHMC5883LMagnetometerBias

void CalibrateHMC5883LMagnetometer(void) {

	InitialHMC5883LMagnetometerBias();

	TxString(
			"\r\nRotate in all directions including upside down\r\nPress the CONTINUE button (x) to FINISH\r\n");
	while (PollRxChar() != 'x') {
		SpareSlotTime = true;
		while ((mSClock() < mS[CompassUpdate])) {
		};

		GetHMC5883LMagnetometer();
	}
	WriteMagCalEE();

	DoTestHMC5883LMagnetometer();

} // CalibrateHMC5883Magnetometer

#define HMC5883L_STATUS		0x09

void DoTestHMC5883LMagnetometer(void) {
	static int32 Temp;
	static uint8 a, i, status;
	static MagStruct * M;

	ShowCompassType();

	status = ReadI2CByteAtAddr(HMC5883L_ID,HMC5883L_STATUS);
	MagHeading = Make2Pi(GetCompass());
	WriteMagCalEE();
	Heading = Make2Pi( MagHeading - CompassOffset );

	if ( F.MagnetometerActive ) {
		TxString("\r\n\r\nStatus:\t0x");
		TxValH(status);
		TxString("\r\nRetries:\t");
		TxVal32(MagRetries - 1 ,0,0);
		
	    TxString("\r\n\t\tMag \tMin \tMax \tBias \tRef.\r\n");
		for ( a = X; a<=(uint8)Z; a++ ) {
			M = &Mag[a];
			TxChar(HT);
			TxChar(a+'X');
			TxString(":\t");
		    TxVal32(M->G, 0, HT);
			TxVal32(M->Min, 0, HT);
			TxVal32(M->Max, 0, HT);
			TxVal32(SRS16(M->Max + M->Min, 1), 0, HT);
			TxVal32(M->Scale, 0, HT);
		    TxNextLine();
		}
	
		TxVal32(ConvertMPiToDDeg(MagHeading), 1, 0);
	 	TxString(" deg (Compass)\r\n");
	    TxVal32(ConvertMPiToDDeg(Heading), 1, 0);
	    TxString(" deg (True)\r\n");

		WriteMagCalEE();
	} else
		TxString(" Fail\r\n");

} // DoHMC5883LTest

//________________________________________________________________________________________


#define TEST_COMP_OPMODE 0b01110000	// standby mode to reliably read EEPROM

void GetHMC6352Parameters(void) {
	#if defined(FULL_COMPASS_TEST)

	static uint8 r;

	UseI2C100KHz = true;
	I2CStart();
		WriteI2CByte(HMC6352_ID);
		WriteI2CByte('G');
		WriteI2CByte(0x74);
		WriteI2CByte(TEST_COMP_OPMODE);
	I2CStop();

	Delay1mS(COMPASS_TIME_MS);

	for (r = 0; r <= (uint8)8; r++) { // do NOT use a block read
		CP[r] = 0xff;

		Delay1mS(10);

		I2CStart();
			WriteI2CByte(HMC6352_ID);
			WriteI2CByte('r');
			WriteI2CByte(r);
		I2CStop();

		Delay1mS(10);

		I2CStart();
			WriteI2CByte(HMC6352_ID+1);
			CP[r] = ReadI2CByte(I2C_NACK);
		I2CStop();
	}

	UseI2C100KHz = false;

	Delay1mS(7);

	#endif // FULL_COMPASS_TEST

} // GetHMC6352Parameters

static uint8 CP[9];

void DoTestHMC6352Compass(void) {
	static uint16 v, prev;
	static int16 Temp;
	static uint8 i;
	static boolean r;

	TxString("HMC6352\r\n");

	UseI2C100KHz = true;

	#if defined(FULL_COMPASS_TEST)

	I2CStart();
		WriteI2CByte(HMC6352_ID);
		WriteI2CByte('G');
		WriteI2CByte(0x74);
		WriteI2CByte(TEST_COMP_OPMODE);
	I2CStop();

	UseI2C100KHz = false;

	Delay1mS(1);

	WriteHMC6352Command('O'); // reset

	Delay1mS(7);

	GetHMC6352Parameters();

	TxString("Registers\r\n");
	TxString("0:\tI2C"); 
	TxString("\t 0x"); TxValH(CP[0]); 
	if ( CP[0] != (uint8)0x42 ) 
		TxString("\t Error expected 0x42 for HMC6352");
	TxNextLine();

	Temp = (CP[1]*256)|CP[2];
	TxString("1:2:\tXOffset\t"); 
	TxVal32((int32)Temp, 0, 0); 
	TxNextLine(); 

	Temp = (CP[3]*256)|CP[4];
	TxString("3:4:\tYOffset\t"); 
	TxVal32((int32)Temp, 0, 0); 
	TxNextLine(); 

	TxString("5:\tDelay\t"); 
	TxVal32((int32)CP[5], 0, 0); 
	TxNextLine(); 

	TxString("6:\tNSum\t"); TxVal32((int32)CP[6], 0, 0);
	TxNextLine(); 

	TxString("7:\tSW Ver\t"); 
	TxString(" 0x"); TxValH(CP[7]); 
	TxNextLine(); 

	TxString("8:\tOpMode:");
	switch ( ( CP[8] >> 5 ) & 0x03 ) {
		case 0: TxString("  1Hz"); break;
		case 1: TxString("  5Hz"); break;
		case 2: TxString("  10Hz"); break;
		case 3: TxString("  20Hz"); break;
		}
 
	if ( CP[8] & 0x10 ) TxString(" S/R"); 

	switch ( CP[8] & 0x03 ) {
		case 0: TxString(" Standby"); break;
		case 1: TxString(" Query"); break;
		case 2: TxString(" Continuous"); break;
		case 3: TxString(" Not-allowed"); break;
	}

	Delay1mS(500);

	#endif // FULL_COMPASS_TEST

	TxNextLine();

	MagHeading = GetCompass();
	Heading = Make2Pi( MagHeading - CompassOffset );

    TxVal32(ConvertMPiToDDeg(MagHeading), 1, 0);
    TxString(" deg (Compass)\r\n");
    TxVal32(ConvertMPiToDDeg(Heading), 1, 0);
    TxString(" deg (True)\r\n");

} // DoTestHMC6352Compass


//________________________________________________________________________________________


void BaroTest(void) {
	static uint8 i;

	TxString("\r\nAltitude test - ");
	ShowBaroType();

	if ( F.BaroActive ) {	
		F.NewBaroValue = false;
		while ( !F.NewBaroValue )
			GetBaroAltitude();	
		F.NewBaroValue = false;
		TxString("\r\nP/T Raw: ");
		TxVal32( BaroPressure, 0, ' ');
		TxVal32( BaroTemperature, 0, 0);
		TxNextLine();
		TxString("Rel. Density Alt.: ");
		TxVal32( (int32) BaroAltitude - OriginAltitude, 2, ' ');
		TxString("M\r\n");
		TxString("ROC: ");
		TxVal32( (int32) BaroROC, 2, ' ');
		TxString("M\r\n");

		if (F.RangefinderActive) {
			TxString("\r\nR.Finder: ");
			GetRangefinderAltitude();
			TxVal32( (int32) RangefinderAltitude, 2, ' ');
			TxString("M\r\n");
		} else
			TxString("No rangefinder\r\n");
	}

} // BaroTest

//________________________________________________________________________________________


void ShowI2CDeviceName(uint8 d) {
	// could be a full table lookup?

    TxChar(' ');
    switch ( d  ) {
        case MPU6050_0xD0_ID:
        case MPU6050_0xD2_ID:
            TxString("MPU6050");
           break; 
       	 case BOSCH_ID:
         	TxString("Bosch/MS  Baro");
            break; 
		 case HMC5883L_ID:
         	TxString("HMC5883L Mag");
            break;
		case HMC6352_ID:
			TxString("HMC6352 Comp");
            break;
        default:
			TxString("unknown");
            break;
    } // switch
    TxChar(' ');

} // ShowI2CDeviceName

uint8 ScanI2CBus(void) {
	uint8 s;
	uint8 d;

	d = 0;

	UseI2C100KHz = true;
	TxString("I2C Bus\r\n");
	for ( s = 0x10 ; s <= 0xf6 ; s += 2 ) {
		if( I2CResponse(s) ) {
			d++;
			TxString("\t0x");
			TxValH(s>>1);
			TxString("/0x");
			TxValH(s);
			ShowI2CDeviceName(s);
			TxNextLine();
		}
		Delay1mS(2);
	}
	UseI2C100KHz = false;

	return(d);
} // ScanI2CBus

//________________________________________________________________________________________


#if defined(INC_I2C_PROG)

void ProgramSlaveAddress(uint8 addr) {
	static uint8 s;

	for (s = 0x10 ; s < 0xf0 ; s += 2 ) {
		ESCI2CStart();
		if( WriteESCI2CByte(s) == I2C_ACK )
			if( s == addr ) { // ESC is already programmed OK
				ESCI2CStop();
				TxString("\tESC at SLA 0x");
				TxValH(addr);
				TxString(" is already programmed OK\r\n");
				return;
			} else
				if( WriteESCI2CByte(0x87) == I2C_ACK ) // select register 0x07
					if( WriteESCI2CByte( addr ) == I2C_ACK ) { // new slave address
						ESCI2CStop();
						TxString("\tESC at SLA 0x");
						TxValH(s);
						TxString(" reprogrammed to SLA 0x");
						TxValH(addr);
						TxNextLine();
						return;
					}
		ESCI2CStop();
	}
	TxString("\tESC at SLA 0x");
	TxValH(addr);
	TxString(" no response - check cabling and pullup resistors!\r\n");
} // ProgramSlaveAddress

void ConfigureESCs(void) {
	static uint8 m;

	if ( (int8)P[ESCType] == ESCYGEI2C ) {
		TxString("\r\nProgram YGE ESCs\r\n");
		for ( m = 0 ; m < (uint8)NO_OF_I2C_ESCS ; m++ ) {
			TxString("Connect ONLY ");
			switch( m ) {
			#if defined(Y6COPTER)
				case 0 : TxString("FrontT"); break;
				case 1 : TxString("LeftT");  break;
				case 2 : TxString("RightT"); break;
				case 3 : TxString("FrontB"); break;
				case 4 : TxString("LeftB"); break;
				case 5 : TxString("RightB"); break;
			#else
				#if defined(HEXACOPTER) | defined(HEXACOPTERX)
					case 0 : TxString("Front"); break;
					case 1 : TxString("LeftFront");  break;
					case 2 : TxString("RightFront"); break;
					case 3 : TxString("LeftBack"); break;
					case 4 : TxString("RightBack"); break;
					case 5 : TxString("Back"); break;
				#else
					#if defined(TRICOPTER)
						case 1 : TxString("Left");  break;
						case 2 : TxString("Right"); break;
					#else
						case 0 : TxString("Front"); break;
						case 1 : TxString("Left");  break;
						case 2 : TxString("Right"); break;
						case 3 : TxString("Back"); break;
					#endif // TRICOPTER
				#endif // HEXACOPTER | HEXACOPTER
			#endif // Y6COPTER
			}
			TxString(" ESC, then the CONTINUE button \r\n");
			while( PollRxChar() != 'x' ); // UAVPSet uses 'x' for CONTINUE button
		//	TxString("\r\n");
			ProgramSlaveAddress( 0x62 + ( m*2 ));
		}
		TxString("\r\nConnect ALL ESCs and power-cycle the Quadrocopter\r\n");
	} else
		TxString("\r\nYGEI2C not selected as ESC?\r\n");
} // ConfigureESCs

#endif // INC_I2C_PROG

#endif // TESTING

void BatteryTest(void) {
	static int32 v;

	TxString("\r\nBattery test\r\n");

	// Battery
	v = SRS32(ADC(ADCBattVoltsChan) * 278L, 10); // resistive divider 
	TxString("Batt:\t");
	TxVal32(v, 1, 'V');
	TxString(" Limit > ");
	v = SRS32(BatteryVoltsLimitADC * 278L, 10); // resistive divider ADCBattVoltsChan
	TxVal32(v, 1, 'V');
	TxNextLine();
	
} // BatteryTest

#if defined(SIMULATE)

#define EMU_DT_MS	100L
#define EMU_DTR		(1000L/EMU_DT_MS)

int32 HRBaroAltitude = 0;

void DoEmulation(void) {
	static uint24 UpdatemS = 0;
	
	if (Armed() && (mSClock() > UpdatemS)) {
		UpdatemS = mSClock() + EMU_DT_MS;

		BaroROC = (DesiredThrottle - CruiseThrottle + AltComp) * 2;
		HRBaroAltitude += BaroROC; 
		BaroAltitude = HRBaroAltitude / EMU_DTR;
		if ((HRBaroAltitude) < 10)
			BaroROC = HRBaroAltitude = BaroAltitude = 0;

		GPS.Altitude = BaroAltitude;
		F.NewBaroValue = true;
		F.BaroActive = true;

		A[Roll].Angle = ((int32)A[Roll].Control * 58);//DEG_TO_ANGLE_UNITS * 45) / RC_NEUTRAL;
		A[Pitch].Angle = ((int32)A[Pitch].Control * 58);//DEG_TO_ANGLE_UNITS * 45) / RC_NEUTRAL;

		Rl = -A[Roll].Control; 
		Pl = -A[Pitch].Control; 
		Yl = -A[Yaw].Control;

		MagHeading -= (A[Yaw].Control);// * DegreesToRadians(180) ) / (EMU_DTR * RC_NEUTRAL);

		MagHeading = Make2Pi(MagHeading);
		Heading = Make2Pi(MagHeading  - CompassOffset );
	}

} // DoEmulation

int32 ConvertdMToGPS(int32 c) {
	return ( ((int32)c * (int32)10000)/((int32)1855) );
} // ConvertdMToGPS

void GPSEmulation(void) {
	
	#define FAKE_NORTH_WIND 0
	#define FAKE_EAST_WIND 0
	#define SIM_CRUISE_DMPS 70L
	#define SIM_MAX_DMPS 100L

	static int16 NorthDiff, EastDiff;
	static int24 PitchDiff, RollDiff;
	static uint24 NextGPS = 0;

	if ( mSClock( )> NextGPS ) {
		NextGPS += 1000/ GPS_UPDATE_HZ ;
		F.GPSPacketReceived = true;

	//		PitchDiff = -((int32)A[Pitch].Control * SIM_MAX_DMPS) / RC_NEUTRAL; 
	//		RollDiff = ((int32)A[Roll].Control * SIM_MAX_DMPS) / RC_NEUTRAL; 
		PitchDiff = -SRS16(A[Pitch].Control, 1); 
		RollDiff = SRS16(A[Roll].Control, 1); 
	
		Rotate(&NorthDiff, &EastDiff, PitchDiff, RollDiff, -Heading);
	
		GPS.Vel = int32sqrt(Sqr(PitchDiff) + Sqr(RollDiff));
		F.ValidGPSVel = true;
	
		NorthDiff += FAKE_NORTH_WIND;
		EastDiff += FAKE_EAST_WIND;
	
		GPS.LongitudeRaw += ConvertdMToGPS(EastDiff) / GPS_UPDATE_HZ;
		GPS.LatitudeRaw += ConvertdMToGPS(NorthDiff) / GPS_UPDATE_HZ;
	
		GPS.Heading = Heading;
	
		GPSPacketTag = GPGGAPacketTag;
		GPS.Fix = 3;
		GPS.NoOfSats = 10;
		GPS.HDOP = 5;
		F.GPSValid = true;
	}

} // EmulateGPS

#endif // SIMULATE

