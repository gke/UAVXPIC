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

// 	GPS. routines

#include "uavx.h"

void UpdateField(void);
int32 ConvertGPSTodM(int32);
int24 ConvertInt(uint8, uint8);
int32 ConvertLatLon(uint8, uint8);
int32 ConvertUTime(uint8, uint8);
void ParseGPRMCSentence(void);
void ParseGPGGASentence(void);
void SetGPSOrigin(void);
void UpdateGPSSolution(void);
void InitGPS(void);
void UpdateGPS(void);

const rom uint8 NMEATags[MAX_NMEA_SENTENCES][5]= {
    // NMEA
    {'G','P','G','G','A'}, // full positioning fix
    {'G','P','R','M','C'}, // main current position and heading
};

#pragma udata gpsbuff
NMEAStruct NMEA;
#pragma udata

#pragma udata gpsvars
gpsstruct GPS;
#pragma udata

#pragma udata gpsvars1
uint8 GPSPacketTag, NMEAPacketTag;
uint8 nll, cc, lo, hi;
boolean EmptyField;
int16 ValidGPSSentences;
#pragma udata

int32 ConvertGPSTodM(int32 c) {
	// conversion max is 21Km
	// 0.11131953098f
	return ( ((int32)c * (int32)11132)/((int32)10000) );
} // ConvertGPSTodM

#if defined(USE_UBLOX_BIN)

#define DEFAULT_BAUD_RATES 6
const uint32 DefaultBaud[] = { 4800, 9600, 19200, 38400, 57600, 115200 };

void TxGPSString(const rom uint8 * pch) {
	while (*pch != (char) 0) {
		TxChar(*pch++);
		Delay1mS(5);
	}
} // TxGPSString

//______________________________________________________________________________

// UBlox Code rewritten from AQ (C) Bill Nesbitt

// We only have GPS tx available so initialise GPS manually
const rom char UBLOX_INIT[] = { 
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
    0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
    0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
    0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
    0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
    0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
    0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
  };

#pragma udata ubxvars
ubxstruct ubx;
#pragma udata

void ParseUbxPacket(void) {

	enum UbxFixTypes {
		FixNone = 0,
		FixDeadReckoning = 1,
		Fix2D = 2,
		Fix3D = 3,
		FixGPSDeadReckoning = 4,
		FixTime = 5
	};

	switch (ubx.class) {
	case UBX_NAV_CLASS:
		switch (ubx.id) {
		case UBX_NAV_STATUS:
			GPS.Fix = ubx.payload.status.fixtype;
			F.GPSValid = (ubx.payload.status.fix_status & 1)
					&& ((ubx.payload.status.fixtype == Fix3D
							|| ubx.payload.status.fixtype == Fix2D));
			break;
		case UBX_NAV_SOL:
			GPS.NoOfSats = ubx.payload.sol.satellites;
			break;
		case UBX_NAV_POSLLH:
			GPS.MissionTime = ubx.payload.posllh.iTOW;
			GPS.LatitudeRaw = ubx.payload.posllh.lat;
			GPS.LongitudeRaw = ubx.payload.posllh.lon;
			GPS.Altitude = ubx.payload.posllh.hMSL / 10; // mm => cm
			break;
		case UBX_NAV_VELNED:
			GPS.Vel = ubx.payload.valned.gSpeed / 10; // cm/s => dm/s
			GPS.Heading = ubx.payload.valned.heading * 3142 / 100000;
			break;
		case UBX_NAV_DOP:
			GPS.HDOP = ubx.payload.dop.hDOP / 10;
			break;
		default:
			break;
		}
		break;
	case UBX_TIM_CLASS:
		switch (ubx.id) {
		case UBX_TIM_TP:
			//GPS.TPtowMS = ubx.payload.tp.towMS;
			break;
		default:
			break;
		}
	}

} // ParseUbxPacket

void RxUbxPacket(uint8 RxCh) {

	switch (RxState) {
	case WaitSentinel:
		if (RxCh == UBX_PREAMBLE1)
			RxState = WaitSentinel2;
		break;
	case WaitSentinel2:
		if (RxCh == UBX_PREAMBLE2)
			RxState = WaitClass;
		else
			RxState = WaitSentinel;
		break;
	case WaitClass:
		ubx.class = RxCh;
		ubx.RxCK_A = ubx.RxCK_B = 0;
		ubx.RxCK_A+=RxCh;ubx.RxCK_B+=ubx.RxCK_A;
		RxState = WaitID;
		break;
	case WaitID:
		ubx.id = RxCh;
		ubx.RxCK_A+=RxCh;ubx.RxCK_B+=ubx.RxCK_A;
		RxState = WaitLength;
		break;
	case WaitLength:
		ubx.length = RxCh;
		ubx.RxCK_A+=RxCh;ubx.RxCK_B+=ubx.RxCK_A;
		RxState = WaitLength2;
		break;
	case WaitLength2:
		ubx.length += (RxCh << 8);
		ubx.RxCK_A+=RxCh;ubx.RxCK_B+=ubx.RxCK_A;
		if (ubx.length > 0) {
			ubx.count = 0;
			RxState = WaitBody;
		} else
			RxState = WaitCheckSum;
		break;
	case WaitBody:
		*((uint8 *) (&ubx.payload) + ubx.count) = RxCh;
		if (++ubx.count == ubx.length)
			RxState = WaitCheckSum;
		ubx.RxCK_A+=RxCh;ubx.RxCK_B+=ubx.RxCK_A;
		break;
	case WaitCheckSum:
		if (RxCh == ubx.RxCK_A)
			RxState = WaitCheckSum2;
		else
			RxState = WaitSentinel;
		break;
	case WaitCheckSum2:
		RxState = WaitSentinel;
		F.GPSPacketReceived = RxCh == ubx.RxCK_B;
		if ( F.GPSPacketReceived )
			ParseUbxPacket();
		break;
	default:
		RxState = WaitSentinel;
		break;
	}	
} // RxUbxPacket

//______________________________________________________________________________

#else

int24 ConvertInt(uint8 lo, uint8 hi) {
	static uint8 i;
	static int24 r;

	r = 0;
	if ( !EmptyField )
		for (i = lo; i <= hi ; i++ )
			r = r * 10 + NMEA.s[i] - '0';

	return (r);
} // ConvertInt

int32 ConvertLatLon(uint8 lo, uint8 hi) {
 	// NMEA coordinates normally assumed as DDDMM.MMMMM ie 5 decimal minute digits
	// but code can deal with 4 and 5 decimal minutes 
	static int32 dd, mm, dm, r;
	static uint8 dp;	
	
	r = 0;
	if ( !EmptyField ) {
		dp = lo + 4; // could do this in initialisation for Lat and Lon?
		while ( NMEA.s[dp] != '.') dp++;

	    dd = ConvertInt(lo, dp - 3);
	    mm = ConvertInt(dp - 2 , dp - 1);
		if ( ( hi - dp ) > (uint8)4 )
			dm = ConvertInt(dp + 1, dp + 5);
		else
			dm = ConvertInt(dp + 1, dp + 4) * 10L;
			
	    r = dd * 10000000;
		r += (mm * 10000000 + dm * 100 + 30) / 60;
	}	
	return(r);
} // ConvertLatLon

int32 ConvertUTime(uint8 lo, uint8 hi) {
	static int32 ival;
	
	ival=0;
	if ( !EmptyField )
		ival=(int32)(ConvertInt(lo, lo+1))*3600+
				(int32)(ConvertInt(lo+2, lo+3)*60)+
				(int32)(ConvertInt(lo+4, hi));
	      
	return(ival);
} // ConvertUTime

void UpdateField(void) {
	static uint8 ch;

	lo = cc;
	ch = NMEA.s[cc];
	while (( ch != ',' ) && ( ch != '*' ) && ( cc < nll )) 
		ch = NMEA.s[++cc];

	hi = cc - 1;
	cc++;
	EmptyField = hi < lo;
} // UpdateField

void ParseGPGGASentence(void) { 

	cc = 0;
	nll = NMEA.length;

    UpdateField();
    
    UpdateField();   //UTime
	GPS.MissionTime = ConvertUTime(lo,hi);

	UpdateField();   	//Lat
    GPS.LatitudeRaw = ConvertLatLon(lo, hi);
    UpdateField();   //LatH
    if (NMEA.s[lo] == 'S') GPS.LatitudeRaw = -GPS.LatitudeRaw;

    UpdateField();   	//Lon   
    GPS.LongitudeRaw = ConvertLatLon(lo, hi);
    UpdateField();   	//LonH
	if (NMEA.s[lo] == 'W') GPS.LongitudeRaw = -GPS.LongitudeRaw;
        
    UpdateField();   	//Fix 
    GPS.Fix = (uint8)(ConvertInt(lo,hi));

    UpdateField();   	//Sats
    GPS.NoOfSats = (uint8)(ConvertInt(lo,hi));

    UpdateField();   	// HDOP
	GPS.HDOP = ConvertInt(lo, hi-3) * 10 + ConvertInt(hi-1, hi-1); 

    UpdateField();   	// Alt assume metres!
	GPS.Altitude = ConvertInt(lo, hi-2) * 10L + ConvertInt(hi, hi) * 10; // Decimetres

   	F.GPSValid = (GPS.Fix >= (uint8)GPS_MIN_FIX) && ( GPS.NoOfSats >= (uint8)GPS_MIN_SATELLITES );

	if ( State == InFlight ) {
		StatsMinMax(GPS.HDOP, MinHDOPS, MaxHDOPS);
		StatsMinMax(GPS.NoOfSats, GPSMinSatsS, GPSMaxSatsS);

		F.GPSFailure = GPS.HDOP > 15; 
	}
} // ParseGPGGASentence

void ParseGPRMCSentence() { 	
	// main current position and heading
	static i32u Temp32;

	cc = 0;
	nll = NMEA.length;

    UpdateField();

    UpdateField();   //UTime

    UpdateField();
    if ( NMEA.s[lo] == 'A' ) {
	
		F.GPSValid = true;

        UpdateField();   //Lat
        GPS.LatitudeRaw = ConvertLatLon(lo,hi);
        UpdateField();   //LatH
        if (NMEA.s[lo] == 'S')
        	GPS.LatitudeRaw = -GPS.LatitudeRaw;

        UpdateField();   //Lon
        GPS.LongitudeRaw = ConvertLatLon(lo,hi);

        UpdateField();   //LonH
        if ( NMEA.s[lo] == 'W' )
        	GPS.LongitudeRaw = -GPS.LongitudeRaw;

        UpdateField();   // Groundspeed (Knots)
		GPS.Vel = SRS32(((int32)ConvertInt(lo, hi-3) * 100L + ConvertInt(hi-1, hi)) * 13L, 8);//  5.144444 dMPS/Kt

        UpdateField();   // True course made good (Degrees)
		GPS.Heading = SRS32(((int32)ConvertInt(lo, hi-3) * 100L + ConvertInt(hi-1, hi)) * 45L, 8); // MilliRadians 3142/18000; 

        F.ValidGPSVel = true;
    } else
        F.ValidGPSVel = false;

} // ParseGPRMCSentence


void RxNMEASentence(uint8 RxCh) {

	switch ( RxState ) {
	case WaitCheckSum:
		if (GPSCheckSumChar < (uint8)2) {
			GPSTxCheckSum *= 16;
			if ( RxCh >= 'A' )
				GPSTxCheckSum += ( RxCh - ('A' - 10) );
			else
				GPSTxCheckSum += ( RxCh - '0' );
			GPSCheckSumChar++;
		} else {
			NMEA.length = ll;	
			F.GPSPacketReceived = GPSTxCheckSum == RxCheckSum;
			if ( F.GPSPacketReceived ) 	
				switch ( GPSPacketTag ) {
			    case GPGGAPacketTag:
					ParseGPGGASentence();
					break;
				case GPRMCPacketTag:
					ParseGPRMCSentence();
					break;
				default:
					break;
				}
			RxState = WaitSentinel;
		}
		break;
	case WaitBody: 
		if ( RxCh == '*' ) {
			GPSCheckSumChar = GPSTxCheckSum = 0;
			RxState = WaitCheckSum;
		} else         
			if ( RxCh == '$' ) { // abort partial Sentence 
				ll = tt = RxCheckSum = 0;
				RxState = WaitID;
			} else {
				RxCheckSum ^= RxCh;
				NMEA.s[ll++] = RxCh; 
				if ( ll > (uint8)( GPSRXBUFFLENGTH-1 ) )
					RxState = WaitSentinel;
			}					
			break;
	case WaitID:
		RxCheckSum ^= RxCh;
		while ( ( RxCh != (uint8)NMEATags[ss][tt] ) && ( ss < (uint8)MAX_NMEA_SENTENCES ) ) ss++;
		if ( RxCh == NMEATags[ss][tt] )
	        if ( tt == (uint8)NMEA_TAG_INDEX ) {
				GPSPacketTag = ss;
				RxState = WaitBody;
			} else
				tt++;
		else
			RxState = WaitSentinel;
		break;
	case WaitSentinel: // highest priority skipping unused sentence types
		if ( RxCh == '$' ) {
			ll = tt = ss = RxCheckSum = 0;
			RxState = WaitID;
		}
		break;	
	} 
} // RxNMEASentence

#endif // USE_UBLOX_BIN

void SetGPSOrigin(void) {
	static int32 Temp;

	if ( ( ValidGPSSentences == GPS_ORIGIN_SENTENCES ) && F.GPSValid ) {
		mS[LastGPS] = mSClock();

		#if defined(SIMULATE)
			GPS.LatitudeRaw = (-311833300L);
			GPS.LongitudeRaw = (1368083300L);
		#endif

		GPS.StartTime = GPS.MissionTime;
		GPS.OriginLatitudeRaw = GPS.LatitudeRaw;
		GPS.OriginLongitudeRaw = GPS.LongitudeRaw;
		
		Temp = GPS.LatitudeRaw/1000000L; // to degrees * 10
		Temp = Abs(Temp);
		Temp = ConvertDDegToMPi(Temp);
		GPS.LongitudeCorrection = int16cos(Temp);
	
		GPS.OriginAltitude = GPS.Altitude;

		GPS.Vel = 0;

		Write16EE(NAV_ORIGIN_ALT, (int16)(GPS.Altitude/100));
		Write32EE(NAV_ORIGIN_LAT, GPS.LatitudeRaw);
		Write32EE(NAV_ORIGIN_LON, GPS.LongitudeRaw);

		if ( !F.OriginValid ) {
			DoBeep100mSWithOutput(8,0);
			Stats[NavValidS] = true;
			F.OriginValid = true;
		}
		F.AcquireNewPosition = true;		
	}
} // SetGPSOrigin


void UpdateGPSSolution(void) {
	static int24 NowmS;
	static int16 GPSdT;

	if ( F.GPSValid ) {
		// all coordinates in 0.00001 Minutes or ~1.8553cm relative to Origin
		// There is a lot of jitter in position - could use Kalman Estimator?

		NowmS = mSClock();
		GPSdT = (NowmS - mS[LastGPS]);
		mS[LastGPS] = NowmS;

	    if ( ValidGPSSentences <  GPS_ORIGIN_SENTENCES ) {   
			F.GPSValid = false;
			if ( GPS.HDOP <= GPS_MIN_HDOP )
				ValidGPSSentences++;
			else
				ValidGPSSentences = 0;
		}

		if (F.OriginValid) {
			Nav.C[NorthC].Pos = ConvertGPSTodM(GPS.LatitudeRaw - GPS.OriginLatitudeRaw);
			Nav.C[EastC].Pos = SRS32(ConvertGPSTodM(GPS.LongitudeRaw - GPS.OriginLongitudeRaw)
							* GPS.LongitudeCorrection, 8);

			Nav.C[NorthC].PosP = Nav.C[NorthC].Pos;
			Nav.C[EastC].PosP = Nav.C[EastC].Pos;
		}

		if ( State == InFlight ) {
			StatsMax(GPS.Altitude, GPSAltitudeS);
			StatsMax(GPS.Vel, GPSVelS);
		}
	} else {
		GPS.NoOfSats = ValidGPSSentences;
		if ( State == InFlight ) 
			Stats[GPSInvalidS]++;
	}

} // UpdateGPSSolution

void UpdateGPS(void) {
	static uint8 ch;

	#if defined(SIMULATE)
		GPSEmulation();
	#else
		while ( (!F.GPSPacketReceived) && (RxQTail != RxQHead)) {
			ch = RxQ[RxQHead];
			RxQHead = (RxQHead + 1) & RX_BUFF_MASK;
			#if defined(USE_UBLOX_BIN)
				RxUbxPacket(ch);
			#else
				RxNMEASentence(ch);
			#endif
		}
	#endif // SIMULATE

	if ( F.GPSPacketReceived ) {
		LEDBlue_ON;
		LEDRed_OFF;		
		F.GPSPacketReceived = false; 
		UpdateGPSSolution(); // 3mS 18f2620 @ 40MHz
		if (F.GPSValid) {
			LEDRed_OFF;	
			F.NavigationActive = mSClock() > mS[NavActiveTime];
#ifndef TESTING
			if (!F.NavigationEnabled)
				AcquireHoldPosition();
#endif
			F.NewNavUpdate = F.NavigationEnabled && !F.Bypass;
			mSTimer(GPSTimeout, GPS_TIMEOUT_MS);
		} else {
			F.NavigationActive = false;
			LEDRed_ON;
		}
		LEDBlue_OFF;
	} else {
		if( mSClock() > mS[GPSTimeout] ) {
			F.GPSValid = false;
			LEDRed_ON;
		}
		LEDBlue_OFF;
	}

} // UpdateGPS

void InitGPS(void) {
	cc = 0;

	GPS.LongitudeCorrection = 256; // =>1.0
	GPS.MissionTime = GPS.Fix = GPS.NoOfSats = GPS.HDOP = 0;
	GPS.Altitude = GPS.Heading = GPS.Vel = 0; 
	GPS.OriginLatitudeRaw = GPS.LatitudeRaw = 0;
	GPS.OriginLongitudeRaw = GPS.LongitudeRaw = 0;

	Write32EE(NAV_ORIGIN_LAT, 0);
	Write32EE(NAV_ORIGIN_LON, 0);
	Write16EE(NAV_ORIGIN_ALT, 0);

	ValidGPSSentences = 0;

	F.OriginValid = F.GPSValid = F.GPSPacketReceived = false;
  	RxState = WaitSentinel;

} // InitGPS
