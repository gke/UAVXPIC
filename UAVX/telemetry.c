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

void SendPacketHeader(void);
void SendPacketTrailer(void);
void CheckTelemetry(void);
void SendCycle(void);
void SendControl(void);
void SendMinPacket(void);
void SendFlightPacket(void);
void SendNavPacket(void);
void SendControlPacket(void);
void SendStatsPacket(void);
void SendParamPacket(uint8, uint8);
void SendParameters(uint8);
void SendArduStation(void);
void SendCustom(void);
void ShowAttitude(void);

uint8 UAVXCurrPacketTag;

void CheckTelemetry(void) 
{
	static uint24 NowmS;

	NowmS = mSClock();
	if ( ( NowmS >= mS[TelemetryUpdate] ) && SpareSlotTime )
	{				
		switch ( P[TelemetryType] ) {
		case UAVXTelemetry:
			SpareSlotTime = false;
			mSTimer(TelemetryUpdate, UAVX_TEL_INTERVAL_MS);
			SendCycle(); 	
			break;
		case UAVXMinTelemetry:
			SpareSlotTime = false;
			mSTimer(TelemetryUpdate, UAVX_MIN_TEL_INTERVAL_MS); 
			SendMinPacket(); 	
			break;
		case UAVXControlTelemetry:
			SpareSlotTime = false;
			mSTimer(TelemetryUpdate, UAVX_CONTROL_TEL_INTERVAL_MS); 
			SendControlPacket(); 	
			break;
		case CustomTelemetry:
			SpareSlotTime = false; 
			mSTimer(TelemetryUpdate, CUSTOM_TEL_INTERVAL_MS);
			SendCustom(); 
			break;
		default:
			break;
		}
	}
} // DoTelemetry


#define NAV_STATS_INTERLEAVE	10
static int8 StatsNavAlternate = 0; 

void SendPacketHeader(void) {

	static uint8 b;

	F.TxToBuffer = true;

	#if defined(TELEMETRY_PREAMBLE)
	for (b=10;b;b--) 
		TxChar(0x55);
	#endif // TELEMETRY_PREAMBLE
	      
	TxChar(0xff); // synchronisation to "jolt" USART	
	TxChar(SOH);	
	TxCheckSum = 0;
} // SendPacketHeader

void SendPacketTrailer(void) {

	TxESCu8(TxCheckSum);	
	TxChar(EOT);
	
	TxChar(CR);
	TxChar(LF); 

	F.TxToBuffer = false; 
} // SendPacketTrailer

void ShowAttitude(void) {

	TxESCi16(A[Roll].Desired);
	TxESCi16(A[Pitch].Desired);
	TxESCi16(A[Yaw].Desired);

	TxESCi16(A[Roll].Rate);
	TxESCi16(A[Pitch].Rate);

	TxESCi16(A[Yaw].Rate);

	TxESCi16(A[Roll].Angle);
	TxESCi16(A[Pitch].Angle);

	TxESCi16(HeadingE); 

	TxESCi16(A[Roll].Acc);
    TxESCi16(A[Pitch].Acc);
	TxESCi16(A[Yaw].Acc);

} // ShowAttitude

void SendFlightPacket(void) {
	static uint8 b;

	SendPacketHeader();

	TxESCu8(UAVXFlightPacketTag);
	TxESCu8(55 + MAX_DRIVES * 2 + TELEMETRY_FLAG_BYTES);
	for ( b = 0; b < (uint8)TELEMETRY_FLAG_BYTES; b++ )
		TxESCu8(F.AllFlags[b]); 
		
	TxESCu8(State);	

	TxESCi16(BatteryVoltsADC);
	TxESCi16(BatteryCurrentADC);
	TxESCi16(BatteryChargeUsedmAH);
 
	TxESCi16(RCGlitches);			
	TxESCi16(DesiredThrottle);

	ShowAttitude();

	TxESCi16(ROC); // cm/S
	TxESCi24(BaroAltitude - OriginAltitude);
	
	TxESCi16(NewCruiseThrottle); 				
	TxESCi16(RangefinderAltitude); // cm

	TxESCi16(Make2Pi(Heading));

	TxESCi16(AltFiltComp);
	TxESCi16(AltComp << 2 );
	TxESCi8(AccConfidence * 100);

	TxESCu8(MAX_DRIVES);
	for (b = 0; b < (uint8) MAX_DRIVES; b++) // motor/servo channels
		TxESCi16(PW[b] << 2);

	TxESCi24(mSClock() - mS[StartTime]);

	SendPacketTrailer();
} // SendFlightPacket

void SendControlPacket(void){
	static uint8 b;

	SendPacketHeader();

	TxESCu8(UAVXControlPacketTag);
	TxESCu8(31 + MAX_DRIVES * 2);

	TxESCi16(DesiredThrottle);
 			
	ShowAttitude();

	TxESCu8(AF_TYPE);

	TxESCu8(MAX_DRIVES);
	for ( b = 0; b < (uint8)MAX_DRIVES; b++ ) // motor/servo channels
		TxESCi16((int16)PW[b] << 2);

	TxESCi24(mSClock() - mS[StartTime]);

	SendPacketTrailer();

} // SendControlPacket

void SendNavPacket(void){

	SendPacketHeader();

	TxESCu8(UAVXNavPacketTag);
	TxESCu8(54);
		
	TxESCu8(NavState);
	TxESCu8(FailState);
	TxESCu8(GPS.NoOfSats);
	TxESCu8(GPS.Fix);
	
	TxESCu8(0);	// CurrWP
	
	TxESCi16(GPS.HDOP * 10);

	TxESCi16(Make2Pi(WPHeading));
	TxESCi16(Nav.CrossTrackE);	// cross track error

	TxESCi16(GPS.Vel);
	TxESCi16(Make2Pi(GPS.Heading)); 				// GPS ROC dm/S
	
	TxESCi24(GPS.Altitude); 				// cm
	TxESCi32(GPS.LatitudeRaw); 			// 5 decimal minute units
	TxESCi32(GPS.LongitudeRaw); 
	
	TxESCi24(DesiredAltitude);
	TxESCi32(Nav.C[NorthC].PosE); 
	TxESCi32(Nav.C[EastC].PosE);
	
	TxESCi24(mS[NavStateTimeout] - mSClock());	// mS
	
	TxESCi16(0);			// 0.1C
	TxESCi32(GPS.MissionTime);

	TxESCi16(Nav.Sensitivity << 2);//1000)/RC_MAXIMUM);
	TxESCi16(A[Roll].NavCorr << 2);
	TxESCi16(A[Pitch].NavCorr << 2);
	TxESCi16(A[Yaw].NavCorr << 2);

	SendPacketTrailer();

} // SendNavPacket

void SendStatsPacket(void) {
	static uint8 i;

	SendPacketHeader();

    TxESCu8(UAVXStatsPacketTag);
    TxESCu8(MAX_STATS * 2 + 2);

    for ( i = 0; i < (uint8)MAX_STATS ; i++)
        TxESCi16(Stats[i]);

    TxESCu8(AF_TYPE);
    TxESCu8(0);//Orientation);

	SendPacketTrailer();

} // SendStatsPacket

void SendMinPacket(void) {

#if defined(INC_MIN_PACKET)

	static uint8 b;

	SendPacketHeader();

	TxESCu8(UAVXMinPacketTag);
	TxESCu8(33 + TELEMETRY_FLAG_BYTES);
	for ( b = 0; b < (uint8)TELEMETRY_FLAG_BYTES; b++ )
		TxESCu8(F.AllFlags[b]); 
		
	TxESCu8(State);	
	TxESCu8(NavState);	
	TxESCu8(FailState);

	TxESCi16(BatteryVoltsADC);
	TxESCi16(BatteryCurrentADC);
	TxESCi16(BatteryChargeUsedmAH);	

	TxESCi16(A[Roll].Angle);
	TxESCi16(A[Pitch].Angle);
		
	TxESCi24(BaroAltitude - OriginAltitude);
	TxESCi16(RangefinderAltitude); 		

	TxESCi16(Heading);
	
	TxESCi32(GPS.LatitudeRaw); 					
	TxESCi32(GPS.LongitudeRaw);

	TxESCu8(UAVXAirframe);
	TxESCu8(0);//Orientation);

	TxESCi24(mSClock() - mS[StartTime]);

	SendPacketTrailer();

#endif

} // SendMinPacket


void SendCycle(void) { // 0.8mS at 40MHz
	
	switch ( UAVXCurrPacketTag ) {
	case UAVXFlightPacketTag:
		SendFlightPacket();
		UAVXCurrPacketTag = UAVXNavPacketTag;
		break;	
	case UAVXNavPacketTag:
		if ( ++StatsNavAlternate < NAV_STATS_INTERLEAVE )
			SendNavPacket();		
		else
		{
			SendStatsPacket();
			StatsNavAlternate = 0;
		}
		UAVXCurrPacketTag = UAVXFlightPacketTag;
		break;	
	default:
		UAVXCurrPacketTag = UAVXFlightPacketTag;
		break;		
	} // switch
			
} // SendCycle

int32 aaa,bbb,ccc,ddd,eee,fff,ggg,hhh;

void SendCustom(void) {
 	// user defined telemetry human readable OK for small amounts of data < 1mS

	F.TxToBuffer = true;
	
	// insert values here using TxVal32(n, dp, separator)
	// dp is the scaling to decimal places, separator
	// separator may be a single 'char', HT for tab, or 0 (no space)
	// -> 

	// add user specific code
//	TxVal32(mSClock(),0,',');
	TxVal32(aaa,0,',');
	TxVal32(bbb,0,',');
	TxVal32(ccc,3,',');
//	TxVal32(ddd,0,',');
//	TxVal32(eee,0,',');
//	TxVal32(fff,0,',');
//	TxVal32(ggg,0,',');
//	TxVal32(hhh,0,',');
	// <-

	TxChar(CR);
	TxChar(LF);

	F.TxToBuffer = false;


} // SendCustom





