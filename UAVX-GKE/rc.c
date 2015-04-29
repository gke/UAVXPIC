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

void InitRC(void);
void MapRC(void);
void CheckSticksHaveChanged(void);
void UpdateControls(void);
void CheckThrottleMoved(void);

uint8 Map[CONTROLS], RMap[CONTROLS];
int16 RC[CONTROLS], RCp[CONTROLS];
int16 CruiseThrottle, NewCruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
int16 DesiredCamPitchTrim;
int16 ThrLow, ThrHigh, ThrNeutral;

void UpdateRCMap(void) {
	static uint8 i;

	for (i = 0; i < (uint8) CONTROLS; i++)
		Map[i] = i;

	Map[ThrottleRC] = P[RxThrottleCh];
	Map[RollRC] = P[RxRollCh];
	Map[PitchRC] = P[RxPitchCh];
	Map[YawRC] = P[RxYawCh];
	Map[RTHRC] = P[RxGearCh];
	Map[RateControlRC] = P[RxAux1Ch];
	Map[NavGainRC] = P[RxAux2Ch];
	Map[BypassRC]= P[RxAux3Ch];
	Map[CamPitchRC] = P[RxAux4Ch];

	for (i = ThrottleRC; i <= (uint8) CamPitchRC; i++)
		Map[i] -= 1;

	for ( i = 0; i < (uint8)CONTROLS; i++) // make reverse map
		RMap[Map[i]] = i;

} // UpdateRCMap

void InitRC(void) {
	static uint8 c;
	
	for (c = 0; c < (uint8)CONTROLS; c++)
		PPM[c].i16 = RC[c] = RCp[c] = 0;

	for (c = RollRC; c <= YawRC; c++)
		RC[c] = RCp[c] = RC_NEUTRAL;
	RC[CamPitchRC] = RCp[CamPitchRC] = RC_NEUTRAL;
	
	F.UsingCompoundPPM = P[RCType] == CompoundPPM;
	PIE1bits.CCP1IE = false;
	CCP1CONbits.CCP1M0 = true;
	PPM_Index = PrevEdge = RCGlitches = 0;
	PIE1bits.CCP1IE = true;

	SignalCount = -RC_GOOD_BUCKET_MAX;
	F.Signal = F.RCNewValues = false;

	UpdateRCMap();

	DesiredThrottle = StickThrottle = 0; 
	DesiredCamPitchTrim = 0;
	Nav.Sensitivity = 0;
	F.ReturnHome = F.Navigate = F.AltControlEnabled = false;

	mS[StickChangeUpdate] = mSClock();
	mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
	SignalCount = -RC_GOOD_BUCKET_MAX;

} // InitRC

void MapRC(void) {  // re-maps captured PPM to Rx channel sequence
	static uint8 c, cc;
	static int16 LastThrottle, Temp;
	static i32u Temp2;

	LastThrottle = RC[ThrottleRC];

	for (c = 0 ; c < (uint8)NoOfRCChannels ; c++) {
		cc = RMap[c];
		//Temp = ((int32)PPM[i].i16 * RC_MAXIMUM + 625L)/1250L; // scale to 4uS res. for now
		Temp2.i32 = ((int32)PPM[c].i16 * (RC_MAXIMUM*53L)  );
		RC[cc] = RxFilter(RC[cc], Temp2.iw1);					
	}
} // MapRC


void CheckSticksHaveChanged(void)
{
	#if !defined(TESTING)

	static uint32 NowmS;
	static boolean Change;
	static uint8 c;

	if ( F.FailsafesEnabled )
	{
		NowmS = mSClock();
		if ( F.ReturnHome ) {
			Change = true;
			mS[RxFailsafeTimeout] = NowmS + RC_NO_CHANGE_TIMEOUT_MS;			
		} else {
			if ( NowmS > mS[StickChangeUpdate] ) {
				mS[StickChangeUpdate] = NowmS + 500;
		
				Change = false;
				for ( c = ThrottleC; c <= (uint8)RTHRC; c++ ) {
					Change |= Abs( RC[c] - RCp[c]) > RC_STICK_MOVEMENT;
					RCp[c] = RC[c];
				}
			}
		
			if ( Change ) {
				mS[RxFailsafeTimeout] = NowmS + RC_NO_CHANGE_TIMEOUT_MS;
				mS[NavStateTimeout] = NowmS;
	
				if ( FailState == Monitoring ) {
					if ( F.LostModel ) {
						Beeper_OFF;
						F.LostModel = false;
						DescentComp = 1;
					}
				}
			} else
				if ( NowmS > mS[RxFailsafeTimeout] ) {
					if ( State == InFlight  )
					{
						//Stats[RCFailsafesS]++;
						mS[NavStateTimeout] = NowmS + NAV_RTH_LAND_TIMEOUT_MS;
						mS[DescentUpdate] = NowmS + ALT_DESCENT_UPDATE_MS;
						DescentComp = 1; // for no Baro case
					}
				}
		}
	} 

	#endif 
} // CheckSticksHaveChanged

void UpdateControls(void) {
	static int16 Temp;
	static uint8 c;

	F.RCNewValues = false;
	MapRC();								// remap channel order for specific Tx/Rx
	StickThrottle = RC[ThrottleRC];

	//_________________________________________________________________________________________

	// Stick Processing

	if (NoOfRCChannels > Map[RTHRC]) {
		Temp = ((uint16)RC[RTHRC] * 3) >> 8; // RC_MAXIMUM is 256
		NavSwState = Limit(Temp, NavSwLow, NavSwHigh);
		UpdateRTHSwState();
	} else {
		F.ReturnHome = F.Navigate = false;
		F.NavigationEnabled = false;
	}

	if (NoOfRCChannels > Map[NavGainRC]) {
		Nav.Sensitivity = RC[NavGainRC];
		Nav.Sensitivity = Limit(Nav.Sensitivity, 0, RC_MAXIMUM);
		F.AltControlEnabled = Nav.Sensitivity > FromPercent(NAV_SENS_ALTHOLD_THRESHOLD, RC_MAXIMUM);	
	} else {
		Nav.Sensitivity = FromPercent(50, RC_MAXIMUM);
		F.AltControlEnabled = true;
	}

	if (NoOfRCChannels > Map[CamPitchRC])
		DesiredCamPitchTrim = RC[CamPitchRC] - RC_NEUTRAL;
	else
		DesiredCamPitchTrim = RC_NEUTRAL;

	#if defined(GKE_TUNE)
		TuneTrim = SRS16(DesiredCamPitchTrim, 4);
	#endif

	F.UsingRateControl = (NoOfRCChannels > Map[RateControlRC])
			&& (RC[RateControlRC] > FromPercent(70, RC_MAXIMUM))
			&& !(F.ReturnHome || F.Navigate);
	if (F.UsingRateControl) F.ReturnHome = false;	// no Nav in rate/aerobatics mode

	#if defined(MULTICOPTER)
		F.Bypass = false;
	#else
		F.Bypass = (NoOfRCChannels > Map[BypassRC]) && (RC[BypassRC] > (RC_MAXIMUM/2L));
	#endif

	//_________________________________________________________________________________________

	// Altitude Hold

	if (( NavState == HoldingStation )||( NavState == PIC))
	{ // Manual
		if ( StickThrottle < RC_THRES_STOP )	// to deal with usual non-zero EPA
			StickThrottle = 0;
	}
	else // Autonomous
		if ( F.AltControlEnabled )
			StickThrottle = CruiseThrottle;

	if ( (! F.HoldingAlt) && !(F.ReturnHome ) ) // cancel any current altitude hold setting 
		SetDesiredAltitude(Altitude);	

	//_________________________________________________________________________________________
			
	// Attitude
		
	A[Roll].Desired = RC[RollRC] - RC_NEUTRAL;
	A[Pitch].Desired = RC[PitchRC] - RC_NEUTRAL;		
	A[Yaw].Desired = RC[YawRC] - RC_NEUTRAL;
	
	CurrMaxRollPitch = Max(A[Roll].Desired, A[Pitch].Desired);
		
	if ( CurrMaxRollPitch > ATTITUDE_HOLD_LIMIT )
		if ( AttitudeHoldResetCount > ATTITUDE_HOLD_RESET_INTERVAL )
			F.AttitudeHold = false;
		else {
			AttitudeHoldResetCount++;
			F.AttitudeHold = true;
		} else {
			F.AttitudeHold = true;	
			if ( AttitudeHoldResetCount > 1 )
				AttitudeHoldResetCount -= 2; // Faster decay
	}

	//_________________________________________________________________________________________
			
	// Rx has gone to failsafe

	CheckSticksHaveChanged();
	
	F.NewCommands = true;

} // UpdateControls

void CheckThrottleMoved(void)
{
	if( mSClock() < mS[ThrottleUpdate] )
		ThrNeutral = DesiredThrottle;
	else {
		ThrLow = ThrNeutral - THROTTLE_MIDDLE;
		ThrLow = Max(ThrLow, THROTTLE_MIN_ALT_HOLD);
		ThrHigh = ThrNeutral + THROTTLE_MIDDLE;
		if ( ( DesiredThrottle <= ThrLow ) || ( DesiredThrottle >= ThrHigh ) ) {
			mSTimer(ThrottleUpdate, THROTTLE_UPDATE_MS);
			F.ThrottleMoving = true;
		} else
			F.ThrottleMoving = false;
	}
} // CheckThrottleMoved




