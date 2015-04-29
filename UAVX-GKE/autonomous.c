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

// Autonomous flight routines

#include "uavx.h"

void DoShutdown(void);
void DecayNavCorr(void);
void DoCompScaling(void);
void Navigate(int24 DesiredNorth, int24 DesiredEast);
void AcquireHoldPosition(void);
void NavGainSchedule(int16);
void DoNavigation(void);
void DoFailsafe(void);
void UAVXNavCommand(void);
void GetWayPointEE(int8);
void InitNavigation(void);

#pragma udata nav_vars
NavStruct Nav;
#pragma udata

uint8 LandingState = CommenceDescent;
boolean StartingNav = true;
int16 DescentComp;
int16 RollPitchMixFrac;
int16 RelHeading;
int16 EffNavSensitivity;

#pragma idata nav_sw_state
uint8 NavSwState = NavSwLow;
uint8 NavSwStateP = NavSwUnknown;
#pragma idata
		
#pragma udata navvars
int24	WPAltitude;
int24 	WPDistance;
int16	OriginalWPHeading, WPHeading;
uint24 	NavRTHTimeoutmS;
#pragma udata


boolean NotDescending(void) {

	return (Abs(ROC) < ALT_MIN_DESCENT_DMPS);

} // NotDescending

boolean DoLanding(void) {
	boolean HasLanded;

	static uint32 LastUpdatemS = 0;
	static int16 bucketmS;
	uint32 NowmS;
	int16 dTmS;

	HasLanded = false;

	NowmS = mSClock();
	dTmS = LastUpdatemS - NowmS;
	LastUpdatemS = NowmS;


#if defined(MULTICOPTER)
	DesiredThrottle = CruiseThrottle;
#else
	DesiredThrottle = 0;
#endif
	SetDesiredAltitude(-1000); // all the way

	switch (LandingState) {
	case CommenceDescent:
		if (Abs(Altitude) < LAND_DM) {
			bucketmS = LAND_TIMEOUT_MS;
			LandingState = Descent;
		}
		break;
	case Descent:
		if (NotDescending()) {
			bucketmS = Max(0, bucketmS - dTmS);
			if (bucketmS <= 0)
				LandingState = DescentStopped;
		} else
			bucketmS = Min(bucketmS + dTmS * 2, LAND_TIMEOUT_MS);

		mSTimer(NavStateTimeout, bucketmS);
		break;
	case DescentStopped:
		LandingState = CommenceDescent; // reset
		HasLanded = true;
		break;
	} // switch

	return (HasLanded);
} // DoLanding

void DoShutdown(void) {
	StopMotors();
	ALL_LEDS_OFF;
	NavState = HoldingStation;
	State = Shutdown;
} // DoShutdown

void DecayNavCorr(void) {
	A[Roll].NavCorr = DecayX(A[Roll].NavCorr, 5);
	A[Pitch].NavCorr = DecayX(A[Pitch].NavCorr, 5);
	A[Yaw].NavCorr = 0;
} // DecayNavCorr

void ResetNavHold(void) {
	uint8 a;

//zzz

/* zzz
	A[Pitch].NavCorr = A[Roll].NavCorr = Nav.Distance = Nav.Bearing
				= 0.0f;
	F.OrbitingWP = false;

	for (a = NorthC; a <= YawC; a++)
		Nav.C[a].CorrP = Nav.C[a].VelEP = Nav.C[a].VelIntE = 0.0f;
	Nav.CorrP = Nav.I.IntE = 0.0f;
*/

	Nav.MaxVelocitydMpS = NAV_MIN_VEL_DMPS;

	F.NewNavUpdate = F.WayPointAchieved = F.WayPointCentred = false;

} // ResetNavHold

#ifndef TESTING

void AcquireHoldPosition(void) {

	Nav.C[NorthC].Hold = Nav.C[NorthC].Pos;
	Nav.C[EastC].Hold = Nav.C[EastC].Pos;

	F.CrossTrackActive = F.WayPointAchieved = F.WayPointCentred = F.AcquireNewPosition = false;

	NavState = HoldingStation;

} // AcquireHoldPosition

#endif // !TESTING

int16 CalculateTurnCorr(void) {
	static int16 A;

	A = MakePi(RelHeading);
	A = Limit1(A, DegreesToRadians(30));
	A = SRS32((int32)A * Nav.YawKp * EffNavSensitivity, 9);

	return(A);
} // CalculateTurnCorr

void Navigate(int24 DesiredNorth, int24 DesiredEast) {
	// F.GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	#ifndef TESTING // not used for testing - make space!

	static int16 AltE;
	static int16 YawCorr;
	static int24 AbsNorthE, AbsEastE;
	static int32 PosE;
	static int32 PosEP = 0;
	static int16 VelP = 0; // zzz
	static int16 VelEP = 0;
	static int16 DesiredVel, Vel, VelE;
	static int32 P, I;
	static int16 NavCorr;
	static int16 Temp;
	static uint8 a;

	EffNavSensitivity = Nav.Sensitivity - FromPercent(NAV_SENS_THRESHOLD, RC_MAXIMUM); 
	EffNavSensitivity = Limit(EffNavSensitivity, 0, 256);

	Nav.C[NorthC].Desired = DesiredNorth; // for telemetry tracking
	Nav.C[EastC].Desired = DesiredEast;

	Nav.C[NorthC].PosE = Nav.C[NorthC].Desired - Nav.C[NorthC].Pos;
	Nav.C[EastC].PosE = Nav.C[EastC].Desired - Nav.C[EastC].Pos;

	PosE = int32sqrt(Sqr(Nav.C[EastC].PosE) + Sqr(Nav.C[NorthC].PosE));

	AbsNorthE = Abs(Nav.C[NorthC].PosE);
	AbsEastE = Abs(Nav.C[EastC].PosE);
	WPDistance = Max(AbsNorthE, AbsEastE); // avoiding sqrt
	if (WPDistance < 2896L) // dM
		WPDistance = int32sqrt(Sqr(AbsEastE) + Sqr(AbsNorthE));
	WPHeading = int32atan2(Nav.C[EastC].PosE, Nav.C[NorthC].PosE);

	PosE = WPDistance;
	Vel = SRS32(PosEP - PosE, 2); 
	PosEP = PosE;

	Vel = SRS16(Vel + VelP, 1);
	if (Abs(Vel) < 50) 
		Vel = 0;
	else {
		VelP = Vel;
		PosE -= SRS16(Vel, 2);
	}

	F.WayPointCentred = Nav.Distance < (GPS.HDOP * 2);

	F.WayPointAchieved = F.WayPointCentred && (Abs(DesiredAltitude - Altitude)
			< Nav.ProximityAltitude);

	if (EffNavSensitivity > 0) {
		RelHeading = MakePi(WPHeading - Heading); // make +/- Pi

		Temp = Limit(WPDistance, 0, 16000);
		DesiredVel = SRS32(Temp * Nav.PosKp * EffNavSensitivity, 16);
		DesiredVel = Limit(DesiredVel, 0, Nav.MaxVelocitydMpS);

		VelE = DesiredVel - Vel;

		VelE = SlewLimit(VelEP, VelE, 1); // zzz check equiv to 0.5MPSPS
		VelEP = VelE;

		P = VelE * Nav.VelKp;

		Nav.VelIntE += SRS16(VelE, 2); // * NavdT assumes 0.25Sec
		Nav.VelIntE = Limit1(Nav.VelIntE, Nav.VelIntLimit);
		I = 0; // zzz Nav.VelIntE * Nav.VelKi;

		NavCorr = SRS16(P + I, 8);
		NavCorr = Limit1(NavCorr, Nav.MaxAngle);

		A[Pitch].NavCorr = -SRS32(NavCorr * int16cos(RelHeading), 8);
		A[Roll].NavCorr = SRS32(NavCorr * int16sin(RelHeading), 8);

		A[Yaw].NavCorr = 0.0f;
		if (F.AllowTurnToWP && (PosE > Nav.ProximityRadius))
			Nav.DesiredHeading = MakePi(RelHeading);
		else
			Nav.DesiredHeading = DesiredHeading;

	} else
	#endif // !TESTING
		DecayNavCorr();

} // Navigate

boolean FailsafeLanding(void) {
	boolean v;
	//NoFailsafes, Monitoring, BatteryLow, LostSignal

	if (F.FailsafesEnabled) {
		if (F.SticksUnchangedFailsafe)
			FailState = LostSignal;
		else
			FailState = Monitoring;
		v = F.SticksUnchangedFailsafe;
	} else {
		FailState = NoFailsafes;
		v = false;
	}

	return (v);

} // FailsafeLanding


void UpdateRTHSwState(void) { // called in rc.c on every rx packet
	static uint24 NextNavSwUpdatemS = 0;
	uint32 NowmS;

	NowmS = mSClock();
	if (NowmS > NextNavSwUpdatemS) {
		if (NavSwState != NavSwStateP) {
			NextNavSwUpdatemS = NowmS + 1500;

			Nav.MaxVelocitydMpS = NAV_MIN_VEL_DMPS;

			switch (NavSwState) {
			case NavSwLow:
				F.NavigationEnabled = F.Navigate = F.ReturnHome = false;
				if (State == InFlight)
					DesiredAltitude = Altitude;
				NavState = PIC;
				break;
			case NavSwMiddle:
				F.NavigationEnabled = true;
				F.Navigate = F.ReturnHome = false;
				break;
			case NavSwHigh:
				F.NavigationEnabled = F.ReturnHome = true;
				F.Navigate = false;
				break;
			} // switch
			NavSwStateP = NavSwState;
		}
	}
} // UpdateRTHSwState


void DoNavigation(void) {
	#ifndef TESTING // not used for testing - make space!

	if (F.NavigationActive) {
		F.LostModel = false;
		if (F.NewNavUpdate && SpareSlotTime) {
			F.NewNavUpdate = false;

			switch ( NavState ) { // most frequent case last - switches in C18 are IF chains not branch tables!
			case Touchdown:
				Navigate(0, 0);
				DoLanding();
				break;
			case Descending:
				Navigate( 0, 0 );
				if ( F.ReturnHome ) {
					if ( Altitude < LAND_CM )
						NavState = Touchdown;
					DoLanding();
				} else
					AcquireHoldPosition();
				break;
			case AtHome:
				Navigate(0, 0);

				if ( F.ReturnHome )
					if ( F.WayPointAchieved ) { // check still @ Home
						if ( F.UsingRTHAutoDescend && ( mSClock() > mS[NavStateTimeout] )) {
							mSTimer(NavStateTimeout, NAV_RTH_LAND_TIMEOUT_MS);
							NavState = Descending;
						}
					} else {
						F.CrossTrackActive = false;
						NavState = ReturningHome;
					} else
					AcquireHoldPosition();
				break;
			case ReturningHome:		
				Navigate(0, 0);				
				if ( F.ReturnHome ) {
					if ( F.WayPointAchieved ) {
						mSTimer(NavStateTimeout, NavRTHTimeoutmS);
						SetDesiredAltitude(RTHAltitude);			
						NavState = AtHome;
					}	
				} else
					AcquireHoldPosition();					
				break;
			case PIC:
			case HoldingStation:	
					if ( F.ReturnHome ) {
						SetDesiredAltitude(RTHAltitude);
						F.CrossTrackActive;
						NavState = ReturningHome;
					} else {
						if ( !F.AttitudeHold )		
							AcquireHoldPosition();
						Navigate(Nav.C[NorthC].Hold, Nav.C[EastC].Hold);
					}									
				break;
			} // switch NavState
		}
	} else {

		ResetNavHold();

		switch (NavState) {
		case Touchdown:
			//
			break;
		case Descending:
			if (F.ReturnHome || FailsafeLanding()) {
				if (F.AltControlEnabled)
					DoAutoLanding();
			} else
				NavState = PIC;
			break;
		case AcquiringAltitude:
		case HoldingStation:
		case PIC:
		default:
			NavState = PIC;
			if (F.ReturnHome || FailsafeLanding()) //&& F.NewCommands  && ( StickThrottle >= IdleThrottle ) )
			{
				F.LostModel = FailsafeLanding();
#if defined(MULTICOPTER)
				// just land

#else
				// landing config for control surfaces zzz
#endif
				LandingState = CommenceDescent;
				NavState = Descending;
			}
			break;
		} // switch NavState
	}

	F.NewCommands = false;	// Navigate modifies Desired Roll, Pitch and Yaw values.

	#endif // !TESTING
} // DoNavigation


void InitNavigation(void) {

	WPHeading = 0;	
	A[Roll].NavCorr = A[Pitch].NavCorr = A[Yaw].NavCorr = 0;
	Nav.C[NorthC].PosE = Nav.C[EastC].PosE = 0;
	Nav.CrossTrackE = 0;

	SetDesiredAltitude(0);
	NavState = HoldingStation;
	AttitudeHoldResetCount = 0;
	CurrMaxRollPitch = 0;
	F.WayPointAchieved = F.WayPointCentred = false;
	F.NewNavUpdate = false;
	Nav.ProximityRadius = NAV_PROXIMITY_RADIUS * 10L; 
	Nav.ProximityAltitude = NAV_PROXIMITY_ALTITUDE * 10L; // Decimetres
	
	F.CrossTrackActive = F.WayPointCentred =  F.WayPointAchieved = false;

} // InitNavigation

