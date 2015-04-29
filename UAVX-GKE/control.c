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

void DoAltitudeHold(void);
void AltitudeHold(void);
void DoControl(void);
void CheckThrottleMoved(void);
void InitControl(void);

#pragma udata axisvars			
AxisStruct A[3];
#pragma udata

#pragma udata hist
uint32 CycleHist[16];
#pragma udata
		
int16 CameraAngle[3];				

int16 HeadingE;

int16 CurrMaxRollPitch;

int16 AttitudeHoldResetCount;
int24 DesiredAltitude, Altitude; 
int16 AltFiltComp, AltComp, HRAltComp, ROC, MinROCCmpS;
int16 AltMinThrCompStick;
int24 RTHAltitude;

#if defined(GKE_TUNE)
	int16 TuneTrim;
#endif

void AcquireAltitude(void) { // Syncronised to baro intervals independant of active altitude source	
	const int16 AltFC = 883; // 3.45;
	const int16 AltKd = 374; // 1.46;
	const int16 AltKop = 256;
	static int24 p, d, pd, AltE;
	static int16 AltKdF = 0;
	static int16 ROCE, DesiredROC;
	static int16 NewComp;

	AltE = DesiredAltitude - Altitude;

	p = AltE * AltKop;
	d = 0;//(AltE * AltKd - AltKdF) * AltFC;
	//AltKdF += AltKdF * AltFC; // need limiting?

	pd = p + d;
	pd = SRS16(pd, 8);

	DesiredROC = Limit(pd, MinROCCmpS, ALT_MAX_ROC_CMPS);

	ROCE = DesiredROC - ROC;

	NewComp = ROCE * P[ROCKpRate];

	NewComp = SRS16(NewComp, 9);
	AltComp = Limit(NewComp, AltMinThrCompStick, ALT_MAX_THR_COMP);
	
} // AcquireAltitude	

void DoAltitudeHold() { 
	static int16 ActualThrottle;
	static int16 ROCF = 0;
	static int16 TransitionAlt;
	static int16 AltOffset = 0;

	if (F.NewBaroValue) {
		F.NewBaroValue = false;
		GetRangefinderAltitude();
		CheckThrottleMoved();

#ifdef USE_BARO_RF_CALIB

		// From MultiWii
		if (F.UsingRangefinderAlt && (RangefinderAltitude < 200)) {
			AltOffset = BaroAltitude - RangefinderAltitude;
			Altitude = RangefinderAltitude;
			ROC = RangefinderROC;
		} else {
			BaroAltitude -= AltOffset;
			if (F.UsingRangefinderAlt) {
				TransitionAlt = 300 - RangefinderAltitude;
				Altitude = RangefinderAltitude * TransitionAlt + BaroAltitude
				* (100 - TransitionAlt);
				ROC = RangefinderROC;
			} else {
				Altitude = BaroAltitude;
				ROC = BaroROC;
			}
		}

#else

		if (F.UsingRangefinderAlt) { 
			Altitude = RangefinderAltitude;
			ROC = RangefinderROC;
		} else {
			Altitude = BaroAltitude - OriginAltitude;
			ROC = BaroROC;
		}

#endif

		ROCF = HardFilter(ROCF, ROC);

		if (F.AltControlEnabled) {				
			if (F.SticksUnchangedFailsafe || ((NavState
					!= HoldingStation) && (NavState != PIC))) { 
				F.HoldingAlt = true;
				AcquireAltitude();
			} else if (F.ThrottleMoving || 
				((Abs(ROCF) > (ALT_MAX_ROC_CMPS >> 3)) && 
				!F.HoldingAlt)) {
				F.HoldingAlt = false;
				SetDesiredAltitude(Altitude);
				AltComp = Decay1(AltComp);
			} else {
				F.HoldingAlt = true;
				ROCF = 0;
				AcquireAltitude(); // not using cruise throttle
				#ifndef SIMULATE
					ActualThrottle = DesiredThrottle + AltComp;
					if ((Abs(ROC) < ALT_HOLD_MAX_ROC_CMPS) && (ActualThrottle
							> THROTTLE_MIN_CRUISE)) {
						NewCruiseThrottle
								= HardFilter(NewCruiseThrottle, ActualThrottle);
						NewCruiseThrottle
								= Limit(NewCruiseThrottle, THROTTLE_MIN_CRUISE, THROTTLE_MAX_CRUISE );
						CruiseThrottle = NewCruiseThrottle;
					}
				#endif // SIMULATE
			}
		} else {
			Altitude = BaroAltitude;
			//zzz ROC = 0;
			AltComp = Decay1(AltComp);
			F.HoldingAlt = false;
		}
	}
} // DoAltitudeHold

void ComputeRateDerivative(AxisStruct *C, int16 Rate) {
	static int32 r;

	r = SoftFilter32(C->Ratep, Rate);
	C->RateD =  r - C->Ratep;
	C->Ratep = r;

	C->RateD = SoftFilter32(C->RateDp, C->RateD);
	C->RateDp = C->RateD;

} // ComputeRateDerivative


void DoWolfControl(void) { // Origin Dr. Wolfgang Mahringer UAVP
	static uint8 a;
	static AxisStruct *C;
	static int32 PD, I;

	for ( a = Roll; a<=(uint8)Pitch; a++) {
		C = &A[a];

		ComputeRateDerivative(C, C->Rate);
		PD =  SRS32((int32)C->Rate * C->RateKp + C->RateD * C->RateKd, 9);
	
	   	I = SRS32((int32)C->Angle * C->RateKi , 6);
		I = Limit1(I, C->IntLimit);
	
		C->Out = (PD + I) - C->Control;  
	}
	
} // DoWolfControl


void DoAngleControl(void) { // PI_PD with Dr. Ming Liu
	static uint8 a;
	static AxisStruct *C;
	static int32 Pa, Ia, PDr, DesRate, AngleE, RateE;

	for ( a = Roll; a<=(uint8)Pitch; a++) {
		C = &A[a];

		AngleE = C->Control * RC_STICK_ANGLE_SCALE - C->Angle;
	
		Pa = AngleE * C->AngleKp;
	
		C->AngleEInt += AngleE * C->AngleKi;
		C->AngleEInt = Limit1(C->AngleEInt, C->IntLimit * 8);
		Ia = 0;//C->AngleEInt;
	
		DesRate = SRS32(Pa + Ia, 3);
		RateE = DesRate - C->Rate; 	
	
		ComputeRateDerivative(C, RateE); // ignore set point change RateE
		PDr = -SRS32(RateE * C->RateKp + C->RateD * C->RateKd, 9);
	
		C->Out = PDr;
	}

} // DoAngleControl

void YawControl(void) {
	static int32 Pa, Pr, Ir, RateE, PIr;
	static boolean YawStickActive;
	static int16 Temp;

	YawStickActive = Abs(A[Yaw].Control) > 20;

	F.UpdateHeading = YawStickActive || ((NavState != HoldingStation) && (NavState != PIC));

	if (F.UpdateHeading)
		DesiredHeading = (NavState == HoldingStation)||(NavState == PIC)  ? Heading
					: Nav.DesiredHeading;

	HeadingE = MinimumTurn(DesiredHeading - Heading);
	HeadingE = Limit1(HeadingE, DegreesToRadians(30));

	RateE = SRS32((int32)HeadingE * A[Yaw].AngleKp, 6) - A[Yaw].Rate;

	Pr  = SRS32((int32)RateE * A[Yaw].RateKp, 10); 

	if (F.UpdateHeading)
		A[Yaw].RateEInt = Ir = 0;
	else {
		A[Yaw].RateEInt += RateE * A[Yaw].RateKi;
		A[Yaw].RateEInt = Limit1(A[Yaw].RateEInt, A[Yaw].IntLimit * 1024);
		Ir = SRS32(A[Yaw].RateEInt, 10);
	}

	A[Yaw].Out = Pr + Ir;

	if (YawStickActive)
		A[Yaw].Out -= A[Yaw].Control;

	Temp = A[Yaw].Control + 100;
	A[Yaw].Out = Limit1(A[Yaw].Out, A[Yaw].RateIntLimit);

} // YawControl


void ZeroPIDIntegrals(void) {
	static uint8 a;
	static AxisStruct *C;

	for ( a = Roll; a<=(uint8)Yaw; a++) {
		C = &A[a];
		C->AngleEInt = C->RateEInt = 0;
	}
} // ZeroPIDIntegrals

void DetermineControl(void) {

	A[Pitch].Control = A[Pitch].Desired;
	A[Roll].Control = A[Roll].Desired;

	if (F.NavigationActive) {
		A[Pitch].Control += A[Pitch].NavCorr;
		A[Roll].Control += A[Roll].NavCorr;
	}
	A[Yaw].Control = A[Yaw].Desired;

} // DetermineControl

void DoControl(void){

	#if defined(SIMULATE)
		DoEmulation();
	#else
		GetAttitude();
		GetBaroAltitude();
		GetHeading();
		DetermineControl();
	
		if ( F.UsingRateControl )
			DoWolfControl();
		else	
			DoAngleControl();
								
		YawControl();
			
		Rl = A[Roll].Out; Pl = A[Pitch].Out; Yl = A[Yaw].Out;

	#endif // SIMULATE

	F.NearLevel = Max(Abs(A[Roll].Angle), Abs(A[Pitch].Angle)) < NAV_RTH_LOCKOUT;

	OutSignals(); 

} // DoControl

void InitControl(void) {
	static uint8 a;
	static AxisStruct *C;

	AltComp = HRAltComp = 0;
	AltMinThrCompStick = -ALT_MAX_THR_COMP;

	for ( a = Roll; a<=(uint8)Yaw; a++) {
		C = &A[a];
		C->Outp = C->Ratep = C->RateDp = C->RateD = C->Angle = 0;
	}	
	ZeroPIDIntegrals();

} // InitControl



