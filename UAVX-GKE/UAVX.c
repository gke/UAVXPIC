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

#pragma	config OSC=HSPLL, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC, XINST = OFF

#include "uavx.h"

Flags 	F;
uint8 p;
uint32 NowmS, NowuS, CyclemS;

#pragma udata access nearvars
int8 near State, NavState, FailState;
boolean near SpareSlotTime;

#pragma udata

void main(void) {

	DisableInterrupts;

	InitPortsAndUSART();
	InitTimersAndInterrupts();

	InitMisc();

	InitADC();
	InitI2C(); // selects 400KHz
	ReadStatsEE();	
	InitRC();
	InitMotors();

    EnableInterrupts;

	LEDYellow_ON;
	Delay1mS(500);

	InitParameters(); // inits Acc/Gyros

	InitHeading();
	InitRangefinder();
	InitGPS();
	InitNavigation();
	InitBarometer();
#ifndef USE_PACKET_COMMS
	ShowSetup();
#endif
	FirstPass = true;
	
	while( true ) {

		StopMotors();

		ReceivingGPSOnly(false);
		EnableInterrupts;

		LightsAndSirens();	// Check for Rx signal, disarmed on power up, throttle closed, gyros ONLINE

		State = Starting;
		mSTimer(RxFailsafeTimeout, FAILSAFE_TIMEOUT_MS);
		FailState = Monitoring;
		F.FirstArmed = false;

		while ( Armed() ) { // no command processing while the Quadrocopter is armed

			ReceivingGPSOnly(true); 

			do {
				NowuS = uSClock();
			} while (NowuS < CycleUpdateuS);
			NowmS = NowuS/1000;		
			CycleUpdateuS = NowuS + PID_CYCLE_US;

			SpareSlotTime = true; // token used by nav, compass, baro and telemetry

			DoControl();

			GetBaroAltitude();
			UpdateGPS();

				if (F.RCNewValues)
					UpdateControls();

				switch ( State  ) {
				case Starting:	// this state executed once only after arming

					LEDYellow_OFF;

					if ( !F.FirstArmed ) {
						mS[StartTime] = 0;
						F.FirstArmed = true;
					}

					InitControl();
					InitGPS();
					InitNavigation();

					DesiredThrottle = 0;
					ErectGyros(256);				// DO NOT MOVE AIRCRAFT!
					ZeroStats();
					WriteMagCalEE();

					DoStartingBeepsWithOutput(3);

					mSTimer(ArmedTimeout, ARMED_TIMEOUT_MS);
					mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
					F.LostModel = false;

					State = Landed;
					break;
				case Landed:
					DesiredThrottle = 0;
					ZeroPIDIntegrals();
					PrimeROC = true;
					DesiredHeading = Heading;
					AltMinThrCompStick = -ALT_MAX_THR_COMP;
					F.HoldingAlt = false;
					DoStickProgramming();
					InitHeading();
					if ( NowmS > mS[ArmedTimeout] )
						DoShutdown();
					else	
						if ( StickThrottle < IdleThrottle ) {
							SetGPSOrigin();
							DecayNavCorr();
	    					if ( F.NewCommands )
								F.LostModel = F.SticksUnchangedFailsafe;
						} else {						
							LEDPattern = 0;
							mSTimer(NavActiveTime, NAV_ACTIVE_DELAY_MS);
							Stats[RCGlitchesS] = RCGlitches; // start of flight
							SaveLEDs();

							mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
							F.LostModel = false;
							if ( ParameterSanityCheck() )
								State = InFlight;
							else
								ALL_LEDS_ON;	
						}						
					break;
				case Landing:
					if ( StickThrottle > IdleThrottle ) {
						DesiredThrottle = 0;
						State = InFlight;
					} else
						if ( NowmS < mS[ThrottleIdleTimeout] )
							DesiredThrottle = IdleThrottle;
						else {
							DecayNavCorr();
							DesiredThrottle = AltComp = HRAltComp = 0; // to catch cycles between Rx updates
							AltMinThrCompStick = -ALT_MAX_THR_COMP;
							F.DrivesArmed = false;
							Stats[RCGlitchesS] = RCGlitches - Stats[RCGlitchesS];	
							WriteStatsEE();
							WriteMagCalEE();
							mSTimer(ArmedTimeout, ARMED_TIMEOUT_MS);
							State = Landed;
						}
					break;
				case Shutdown:
					LEDChaser();
					if ((StickThrottle < IdleThrottle) && !(F.ReturnHome)) {
						mSTimer(ArmedTimeout, ARMED_TIMEOUT_MS);
						mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
						F.LostModel = false;
						DoStartingBeepsWithOutput(3);
						State = Landed;
					}
					break;
				case InFlight:
					F.DrivesArmed = true;		
					LEDChaser();
					DesiredThrottle = SlewLimit(DesiredThrottle, StickThrottle, 1);
					DoNavigation();	
					if (State != Shutdown) {
						if (StickThrottle < IdleThrottle) {
							AltComp = 0;
							mSTimer(ThrottleIdleTimeout, THROTTLE_LOW_DELAY_MS);
							RestoreLEDs();
							State = Landing;
						} else
							DoAltitudeHold();
					}
					break;
				} // Switch State

			CheckTelemetry();

			CheckBatteries();
			CheckAlarms();

		} // flight while armed
	
		Delay1mS(1); // zzz to prevent a fast loop around through startup
	}

} // main

