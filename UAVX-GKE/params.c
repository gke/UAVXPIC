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

void ReadParametersEE(void);
void WriteParametersEE(uint8);
void UseDefaultParameters(void);
void UpdateWhichParamSet(void);
boolean ParameterSanityCheck(void);
void InitParameters(void);

const rom uint8 ESCLimits [] = { OUT_MAXIMUM, OUT_HOLGER_MAXIMUM, OUT_X3D_MAXIMUM, OUT_YGEI2C_MAXIMUM };

#include "uavx_default_parameters.h"

volatile boolean StickArmed = false;
uint8 ParamSet;
uint8 PrevSensorHint;
boolean ParametersChanged;
boolean UsingMPU6050;
int16 OrientationMRad;
uint8 UAVXAirframe;

#pragma udata params
int8 P[MAX_PARAMETERS];
#pragma udata

void MakeSensorsUnknown(void) {

	AccType = AccUnknown;
	GyroType = GyroUnknown;
	CompassType = CompassUnknown;
	PrevSensorHint = GyroUnknown;

	ParametersChanged = true;
	ParamSet = 1;

} // MakeSensorsUnknown

void CheckConfig(void) {

	F.ConfigError = (P[TxMode] == ModeUnknown)
		|| (P[RCType] == RCUnknown) || (P[SensorHint] == GyroUnknown);
} // CheckConfig

void InitAttitudeSensors(void) {

	UsingMPU6050 = (P[SensorHint] == FreeIMU) || (P[SensorHint] == UAVXArm32IMU);

	AccType = AccUnknown;
	GyroType = P[SensorHint];

	if (P[SensorHint] != GyroUnknown) {	
		if (UsingMPU6050) {
			GyroType = P[SensorHint];
			F.UsingAnalogGyros = false;
			F.IMUActive = MPU6050Active(); 
			if (F.IMUActive)
				InitMPU6050();
			AccType = MPU6050Acc;
		}
		else {
			InitAnalogGyros();
			F.IMUActive = LISLAccActive();
			if (F.IMUActive)
				InitLISLAcc();
			AccType = LISLAcc;
		}
		InitCompass();
		PrevSensorHint = P[SensorHint];
	}

} // InitAttitudeSensors

void ReadParametersEE(void)
{
	static uint8 i, b;
	static uint16 a;

	if ( ParametersChanged ) { // overkill if only a single parameter has changed but is not in flight loop

		a = (ParamSet - 1)* MAX_PARAMETERS;	
		for ( i = 0; i < (uint8)MAX_PARAMETERS; i++)
			P[i] = ReadEE(a + i);

		CheckConfig();

		if (P[SensorHint] != PrevSensorHint)
			InitAttitudeSensors();

		A[Roll].AngleKp = P[RollAngleKp];
		A[Roll].AngleKi = P[RollAngleKi];

		A[Roll].RateKp = P[RollRateKp];
		A[Roll].RateKi = P[RollRateKi];
		A[Roll].RateKd = P[RollRateKd];

		A[Roll].IntLimit = P[RollIntLimit];
		
		A[Pitch].AngleKp = P[PitchAngleKp];
		A[Pitch].AngleKi = P[PitchAngleKi];

		A[Pitch].RateKp = P[PitchRateKp];
		A[Pitch].RateKi = P[PitchRateKi];
		A[Pitch].RateKd = P[PitchRateKd];

		A[Pitch].IntLimit = P[PitchIntLimit];
		
		A[Yaw].AngleKp = P[YawAngleKp];
		A[Yaw].RateKp = P[YawRateKp];
		A[Yaw].RateIntLimit = P[YawRateIntLimit];

		F.UsingMWStickProg = (P[ConfigBits] & UseMWStickProgMask) != 0;

		// Navigation
		#if defined(TRICOPTER) | defined(Y6COPTER) | defined(HELICOPTER) | defined(VTCOPTER)
			F.AllowTurnToWP = true;	
		#endif

		Nav.MaxVelocitydMpS = P[NavMaxVelMPS]*10;
		Nav.PosKp = 1; // zzz
		Nav.VelKp = 5;
		Nav.VelKi = P[NavWindKi];
		Nav.VelIntLimit = 150 / Nav.VelKi; // cm/S
		Nav.YawKp = 2;
		Nav.CrossTrackKp = P[NavCrossTrackKp];

		Nav.MaxAngle = DegreesToRadians((int16)P[NavMaxAngle]);
		
		Nav.MaxVelocitydMpS = P[NavMaxVelMPS] * 10; 

		ESCMax = ESCLimits[P[ESCType]];
		if ( P[ESCType] == ESCPPM )
			TRISB = 0b00000000; // make outputs
		else
			for ( i = 0; i < (uint8)NO_OF_I2C_ESCS; i++ )
				ESCI2CFail[i] = 0;

		KpAcc = (real32)P[MadgwickKpAcc] * 0.001f;

		InitAccelerations();
		ErectGyros(16);

		b = P[ServoSense];
		for ( i = 0; i < (uint8)6; i++ ) {
			PWSense[i]  = ((b & 1) ? -1 : 1);
			b >>=1;
		}

		RollPitchMixFrac = FromPercent(P[RollPitchMix], 256);

		#if defined(SIMULATE)
			CruiseThrottle = NewCruiseThrottle = FromPercent(45, RC_MAXIMUM);
		#else
			CruiseThrottle = NewCruiseThrottle = FromPercent((int16)P[PercentCruiseThr], RC_MAXIMUM);
		#endif

		#if defined(MULTICOPTER)
			IdleThrottle = Limit((int16)P[PercentIdleThr], 10, 20);
		#else
			IdleThrottle = Limit((int16)P[PercentIdleThr],2, 5);
		#endif // MULTICOPTER

		IdleThrottle = FromPercent(IdleThrottle, RC_MAXIMUM);

		MinROCCmpS = -(int16)P[MaxDescentRateDmpS] * 10;

		RTHAltitude = (int24)P[NavRTHAlt]*100;

		CompassOffset = - (int16)P[NavMagVar]; // changed sign of MagVar AGAIN!
		if ( CompassType == HMC6352Compass )
			CompassOffset += (int16)COMPASS_OFFSET_QTR * 90L; 
		CompassOffset = ((int32)CompassOffset *MILLIPI)/180;

		OrientationMRad = ((int16)AFOrientation[AF_TYPE] * 1309) / 10; // 7.5deg steps
		OSin = int16sin(OrientationMRad);
		OCos = int16cos(OrientationMRad);

		NoOfRCChannels = Limit(P[RCChannels], 4, CONTROLS);
		if ( (( NoOfRCChannels&1 ) != (uint8)1 ) && !F.UsingCompoundPPM )
			NoOfRCChannels--;

		InitRC();

		F.FailsafesEnabled = ((P[ConfigBits] & UseFailsafeMask) == 0);

		F.UsingRTHAutoDescend = (P[ConfigBits] & UseRTHDescendMask) != 0;
		NavRTHTimeoutmS = (uint24)P[DescentDelayS]*1000L;

		BatteryVoltsLimitADC = BatteryVoltsADC = ((int24)P[LowVoltThres] * 1024 + 70L) / 139L; // UAVPSet 0.2V units
		BatteryCurrentADC = 0;
		
		F.ParametersValid = ParameterSanityCheck();

		ParametersChanged = false;
	}
	
} // ReadParametersEE

void WriteParametersEE(uint8 s)
{
	static uint8 p;
	static uint16 addr;
	
	addr = (s - 1)* MAX_PARAMETERS;

	for ( p = 0; p < (uint8)MAX_PARAMETERS; p++)
		WriteEE( addr + p,  P[p]);
} // WriteParametersEE

void UseDefaultParameters(void)
{ // loads a representative set of initial parameters as a base for tuning
	static uint16 p;

//	for ( p = 0; p < (uint16)MAX_EEPROM; p++ )
//		WriteEE( p,  0xff);

	for ( p = 0; p < (uint16)MAX_PARAMETERS; p++ )
		P[p] = DefaultParams[p][0];

	WriteParametersEE(1);
	WriteParametersEE(2);

	MakeSensorsUnknown();
	ParamSet = 1;
	ParametersChanged = true;

	WriteEE(NAV_NO_WP, 0); // set NoOfWaypoints to zero

	TxString("\r\nUse READ CONFIG to refresh UAVPSet display\r\n");	
} // UseDefaultParameters

#define THR_LO  (1<<(2*ThrottleRC))
#define THR_CE  (3<<(2*ThrottleRC))
#define THR_HI  (2<<(2*ThrottleRC))
#define ROL_LO  (1<<(2*RollRC))
#define ROL_CE  (3<<(2*RollRC))
#define ROL_HI  (2<<(2*RollRC))
#define PIT_LO  (1<<(2*PitchRC))
#define PIT_CE  (3<<(2*PitchRC))
#define PIT_HI  (2<<(2*PitchRC))
#define YAW_LO  (1<<(2*YawRC))
#define YAW_CE  (3<<(2*YawRC))
#define YAW_HI  (2<<(2*YawRC))

enum StickStates {
	MonitorSticks, SticksChanging, SticksChanged
};

#pragma idata stick
static uint8 StickHoldState = MonitorSticks;
static uint8 SticksState = 0;
static uint32 StickTimermS = 0;
#pragma idata

void UpdateSticksState(void) {
	// pattern scheme from MultiWii
	uint8 pattern = 0;
	uint8 i;

	for (i = 0; i < 4; i++) {
		pattern >>= 2;
		if (RC[i] > FromPercent(30, RC_MAXIMUM))
			pattern |= 0x80; // check for MIN
		if (RC[i] < FromPercent(70, RC_MAXIMUM))
			pattern |= 0x40; // check for MAX
	}

	switch (StickHoldState) {
	case MonitorSticks:
		if (SticksState != pattern) {
			SticksState = pattern;
			StickTimermS = mSClock() + 2000;
			StickHoldState = SticksChanging;
		}
		break;
	case SticksChanging:
		if (SticksState == pattern) {
			if (mSClock() > StickTimermS)
				StickHoldState = SticksChanged;
		} else
			StickHoldState = MonitorSticks;
		break;
	default:
		break;
	}// switch

} // UpdateSticksState

void DoStickProgramming(void) {
	uint8 NewParamSet, i;
	boolean EEChanged, NewAllowNavAltControl, NewAllowTurnToWP;

	if ( !Armed() ) {

		UpdateSticksState();

		if (StickHoldState == SticksChanged) {
			if (F.UsingMWStickProg && (SticksState == (THR_LO | YAW_CE
					| PIT_CE | ROL_HI)) && ArmingSwitch && !StickArmed) {
				LEDBlue_ON;
				DoBeep100mSWithOutput(3, 0);
				StickArmed = true;
				SticksState = MonitorSticks;
				LEDBlue_OFF;
			} else {

				NewParamSet = ParamSet;
				NewAllowTurnToWP = F.AllowTurnToWP;
	
				switch (SticksState) {
				case THR_LO + YAW_CE + PIT_CE + ROL_HI:
					// BR
					NewAllowTurnToWP = true;
					break;
				case THR_LO + YAW_CE + PIT_CE + ROL_LO:
					// BL
					NewAllowTurnToWP = false;
					break;
	
				case THR_LO + YAW_HI + PIT_LO + ROL_CE:
					// TR
					NewParamSet = 1;
					NewAllowNavAltControl = true;
					break;
				case THR_LO + YAW_LO + PIT_LO + ROL_CE:
					// TL
					NewParamSet = 2;
					NewAllowNavAltControl = true;
					break;
				default:
					break;
				} // switch
	
				if (NewParamSet != ParamSet) {
					LEDBlue_ON;
					ParamSet = NewParamSet;
					for (i = 0; i<=ParamSet; i++)
						DoBeep100mSWithOutput(1, 1);
					ParametersChanged = true;
					LEDBlue_OFF;
				}
				StickHoldState = MonitorSticks;
			}
		}
	} else if (State == Landed) {

		UpdateSticksState();

		if (StickHoldState == SticksChanged) {
			if (F.UsingMWStickProg && (SticksState == (THR_LO | YAW_CE
					| PIT_CE | ROL_LO)) && StickArmed) {
				LEDBlue_ON;
				DoBeep100mSWithOutput(1, 0);
				StickArmed = false;
				SticksState = MonitorSticks;
				LEDBlue_OFF;
			} else {

				EEChanged = true;
				switch (SticksState) {
				case THR_LO | YAW_CE | PIT_HI | ROL_CE:
					A[FB].AccBias += ACC_TRIM_STEP;
					break;
				case THR_LO | YAW_CE | PIT_LO | ROL_CE:
					A[FB].AccBias -= ACC_TRIM_STEP;
					break;
				case THR_LO | YAW_LO | PIT_CE | ROL_CE:
					A[LR].AccBias -= ACC_TRIM_STEP;
					break;
				case THR_LO | YAW_HI | PIT_CE | ROL_CE:
					A[LR].AccBias += ACC_TRIM_STEP;
					break;
				default:
					EEChanged = false;
					break;
				} // switch
	
				if (EEChanged) {
					WriteAccCalEE();
				//	UpdateMPU6050AccAndGyroBias();
					DoBeep100mSWithOutput(1, 0);
					StickHoldState = SticksChanging;
					StickTimermS = mSClock() + 100;
				} else
					StickHoldState = MonitorSticks;
			}
		}
	} else
		StickHoldState = MonitorSticks;
} // DoStickProgramming


boolean ParameterSanityCheck(void) {
	return ((P[RollRateKp] != 0) &&
			(P[PitchRateKp]!= 0) &&
			(P[YawRateKp] != 0) );
} // ParameterSanityCheck

void InitParameters(void) {
	static uint8 i;
	static int16 A;

	ALL_LEDS_ON;
	ParamSet = 1;
	MakeSensorsUnknown();

	F.AllowTurnToWP = true;	

	if ( ( ReadEE((uint16)RCChannels) == -1 ) || (ReadEE(MAX_PARAMETERS + (uint16)RCChannels) == -1 ) )
		UseDefaultParameters();

	ReadParametersEE();

	ALL_LEDS_OFF;  
} // InitParameters


