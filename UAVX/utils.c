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

#pragma udata battery_vars
int8 BatteryVolts;
int16 BatteryVoltsADC, BatteryCurrentADC, BatteryVoltsLimitADC, BatteryCurrentADCEstimated, BatteryChargeUsedmAH;
int32 BatteryChargeADC, BatteryCurrent;
#pragma udata

boolean	FirstPass;

void DoCalibrationAlarm(void) {
	static uint32 TimeoutmS = 0;

	if (!(F.IMUCalibrated && F.MagnetometerCalibrated))
		if (mSClock() > TimeoutmS) {
			TimeoutmS = mSClock() + 500;
			LEDYellow_TOG;
		}
} // DoAccCalibrationAlarm

void LightsAndSirens(void) {
	static int8 RCStart = RC_INIT_FRAMES;
	static uint32 NowmS;
	static int24 Ch5Timeout;
	static uint8 s;

	LEDYellow_TOG;
	if ( F.Signal ) 
		LEDGreen_ON; 
	else 
		LEDGreen_OFF;

	Beeper_OFF;
	Ch5Timeout = mSClock() + 500; // mS.
	do {
#ifdef USE_PACKET_COMMS

#else
		ProcessCommand();
#endif
		SpareSlotTime = true; // for "tests"
		GetHeading();
		SpareSlotTime = true; // for "tests"
		GetBaroAltitude();
		F.HoldingAlt = false;
	
		if( F.Signal ) {
			LEDGreen_ON;
			if( F.RCNewValues ) {
				UpdateControls();
			
				InitialThrottle = StickThrottle;
				StickThrottle = DesiredThrottle = 0; 
				OutSignals(); // synced to New RC signals
				if(( mSClock() > (uint24)Ch5Timeout ) && !F.Bypass) {
					F.ParametersValid = ParameterSanityCheck();
					if (  F.ReturnHome || !F.ParametersValid ){
						Beeper_TOG;					
						LEDRed_TOG;
					} else
						LEDRed_OFF;
							
					Ch5Timeout += 500;
				}
	
				if (RCStart == 0) // wait until RC filters etc. have settled
					DoStickProgramming();
				else
					RCStart--;
			}
		} else {
			LEDRed_ON;
			LEDGreen_OFF;
		}	
		ReadParametersEE();	
	}
	while( (!F.Signal) 
		|| (Armed() && FirstPass) 
		|| (F.ConfigError)
		|| F.ReturnHome 
		|| (!F.MagnetometerActive) 
		|| ( InitialThrottle >= RC_THRES_START ) 
		|| (!F.ParametersValid)  
		|| (!F.IMUActive)
	);
			
	FirstPass = false;

	Beeper_OFF;
	LEDRed_OFF;
	LEDGreen_ON;

	DoCalibrationAlarm();

	NowmS = mSClock();
	mS[LastBattery] = NowmS;
	mSTimer(FailsafeTimeout, FAILSAFE_TIMEOUT_MS);
	CycleUpdateuS = uSClock() + PID_CYCLE_US;
	F.LostModel = false;
	FailState = Monitoring;

} // LightsAndSirens

void DumpBlackBox(void) {
	static uint8 i;
 
	TxString("\r\nNo inflight logs for UAVXPIC\r\n");

	#if defined(TESTING)	
		ShowSetup();
		ScanI2CBus();
		GyrosAndAccsTest();
		DoCompassTest();
		BaroTest();
		ReceiverTest();
		BatteryTest();
		ShowStats();
	#else
#ifndef USE_PACKET_COMMS
		ShowSetup();
		ShowStats();
		BatteryTest();
#endif
	#endif // TESTING

	TxString("\r\nParameters:");
	for (i = 0; i < MAX_PARAMETERS; i++) {
		TxString("\r\n ");
		TxValU(i + 1);
		TxString(" = ");
		TxValS(P[i]);
		if (DefaultParams[i][1])
			TxString(" .");
		else {
			TxChar(' ');
			TxValS(ReadEE(MAX_PARAMETERS + i));
		}
	}

} // DumpBlackBox

void InitPortsAndUSART(void) {
	// general ports setup
	TRISA = 0b00111111;				// all inputs
	ADCON1 = 0b00000010;			// Vref used for Rangefinder

	#if defined(UAVX_HW)
		PORTB = 0b00000000;			// all outputs - motors/servos LEDs, buzzer
		TRISB = 0b00000000;					
	#else // UAVX_HW
		PORTB = 0b11000000;			// all outputs to low, except RB6 & 7 (I2C)!
		TRISB = 0b01000000;			// all servo and LED outputs
	#endif // UAVX_HW

	PORTC = 0b01100000;				// all outputs to low, except TxD and CS
	#if defined(HAVE_CUTOFF_SW)
		TRISC = 0b10000101;			// RC7, RC2, RC0 are inputs
	#else
		TRISC = 0b10000100;			// RC7, RC2 are inputs
	#endif // HAVE_CUTOFF_SW

	SSPSTATbits.CKE = true;			// low logic threshold for LISL
	
	INTCON2bits.NOT_RBPU = false;	// WEAK PULLUPS MUST BE ENABLED OTHERWISE I2C VERSIONS 
									// WITHOUT ESCS INSTALLED WILL PREVENT ANY FURTHER BOOTLOADS

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B9600);
} // InitPortsAndUSART

void InitMisc(void) {
	static uint8 i;

	State = Starting;				// For trace preconditions
	
	for ( i = 0; i < (uint8)FLAG_BYTES ; i++ )
		F.AllFlags[i] = false;

	F.ParametersValid = F.AcquireNewPosition = F.AltControlEnabled = true;

	#if defined(SIMULATE)
	F.Emulation = true;
	#endif // SIMULATE

	BatteryChargeADC = 0;

	ThrNeutral = ThrLow = ThrHigh = MAXINT16;
	IdleThrottle = ((10L*OUT_MAXIMUM)/100);
	InitialThrottle = RC_MAXIMUM;
	ESCMax = OUT_MAXIMUM;	

	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;
} // InitMisc

boolean Armed(void) {

	if (F.UsingMWStickProg)
		return (ArmingSwitch && StickArmed);
	else
		return (ArmingSwitch);

} // Armed

void Delay1mS(int16 d) { 
	static uint24 Timeout;

	Timeout = mSClock() + d + 1;
	while ( mSClock() < Timeout ) {};

} // Delay1mS

void Delay100mSWithOutput(int16 dur) {
  // Motor and servo pulses are still output

	static uint24 Timeout;

	Timeout = mSClock() + dur * 100;
	while ( mSClock() < Timeout ) {
		Delay1mS(PID_CYCLE_MS - 2);		
		OutSignals(); // 1-3 ms Duration
	}

} // Delay100mSWithOutput

void DoBeep100mSWithOutput(uint8 t, uint8 d) {
	Beeper_ON;
	Delay100mSWithOutput(t);
	Beeper_OFF;
	Delay100mSWithOutput(d);
} // DoBeep100mSWithOutput

void DoStartingBeepsWithOutput(uint8 b) {
	static uint8 i;

	for ( i = 0; i < b; i++ )
		DoBeep100mSWithOutput(2, 8);

	DoBeep100mSWithOutput(8,0);
} // DoStartingBeeps

void CheckBatteries(void) {
	static uint24 NowmS;
	static int16 Temp;

	NowmS = mSClock();

	if (( NowmS >= mS[BatteryUpdate] ) && SpareSlotTime ) {
		mSTimer(BatteryUpdate, BATTERY_UPDATE_MS);
		SpareSlotTime = false;
		//No spare ADC channels yet. Temp = ADC(ADCBattCurrentChan);
		BatteryCurrentADC  = CurrThrottle * THROTTLE_CURRENT_SCALE; // Mock Sensor
	
		BatteryChargeADC += BatteryCurrentADC * (NowmS - mS[LastBattery]);
		mS[LastBattery] = NowmS;
		BatteryChargeUsedmAH = CURRENT_SENSOR_MAX * (BatteryChargeADC/3686400L); // 1024*1000*3600
	
		Temp = ADC(ADCBattVoltsChan);
		BatteryVoltsADC  = SoftFilter(BatteryVoltsADC, Temp);
		F.LowBatt = (BatteryVoltsADC < BatteryVoltsLimitADC ) & 1;
	}
} // CheckBatteries

static int16 BeeperOffTime = 100;
static int16 BeeperOnTime = 100;

void CheckAlarms(void) {
	if ( SpareSlotTime ) {
		SpareSlotTime = false;
		F.BeeperInUse = F.LowBatt || F.LostModel  || (F.AltControlEnabled && F.HoldingAlt) || (State == Shutdown);
	
		if ( F.BeeperInUse ) {
			if( F.LowBatt ) {
				BeeperOffTime = 600;
				BeeperOnTime = 600;
			} else
				if (F.AltControlEnabled && F.HoldingAlt) {
					BeeperOffTime = 200;
					BeeperOnTime = 20;
				} else
					if ( State == Shutdown ) {
						BeeperOffTime = 4750;
						BeeperOnTime = 250;
					} else
						if ( F.LostModel ) {
							BeeperOffTime = 125;
							BeeperOnTime = 125;		
						}		
	
			if ( mSClock() > mS[BeeperUpdate] )
				if ( BEEPER_IS_ON ) {
					mSTimer(BeeperUpdate, BeeperOffTime);
					Beeper_OFF;
					LEDRed_OFF;					
				} else {
					mSTimer(BeeperUpdate, BeeperOnTime);
					Beeper_ON;
					LEDRed_ON;		
				}	
		}	
		 else 
			if (mSClock() > mS[BeeperTimeout])
				Beeper_OFF;
	}

} // CheckAlarms

#ifndef USE_LIMIT_MACRO
int32 ProcLimit(int32 i, int32 l, int32 u) {
	return ((i<l) ? l : ((i>u) ? u : i));	
} // ProcLimit
#endif // USE_LIMIT_MACRO

int16 DecayX(int16 i, int16 d) {
	if ( i < 0 ) {
		i += d;
		if ( i >0 )
			i = 0;
	} else 
		if ( i > 0 ) {
			i -= d;
			if ( i < 0 )
				i = 0;
		}
	return (i);
} // DecayX


int32 Threshold(int32 v, int16 t) {

	if (v > t)
		v -= t;
	else
		if (v < -t)
			v += t;
		else
			v = 0;
	return (v);
} // Threshold

int32 SlewLimit(int32 Old, int32 New, int16 Slew){
  static int32 Low, High;
  
  Slew = Abs(Slew);

  Low = Old - Slew;
  High = Old + Slew; 
  return(( New < Low ) ? Low : (( New > High ) ? High : New));
} // SlewLimit

void Rotate(int16 * nx, int16 * ny, int24 x, int24 y, int16 A) { // A is CW rotation
	static int16 CosA, SinA, Temp;

	if (A == 0) { // Angle is integer so OK
		*nx = x;
		*ny = y;
	} else {
		CosA = int16cos(A);
		SinA = int16sin(A);
	
		Temp = SRS32(x * CosA + y * SinA, 8);
		*ny = SRS32(-x * SinA + y * CosA, 8);
		*nx = Temp;
	}
} // Rotate



