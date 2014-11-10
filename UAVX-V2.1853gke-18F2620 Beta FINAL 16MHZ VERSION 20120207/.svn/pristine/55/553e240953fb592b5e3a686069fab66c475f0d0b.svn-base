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

void LightsAndSirens(void);
void InitPortsAndUSART(void);
void InitMisc(void);
void Delay1mS(int16);
void Delay100mSWithOutput(int16);
void DoBeep100mSWithOutput(uint8, uint8);
void DoStartingBeepsWithOutput(uint8);
void CheckAlarms(void);
int32 SlewLimit(int32, int32, int32);
int32 ProcLimit(int32, int32, int32);
int16 DecayX(int16, int16);
void LPFilter16(int16*, i32u*, int16);
void LPFilter24(int24* i, i32u* iF, int16 FilterA);

int8 BatteryVolts;
int16 BatteryVoltsADC, BatteryCurrentADC, BatteryVoltsLimitADC, BatteryCurrentADCEstimated, BatteryChargeUsedmAH;
int32 BatteryChargeADC, BatteryCurrent;

int8 RCStart = RC_INIT_FRAMES;
boolean	FirstPass;

void LightsAndSirens(void)
{
	static int24 Ch5Timeout;

	LEDYellow_TOG;
	if ( F.Signal ) LEDGreen_ON; else LEDGreen_OFF;

	Beeper_OFF;
	Ch5Timeout = mSClock() + 500; // mS.
	do
	{
		SpareSlotTime = true; // for "tests"
		ProcessCommand();
		GetHeading();
		GetBaroAltitude();
		if( F.Signal )
		{
			LEDGreen_ON;
			if( F.RCNewValues )
			{
				UpdateControls();
				if ( --RCStart == 0 ) // wait until RC filters etc. have settled
				{
					UpdateParamSetChoice();
					MixAndLimitCam();
					RCStart = 1;
				}
				InitialThrottle = StickThrottle;
				StickThrottle = DesiredThrottle = 0; 
				OutSignals(); // synced to New RC signals
				if( mSClock() > (uint24)Ch5Timeout )
				{
					if ( F.Ch5Active || !F.ParametersValid )
					{
						Beeper_TOG;					
						LEDRed_TOG;
					}
					else
						if ( Armed )
							LEDRed_TOG;
							
					Ch5Timeout += 500;
				}	
			}
		}
		else
		{
			LEDRed_ON;
			LEDGreen_OFF;
		}	
		ReadParametersEE();	
	}
	while( (!F.Signal) || (Armed && FirstPass) || F.Ch5Active || F.GyroFailure || 
		( InitialThrottle >= RC_THRES_START ) || (!F.ParametersValid)  );
			
	FirstPass = false;

	Beeper_OFF;
	LEDRed_OFF;
	LEDGreen_ON;

	if ( F.AccelerationsValid )
		LEDYellow_ON;
	else
		LEDYellow_OFF;

	mS[LastBattery] = mSClock();
	mS[FailsafeTimeout] = mSClock() + FAILSAFE_TIMEOUT_MS;
	PIDUpdate = mSClock() + PID_CYCLE_MS;
	F.LostModel = false;
	FailState = MonitoringRx;

} // LightsAndSirens

void InitPortsAndUSART(void)
{
	// general ports setup
	TRISA = 0b00111111;				// all inputs
	ADCON1 = 0b00000010;			// Vref used for Rangefinder

	#ifdef UAVX_HW
		PORTB = 0b00000000;			// all outputs - motos/servos LEDs, buzzer
		TRISB = 0b00000000;					
	#else // UAVX_HW
		PORTB = 0b11000000;			// all outputs to low, except RB6 & 7 (I2C)!
		TRISB = 0b01000000;			// all servo and LED outputs
	#endif // UAVX_HW

	PORTC = 0b01100000;				// all outputs to low, except TxD and CS
	#ifdef HAVE_CUTOFF_SW
		TRISC = 0b10000101;			// RC7, RC2, RC0 are inputs
	#else
		TRISC = 0b10000100;			// RC7, RC2 are inputs
	#endif // HAVE_CUTOFF_SW

	SSPSTATbits.CKE = true;			// low logic threshold for LISL
	
	INTCON2bits.NOT_RBPU = false;	// WEAK PULLUPS MUST BE ENABLED OTHERWISE I2C VERSIONS 
									// WITHOUT ESCS INSTALLED WILL PREVENT ANY FURTHER BOOTLOADS

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
} // InitPortsAndUSART

void InitMisc(void)
{
	static int8 i;

	State = Starting;				// For trace preconditions
	
	for ( i = 0; i < FLAG_BYTES ; i++ )
		F.AllFlags[i] = false;
 
	F.ParametersValid = F.AcquireNewPosition = F.AllowNavAltitudeHold = true;

	#ifdef SIMULATE
	F.Simulation = true;
	#endif // SIMULATE

//	YawFilterA = ( PID_CYCLE_MS * 256L) / ( 1000L / ( 6L * (int24) FILT_YAW_HZ ) + PID_CYCLE_MS );
//	HeadingFilterA	= ( COMPASS_TIME_MS * 256L) / ( 1000L / ( 6L * (int24) FILT_COMPASS_HZ*8 ) + COMPASS_TIME_MS );

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

void Delay1mS(int16 d)
{ 
	static uint24 Timeout;

	Timeout = mSClock() + d + 1;
	while ( mSClock() < Timeout ) {};

} // Delay1mS

void Delay100mSWithOutput(int16 dur)
{  // Motor and servo pulses are still output

	static uint24 Timeout;

	Timeout = mSClock() + dur * 100;
	while ( mSClock() < Timeout )
	{
		Delay1mS(PID_CYCLE_MS - 2);		
		OutSignals(); // 1-3 ms Duration
	}

} // Delay100mSWithOutput

void DoBeep100mSWithOutput(uint8 t, uint8 d)
{
	Beeper_ON;
	Delay100mSWithOutput(t);
	Beeper_OFF;
	Delay100mSWithOutput(d);
} // DoBeep100mSWithOutput

void DoStartingBeepsWithOutput(uint8 b)
{
	static uint8 i;

	for ( i = 0; i < b; i++ )
		DoBeep100mSWithOutput(2, 8);

	DoBeep100mSWithOutput(8,0);
} // DoStartingBeeps

void CheckBatteries(void)
{
	static uint24 Now;
	static int16 Temp;

	Now = mSClock();

	if (( Now >= mS[BatteryUpdate] ) && SpareSlotTime )
	{
		mS[BatteryUpdate] = Now + BATTERY_UPDATE_MS;
		SpareSlotTime = false;
		//No spare ADC channels yet. Temp = ADC(ADCBattCurrentChan);
		BatteryCurrentADC  = CurrThrottle * THROTTLE_CURRENT_SCALE; // Mock Sensor
	
		BatteryChargeADC += BatteryCurrentADC * (Now - mS[LastBattery]);
		mS[LastBattery] = Now;
		BatteryChargeUsedmAH = CURRENT_SENSOR_MAX * (BatteryChargeADC/3686400L); // 1024*1000*3600
	
		Temp = ADC(ADCBattVoltsChan);
		BatteryVoltsADC  = SoftFilter(BatteryVoltsADC, Temp);
		F.LowBatt = (BatteryVoltsADC < BatteryVoltsLimitADC ) & 1;
	}
} // CheckBatteries

static int16 BeeperOffTime = 100;
static int16 BeeperOnTime = 100;

void CheckAlarms(void)
{
	if ( SpareSlotTime )
	{
		SpareSlotTime = false;
		F.BeeperInUse = F.LowBatt || F.LostModel  || (State == Shutdown);
	
		if ( F.BeeperInUse )
		{
			if( F.LowBatt ) 
			{
				BeeperOffTime = 600;
				BeeperOnTime = 600;
			}	
			else
				if ( State == Shutdown )
				{
					BeeperOffTime = 4750;
					BeeperOnTime = 250;
				}
				else
					if ( F.LostModel )
					{
						BeeperOffTime = 125;
						BeeperOnTime = 125;		
					}		
	
			if ( mSClock() > mS[BeeperUpdate] )
				if ( BEEPER_IS_ON )
				{
					mS[BeeperUpdate] = mSClock() + BeeperOffTime;
					Beeper_OFF;
					LEDRed_OFF;					
				}
				else
				{
					mS[BeeperUpdate] = mSClock() + BeeperOnTime;
					Beeper_ON;
					LEDRed_ON;		
				}	
		}	
		#ifdef NAV_ACQUIRE_BEEPER
		else
			if ( (State == InFlight) && (!F.AcquireNewPosition) && (mSClock() > mS[BeeperTimeout]) )
				Beeper_OFF;
		#endif // NAV_ACQUIRE_BEEPER 
	}

} // CheckAlarms

#ifndef USE_LIMIT_MACRO
int32 ProcLimit(int32 i, int32 l, int32 u)
{
	return ((i<l) ? l : ((i>u) ? u : i));	
} // ProcLimit
#endif // USE_LIMIT_MACRO

int16 DecayX(int16 i, int16 d)
{
	if ( i < 0 )
	{
		i += d;
		if ( i >0 )
			i = 0;
	}
	else
	if ( i > 0 )
	{
		i -= d;
		if ( i < 0 )
			i = 0;
	}
	return (i);
} // DecayX

int32 SlewLimit(int32 Old, int32 New, int32 Slew)
{
  int32 Low, High;
  
  Low = Old - Slew;
  High = Old + Slew; 
  return(( New < Low ) ? Low : (( New > High ) ? High : New));
} // SlewLimit

/* abandon filters in favour of simple weighted averaging on last two samples

void LPFilter16(int16* i, i32u* iF, int16 FilterA)
{
	static i24u Temp;

	Temp.b0 = 0;
	Temp.i2_1 = *i;
	iF->i32 += ((int32)Temp.i24 - iF->i3_1) * FilterA;
	*i = iF->iw1;

} // LPFilter16

void LPFilter24(int24* i, i32u* iF, int16 FilterA)
{
	static i32u Temp;

	Temp.b0 = 0;
	Temp.i3_1 = *i;
	iF->i32 += ((int32)Temp.i32 - iF->i3_1) * FilterA;
	*i = iF->iw1;

} // LPFilter24

LPFilter16(&Heading, &HeadingValF, HeadingFilterA);

*/
