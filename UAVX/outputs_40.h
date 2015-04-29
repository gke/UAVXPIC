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

void OutSignals(void);

#if defined(TRICOPTER)
	#define MOTOR_MASK 0b00000111
	#define SERVO_MASK 0b00111000
#else
	#if defined(Y6COPTER) | defined(HEXACOPTER) | defined(HEXACOPTERX)
		#define MOTOR_MASK 0b00111111
		#define SERVO_MASK 0b00000000
	#else
		#if defined MULTICOPTER // QUADROCOPTER | VTCOPTER
			#define MOTOR_MASK 0b00001111
			#define SERVO_MASK 0b00110000
		#else // CONVENTIONAL | VTOL
			#define MOTOR_MASK 0b00000000
			#define SERVO_MASK 0b00111111
		#endif
	#endif
#endif

void OutSignals(void) {	
	static uint8 m;
	static boolean UpdateServo = true;
	static volatile uint32 FinishTicks, LatencyTicks;
	static uint32 NowmS;
	static i32u Ticks;

	if ( !F.DrivesArmed )
		StopMotors();

	if ( F.Bypass || !Armed() ) {
		Rl = (int16) (-A[Roll].Desired);
		Pl = (int16) (-A[Pitch].Desired);
		Yl = (int16) (-A[Yaw].Desired);
	}

	#if defined(SIMULATE) | defined(TESTING)

	MixAndLimitMotors();
	MixAndLimitCam();

	#else

	NowmS = mSClock();
	UpdateServo = NowmS >= mS[ServoUpdate];
	if (UpdateServo)
		mSTimer(ServoUpdate, SERVO_CYCLE_MS);

	if ( P[ESCType] == ESCPPM ) {

		DisableInterrupts;
		FinishTicks = Timer0Ticks() + 2500;

		if ( UpdateServo ) {
			PORTB = (MOTOR_MASK | SERVO_MASK);
	
			_asm
			MOVLB	0
			MOVLW	(MOTOR_MASK | SERVO_MASK)
			MOVWF	SHADOWB,1
			_endasm	
		} else {
			PORTB |= MOTOR_MASK;
	
			_asm
			MOVLB	0
			MOVLW	MOTOR_MASK				
			MOVWF	SHADOWB,1
			_endasm
			Delay1TCY();
			Delay1TCY();
		}

		EnableInterrupts;

		MixAndLimitMotors();
		MixAndLimitCam();

		if ( P[DriveFilt] > 0 )
			for (m = 0; m < MAX_DRIVES; m++) 
				PWp[m] = SRS16(PWp[m] + PW[m], 1);

		PW0 = PWLimit(PWp[K1]);
		PW1 = PWLimit(PWp[K2]);
		PW2 = PWLimit(PWp[K3]);
		PW3 = PWLimit(PWp[K4]);	
		PW4 = PWLimit(PWp[K5]);
		PW5 = PWLimit(PWp[K6]);

		LatencyTicks = FinishTicks - INT_LATENCY;
		while ( Timer0Ticks() < LatencyTicks){};

		DisableInterrupts;
		do {
			if (INTCONbits.T0IF & INTCONbits.TMR0IE) { // rarely if ever happens!
				uSTop++;	
				INTCONbits.TMR0IF = false;
				Ticks.w0 = 0;	
			}
			else {
				Ticks.b0 = TMR0L;
				Ticks.b1 = TMR0H;
			}
			Ticks.w1 = uSTop;
		} while ( Ticks.u32 < FinishTicks);
	
		_asm
		MOVLB	0
OS005:
		MOVF	SHADOWB,0,1	
		MOVWF	PORTB,0
		ANDLW	0x3f		
		
		BZ		OS006
					
		DECFSZ	PW0,1,1				
		GOTO	OS007
							
		BCF		SHADOWB,0,1
OS007:
		DECFSZ	PW1,1,1		
		GOTO	OS008
							
		BCF		SHADOWB,1,1
OS008:
		DECFSZ	PW2,1,1
		GOTO	OS009
							
		BCF		SHADOWB,2,1
OS009:
		DECFSZ	PW3,1,1	
		GOTO	OS010
								
		BCF		SHADOWB,3,1			
OS010:
		DECFSZ	PW4,1,1
		GOTO	OS011
		
		BCF		SHADOWB,4,1			
OS011:
		DECFSZ	PW5,1,1
		GOTO	OS012
			
		BCF		SHADOWB,5,1
		OS012:
		_endasm
		
		Delay10TCY(); 
		Delay10TCY(); 
		Delay1TCY();
		Delay1TCY();

		_asm						
		GOTO	OS005
OS006:
		_endasm
		
		EnableInterrupts;

	} else {
		MixAndLimitMotors();
		MixAndLimitCam();
	
		DoI2CESCs(); // no camera servos for now - check how long this takes
	}
	
	#endif // SIMULATE | TESTING

} // OutSignals
