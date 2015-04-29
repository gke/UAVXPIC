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

// Interrupt Routines

#include "uavx.h"

#define MIN_PPM_SYNC_PAUSE 3750  	// 3000 *0.8us = 2.4ms // was 6250 5mS

// no less than 1500 ?? 

void InitTimersAndInterrupts(void);
uint32 Timer0Ticks(void);
void low_isr_handler(void);
void high_isr_handler(void);

#pragma udata clocks
volatile uint24	mS[CompassUpdate+1];
volatile uint24 CycleUpdateuS;
#pragma udata

#pragma udata access nearvars
near volatile uint16 uSTop;
near i16u 	PPM[CONTROLS];
near uint8 	PPM_Index, NoOfRCChannels;
near int24 	PrevEdge, CurrEdge;
near i16u 	Width, Timer0;
near int24 	PauseTime;
near uint8 	RxState;

near uint8 	ll, ss, tt, RxCh;
near uint8 	RxCheckSum, TxCheckSum, GPSCheckSumChar, GPSTxCheckSum;
near uint16 RxQTail, RxQHead, TxQTail, TxQHead;

#pragma udata

int8	SignalCount;
uint16	RCGlitches;

void ReceivingGPSOnly(boolean r) {

	if ( r != F.ReceivingGPS ) {
		PIE1bits.RCIE = false;
		F.ReceivingGPS = r;

		if ( F.ReceivingGPS )		
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_LOW, _B9600);
		else
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
   		PIE1bits.RCIE = r;

		DisableInterrupts;
		RxQTail = RxQHead = 0;
		EnableInterrupts;
	}
} // ReceivingGPSOnly

void InitTimersAndInterrupts(void) {
	static uint8 i;

	OpenTimer0(TIMER_INT_OFF&T0_16BIT&T0_PS_1_4&T0_SOURCE_INT); //15
	
	OpenTimer1(TIMER_INT_OFF&T1_16BIT_RW&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);
	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge

	CCP1CONbits.CCP1M0 = true; 

	RxQHead = RxQTail = TxQHead = TxQTail = RxCheckSum = 0;

	for (i = StartTime; i<= (uint8)CompassUpdate; i++)
		mS[i] = 0;

	uSTop = 0;

	INTCONbits.PEIE = true;	
	INTCONbits.TMR0IE = true; 

} // InitTimersAndInterrupts

uint32 Timer0Ticks(void) {
	static i32u m;

	DisableInterrupts;
	m.b0 = TMR0L;
	m.b1 = TMR0H;
	m.w1 = uSTop;
	EnableInterrupts;

	return(m.u32);

} // Timer0Ticks

uint32 uSClock(void) {
	return((Timer0Ticks() * 4) / 10);
} // mSClock

uint24 mSClock(void) {
	return((Timer0Ticks() * 4) / 10000);
} // mSClock

void mSTimer(uint8 t, int32 TimePeriod) {
	mS[t] = mSClock() + TimePeriod;
} // mSTimer

#pragma interrupt low_isr_handler
void low_isr_handler(void) {
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void) {

	if( PIR1bits.CCP1IF ) {	// An Rx PPM pulse edge has been detected
		CurrEdge = CCPR1;
		if ( CurrEdge < PrevEdge )
			PrevEdge -= (int24)0x00ffff;		// Deal with wraparound

		Width.i16 = (int16)(CurrEdge - PrevEdge);
		PrevEdge = CurrEdge;		

		if ( Width.i16 > MIN_PPM_SYNC_PAUSE ) {	// A pause  > 5ms
			PPM_Index = 0;						// Sync pulse detected - next CH is CH1
			F.RCFrameOK = true;
			F.RCNewValues = false;
			PauseTime = Width.i16;	
		} else 
			if (PPM_Index < NoOfRCChannels) {
					PPM[PPM_Index].i16 = (int16) Width.i16 - 1250;		
				
				PPM_Index++;
				// MUST demand rock solid RC frames for autonomous functions not
				// to be cancelled by noise-generated partially correct frames
				if ( PPM_Index == NoOfRCChannels ) {
					if ( F.RCFrameOK ) {
						F.RCNewValues = true;
 						SignalCount++;
					} else {
						F.RCNewValues = false;
						SignalCount -= RC_GOOD_RATIO;
					}

					SignalCount = Limit1(SignalCount, RC_GOOD_BUCKET_MAX);
					F.Signal = SignalCount > 0;
				}
			}

		if ( !F.UsingCompoundPPM )						
			CCP1CONbits.CCP1M0 ^= 1;

		PIR1bits.CCP1IF = false;
	}

	if ( PIR1bits.RCIF & PIE1bits.RCIE ) { // RCIE enabled for GPS
		if ( RCSTAbits.OERR | RCSTAbits.FERR ) {
			RxCh = RCREG; // flush
			RCSTAbits.CREN = false;
			RCSTAbits.CREN = true;
		} else { // PollGPS in-lined to avoid EXPENSIVE context save and restore within irq

			RxCh = RCREG;
			RxQTail = (RxQTail + 1) & RX_BUFF_MASK;
			RxQ[RxQTail] = RxCh;
		}
	
		PIR1bits.RCIF = false;
	}

	if ( INTCONbits.T0IF & INTCONbits.TMR0IE) {
		uSTop++;
		
		INTCONbits.TMR0IF = false;	
	}

	if (PIR1bits.TXIF & PIE1bits.TXIE) {
		if (TxQHead != TxQTail) {
			TXREG = TxQ[TxQHead];
			TxQHead = (TxQHead + 1) & TX_BUFF_MASK;
		}
		if ( TxQHead == TxQTail)
			PIE1bits.TXIE = false;
		PIR1bits.TXIF = false;
	}

} // high_isr_handler
	
#pragma code high_isr = 0x08
void high_isr (void) {
  _asm goto high_isr_handler _endasm
} // high_isr
#pragma code

#pragma code low_isr = 0x18
void low_isr (void) {
  _asm goto low_isr_handler _endasm
} // low_isr
#pragma code

