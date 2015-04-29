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

#ifndef USE_PACKET_COMMS

void ShowPrompt(void);
void ShowRxSetup(void);
void ShowSetup(void);
void ProcessCommand(void);

int8 PTemp[64];

#if defined(TESTING)
const rom uint8 SerHello[] = "UAVX TEST " Version 
#else
const rom uint8 SerHello[] = "UAVX " Version 							 
#endif // TESTING
 							  " Copyright 2008 G.K. Egan & 2007 W. Mahringer\r\n"
							  "This is FREE SOFTWARE and comes with ABSOLUTELY NO WARRANTY "
							  "see http://www.gnu.org/licenses/!\r\n";
#pragma idata

#pragma idata chanchar
const char rom ChanChar[] = {'T','R','P','Y','5','6','7','8','9'};
#pragma idata

#pragma idata menuhelp
const rom uint8 SerHelp[] = "\r\nCommands:\r\n"
	#if defined(TESTING)

//	"B..Load UAVX hex file\r\n"
	"C..Comp/Mag test\r\n"
	"D..Load default params\r\n"
//	"G..Dump black box\r\n"
	"H..Baro & RF test\r\n"
	"I..Gyro & Acc test\r\n"
	"K..Cal Comp/Mag\r\n"
//	"M..Modify params\r\n"
	"N..Cal Acc\r\n"
	"P..Rx test\r\n"
	"S..Setup\r\n"
	"T..I2C bus scan\r\n"
	"V..Battery test\r\n"
	"X..Flight stats\r\n"
#if defined(INC_I2C_PROG)
	"Y..Program YGE I2C ESC\r\n"
#endif

	#else

//	"B..Load UAVX hex file\r\n"
	"D..Load default param set\r\n"
	"S..Setup\r\n"
	"V..Battery test\r\n"
	"X..Flight stats\r\n"

	#endif // TESTING
	"1-8..Individual LED/buzzer test\r\n"; // last line must be in this form for UAVPSet
#pragma idata

#pragma idata rx_mnem_chars
const rom uint8 RxChMnem[] = "TAERG12345";
#pragma idata

void ShowPrompt(void) {
	TxString("\r\n>");
} // ShowPrompt

void ShowRxSetup(void) {
	if ( F.UsingCompoundPPM )
		TxString("Serial PPM frame");
	else
		TxString("Odd Rx Channels PPM");
} // ShowRxSetup

#pragma idata airframenames
const char * AFName[] = { "Tri", "CoaxTri",
		"VTail", "Quad", "QuadX", "CoaxQuad","CoaxQuadX", "Hexa", "HexaX",
		"Octo", "OctoX", "Heli90", "Heli120",
		"Wing", "Air", "VTOL", "Gimbal",  "unknown" };
#pragma idata

void ShowAFType(void) {
	TxString(AFName[AF_TYPE]);
} // ShowAFType

void ShowSetup(void) {
	static uint8 i;

	TxNextLine();
	TxString(SerHello);

	TxString("\r\nClock: 40MHz");

	TxString("\r\nAircraft: "); ShowAFType();

	TxString("\r\nPID Cycle (mS): ");
	TxVal32(PID_CYCLE_MS,0,' ');
	TxNextLine();

	#if defined(MULTICOPTER)
		TxString("\r\nForward Flight: ");
		TxVal32((int16)AFOrientation[AF_TYPE] * 75L, 1, 0);
		TxString("deg CW from K1 motor(s)");
	#endif // MULTICOPTER

	TxString("\r\nAccs: ");
	ShowAccType();	

	TxString("\r\nGyros: "); 
	ShowGyroType(GyroType);

	TxString("\r\nBaro: "); ShowBaroType();

	TxString("\r\nCompass: ");
	ShowCompassType();
	if(( F.MagnetometerActive) && ( CompassType != HMC6352Compass )) {
		TxString(" Offset ");
		TxVal32(COMPASS_OFFSET_QTR * 90,0,0);
		TxString("deg.");
	}

	TxString("\r\nESCs: ");	
	ShowESCType();
	if ( P[ESCType] != ESCPPM ) {
		TxString(" {");
		for ( i = 0; i < (uint8)NO_OF_I2C_ESCS; i++ )
			if ( ESCI2CFail[i] )
				TxString(" Fail");
			else
				TxString(" OK");
		TxString(" }");
	}	
	
	TxString("\r\nTx/Rx: ");
	ShowRxSetup();
	TxString("Tx Mode ");
	switch (P[TxMode]) {
	case TxMode1:
		TxChar('1' + P[TxMode]); 
		break;
	case TxMode2:
		TxChar('1' + P[TxMode]); 
		break;
	default: 
		TxString("unknown"); 
		break;
	}

	TxString("\r\nControl: ");
	if (F.UsingRateControl)
		TxString("Rate");
	else
		TxString("Angle");

	TxString("\r\nParam set: "); // must be exactly this string as UAVPSet expects it
	TxChar('0' + ParamSet);	

	TxString("\r\n\r\nNav:\r\n\tAutoland ");
	if ( F.UsingRTHAutoDescend )
		TxString("ENABLED\r\n");
	else
		TxString("disabled\r\n");

	if ( F.AllowTurnToWP )
		TxString("\tTurn to WP\r\n");
	else
		TxString("\tHold heading\r\n");

	TxString("\r\nALARM (if any):\r\n");
	if ( (( NoOfRCChannels&1 ) != (uint8)1 ) && !F.UsingCompoundPPM ) {
		TxString("\tODD Ch Inp selected, EVEN number used - reduced to ");
		TxVal32(NoOfRCChannels,0,0);
		TxNextLine();
	}
	if ( !F.FailsafesEnabled )
		TxString("\tYOU have DISABLED Failsafes\r\n");

	#if defined(TESTING)
		TxString("\tTEST VER. - No Motors\r\n");
	#endif // TESTING

	if ( !F.ParametersValid )
		TxString("\tINVALID flight params (PID)!\r\n");

	if ( F.ConfigError )
		TxString("\tRx, Rx Mode or PIC Sensors NOT DEFINED\r\n");
	
	if ( !F.BaroActive )
		TxString("\tBaro. FAIL\r\n");

	if ( !F.RangefinderActive )
		TxString("\tRangefinder OFFLINE\r\n");

	if ( ( AccType == AccUnknown ) || !F.IMUActive )
		TxString("\tAccs. FAIL\r\n");

	if ( !F.MagnetometerActive)
		TxString("\tCompass OFFLINE\r\n");

	if ( !F.Signal )
		TxString("\tBad EPAs or Tx switched off?\r\n");
	if ( Armed() && FirstPass ) 
		TxString("\tUAVX is armed - DISARM!\r\n");

	if ( F.ReturnHome )
		TxString("\tNav/RTH selected - DESELECT!\r\n");

	if ( InitialThrottle >= RC_THRES_START )
		TxString("\tClose Throttle!\r\n");
	
	ShowPrompt();
} // ShowSetup

void ProcessCommand(void) {
	static uint8 ch;
	static uint8 p;
	static int8 d;
	static int16 dd;
	static int32 Temp;

	if ( !Armed() ) {
		ch = PollRxChar();
		if ( ch != NUL ) {
			if( islower(ch))							// check lower case
				ch = toupper(ch);
			
			switch( ch ) {
			case 'B':	// call bootloader
				{ // arming switch must be OFF to call bootloader!!!
					DisableInterrupts;
					BootStart();		// never comes back!
				}
			case 'D':
				TxString("\r\nLoad default parameters - ");
				TxString("Click CONTINUE to confirm or CANCEL\r\n");
				do {
					ch = PollRxChar();
				} while ((ch != 'x') && (ch != 'z'));

				if (ch == 'x')
					UseDefaultParameters();
				else
					TxString("\r\nCancelled");
				ShowPrompt();
				break;
			case 'L'  :	// List parameters
				TxString("\r\nParameter list for set #");	// do not change (UAVPset!)
				TxChar('0' + ParamSet);
				ReadParametersEE();
				for( p = 0; p < MAX_PARAMETERS; p++) {
					TxString("\r\nRegister ");
					TxValU((uint8)(p+1));
					TxString(" = ");
					TxValS(P[p]);
				}
				ShowPrompt();
				break;
			case 'M'  : // modify parameters
				// no reprogramming in flight!!!!!!!!!!!!!!!
				LEDBlue_ON;
				TxString("\r\nRegister ");
				p = (uint8)(RxNumU()-1);
				TxString(" = ");
				dd = RxNumS();
				d = Limit(dd, -128, 127);
				if ( p == AFType)
					PTemp[p] = AF_TYPE;
				else
					PTemp[p] = d;
				
				if ( ( p == (MAX_PARAMETERS-1)) && ( P[RollRateKp] != 0 ) ) {
					#if defined(GKE_TUNE)
						PTemp[CamPitchKp] = TuneTrim;
					#endif	
					PTemp[AFType] = AF_TYPE;
					PTemp[IMU] = Wolferl;
					PTemp[GPSProtocol] = DefaultParams[GPSProtocol][1];
					for (p = 0; p<MAX_PARAMETERS;p++)
						if( ParamSet == (uint8)1 ) {
							WriteEE(p, PTemp[p]);
							if ( DefaultParams[p][1] )
								WriteEE(MAX_PARAMETERS + p, PTemp[p]);
						} else {
							if ( !DefaultParams[p][1] )
								WriteEE(MAX_PARAMETERS + p, PTemp[p]);
						}

					F.ParametersValid = true; 	// ALL parameters must be written 
					ParametersChanged = true;
				}
				LEDBlue_OFF;
				ShowPrompt();
				break;
			case 'Z' : // set Paramset
				p = RxNumU();
				if ( p != (int8)ParamSet ) {
					ParamSet = p;
					ParametersChanged = true;
					ReadParametersEE();
				}
				break;
			case 'R':	// receiver values
				TxString("\r\n");
				for (d = ThrottleRC; d <= CamPitchRC; d++) {
					TxChar(ChanChar[d]);
					TxChar(':');
					Temp = ((int32)RC[d]*500)/RC_MAXIMUM;
					TxValS(Limit1(Temp,999));
					TxChar(',');
				}
				TxString("X:+000");
				ShowPrompt();
				break;
			case 'S' :	// show status
				ShowSetup();
				break;
			case 'X' :	// flight stats
				ShowStats();
				ShowPrompt();
				break;
			case 'G':
				DumpBlackBox();
				ShowPrompt();
				break;
			#if defined(TESTING)
			case 'T':
				TxString("\r\nI2C devices ...\r\n");
				TxVal32(ScanI2CBus(),0,0);
				TxString(" device(s) found\r\n");
				ShowPrompt();
				break;
			case 'C':
				DoCompassTest();
				ShowPrompt();
				break;		
			case 'H':	// barometer
				BaroTest();
				ShowPrompt();
				break;
			case 'I':	// gyro
				GyrosAndAccsTest();
				ShowPrompt();
				break;	
			case 'K':
				CalibrateCompass();
				ShowPrompt();
				break;	
			case 'N' :	// neutral values
				GetNeutralAccelerations();
				ShowPrompt();
				break;		
			case 'P'  :	// Receiver test			
				ReceiverTest();
				ShowPrompt();
				break;
			#if defined(INC_I2C_PROG)
			case 'Y':	// configure YGE30i EScs
				ConfigureESCs();
				ShowPrompt();
				break;
			#endif
			#endif // TESTING
			case 'V' :	// Battery test
				BatteryTest();
				ShowPrompt();
				break;
			case '?'  :  // help
				TxString(SerHelp);
				ShowPrompt();
				break;
			default: break;
			} // switch
		}
	}
} // ProcessCommand

#endif // USE_PACKET_COMMS
