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
	
	#pragma idata paramdefaults
	const rom int8 DefaultParams[MAX_PARAMETERS][2] = 
/*
	{
			{17,0},				// RollRateKp, 				01
			{8,0},	 			// RollRateKd,				02
			{15, 0},			// RollAngleKp,				03
			{ModeUnknown,true},	// TxMode,					04
			{30,0},	 			// RollIntLimit,			05

			{17,0},	 			// PitchRateKp,				06
			{8,0},	 			// PitchRateKd,				07
			{15,0},	 			// PitchAngleKp,			08
			{0, 0},	 			// Unused9,					09
			{30,0},	 			// PitchIntLimit,			10

			{25,0},	 			// YawRateKp, 				11
			{50,0},	 			// RollRateKd was YawKi,	12
			{Wolferl, true},	// IMU,						13
			{20,0},	 			// YawRateIntLimit,			14
			{RCUnknown, true},	// RCType,					15

	
			{UseMWStickProgMask,true}, // ConfigBits,		16c
			{1,true},			// RxThrottleCh,			17
			{51,true}, 			// LowVoltThres,			18c 
			{20,true}, 			// CamRollKp,				19c
			{45,true}, 			// PercentCruiseThr,		20c

			{20,true}, 			// AltCFKpTrim,				21c 
			{30,true}, 			// RollPitchMix,			22c
			{5,true}, 			// PercentIdleThr,			23c
			{15,0}, 			// RollAngleKi,				24c
			{15,0}, 			// PitchAngleKi,			25c
			{20,true}, 			// CamPitchKp,				26c
			{12,0}, 			// YawAngleKp (Compass),	27
			{50,0},				// PitchRateKd,				28
			{10,true}, 			// NavMaxAngle,				29
			{19,true}, 			// ROCRateKp,				30

			{0,0}, 				// YawRateKd,	    		31
			{50,0},				// MadgwickKpMag,	   		32
			{10,true}, 		    // NavRTHAlt,				33
			{13,true},			// NavMagVar,				34c
			{FreeIMU,true}, 	// SensorHint,     			35c

			{ESCPPM,true}, 		// ESCType,					36c
			{7,true}, 			// RxChannels,				37c
			{2,true},			// RxRollCh,				38
			{5,true},			// MadgwickKpAcc,			39c
			{1,true},			// CamRollTrim,				40c

			{2,true},			// NavMaxVelMpS,			41
			{3,true},			// RxPitchCh,				42
			{4,true},			// RxYawCh,					43
			{QuadAF,true},		// AFType,					44c
			{NoTelemetry, true}, // TelemetryType,			45c
			{10,true},		    // MaxDescentRatedMpS, 		46
			{30,true},			// DescentDelayS			47
			{MPU_RA_DLPF_BW_98, true},	// GyroLPF,			48
			{0,true},			// NavCrossTrackKp,			49
			{5,true},			// RxGearCh,				50c

			{6,true},			// RxAux1Ch,				51
			{0,true},			// ServoSense				52c
			{2,true},			// AccConfSD,				53c
			{22,true},			// BatteryCapacity			54c
			{7,true},			// RxAux2Ch,				55c
			{8,true},			// RxAux3Ch,				56
			{20,true},			// NavWindKi				57

			{90,true},			// GPSCFKpTrim				58
			{50,0},				// Balance					59
			{9,true},			// RxAux4Ch					60
			{50,true},			// DriveFilt				61
#if defined(USE_UBLOX_BIN)
			{UBXBinGPS,true},	// GPSProtocol,				62
#else
			{NMEAGPS, true},	// GPSProtocol,				62
#endif
			{20,true},			// StickScalePitchRoll,		63
			{10,true}			// StickScaleYaw,			64
		};
*/
		{ // MULTICOPTERS
			{17,0},				// RollKpRate, 			01 UAVP 21
			{8,0},	 			// RollKiRate,			02 UAVP 10
			{15,0},				// RollKpAngle,			03 // 25
			{ModeUnknown,true},	// TxMode,				04
			{30,0},	 			// RollIntLimit,		05 UAVP 80

			{17,0},	 			// PitchKpRate,			06 UAVP 21
			{8,0},	 			// PitchKiRate,			07 UAVP 10
			{15,0},	 			// PitchKpAngle			08 // 25
			{0, 0},	 			// RFUsed,				09 UAVP 1
			{30,0},	 			// PitchIntLimit,		10 UAVP 80

			{25,0},	 			// YawKpRate, 			11 // 40
			{50,0},	 			// RollKdRate,			12
			{MadgwickIMU, true},// IMU,					13
			{20,0},	 			// YawRateIntLimit,		14 // 12
			{RCUnknown, true},	// RCType,				15 was CompoundPPM

			{UseRTHDescendMask | UseMWStickProgMask, true},
								// ConfigBits,			16c
			{1,true},			// RxThrottleCh,		17
			{68,true}, 			// LowVoltThres,		18c
			{10,true}, 			// CamRollKp,			19c
			{45,true}, 			// PercentCruiseThr,	20c

			{0,true}, 			// AltCFKpTrim,			21c // 50
			{0,true}, 			// RollPitchMix,		22c
			{10,true}, 			// PercentIdleThr,		23c
			{15,0}, 			// RollKiAngle,			24
			{15,0}, 			// PitchKiAngle,		25
			{10,true}, 			// CamPitchKp,			26c
			{8,0}, 				// YawKpAngle(Compass),	27
			{45,0},				// PitchKdRate,			28 UAVP 10
			{10,true}, 			// NavMaxAngle,			29
			{20,true}, 			// ROCKpRate,			30

			{0,0}, 				// YawRateKi,	    	31
			{50,true},			// MadgwickKpMag,	    32
			{15,true}, 		    // NavRTHAlt,			33
			{13,true},			// NavMagVar,			34c 13 Melbourne
			{UAVXArm32IMU, true},// SensorHint,     		35c
			{ESCPPM,true}, 		// ESCType,				36c
			{7,true}, 			// RxChannels,			37c
			{2,true},			// RxRollCh,			38
			{20,true},			// MadgwickKpAcc,		39c
			{1,true},			// CamRollTrim,			40c

			{3,true},			// NavMaxVelMPS,		41
			{3,true},			// RxPitchCh,			42
			{4,true},			// RxYawCh,				43
			{AFUnknown,true},	// AFType,				44c
			{UAVXTelemetry, true}, // TelemetryType,	45c
			{10,true},		    // MaxDescentRateDmpS, 	46
			{30,true},			// DescentDelayS,		47
			{MPU_RA_DLPF_BW_98, true},	// GyroLPF,		48
			{4,true},			// NavCrossTrackKp,		49
			{5,true},			// RxGearCh,			50c

			{6,true},			// RxAux1Ch,			51
			{0,true},			// ServoSense			52c
			{4,true},			// AccConfSD,			53c
			{22,true},			// BatteryCapacity,		54c
			{7,true},			// RxAux2Ch,			55c
			{8,true},			// RxAux3Ch,			56
			{20,true}, 			// NavWindKi,			57

			{0,true},			// GPSCFKp,				58 // 90
			{50,true},			// Balance,				59
			{9,true},			// RxAux4Ch,			60
			{50,true},			// DriveFilt,			61

#if defined(USE_UBLOX_BIN)
			{UBXBinGPS,true},	// GPSProtocol,				62
#else
			{NMEAGPS, true},	// GPSProtocol,				62
#endif
			{10,true},			// StickScalePitchRoll,	63
			{10,true}			// StickScaleYaw,		64
	};
	#pragma idata


