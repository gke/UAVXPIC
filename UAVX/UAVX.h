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

#ifndef BATCHMODE
	//#define DO NOT USE USE_PACKET_COMMS
	#define USE_PID_DT_CLIP
	//#define GKE_TUNE
	#define USE_UBLOX_BIN
	//#define TESTING
	//#define FULL_TEST			// extended compass test etc.					
	#define SIMULATE
	#define QUADROCOPTER
	//#define QUADROCOPTERX
	//#define TRICOPTER
	//#define Y6COPTER
	//#define HEXACOPTER
	//#define HEXACOPTERX
	//#define VTCOPTER
	//#define VTOL
	//#define HAVE_CUTOFF_SW	// Ground PortC Bit 0 (Pin 11) for landing cutoff otherwise 4K7 pullup.
	//#define I2C100KHZ
#endif // !BATCHMODE

//________________________________________________________________________________________________


//#if defined(SIMULATE) | defined(TESTING)
	#define USE_LIMIT_MACRO		// squeek some more space
//#endif

//#define USE_BARO_RF_CALIB
#define USE_NAV_STICK_ENABLE

#if defined(FULL_TEST)
	#define FULL_BARO_TEST
	#define FULL_ACC_TEST
	#define FULL_COMPASS_TEST
#endif // FULL_TEST

#define DEG_TO_ANGLE_UNITS		18L	// approximate!
#define MAX_BANK_ANGLE_DEG		45L

// Airframe

#if defined(UAVXBOARD)
	#define UAVX_HW
#endif

#if defined(TRICOPTER) | defined(QUADROCOPTER) | defined(QUADROCOPTERX) | defined(Y6COPTER) | defined(VTCOPTER) | defined(HEXACOPTER) | defined(HEXACOPTERX)
	#define MULTICOPTER
#endif

#if defined(QUADROCOPTER)
	#define AF_TYPE QuadAF
#elif defined(QUADROCOPTERX)
	#define AF_TYPE QuadXAF
#elif defined(TRICOPTER)
	#define AF_TYPE TriAF
#elif defined(VTCOPTER)
	#define AF_TYPE VTailAF
#elif defined(Y6COPTER)
	#define AF_TYPE TriCoaxAF
#elif defined(HEXACOPTER)
	#define AF_TYPE HexAF
#elif defined(VTOL)
	#define AF_TYPE VTOLAF
#else
#error aircraft configuration not defined or not supported !
#endif


// Filters

// Timeouts and Update Intervals

#define ANGLE_LIMIT_MRAD			3100	// make large otherwise angle estimate latches up!!
	
#define PID_CYCLE_MS				4		// 2 I2C ESCs
#define PID_CYCLE_US				(PID_CYCLE_MS*1000)
#define	SERVO_CYCLE_MS 				20
	
// DISABLED AS UNSAFE ON BENCH 
#define ENABLE_STICK_CHANGE_FAILSAFE

#define FAILSAFE_TIMEOUT_MS			1000L 	// mS. hold last "good" settings and then restore flight or abort
#define ABORT_TIMEOUT_GPS_MS		5000L	// mS. go to descend on position hold if GPS valid.
#define ABORT_TIMEOUT_NO_GPS_MS		0L		// mS. go to descend on position hold if GPS valid.  
#define ABORT_UPDATE_MS				1000L	// mS. retry period for RC Signal and restore Pilot in Control
#define ARMED_TIMEOUT_MS			150000L	// mS. automatic disarming if armed for this long and landed

#define	ALT_DESCENT_UPDATE_MS		1000L	// mS time between throttle reduction clicks in failsafe descent without baro	
#define ALT_DESCENT_STEP			5		// Stick units

#define RC_STICK_MOVEMENT			4L		// minimum to be recognised as a stick input change without triggering failsafe
#define RC_STICK_ANGLE_SCALE		12L 	// 96 zzz48L		// convert stick to desired "angle"

#define THROTTLE_LOW_DELAY_MS		1000L	// mS. that motor runs at idle after the throttle is closed
#define THROTTLE_UPDATE_MS			3000L	// mS. constant throttle time for altitude hold

#define NAV_ACTIVE_DELAY_MS			10000L	// mS. after throttle exceeds idle that Nav becomes active
#define NAV_RTH_LAND_TIMEOUT_MS		15000L	// mS. Shutdown throttle if descent lasts too long

#define UAVX_TEL_INTERVAL_MS		200L	// mS. emit an interleaved telemetry packet
#define UAVX_MIN_TEL_INTERVAL_MS	500L	// mS. emit minimum data packet for example to FrSky
#define ARDU_TEL_INTERVAL_MS		200L	// mS. alternating 1:5
#define UAVX_CONTROL_TEL_INTERVAL_MS 100L	// mS. flight control only
#define CUSTOM_TEL_INTERVAL_MS		100L	// mS.

#define GPS_TIMEOUT_MS				2000L	// mS.

// Altitude Hold

#define ALT_HOLD_MAX_ROC_CMPS		50L		// Must be changing altitude at less than this for alt. hold to be detected

// Altitude Hold

#define LAND_CM						300L	// centimetres deemed to have landed when below this height

#define ALT_MAX_ROC_CMPS			500		// cm/S maximum climb rate
#define ALT_MAX_THR_COMP			FromPercent(20,RC_MAXIMUM)  // simulation says 7	

#define ALT_RF_ENABLE_CM			300L //500L	// altitude below which the rangefinder is selected as the altitude source
#define ALT_RF_DISABLE_CM			400L //600L	// altitude above which the rangefinder is deselected as the altitude source

#define ALT_SLEW_LIMIT_CMPS			500L	// cm/S  
#define ALT_SANITY_CHANGE_CMPS		1000L	// cm/S 

// Navigation

#define	M_TO_GPS					54L		// approximate for constant folding

#define NAV_RTH_LOCKOUT				1000L	// ~100 ~ angle units per degree

#define NAV_MAX_ROLL_PITCH 			64L		// Rx stick units
#define NAV_MAX_FAKE_COMPASS_SLEW	25L		// Rx stick units
#define NAV_CONTROL_HEADROOM		10L		// at least this much stick control headroom above Nav control	
#define NAV_DIFF_LIMIT				24L		// Approx double NAV_INT_LIMIT
#define NAV_INT_WINDUP_LIMIT		64L		// ???

#define NAV_ENFORCE_ALTITUDE_CEILING		// limit all autonomous altitudes
#define NAV_CEILING					120L	// 400 feet
#define NAV_MAX_RADIUS				99L		// Metres
#define NAV_CLOSING_RADIUS_SHIFT	9		// Using shift to avoid division 
#define NAV_CLOSING_RADIUS			((int32)1<<NAV_CLOSING_RADIUS_SHIFT) // GPS units ~54 per metre

#define NAV_PROXIMITY_RADIUS	5L		// Metres if there are no WPs	
#define NAV_PROXIMITY_ALTITUDE	3L		// Metres

// reads $GPGGA sentence - all others discarded

#define GPS_OUTLIER_SLEW_LIMIT		(1*M_TO_GPS)	// maximum slew towards a detected outlier	
#define GPS_OUTLIER_LIMIT			(10*M_TO_GPS)	// maximum lat/lon change in one GPS update

#define	GPS_UPDATE_HZ				5		// Hz - can obtain from GPS updates
#define GPS_UPDATE_MS				(1000/GPS_UPDATE_HZ)
#if defined(SIMULATE)

#define	SIM_MULTI_YAW_RATE_DPS		90		// Deg/S	

#define	GPS_MIN_SATELLITES			4		// preferably > 5 for 3D fix
#define GPS_MIN_FIX					1		// must be 1 or 2 
#define GPS_ORIGIN_SENTENCES 		5L		// Number of sentences needed to obtain reasonable Origin
#define GPS_MIN_HDOP				25L		// HDilute * 10

#else

#define	GPS_MIN_SATELLITES			6		// preferably > 5 for 3D fix
#define GPS_MIN_FIX					1		// must be 1 or 2 
#define GPS_ORIGIN_SENTENCES 		30L		// Number of sentences needed to obtain reasonable Origin
#define GPS_MIN_HDOP				15L		// HDilute * 10

#endif // SIMULATE

#define	NAV_SENS_THRESHOLD 			30L		// Navigation disabled if Ch7 is less than this
#define	NAV_SENS_ALTHOLD_THRESHOLD 	20L		// Altitude hold disabled if Ch7 is less than this
#define NAV_SENS_6CH				80L		// Low GPS gain for 6ch Rx

#define	NAV_SENS_ACC_THRESHOLD 		20L		// Accelerometers disabled if Ch7 is less than this

#define NAV_MIN_VEL_DMPS 			1
#define NAV_MAX_TRIM				20L		// max trim offset for altitude hold

#define ATTITUDE_HOLD_LIMIT 		8L		// dead zone for roll/pitch stick for position hold
#define ATTITUDE_HOLD_RESET_INTERVAL 25L	// number of impulse cycles before GPS position is re-acquired

//#define NAV_PPM_FAILSAFE_RTH				// PPM signal failure causes RTH with Signal sampled periodically

// Throttle

#define	THROTTLE_MAX_CURRENT		40L		// Amps total current at full throttle for estimated mAH
#define	CURRENT_SENSOR_MAX			50L		// Amps range of current sensor - used for estimated consumption - no actual sensor yet.
#define	THROTTLE_CURRENT_SCALE		((THROTTLE_MAX_CURRENT * 1024L)/(200L * CURRENT_SENSOR_MAX ))

#define THROTTLE_MIDDLE				10  	// throttle stick dead zone for baro 
#define THROTTLE_MIN_ALT_HOLD		75		// min throttle stick for altitude lock

// RC

#define RC_INIT_FRAMES				32		// number of initial RC frames to allow filters to settle

//________________________________________________________________________________________

#include "UAVXRevision.h"

#include "MPU6050.h"

// 18Fxxx C18 includes

#include <p18cxxx.h> 
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <capture.h>
#include <adc.h>

// Useful Constants
#define NUL 	(uint8)0
#define SOH 	(uint8)1
#define EOT 	(uint8)4
#define ACK		(uint8)6
#define HT 		(uint8)9
#define LF 		(uint8)10
#define CR 		(uint8)13
#define NAK 	(uint8)21
#define ESC 	(uint8)27
#define true 	(uint8)1
#define false 	(uint8)0

#define MILLIPI 			3142 
#define CENTIPI 			314 
#define HALFMILLIPI 		1571 
#define QUARTERMILLIPI		785
#define SIXTHMILLIPI		524
#define TWOMILLIPI 			6284

#define MILLIRAD 			18 
#define CENTIRAD 			2

#define DegreesToRadians(n)	(n*MILLIRAD)

#define MAXINT32 			0x7fffffff
#define	MAXINT16 			0x7fff

// Additional Types
typedef unsigned char 		uint8 ;
typedef signed char 		int8;
typedef unsigned int 		uint16;
typedef int 				int16;
typedef short long 			int24;
//typedef long 			int24;
typedef unsigned short long uint24;
typedef long 				int32;
typedef unsigned long 		uint32;
typedef uint8 				boolean;
typedef float 				real32;

typedef union {
	char c[8];
	int16 i16[4];
} charint16x4u;

typedef union {
	int16 i16;
	uint16 u16;
	struct {
		uint8 b0;
		uint8 b1;
	};
	struct {
		int8 pad;
		int8 i1;
	};
} i16u;

typedef union {
	int24 i24;
	uint24 u24;
	struct {
		uint8 b0;
		uint8 b1;
		uint8 b2;
	};
	struct {
		uint8 pad;
		int16 i2_1;
	};
} i24u;

typedef union {
	int32 i32;
	uint32 u32;
	struct {
		uint8 b0;
		uint8 b1;
		uint8 b2;
		uint8 b3;
	};
	struct {
		uint16 w0;
		uint16 w1;
	};
	struct {
		int16 iw0;
		int16 iw1;
	};
	
	struct {
		uint8 pad2;
		int24 i3_1;
	};
} i32u;

typedef struct { // Tx
	uint8 Head, Tail;
	uint8 B[128];
	} uint8x128Q;

typedef struct { // PPM
	uint8 Head;
	int16 B[4][8];
	} int16x8x4Q;	

typedef struct { 
	int32 S;
	uint8 Head, Tail;
	int32 B[8];
	} int32x8Q;

typedef struct { 
	int32 S;
	uint8 Head, Tail;
	int16 B[4];
	boolean Prime;
	} int16x4Q;

typedef struct {
	int24 c[3], h[8]; // 8 for rate of change use
	int24 S;
	boolean Primed;
	uint8 Head, Tail;
} HistStruct;	

#define NAVQ_MASK 3
#define NAVQ_SHIFT 2
typedef struct { // GPS
	uint8 Head, Tail;
	int32 LatDiffSum, LonDiffSum;
	int32 LatDiff[NAVQ_MASK], LonDiff[NAVQ_MASK];
	} int32Q;

typedef struct {
	// from Parameter Sets
	int16 RateIntLimit, IntLimit;
	int16 AngleKp, AngleKi, RateKp, RateKi, RateKd;
	// run time
	int16 Desired;
	int16 GyroADC, GyroBias;
	real32 AngleHR;	
	int16 Angle, AngleE;
	int32 Rate, Ratep, RateD, RateDp;
	int24 AngleEInt, RateEInt;
	int16 Acc, AccADC, AccBias;
	int16 Control;
	int16 NavCorr;
	int32 NavPosE, NavVel;
	int16 Out, Outp;	
} AxisStruct;

typedef struct {
	int16 G, Max, Min, Scale;
} MagStruct;

// Macros
#define Set(S,b) 			((uint8)(S|=(1<<b)))
#define Clear(S,b) 			((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 			((uint8)((S>>b)&1))
#define IsClear(S,b) 		((uint8)(!(S>>b)&1))
#define Invert(S,b) 		((uint8)(S^=(1<<b)))

#define Abs(i)				(((i)<0) ? -(i) : (i))
#define Sign(i)				(((i)<0) ? -1 : 1)
#define Sqr(r)				( r * r )

#define Max(i,j) 			((i<j) ? j : i)
#define Min(i,j) 			((i<j) ? i : j )
#define Decay1(i) 			(((i) < 0) ? (i+1) : (((i) > 0) ? (i-1) : 0))

#define Limit(i,l,u) 	(((i) < l) ? l : (((i) > u) ? u : (i)))
#define Limit1(i,l) 	(((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))

#define StatsMinMax(v, l, u) 	if(v>Stats[u])Stats[u]=v;else if (v<Stats[l])Stats[l]=v
#define StatsMax(v, u)		if(v>Stats[u])Stats[u]=v

// To speed up NMEA sentence processing 
// must have a positive argument

#define ConvertDDegToMPi(d) (((int32)d * 3574L)>>11)
#define ConvertMPiToDDeg(d) (((int32)d * 2048L)/3574L)

#define ToPercent(n, m) (((n)*100L+(m)/2)/(m))
#define FromPercent(n, m) (((n)*(m)+50)/100L)

// Simple filters using weighted averaging
#define VerySoftFilter(O,N) 	(SRS16((O)+(N)*3, 2))
#define SoftFilter(O,N) 		(SRS16((O)+(N), 1))
#define MediumFilter(O,N) 		(SRS16((O)*3+(N), 2))
#define SoftFilter32(O,N) 		(SRS32((O)+(N), 1))
#define MediumFilter32(O,N) 	(SRS32((int32)(O)*3+(N), 2))
#define HardFilter(O,N) 		(SRS16((O)*7+(N), 3))
#define HardFilter32(O,N) 		(SRS32((int32)(O)*7+(N), 3))

// Unsigned
#define VerySoftFilterU(O,N)	(((O)+(N)*3+2)>>2)
#define SoftFilterU(O,N) 		(((O)+(N)+1)>>1)
#define MediumFilterU(O,N) 		(((O)*3+(N)+2)>>2)
#define HardFilterU(O,N) 		(((O)*7+(N)+4)>>3)

#define NoFilter(O,N)			(N)

#define DisableInterrupts 	(INTCONbits.GIEH=0)
#define EnableInterrupts 	(INTCONbits.GIEH=1)
#define InterruptsEnabled 	(INTCONbits.GIEH)

// Clock

//#define mSTimer(t,p) mS[t]=mSClock()+p

// Parameters for UART port ClockHz/(16*(BaudRate+1))
#define _B9600			65
#define _B38400			65

// This is messy - trial and error to determine worst case interrupt latency!
#define INT_LATENCY		140 // (uint16)(65536 - 35) // x 1.6uS
#define FastWriteTimer0(t) Timer0.u16=t;TMR0H=Timer0.b1;TMR0L=Timer0.b0	
#define GetTimer0		{Timer0.b0=TMR0L;Timer0.b1=TMR0H;}	

// Bit definitions
#define ArmingSwitch		(PORTAbits.RA4)

#if defined(HAVE_CUTOFF_SW)
	#define InTheAir		(PORTCbits.RC0) // normally open micro switch to ground
#else
	#define InTheAir		true	 
#endif // HAVE_CUTOFF_SW

// EEPROM

#define MAX_EEPROM			1024
#define PARAMS_ADDR_EE		0		// code assumes zero!
#define MAX_PARAMETERS		64		// parameters in EEPROM start at zero

#define STATS_ADDR_EE	 	( PARAMS_ADDR_EE + (MAX_PARAMETERS *2) )
#define MAX_STATS			32 //64

#define SETTINGS_ADDR_EE	(STATS_ADDR_EE + (MAX_STATS * 2) )
#define MAG_BIAS_ADDR_EE	SETTINGS_ADDR_EE
#define ACC_BIAS_ADDR_EE	(MAG_BIAS_ADDR_EE + 16)

#define MAX_SETTINGS		64

// uses second Page of EEPROM
#define NAV_ADDR_EE			256L
// 0 - 8 not used

#define NAV_NO_WP			(NAV_ADDR_EE + 9)
#define NAV_PROX_ALT		(NAV_ADDR_EE + 10) 	
#define NAV_PROX_RADIUS		(NAV_ADDR_EE + 11)
#define NAV_ORIGIN_ALT		(NAV_ADDR_EE + 12)
#define NAV_ORIGIN_LAT		(NAV_ADDR_EE + 14)
#define NAV_ORIGIN_LON 		(NAV_ADDR_EE + 18)
#define NAV_RTH_ALT_HOLD 	(NAV_ADDR_EE + 22)	// P[NavRTHAlt]
#define NAV_WP_START		(NAV_ADDR_EE + 24)

#define WAYPOINT_REC_SIZE 	(4 + 4 + 2 + 1)		// Lat + Lon + Alt + Loiter
#define NAV_MAX_WAYPOINTS	((256 - 24 - 1)/WAYPOINT_REC_SIZE)
	 
//______________________________________________________________________________________________

extern int32 aaa,bbb,ccc,ddd, eee, fff, ggg, hhh;

// main.c

#define FLAG_BYTES 10
#define TELEMETRY_FLAG_BYTES 6
typedef union {
	uint8 AllFlags[FLAG_BYTES];
	struct { // Order of these flags subject to change
		uint8 // 0
				AltControlEnabled :1,
				AllowTurnToWP :1,
				VRSHazard :1,
				LostModel :1,
				NearLevel :1,
				LowBatt :1,
				GPSValid :1,
				OriginValid :1,

				// 1
				BaroFailure :1,
				IMUFailure :1,
				MagnetometerFailure :1,
				GPSFailure :1,
				AttitudeHold :1,
				ThrottleMoving :1,
				HoldingAlt :1,
				Navigate :1,

				// 2
				ReturnHome :1,
				WayPointAchieved :1,
				WayPointCentred :1,
				OrbitingWP :1,
				UsingRTHAutoDescend :1,
				BaroActive :1,
				RangefinderActive :1,
				UsingRangefinderAlt :1,

				// 3
				UsingPOI :1,
				Bypass :1,
				UsingRateControl :1,
				Emulation :1,
				AcquireNewPosition :1,
				DrivesArmed :1,
				NavigationActive :1,
				SticksUnchangedFailsafe :1,

				// 4
				Signal :1,
				RCFrameOK :1,
				ParametersValid :1,
				RCNewValues :1,
				NewCommands :1,
				IMUActive :1,
				MagnetometerActive :1,
				NewMagValues :1,

				// 5
				NewGPSPosition :1,
				InvertMagnetometer:1,
				MagnetometerCalibrated :1,
				UsingUplink :1,
				NewAltitudeValue :1,
				IMUCalibrated :1,
				CrossTrackActive :1,
				FailsafesEnabled :1,

				// 6
				NewBaroValue :1,
				BeeperInUse :1,
				ParametersChanged:1,
				FirstArmed :1,
				ValidGPSVel :1,
				RCFrameReceived :1, // zzz
				ValidGPSPos :1,
				NewMagnetometerValue :1,

				// 7
				UpdateHeading :1, HaveGPS :1, UsingCompoundPPM :1,
				NewNavUpdate:1, HaveExtMem :1,
				UsingRapidDescent :1, // was MPU6050
				ConfigError :1,
				SetupError :1,

				// 8
				TxToBuffer :1, UsingAnalogGyros :1, i2cFatal :1,
				GPSPacketReceived :1, ThrottleOpen :1, UsingMWStickProg :1,
				UsingInertialFilters :1, NavigationEnabled :1,

				// 9
				UsingWPNavigation:1, ReceivingGPS:1, spiFatal:1, BatteryFailsafe:1,
				UsingMagic :1;
	};
} Flags;

enum FlightStates {Starting, Landing, Landed, Shutdown, InFlight};
extern Flags F;
extern int8 near State, NavState, FailState;
extern boolean near SpareSlotTime;

//______________________________________________________________________________________________

// adc.c

#define ADC_TOP_CHANNEL		(uint8)4

#define ADCPORTCONFIG 		0b00001010 // AAAAA
#define ADCBattVoltsChan 	0 
#define ADCRollChan 		1
#define ADCPitchChan 		2
#define ADCAltChan 			3 	// Altitude
#define ADCYawChan			4
#define TopADCChannel		4

extern int16 ADC(uint8);
extern void InitADC(void);

//______________________________________________________________________________________________

// autonomous.c

#define SetDesiredAltitude(n) DesiredAltitude = n
#define DoAutoLanding() if(DoLanding())DoShutdown()

enum Coords {
	NorthC, EastC, AltC
};

typedef struct {
	int24 Desired;
	int24 Hold;
	int24 Pos, PosE, PosP;
	int16 Corr;
} CoordStruct;

typedef struct {
	CoordStruct C[3];
	int16 Sensitivity;
	int16 ProximityAltitude, ProximityRadius;
	int16 MaxVelocitydMpS;
	int24 CrossTrackE;
	int8 CrossTrackKp;
	int16 Bearing;
	int24 Distance;
	int8 PosKp;
	int8 VelKp;
	int8 VelKi;
	int16 VelIntLimit;
	int32 VelIntE;
	int16 DesiredHeading;
	int16 MaxAngle;
	int8 YawKp;
} NavStruct;

extern NavStruct Nav;

extern void DoShutdown(void);
extern void DecayNavCorr(void);
extern void FailsafeHoldPosition(void);
extern void DoCompScaling(void);
extern void Navigate(int24 DesiredNorth, int24 DesiredEast);
extern void AcquireHoldPosition(void);
extern void DoNavigation(void); 
extern void DoFailsafe(void);
extern void UAVXNavCommand(void);
extern void InitNavigation(void);

typedef struct { int32 E, N; int16 A; uint8 L; } WayPoint;

enum NavStates {
	HoldingStation,
	ReturningHome,
	AtHome,
	Descending,
	Touchdown,
	Transiting,
	Loitering,
	OrbitingPOI,
	Perching,
	Takeoff,
	PIC,
	AcquiringAltitude,
	NavStateUndefined
};

extern void UpdateRTHSwState(void);

enum NavSwStates { 
	NavSwLow, NavSwMiddle, NavSwHigh, NavSwUnknown
};

extern uint8 NavSwState, NavSwStateP;

enum NavComs { navVia, navOrbit, navPerch, navPOI, navUnknown };

extern const char * NavComNames [];

enum FailStates {
	NoFailsafes, Monitoring, BatteryLow, LostSignal
};

enum LandingStates {
	CommenceDescent, Descent, DescentStopped
};

extern int16 WPHeading, OriginalWPHeading;
extern uint24 NavRTHTimeoutmS;
extern int16 DescentComp;
extern int16 RollPitchMixFrac;
extern int16 EffNavSensitivity;

//______________________________________________________________________________________________

// baro.c

#define AltFilter32(o,n)	(SRS32(o*7+n,3))	// ~1Hz

#define ALT_UPDATE_MS			25
#define ALT_UPDATE_HZ			(1000/ALT_UPDATE_MS)

#define ALT_SANITY_CHECK_CM	((ALT_SANITY_CHANGE_CMPS*ALT_UPDATE_MS)/1000L)
#define ALT_SLEW_LIMIT_CM		((ALT_SLEW_LIMIT_CMPS*ALT_UPDATE_MS)/1000L)

#define BARO_INIT_RETRIES		100	// max number of initialisation retries

enum BaroTypes { BaroBMP085, BaroMS5611, BaroUnknown };

#define MS5611_ID	0xee

// Bosch Baro

#define BOSCH_ID	0xee

extern void ReadBoschBaro(void);
extern boolean IsBoschBaroActive(void);

extern int24 AltitudeCF(int24);
extern void GetBaroAltitude(void);
extern void InitBarometer(void);

extern void ShowBaroType(void);
extern void BaroTest(void);
extern void ReadI2CBaro(void);
extern int24 CompensatedPressure(uint24, uint24);
extern int24 BaroToCm(void);
extern void InitI2CBarometer(void);
extern void StartI2CBaroADC(boolean);
extern void GetI2CBaroAltitude(void);

extern const rom uint8 BaroError[];
extern int32 OriginBaroTemperature, OriginBaroPressure;
extern uint32 BaroPressure, BaroTemperature;
extern int24 OriginAltitude, BaroAltitude;
extern int16 BaroROC;
extern boolean AcquiringPressure;
extern int24 BaroAltitude;
extern i32u	BaroVal;
extern uint8 BaroType;
extern boolean PrimeROC;

extern uint16 MS5611Constants[];
extern uint16 BMP085Constants[];

//______________________________________________________________________________________________

// compass.c

#define COMPASS_MIDDLE		FromPercent(20, RC_NEUTRAL) // 8			// yaw stick neutral dead zone
#define COMPASS_TIME_MS		50			// 20Hz
#define COMPASS_UPDATE_HZ	(1000/COMPASS_TIME_MS)
#define COMPASS_OFFSET_QTR	3

#define COMPASS_MAX_SLEW	SIXTHMILLIPI
#define YAW_COMP_LIMIT		30			// maximum yaw compensation of compass heading 

#define MAG_INIT_RETRIES	10

enum CompassTypes { HMC6352Compass, HMC5883LMagnetometer, CompassUnknown };

extern void ShowCompassType(void);
extern int16 GetCompass(void);
extern void GetHeading(void);
extern int16 MinimumTurn(int16);
extern void GetCompassParameters(void);
extern void DoCompassTest(void);
extern void CalibrateCompass(void);
extern void InitHeading(void);
extern void InitCompass(void);
extern void DoCompassTest(void);
extern void CalibrateCompass(void);

// HMC5883L Bosch Magnetometer
     
#define HMC5883L_ID 	0x3C

extern int16 GetHMC5883LMagnetometer(void);
extern void DoTestHMC5883LMagnetometer(void);
extern void CalibrateHMC5883LMagnetometer(void);
extern void InitHMC5883LMagnetometer(void);
extern boolean HMC5883LMagnetometerActive(void);
extern void ReadMagCalEE(void);
extern void WriteMagCalEE(void);

// HMC6352 Bosch Compass

#define HMC6352_ID		0x42

extern int16 GetHMC6352Compass(void);
extern void DoTestHMC6352Compass(void);
extern void CalibrateHMC6352Compass(void);
extern void InitHMC6352Compass(void);
extern boolean HMC6352CompassActive(void);
extern void WriteHMC6352Command(uint8);

extern MagStruct Mag[];
extern i24u Compass;
extern int16 MagHeading, Heading, DesiredHeading, CompassOffset;
extern uint8 CompassType;
extern uint8 MagRetries;

//______________________________________________________________________________________________

// control.c

#define LAND_DM 50
#define LAND_TIMEOUT_MS 3000
#define ALT_MAX_ROC_DMPS 10 // maximum climb rate
#define ALT_MAX_THR_COMP_STICK FromPercent(20, RC_MAXIMUM)
#define ALT_RF_ENABLE_DM 30 // was 5M	// altitude below which the rangefinder is selected as the altitude source
#define ALT_RF_DISABLE_DM 40 //was 6M	// altitude above which the rangefinder is deselected as the altitude source
#define YAW_RATE_MAX_RAD_S DegreesToRadians(180) // zzz kill nav or Madgwick fusion if yaw too rapid

#define ALT_MIN_DESCENT_DMPS 	(4)
#define ALT_MAX_DESCENT_DMPS	(20)

enum Attitudes { Roll, Pitch, Yaw };
enum Sensors {X, Y, Z};
enum MagSensors {MX, MZ, MY};
enum Directions { LR, FB, DU }; // Roll, Pitch & Yaw

extern void DoAltitudeHold(void);
extern void AcquireAltitude(void);
extern void DoAttitudeAngle(AxisStruct *);
extern void CompensateYawGyro(void);
extern void DoYawControl(void);
extern void DoOrientationTransform(void);
extern void ZeroPIDIntegrals(void);
extern void DoControl(void);
extern void InitControl(void);

extern uint16 PIDCycleuS;
extern uint32 CycleHist[];

extern AxisStruct A[3];
	
extern int16 CameraAngle[3];
#if defined(GKE_TUNE)
	extern int16 TuneTrim;
#endif				

extern int16 HeadingE;

extern int16 ControlRoll, ControlPitch, CurrMaxRollPitch;

extern int16 AttitudeHoldResetCount;
extern int24 DesiredAltitude, Altitude; 
extern int16 AltFiltComp, AltComp, HRAltComp, ROC, MinROCCmpS;
extern int24 RTHAltitude;
extern int16 AltMinThrCompStick;

extern int16 OrientationMRad;

//______________________________________________________________________________________________

// eeprom.c

extern int8 ReadEE(uint16);
extern int16 Read16EE(uint16);
extern int32 Read32EE(uint16);
extern void WriteEE(uint16, int8);
extern void Write16EE(uint16, int16);
extern void Write32EE(uint16, int32);

//______________________________________________________________________________________________

// gps.c

enum WaitStates {
	WaitSentinel,
	WaitSentinel2,
	WaitID,
	WaitClass,
	WaitLength,
	WaitLength2,
	WaitBody,
	WaitCheckSum,
	WaitCheckSum2
};
enum GPSProtcols {
	NMEAGPS, UBXNMEAGPS, UBXBinGPS, MTKNMEAGPS, MTKBinGPS, UnknownGPS
};

#if defined(USE_UBLOX_BIN)

#define UBX_PREAMBLE1	    0xB5	// u
#define UBX_PREAMBLE2	    0x62	// b
#define UBX_NAV_CLASS	    0x01
#define UBX_RXM_CLASS	    0x02
#define UBX_INF_CLASS		0x04
#define UBX_ACK_CLASS		0x05
#define UBX_CFG_CLASS		0x06
#define UBX_MON_CLASS		0x0a
#define UBX_AID_CLASS	    0x0b
#define UBX_TIM_CLASS	    0x0d
#define UBX_ESF_CLASS		0x10

#define UBX_AID_REQ	    	0x00
#define UBX_TIM_TP	    	0x01

#define UBX_NAV_POSLLH	    0x02
#define UBX_NAV_STATUS	    0x03
#define UBX_NAV_DOP	    	0x04
#define UBX_NAV_SOL	    	0x06
#define UBX_NAV_VELNED	    0x12
#define UBX_NAV_TIMEUTC		0x21
#define UBX_NAV_SVINFO	    0x30

#define UBX_CFG_MSG			0x01
#define UBX_CFG_DAT			0x09
#define UBX_CFG_TP			0x07
#define UBX_CFG_RATE		0X08
#define UBX_CFG_CFG			0X09
#define UBX_CFG_EKF			0x12
#define UBX_CFG_NAV5		0x24

#define UBX_RXM_RAW	    	0x10

#define UBX_SFRB_RAW	    0x11

#define UBX_MAX_PAYLOAD   	64 //zzz 384

#define GPS_LATENCY	    75000	// us (comment out to use Ubx timepulse)

typedef struct { // 28
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	int32 lon; // Longitude (deg * 1e-7)
	int32 lat; // Latitude (deg * 1e-7)
	int32 height; // Height above Ellipsoid (mm)
	int32 hMSL; // Height above mean sea level (mm)
	uint32 hAcc; // Horizontal Accuracy Estimate (mm)
	uint32 vAcc; // Vertical Accuracy Estimate (mm)
} UbxStructPOSLLH;

typedef struct { // 18
	uint32 iTOW; // ms GPS Millisecond Time of Week
	uint16 gDOP; // Geometric DOP
	uint16 pDOP; // Position DOP
	uint16 tDOP; // Time DOP
	uint16 vDOP; // Vertical DOP
	uint16 hDOP; // Horizontal DOP
	uint16 nDOP; // Northing DOP
	uint16 eDOP; // Easting DOP
} UbxStructDOP;

typedef struct { // 52
	uint32 time;
	int32 time_nsec;
	int16 week;
	uint8 fixtype;
	uint8 fix_status;
	int32 ecef_x; // cm
	int32 ecef_y; // cm
	int32 ecef_z; // cm
	uint32 position_accuracy_3d; // cm
	int32 ecef_x_velocity; // cm/S
	int32 ecef_y_velocity; // cm/S
	int32 ecef_z_velocity; // cm/S
	uint32 speed_accuracy; // cm/S
	uint16 position_DOP;
	uint8 res;
	uint8 satellites;
	uint32 res2;
} UbxStructSOL;

typedef struct { // 15
	uint32 iTOW;
	uint8 fixtype;
	uint8 fix_status;
	uint8 flags2;
	uint32 ttff;
	uint32 msss;
} UbxStructSTATUS;

typedef struct { // 36
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	int32 velN; // NED north velocity (cm/s)
	int32 velE; // NED east velocity (cm/s)
	int32 velD; // NED down velocity (cm/s)
	uint32 speed; // Speed (3-D) (cm/s)
	uint32 gSpeed; // Ground Speed (2-D) (cm/s)
	int32 heading; // Heading 2-D (deg * 1e-5)
	uint32 sAcc; // Speed Accuracy Estimate (cm/s)
	uint32 cAcc; // Course / Heading Accuracy Estimate (deg * 1e-5)
} UbxStructVALNED;

typedef struct { // 16
	uint32 towMS;
	uint32 towSubMS;
	int32 qErr;
	uint16 week;
	uint8 flags;
	uint8 res;
} UbxStructTP;

typedef struct { // 20
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	uint32 tAcc; // Time Accuracy Estimate
	int32 nano; // Nanosecond of second (UTC)
	uint16 year; // Year, range 1999..2099 (UTC)
	uint8 month; // Month, range 1..12 (UTC)
	uint8 day; // Day of Month, range 1..31 (UTC)
	uint8 hour; // Hour of Day, range 0..23 (UTC)
	uint8 min; // Minute of Hour, range 0..59 (UTC)
	uint8 sec; // Second of Minute, range 0..59 (UTC)
	uint8 valid; // Validity Flags
} UbxStructTIMEUTC;

typedef struct {
	uint8 state;
	uint8 count;
	uint8 class;
	uint8 id;
	uint8 length;
	union {
		UbxStructPOSLLH posllh;
		UbxStructVALNED valned;
		UbxStructSOL sol;
		UbxStructSTATUS status;
		UbxStructDOP dop;
		UbxStructTP tp;
		UbxStructTIMEUTC timeutc;
		char other[UBX_MAX_PAYLOAD];
	} payload;

	uint8 RxCK_A;
	uint8 RxCK_B;

	uint8 TxUbxCK_A;
	uint8 TxUbxCK_B;
} ubxstruct;

extern ubxstruct ubx;

#endif // USE_UBLOX_BIN

typedef struct {
	int32 	MissionTime, StartTime;
	int24 	Altitude, OriginAltitude;
	int32	LatitudeRaw, LongitudeRaw;
	int32 	OriginLatitudeRaw, OriginLongitudeRaw;
	int16 	LongitudeCorrection;
	int16	Heading;
	int16 	Vel;
	uint8 	NoOfSats;
	uint8 	Fix;
	int16 	HDOP;
} gpsstruct;

extern gpsstruct GPS;

extern void UpdateField(void);
extern int32 ConvertGPSToM(int32);
extern int32 ConvertGPSTodM(int32);
extern int24 ConvertInt(uint8, uint8);
extern int32 ConvertLatLon(uint8, uint8);
extern int32 ConvertUTime(uint8, uint8);
extern void ParseGPRMCSentence(void);
extern void ParseGPGGASentence(void);
extern void SetGPSOrigin(void);
extern void UpdateGPSSolution(void);
extern void UpdateGPS(void);
extern void InitGPS(void);

#define MAXTAGINDEX 		4
#define GPSRXBUFFLENGTH 	80
typedef struct {
		uint8 	s[GPSRXBUFFLENGTH];
		uint8 	length;
	} NMEAStruct;

#define MAX_NMEA_SENTENCES 2
#define NMEA_TAG_INDEX 4

enum GPSPackeType { GPGGAPacketTag, GPRMCPacketTag,  GPSUnknownPacketTag };
extern NMEAStruct NMEA;
extern const rom uint8 NMEATags[MAX_NMEA_SENTENCES][5];

extern uint8 GPSPacketTag, NMEAPacketTag;

extern uint8 nll, cc, lo, hi;
extern boolean EmptyField;
extern int16 ValidGPSSentences;


//______________________________________________________________________________________________

// irq.c

#define CONTROLS 			9

#define RxFilter			MediumFilterU

#define	RC_GOOD_BUCKET_MAX	20
#define RC_GOOD_RATIO		4

#define RC_MINIMUM			0
#define RC_MAXIMUM		240	// adjust for faster arithmetic in RCMap

#define RC_NEUTRAL			((RC_MAXIMUM-RC_MINIMUM+1)/2)
	
#define RC_THRES_STOP		FromPercent(6, RC_MAXIMUM)
#define RC_THRES_START		FromPercent(10, RC_MAXIMUM) 

#define THROTTLE_MAX_CRUISE			FromPercent(60, RC_MAXIMUM) 
#define THROTTLE_MIN_CRUISE			FromPercent(30, RC_MAXIMUM)

#define RC_FRAME_TIMEOUT_MS 	25
#define RC_SIGNAL_TIMEOUT_MS 	(5L*RC_FRAME_TIMEOUT_MS)
#define RC_THR_MAX 				RC_MAXIMUM

#define RC_NO_CHANGE_TIMEOUT_MS 20000L		// mS.

#define MAX_ROLL_PITCH		RC_NEUTRAL	// Rx stick units - rely on Tx Rate/Exp

extern uint32 Timer0Ticks(void);

extern void InitTimersAndInterrupts(void);
extern uint24 mSClock(void);
extern uint32 uSClock(void);
extern void mSTimer(uint8 t, int32 TimePeriod);
extern void InitUSART(void);

enum { StartTime, GeneralCountdown, UpdateTimeout, RCSignalTimeout, BeeperTimeout, ThrottleIdleTimeout, 
	FailsafeTimeout, AbortTimeout, NavStateTimeout, DescentUpdate, LastValidRx, LastGPS, LastNavUpdate, AccTimeout, 
	GPSTimeout, RxFailsafeTimeout, StickChangeUpdate, LEDChaserUpdate, ServoUpdate, LastBattery, BatteryUpdate, 
  	TelemetryUpdate, NavActiveTime, BeeperUpdate, ArmedTimeout,
	ThrottleUpdate, BaroUpdate, CompassUpdate};

extern volatile uint24 mS[];
extern volatile uint24 CycleUpdateuS;

extern near volatile uint16 uSTop;
extern near i16u 	PPM[CONTROLS];
extern near uint8 	PPM_Index, NoOfRCChannels;
extern near int24 	PrevEdge, CurrEdge;
extern near i16u 	Width, Timer0;
extern near int24 	PauseTime;
extern near uint8 	RxState;

extern near uint8 	ll, ss, tt;
extern near uint8 	RxCheckSum, TxCheckSum, GPSCheckSumChar, GPSTxCheckSum;
extern near uint16	RxQTail, RxQHead, TxQTail, TxQHead;

extern int8	SignalCount;
extern uint16 RCGlitches;

//______________________________________________________________________________________________

// inertial.c

enum AttitudeEstimators {
	MadgwickIMU, MadgwickAHRS, UKF, Wolferl, EstUnknown
};

extern void CompensateRollPitchGyros(void);
extern void CompensateYawGyro(void);
extern void DoAttitudeAngles(void);
extern void GetAttitude(void);
extern void GyrosAndAccsTest(void);

#define GRAVITY 1024
#define GRAVITYR (1.0f/(real32)GRAVITY)

#define ACC_TRIM_STEP 2

enum AccTypes { LISLAcc, MPU6050Acc, AccUnknown };

extern void ShowAccType(void);
extern void GetRatesAndAccelerations(void);
extern void GetNeutralAccelerations(void);
extern void InitAccelerations(void);
extern void ReadAccCalEE(void); 
extern void WriteAccCalEE(void); 

#define MPU6050_0xD0_ID     0xd0		//0x68
#define MPU6050_0xD2_ID     0xd2		//0x69

extern void GetMPU6050Values(void);
extern void InitMPU6050(void);
extern boolean MPU6050Active(void);

extern uint8 MPU6050_ID;
 
enum GyroTypes {
	MLX90609Gyro,
	ADXRS150Gyro,
	LY530Gyro,
	ADXRS300Gyro,
	UAVXArm32IMU,
	FreeIMU,
	GyroUnknown
};

extern void ShowGyroType(uint8);
extern void InitAnalogGyros(void);
extern void GetAnalogGyroValues(void);
extern void GetAnalogGyros(void);
extern void CalculateGyroRates(void);
extern void ErectGyros(uint16);

extern uint8 GyroType;

extern void SendCommand(int8);
extern uint8 ReadLISL(uint8);
extern uint8 ReadLISLNext(void);
extern void WriteLISL(uint8, uint8);
extern void InitLISLAcc(void);
extern boolean LISLAccActive(void);
extern void GetLISLValues(void);

extern uint8 MPU6050DLPF, MPU6050DHPF;
extern uint8 AccType;
extern int16 RawAcc[];
extern real32 AccConfidence;
extern real32 KpAcc;

//extern const rom uint8 MPUDLPFMask[];
#if defined(TESTING)
extern const rom uint16 InertialLPFHz[];
extern const rom uint8 * DHPFName[];
#endif


//______________________________________________________________________________________________

// i2c.c

#define	I2C_ACK			((uint8)(0))
#define	I2C_NACK		((uint8)(1))

#define SPI_CS			PORTCbits.RC5
#define SPI_SDA			PORTCbits.RC4
#define SPI_SCL			PORTCbits.RC3
#define SPI_IO			TRISCbits.TRISC4

#define	RD_SPI			1
#define WR_SPI			0
#define DSEL_LISL  		1
#define SEL_LISL  		0

extern void InitI2C(void);
extern boolean I2CWaitClkHi(void);
extern void I2CStart(void);
extern void I2CStop(void);
extern uint8 WriteI2CByte(uint8);
extern uint8 ReadI2CByte(uint8);
extern uint8 ReadI2CByteAtAddr(uint8, uint8);
extern void WriteI2CByteAtAddr(uint8, uint8, uint8);
extern boolean ReadI2Ci16vAtAddr(uint8, uint8, int16 *, uint8, boolean);
extern void ShowI2CDeviceName(uint8);
extern uint8 ScanI2CBus(void);
extern boolean I2CResponse(uint8);

extern boolean ESCWaitClkHi(void);
extern void ESCI2CStart(void);
extern void ESCI2CStop(void);
extern uint8 WriteESCI2CByte(uint8);

extern void ProgramSlaveAddress(uint8);
extern void ConfigureESCs(void);

extern boolean UseI2C100KHz;

//______________________________________________________________________________________________

// leds.c

#define AUX2M			0x01
#define BlueM			0x02
#define RedM			0x04
#define GreenM			0x08
#define AUX1M			0x10
#define YellowM			0x20
#define AUX3M			0x40
#define BeeperM			0x80

#define ALL_LEDS_ON		LEDsOn(BlueM|RedM|GreenM|YellowM)
#define AUX_LEDS_ON		LEDsOn(AUX1M|AUX2M|AUX3M)

#define ALL_LEDS_OFF	LEDsOff(BlueM|RedM|GreenM|YellowM)
#define AUX_LEDS_OFF	LEDsOff(AUX1M|AUX2M|AUX3M)

#define ALL_LEDS_ARE_OFF	( (LEDShadow&(BlueM|RedM|GreenM|YellowM))== (uint8)0 )

#define LEDRed_ON		LEDsOn(RedM)
#define LEDBlue_ON		LEDsOn(BlueM)
#define LEDGreen_ON		LEDsOn(GreenM)
#define LEDYellow_ON	LEDsOn(YellowM) 
#define LEDAUX1_ON		LEDsOn(AUX1M)
#define LEDAUX2_ON		LEDsOn(AUX2M)
#define LEDAUX3_ON		LEDsOn(AUX3M)
#define LEDRed_OFF		LEDsOff(RedM)
#define LEDBlue_OFF		LEDsOff(BlueM)
#define LEDGreen_OFF	LEDsOff(GreenM)
#define LEDYellow_OFF	LEDsOff(YellowM)
#define LEDYellow_TOG	if( (LEDShadow&YellowM) == (uint8)0 ) LEDsOn(YellowM); else LEDsOff(YellowM)
#define LEDRed_TOG		if( (LEDShadow&RedM) == (uint8)0 ) LEDsOn(RedM); else LEDsOff(RedM)
#define LEDBlue_TOG		if( (LEDShadow&BlueM) == (uint8)0 ) LEDsOn(BlueM); else LEDsOff(BlueM)
#define LEDGreen_TOG	if( (LEDShadow&GreenM) == (uint8)0 ) LEDsOn(GreenM); else LEDsOff(GreenM)
#define Beeper_OFF		LEDsOff(BeeperM)
#define Beeper_ON		LEDsOn(BeeperM)

#define BEEPER_IS_ON    ((LEDShadow&BeeperM)!=(uint8)0)
#define BEEPER_IS_OFF   ((LEDShadow&BeeperM)==(uint8)0)
#define Beeper_TOG		if (BEEPER_IS_OFF) LEDsOn(BeeperM); else LEDsOff(BeeperM)

extern void SaveLEDs(void);
extern void RestoreLEDs(void);
extern void SendLEDs(void);
extern void LEDsOn(uint8);
extern void LEDsOff(uint8);
extern void LEDChaser(void);

extern near uint8 LEDShadow, LEDShadowp;

extern uint8 SavedLEDs, LEDPattern;

//______________________________________________________________________________________________

// math.c

extern int16 SRS16(int16, uint8);
extern int32 SRS32(int32, uint8);
extern int16 Make2Pi(int16);
extern int16 MakePi(int16);
extern int16 Table16(int16, const int16 *);
extern int16 int16sin(int16);
extern int16 int16cos(int16);
extern int16 int32atan2(int32, int32);
extern int16 int16sqrt(int16);
extern int32 int32sqrt(int32);

//______________________________________________________________________________________________

// menu.c

extern void ShowPrompt(void);
extern void ShowRxSetup(void);
extern void ShowSetup(void);
extern void ProcessCommand(void);

extern const rom uint8 SerHello[];
extern const rom uint8 SerSetup[];
extern const rom uint8 SerPrompt[];

extern const rom uint8 RxChMnem[];

//______________________________________________________________________________________________

// outputs.c

#define MAX_DRIVES	6 // 10	// 6

// The minimum value for PW width is 1 for the pulse generators
#define OUT_MAXIMUM			208	//210 222	//  to reduce Rx capture and servo pulse output interaction
#define OUT_NEUTRAL			111			//  1.503mS @ 105 16MHz

#define OUT_HOLGER_MAXIMUM	225
#define OUT_YGEI2C_MAXIMUM	240
#define OUT_X3D_MAXIMUM		200

extern void ShowESCType(void);
extern uint8 PWLimit(int16);
extern uint8 I2CESCLimit(int16);
extern void DoMulticopterMix(int16 CurrThrottle);
extern void CheckDemand(int16 CurrThrottle);
extern void MixAndLimitMotors(void);
extern void MixAndLimitCam(void);
extern void OutSignals(void);
extern void StopMotors(void);
extern void InitMotors(void);

extern void DoI2CESCs(void);

enum PWTagsQuad {FrontC=0, LeftC, RightC, BackC, CamRollC, CamPitchC}; // order is important for X3D & Holger ESCs
enum PWTagsVT {FrontLeftC=0, FrontRightC};
enum PWTagsY6 {FrontTC=0, LeftTC, RightTC, FrontBC, LeftBC, RightBC };
enum PWTagsHexa {HFrontC=0, HLeftFrontC, HRightFrontC, HLeftBackC, HRightBackC, HBackC }; 
enum PWTagsAileron {ThrottleC=0, AileronC, ElevatorC, RudderC};
enum PWTagsElevon {RightElevonC=1, LeftElevonC=2};
enum PWTagsVTOL { RightPitchYawC=1, LeftPitchYawC=2, RollC=3 };
enum PWTags {K1=0, K2, K3, K4, K5, K6};

#if ( defined TRICOPTER ) | ( defined HEXACOPTER )
	#define NO_OF_I2C_ESCS 	6
#else
	#if defined(TRICOPTER)
		#define NO_OF_I2C_ESCS 	2
	#else
		#define NO_OF_I2C_ESCS 	4
	#endif
#endif

extern const int8 AFOrientation[];
extern int16 PW[], PWp[];
extern int16 PWSense[6];
extern int16 ESCI2CFail[NO_OF_I2C_ESCS];
extern int16 CurrThrottle;
extern int16 OCos, OSin;

extern near uint8 SHADOWB, PW0, PW1, PW2, PW4, PW5;
extern near int16 Rl, Pl, Yl;

extern int16 ESCMax;

//______________________________________________________________________________________________

// params.c

extern void ReadParametersEE(void);
extern void WriteParametersEE(uint8);
extern void UseDefaultParameters(void);
extern void DoStickProgramming(void);
extern boolean ParameterSanityCheck(void);
extern void InitParameters(void);

enum RCControls {ThrottleRC, RollRC, PitchRC, YawRC, RTHRC, RateControlRC,
	NavGainRC, BypassRC, CamPitchRC, NullRC}; 
enum ESCTypes { ESCPPM, ESCHolger, ESCX3D, ESCYGEI2C, ESCLRCI2C, ESCUnknown };

enum TxModes {TxMode1, TxMode2, ModeUnknown};

enum RCTypes {
	CompoundPPM, Spektrum1024, Spektrum2048, FutabaSBUS, ParallelPPM, RCUnknown
};
enum ESCTypes {
	ESCPPM, ESCHolger, ESCX3D, ESCYGEI2C, ESCUnknown
};

enum AFs {
	TriAF, TriCoaxAF, // aka Y6
	VTailAF,
	QuadAF,
	QuadXAF,
	QuadCoaxAF, // aka OctCoax
	QuadCoaxXAF,
	HexAF,
	HexXAF,
	OctAF,
	OctXAF,
	Heli90AF,
	Heli120AF,
	ElevonAF,
	AileronAF,
	VTOLAF,
	GimbalAF,
	AFUnknown,
};

enum Params { // MAX 64
	RollRateKp, // 01
	RollRateKi, // 02
	RollAngleKp, // 03
	TxMode, // 04
	RollIntLimit, // 05
	PitchRateKp, // 06
	PitchRateKi, // 07
	PitchAngleKp, // 08
	RFUsed, // 09
	PitchIntLimit, // 10

	YawRateKp, // 11
	RollRateKd, // 12
	IMU, // 13
	YawRateIntLimit, // 14
	RCType, // 15
	ConfigBits, // 16
	RxThrottleCh, // 17
	LowVoltThres, // 18
	CamRollKp, // 19
	PercentCruiseThr, // 20

	AltCFKpTrim, // 21
	RollPitchMix, //  22
	PercentIdleThr, // 23
	RollAngleKi, //  24
	PitchAngleKi, //  25
	CamPitchKp, // 26
	YawAngleKp, // 27
	PitchRateKd, // 28
	NavMaxAngle, // 29
	ROCKpRate, // 30

	YawRateKi, // 31
	MadgwickKpMag, //Acro, // 32
	NavRTHAlt, // 33
	NavMagVar, // 34
	SensorHint, // 35 UAVXPIC only
	ESCType, // 36
	RCChannels, // 37
	RxRollCh, // 38
	MadgwickKpAcc, // 39
	CamRollTrim, // 40
	NavMaxVelMPS, // 41
	RxPitchCh, // 42
	RxYawCh, // 43
	AFType, // 44
	TelemetryType, // 45
	MaxDescentRateDmpS, // 46
	DescentDelayS, // 47
	GyroLPF, // 48
	NavCrossTrackKp, // 49
	RxGearCh, // 50
	RxAux1Ch, // 51
	ServoSense, // 52
	AccConfSD, // 53
	BatteryCapacity, // 54
	RxAux2Ch, // 55
	RxAux3Ch, // 56
	NavWindKi, // 57
	GPSCFKpTrim, // 58
	Balance, // 59
	RxAux4Ch, // 60
	DriveFilt, // 61
	GPSProtocol, // 62
	StickScalePitchRoll, // 63
	StickScaleYaw
// 64
};

#define UseInvertMagMask 		0x01
#define	UseRTHDescendMask		(1<<1)
#define UseMWStickProgMask 		(1<<2)
#define EmulationEnableMask		(1<<3)
#define UseWPNavigationMask		(1<<4)
#define	UseFailsafeMask			(1<<5)
#define	UseRapidDescentMask		(1<<6)

// bit 7 unusable in UAVPSet

// In Servo Sense Byte
#define	UseMagicMask			(1<<6)

extern const rom int8 DefaultParams[MAX_PARAMETERS][2];
extern const rom uint8 ESCLimits [];

extern volatile boolean StickArmed;
extern uint8 ParamSet;
extern boolean ParametersChanged;
extern boolean UsingMPU6050;
extern int8 P[];

//__________________________________________________________________________________________

// rangefinder.c

extern void GetRangefinderAltitude(void);
extern void InitRangefinder(void);

extern int16 RangefinderAltitude, RangefinderROC;

//__________________________________________________________________________________________

// rc.c

extern void InitRC(void);
extern void MapRC(void);
extern void CheckSticksHaveChanged(void);
extern void UpdateControls(void);
extern void CheckRCTimeout(void);
extern void CheckThrottleMoved(void);
extern void UpdateRCMap(void);

extern uint8 Map[], RMap[];
extern int16 RC[], RCp[], Trim[];
extern int16 CruiseThrottle, NewCruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
extern int16 DesiredCamPitchTrim;
extern int16 ThrLow, ThrHigh, ThrNeutral;

//__________________________________________________________________________________________

// serial.c

extern void TxString(const rom uint8 *);
extern void TxChar(uint8);
extern void TxValU(uint16);
extern void TxValS(int16);
extern void TxNextLine(void);
extern void TxNibble(uint8);
extern void TxValH(uint8);
extern void TxValH16(uint16);
extern uint8 PollRxChar(void);
extern uint8 RxChar(void);
extern uint8 RxNumU(void);
extern int16 RxNumS(void);
extern void TxVal32(int32, int8, uint8);
extern void SendByte(uint8);
extern void TxESCu8(uint8);
extern void TxESCi8(int8);
extern void TxESCi16(int16);
extern void TxESCi24(int24);
extern void TxESCi32(int32);
extern void SendPacket(uint8, uint8, uint8 *, boolean);

#define TX_BUFF_MASK	255
extern uint8 	TxQ[];

#define RX_BUFF_MASK	255
extern uint8 	RxQ[];

//______________________________________________________________________________________________

// stats.c

extern void ZeroStats(void);
extern void ReadStatsEE(void);
extern void WriteStatsEE(void);
extern void ShowStats(void);

enum Statistics { 
	GPSAltitudeS, BaroAltitudeS, ESCI2CFailS, GPSMinSatsS, MinROCS, MaxROCS, GPSVelS,  
	AccFailS, CompassFailS, BaroFailS, GPSInvalidS, GPSMaxSatsS, NavValidS, 
	MinHDOPS, MaxHDOPS, RCGlitchesS, GPSBaroScaleS, GyroFailS, RCFailsafesS, 
	I2CFailS, MinTempS, MaxTempS, BadS, BadNumS, RollAccCorrAvS, RollAccCorrMeanS, PitchAccCorrAvS, PitchAccCorrMeanS}; // NO MORE THAN 32 or 64 bytes

extern int16 Stats[];

//______________________________________________________________________________________________

// telemetry.c

extern void SendPacketHeader(void);
extern void SendPacketTrailer(void);
extern void DoTelemetry(void);
extern void SendCycle(void);
extern void SendControl(void);
extern void SendMin(void);
extern void SendFlightPacket(void);
extern void SendNavPacket(void);
extern void SendControlPacket(void);
extern void SendParamPacket(uint8, uint8);
extern void SendParameters(uint8);
extern void SendStatsPacket(void);
extern void SendArduStation(void);
extern void SendCustom(void);
extern void CheckTelemetry(void);

extern uint8 UAVXCurrPacketTag;

enum PacketTags {UnknownPacketTag = 0, LevPacketTag, NavPacketTag, MicropilotPacketTag, WayPacketTag, 
	AirframePacketTag, NavUpdatePacketTag, BasicPacketTag, RestartPacketTag, TrimblePacketTag, 
	MessagePacketTag, EnvironmentPacketTag, BeaconPacketTag, UAVXFlightPacketTag, 
	UAVXNavPacketTag, UAVXStatsPacketTag, UAVXControlPacketTag, UAVXParamPacketTag, UAVXMinPacketTag, 
	UAVXArmParamPacketTag, UAVStickPacketTag, 
	FrSkyPacketTag = 99 };

enum TelemetryTypes {
	NoTelemetry,
	UAVXTelemetry,
	UAVXControlTelemetry,
	UAVXMinTelemetry,
	MWTelemetry,
	FrSkyTelemetry,
	UAVPSetSensorTelemetry,
	CustomTelemetry
};

//______________________________________________________________________________________________

// temperature.c

// deleted

//______________________________________________________________________________________________

// tests.c

extern void DoLEDs(void);
extern void ReceiverTest(void);
extern void PowerOutput(int8);
extern void LEDsAndBuzzer(void);
extern void BatteryTest(void);
extern void DoEmulation(void);
extern void GPSEmulation(void);

//______________________________________________________________________________________________

// utils.c

#define BATTERY_UPDATE_MS	1000

extern void LightsAndSirens(void);
extern void InitPorts(void);
extern void InitPortsAndUSART(void);
extern void InitMisc(void);
extern boolean Armed(void);
extern void DumpBlackBox(void);
extern void Delay1mS(int16);
extern void Delay100mSWithOutput(int16);
extern void DoBeep100mSWithOutput(uint8, uint8);
extern void DoStartingBeepsWithOutput(uint8);
extern int32 SlewLimit(int32, int32, int16);
extern int32 ProcLimit(int32, int32, int32);
extern int16 DecayX(int16, int16);
extern int32 Threshold(int32 v, int16 t);
extern void Rotate(int16 * nx, int16 * ny, int24 x,
		int24 y, int16 A);
extern void CheckBatteries(void);
extern void CheckAlarms(void);

extern int16 BatteryVoltsADC, BatteryCurrentADC, BatteryVoltsLimitADC, BatteryCurrentADCEstimated, BatteryChargeUsedmAH;
extern int32 BatteryChargeADC, BatteryCurrent;

extern boolean FirstPass;

//______________________________________________________________________________________________

// bootl18f.asm

extern void BootStart(void);

//______________________________________________________________________________________________

// tests.c

extern void DoLEDs(void);
extern void ReceiverTest(void);
extern void PowerOutput(int8);
extern void LEDsAndBuzzer(void);
extern void BatteryTest(void);

//______________________________________________________________________________________________

// Sanity checks

#if RC_MINIMUM >= RC_MAXIMUM
#error RC_MINIMUM < RC_MAXIMUM!
#endif
#if (RC_MAXIMUM < RC_NEUTRAL)
#error RC_MAXIMUM < RC_NEUTRAL !
#endif






