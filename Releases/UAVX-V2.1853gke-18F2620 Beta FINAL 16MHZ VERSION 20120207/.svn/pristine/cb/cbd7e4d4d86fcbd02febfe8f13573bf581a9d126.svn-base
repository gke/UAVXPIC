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

// Misc defines for testing

#define INC_HMC6352				// SLOW 100KHz I2C preferably REPLACE with HMC5883L

//#define FLAT_LISL_ACC			// LISL acc lying flat components up pins forward
//#define USE_I2C_ESCS_ONLY		// Flat chat 500KHz update rate - as good as it gets on a PIC!

//#define INC_RAW_ANGLES		// for debugging only

//#define HAVE_CUTOFF_SW		// Pin11 (RC0) short to ground when landed otherwise 10K pullup.


#ifndef BATCHMODE
	//#define EXPERIMENTAL
	//#define TESTING
	//#define FULL_TEST			// extended compass test etc.
	//#define FORCE_NAV					
	//#define SIMULATE
	#define QUADROCOPTER
	//#define TRICOPTER
	//#define Y6COPTER
	//#define HEXACOPTER
	//#define VTCOPTER
	//#define HELICOPTER
	//#define AILERON
	//#define ELEVON
	//#define VTOL
	//#define HAVE_CUTOFF_SW	// Ground PortC Bit 0 (Pin 11) for landing cutoff otherwise 4K7 pullup.						
#endif // !BATCHMODE

//________________________________________________________________________________________________

#if ( defined SIMULATE | defined TESTING )
	#define USE_LIMIT_MACRO		// squeek some more space
#endif

#ifdef FULL_TEST
	#define FULL_BARO_TEST
	//#define FULL_ACC_TEST
	//#define FULL_COMPASS_TEST
#endif // FULL_TEST

#define DEG_TO_ANGLE_UNITS		156L

#define MAX_BANK_ANGLE_DEG		45L

#define INC_MS5611			// FreeIMU etc.
#define INC_BMA180			// include BMA180 accelerometer code
#define INC_ADXL345			// include ADXL345 accelerometer code SF6DOF/9DOF	
#define INC_HMC58X3			// preferable Honeywell magnetometer

#ifdef INC_HMC6352
	#define USE_I2C100KHZ			// uses slow I2C routines because of HMC6352
#else
	#define INC_CYCLE_STATS	// tracks actual PID cycle times achieved - view in Setup
#endif // INC_HMC6352

// Desperate for space!!!!!
#ifndef VTCOPTER		
	#define INC_MPU6050		// include MPU6050 accelerometer/gyros
#endif 

#ifdef INC_HMC6352
#endif // INC_HMC6352

// Airframe

#ifdef UAVXBOARD
	#define UAVX_HW
#endif

#if ( defined TRICOPTER | defined QUADROCOPTER | defined Y6COPTER | defined VTCOPTER | defined HEXACOPTER )
	#define MULTICOPTER
#endif

#if ( defined HELICOPTER | defined AILERON | defined ELEVON | defined VTOL )	
	#if ( defined AILERON | defined ELEVON )
		#define NAV_WING
	#endif
#endif

#if defined QUADROCOPTER
	#define AF_TYPE QuadAF
#elif defined TRICOPTER
	#define AF_TYPE TriAF
#elif defined VTCOPTER
	#define AF_TYPE VAF
#elif defined Y6COPTER
	#define AF_TYPE Y6AF
#elif defined HEXACOPTER
	#define AF_TYPE HexAF
#elif defined VTOL
	#define AF_TYPE VTOLAF
#elif defined HELICOPTER
	#define AF_TYPE HeliAF
#elif defined ELEVON
	#define AF_TYPE ElevAF
#elif defined AILERON
	#define AF_TYPE AilAF
#endif

// Filters

#define YawFilter		MediumFilter
#define HeadingFilter	MediumFilter

// Timeouts and Update Intervals

#define SERVO_UPDATE_INTERVAL		18		// mS.

// Rescale angle to accelerometer units 
// MAGIC numbers assume 5mS for 40MHz and 8mS for 16MHz

#define RESCALE_TO_ACC				26

#define ANGLE_LIMIT					28000	// make large otherwise angle estimate latches up!!

#ifdef USE_I2C_ESCS_ONLY
	#define PID_CYCLE_MS			2		// 500KHz flat chat with I2C ESCs
#else	
	#define PID_CYCLE_MS			4		// 250KHz
#endif // USE_I2C_ESCS_ONLY
	
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
#define RC_STICK_ANGLE_SCALE		48L		// convert stick to desired "angle"

#define THROTTLE_LOW_DELAY_MS		1000L	// mS. that motor runs at idle after the throttle is closed
#define THROTTLE_UPDATE_MS			3000L	// mS. constant throttle time for altitude hold

#define NAV_ACTIVE_DELAY_MS			10000L	// mS. after throttle exceeds idle that Nav becomes active
#define NAV_RTH_LAND_TIMEOUT_MS		15000L	// mS. Shutdown throttle if descent lasts too long

#define UAVX_TEL_INTERVAL_MS		200L	// mS. emit an interleaved telemetry packet
#define UAVX_MIN_TEL_INTERVAL_MS	500L	// mS. emit minimum data packet for example to FrSky
#define ARDU_TEL_INTERVAL_MS		200L	// mS. alternating 1:5
#define UAVX_CONTROL_TEL_INTERVAL_MS 100L	// mS. flight control only
#define CUSTOM_TEL_INTERVAL_MS		50L	// mS.

#define GPS_TIMEOUT_MS				2000L	// mS.

// Altitude Hold

//#define ALT_SCRATCHY_BEEPER					// Scratchy beeper noise on altitude hold
#define ALT_HOLD_MAX_ROC_CMPS		50L		// Must be changing altitude at less than this for alt. hold to be detected

// Altitude Hold

#define LAND_CM						300L	// centimetres deemed to have landed when below this height

#define ALT_MAX_ROC_CMPS			100		// cm/S maximum climb rate

#define ALT_MAX_THR_COMP			80L		// Stick units was 40

#define ALT_RF_ENABLE_CM			300L //500L	// altitude below which the rangefiner is selected as the altitude source
#define ALT_RF_DISABLE_CM			400L //600L	// altitude above which the rangefiner is deselected as the altitude source

#define	ALT_UPDATE_HZ				20L		// Hz
#define FILT_ALT_HZ					(ALT_UPDATE_HZ/2)

#define BARO_SLEW_LIMIT_CMPS		1500L	//500L	// cm/S  
#define BARO_SANITY_CHANGE_CMPS		3000L	//500L	// cm/S 
#define BARO_UPDATE_MS				(1000/ALT_UPDATE_HZ)

// Navigation

#define	M_TO_GPS					54L		// approximate for constant folding

#define NAV_ACQUIRE_BEEPER

//#define ATTITUDE_NO_LIMITS				// full stick range is available otherwise it is scaled to Nav sensitivity

#define NAV_RTH_LOCKOUT				1000L	// ~100 ~ angle units per degree

#define NAV_MAX_ROLL_PITCH 			64L		// Rx stick units
#define NAV_MAX_FAKE_COMPASS_SLEW	25L		// Rx stick units
#define NAV_CONTROL_HEADROOM		10L		// at least this much stick control headroom above Nav control	
#define NAV_DIFF_LIMIT				24L		// Approx double NAV_INT_LIMIT
#define NAV_INT_WINDUP_LIMIT		64L		// ???

#define NAV_ENFORCE_ALTITUDE_CEILING		// limit all autonomous altitudes
#define NAV_CEILING					120L	// 400 feet
#define NAV_MAX_NEUTRAL_RADIUS		3L		// Metres also minimum closing radius
#define NAV_MAX_RADIUS				99L		// Metres
#define NAV_CLOSING_RADIUS_SHIFT	9		// Using shift to avoid division 
#define NAV_CLOSING_RADIUS			((int32)1<<NAV_CLOSING_RADIUS_SHIFT) // GPS units ~54 per metre

#ifdef NAV_WING
	#define NAV_PROXIMITY_RADIUS	20L		// Metres if there are no WPs
	#define NAV_PROXIMITY_ALTITUDE	5L		// Metres
#else
	#define NAV_PROXIMITY_RADIUS	5L		// Metres if there are no WPs
	#define NAV_PROXIMITY_ALTITUDE	3L		// Metres
#endif // NAV_WING

// reads $GPGGA sentence - all others discarded

#define GPS_OUTLIER_SLEW_LIMIT		(1*M_TO_GPS)	// maximum slew towards a detected outlier	
#define GPS_OUTLIER_LIMIT			(10*M_TO_GPS)	// maximum lat/lon change in one GPS update

#ifdef SIMULATE

#define SIM_CRUISE_MPS				8		// M/S
#define	GPS_UPDATE_HZ				5		// Hz - can obtain from GPS updates
#define	SIM_WING_YAW_RATE_DPS		30		// Deg/S
#define	SIM_MULTI_YAW_RATE_DPS		90		// Deg/S	

#define	GPS_MIN_SATELLITES			4		// preferably > 5 for 3D fix
#define GPS_MIN_FIX					1		// must be 1 or 2 
#define GPS_ORIGIN_SENTENCES 		5L		// Number of sentences needed to obtain reasonable Origin
#define GPS_MIN_HDILUTE				250L	// HDilute * 100

#else

#define	GPS_UPDATE_HZ				5		// Hz - can obtain from GPS updates
#define	GPS_MIN_SATELLITES			6		// preferably > 5 for 3D fix
#define GPS_MIN_FIX					1		// must be 1 or 2 
#define GPS_ORIGIN_SENTENCES 		30L		// Number of sentences needed to obtain reasonable Origin
#define GPS_MIN_HDILUTE				130L	// HDilute * 100

#endif // SIMULATE

#define	NAV_SENS_THRESHOLD 			40L		// Navigation disabled if Ch7 is less than this
#define	NAV_SENS_ALTHOLD_THRESHOLD 	20L		// Altitude hold disabled if Ch7 is less than this
#define NAV_SENS_6CH				80L		// Low GPS gain for 6ch Rx

#define	NAV_SENS_ACC_THRESHOLD 		20L		// Accelerometers disabled if Ch7 is less than this

#define NAV_MAX_TRIM				20L		// max trim offset for altitude hold

#define ATTITUDE_HOLD_LIMIT 		8L		// dead zone for roll/pitch stick for position hold
#define ATTITUDE_HOLD_RESET_INTERVAL 25L	// number of impulse cycles before GPS position is re-acquired

//#define NAV_PPM_FAILSAFE_RTH				// PPM signal failure causes RTH with Signal sampled periodically

// Throttle

#define	THROTTLE_MAX_CURRENT		40L		// Amps total current at full throttle for estimated mAH
#define	CURRENT_SENSOR_MAX			50L		// Amps range of current sensor - used for estimated consumption - no actual sensor yet.
#define	THROTTLE_CURRENT_SCALE		((THROTTLE_MAX_CURRENT * 1024L)/(200L * CURRENT_SENSOR_MAX ))

#define THROTTLE_SLEW_LIMIT			0		// limits the rate at which the throttle can change (=0 no slew limit, 5 OK)
#define THROTTLE_MIDDLE				10  	// throttle stick dead zone for baro 
#define THROTTLE_MIN_ALT_HOLD		75		// min throttle stick for altitude lock

// RC

#define RC_INIT_FRAMES				32		// number of initial RC frames to allow filters to settle

//________________________________________________________________________________________

#include "UAVXRevision.h"

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

typedef struct {
	i32u v;
	int16 a;
	int16 f;
	uint8 dt;
	} SensorStruct;

typedef struct { // Tx
	uint8 Head, Tail;
	uint8 B[128];
	} uint8x128Q;

typedef struct { // PPM
	uint8 Head;
	int16 B[4][8];
	} int16x8x4Q;	

typedef struct { // Baro
	uint8 Head, Tail;
	int24 B[8];
	} int24x8Q;	

#define NAVQ_MASK 3
#define NAVQ_SHIFT 2
typedef struct { // GPS
	uint8 Head, Tail;
	int32 LatDiffSum, LonDiffSum;
	int32 LatDiff[NAVQ_MASK], LonDiff[NAVQ_MASK];
	} int32Q;

typedef struct {
	// from Parameter Sets
	int16 IntLimit, Limiter;
	int32 Kp, Ki, Kd, Kp2;
	// run time
	int16 Trim, Hold; 
	int16 Desired, FakeDesired;
	int16 FirstGyroADC, GyroADC, GyroBias;
	int16 RawAngle, Angle, AngleE, AngleIntE;	
	int16 Rate, Ratep, RateEp, RateIntE;
	int16 Acc, AccADC, AccBias, AccOffset;
	int8 AngleCorr;
	int32 AccCorrAv, AccCorrMean;
	int16 Control;
	int16 NavCorr, NavIntE;
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

//#define Abs(i)				(((i)<0) ? -(i) : (i))
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

//#define ConvertGPSToM(c) (((int32)c*(int32)1855)/((int32)100000))
//#define ConvertMToGPS(c) (((int32)c*(int32)100000)/((int32)1855))

#define ToPercent(n, m) (((n)*100L+(m)/2)/(m))
#define FromPercent(n, m) (((n)*(m)+50)/100L)

// Simple filters using weighted averaging
#define VerySoftFilter(O,N) 	(SRS16((O)+(N)*3, 2))
#define SoftFilter(O,N) 		(SRS16((O)+(N), 1))
#define MediumFilter(O,N) 		(SRS16((O)*3+(N), 2))
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

#define	TMR0_1MS		(65536-640) // actually 1.0248mS to clear PW 

// Parameters for UART port ClockHz/(16*(BaudRate+1))

#define _B9600			65
#define _B38400			65

// This is messy - trial and error to determine worst case interrupt latency!

#define INT_LATENCY		(uint16)(65536 - 35) // x 1.6uS
#define FastWriteTimer0(t) Timer0.u16=t;TMR0H=Timer0.b1;TMR0L=Timer0.b0
#define GetTimer0		{Timer0.b0=TMR0L;Timer0.b1=TMR0H;}	

// Bit definitions
#define Armed			(PORTAbits.RA4)

#ifdef HAVE_CUTOFF_SW
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
		uint8
		AltHoldEnabled:1,	
		AllowTurnToWP:1,			// stick programmed
		GyroFailure:1,
		LostModel:1,
		NearLevel:1,
		LowBatt:1,
		GPSValid:1,
		NavValid:1,

		BaroFailure:1,
		AccFailure:1,
		CompassFailure:1,
		GPSFailure:1,
		AttitudeHold:1,
		ThrottleMoving:1,
		HoldingAlt:1,
		Navigate:1,

		ReturnHome:1,
		WayPointAchieved:1,
		WayPointCentred:1,
		AccelerometersEnabled:1,
		UsingRTHAutoDescend:1,
		BaroAltitudeValid:1,
		RangefinderAltitudeValid:1,
		UsingRangefinderAlt:1,

		// internal flags not really useful externally

		AllowNavAltitudeHold:1,	// stick programmed
		UsingPositionHoldLock:1,
		Ch5Active:1,
		Simulation:1,
		AcquireNewPosition:1, 
		MotorsArmed:1,
		NavigationActive:1,
		ForceFailsafe:1,

		Signal:1,
		RCFrameOK:1, 
		ParametersValid:1,
		RCNewValues:1,
		NewCommands:1,
		AccelerationsValid:1,
		CompassValid:1,
		CompassMissRead:1,

		UsingAltControl:1,
		ReceivingGPS:1,
		PacketReceived:1,
		NavComputed:1,
		AltitudeValid:1,		
		UsingSerialPPM:1,
		UsingTxMode2:1,
		FailsafesEnabled:1,

		// outside telemetry flags

		UsingTelemetry:1,
		TxToBuffer:1,
		NewBaroValue:1,
		BeeperInUse:1,
		RFInInches:1,
		FirstArmed:1,
		HaveGPRMC:1,

		NormalFlightMode:1,
		MPU6050Initialised:1,
		UsingAnalogGyros:1;	
		};
} Flags;

enum FlightStates { Starting, Landing, Landed, Shutdown, InFlight};
extern Flags F;
extern int8 near State, NavState, FailState;
extern boolean near SpareSlotTime;

//______________________________________________________________________________________________

// accel.c

#define GRAVITY 1024

enum AccTypes { LISLAcc, ADXL345Acc, BMA180Acc, MPU6050Acc, AccUnknown };

extern void ShowAccType(void);
extern void ReadAccelerations(void);
extern void GetNeutralAccelerations(void);
extern void AccelerometerTest(void);
extern void InitAccelerometers(void);

#define ADXL345_ID          0xa6

extern void ReadADXL345Acc(void);
void InitADXL345Acc(void);
extern boolean ADXL345AccActive(void);

#define MPU6050_ID     		0xd0		//0x68

extern void ReadMPU6050Acc(void);
extern void InitMPU6050Acc(void);
extern boolean MPU6050AccActive(void);

#define BMA180_ID_0x80          	0x80
#define BMA180_ID_0x82          	0x82

extern uint8 BMA180_ID;

extern void ReadBMA180Acc(void);
extern void InitBMA180Acc(void);
extern boolean BMA180AccActive(void);

extern void SendCommand(int8);
extern uint8 ReadLISL(uint8);
extern uint8 ReadLISLNext(void);
extern void WriteLISL(uint8, uint8);
extern void InitLISLAcc(void);
extern boolean LISLAccActive(void);
extern void ReadLISLAcc(void);

extern uint8 AccType;
extern int16 RawAcc[];

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

extern void DoShutdown(void);
extern void DecayNavCorr(uint8);
extern void FailsafeHoldPosition(void);
extern void DoCompScaling(void);
extern void Navigate(int32, int32);
extern void SetDesiredAltitude(int24);
extern void DoFailsafeLanding(void);
extern void AcquireHoldPosition(void);
extern void NavGainSchedule(int16);
extern void DoNavigation(void);
extern void FakeFlight(void); 
extern void DoFailsafe(void);
extern void WriteWayPointEE(uint8, int32, int32, int16, uint8);
extern void UAVXNavCommand(void);
extern void GetWayPointEE(int8);
extern void InitNavigation(void);

typedef struct { int32 E, N; int16 A; uint8 L; } WayPoint;

enum NavStates { HoldingStation, ReturningHome, AtHome, Descending, Touchdown, Navigating, Loitering};
enum FailStates { MonitoringRx, Aborting, Terminating, Terminated, RxTerminate };

#ifdef SIMULATE
extern int16 FakeMagHeading;
#endif // SIMULATE

extern WayPoint WP;
extern uint8 CurrWP;
extern int8 NoOfWayPoints;
extern int16 WPAltitude;
extern int32 WPLatitude, WPLongitude;
extern int16 WayHeading;
extern int16 NavClosingRadius, NavProximityRadius, NavNeutralRadius, NavProximityAltitude; 
extern uint24 NavRTHTimeoutmS;
extern int16 NavSensitivity, RollPitchMax;
extern int16 DescentComp;
extern int32 NavScale[];
extern int16 NavSlewLimit;

//______________________________________________________________________________________________

// baro.c

#define BARO_SANITY_CHECK_CM	((BARO_SANITY_CHANGE_CMPS*BARO_UPDATE_MS)/1000L)
#define BARO_SLEW_LIMIT_CM		((BARO_SLEW_LIMIT_CMPS*BARO_UPDATE_MS)/1000L)

#define BARO_INIT_RETRIES		100	// max number of initialisation retries

enum BaroTypes { BaroBMP085, BaroSMD500, BaroMPX4115, BaroMS5611, BaroUnknown };

// Freescale Baro

#define ADS7823_ID		0x90 	// ADS7823 ADC
#define MCP4725_ID		0xC8

extern void SetFreescaleMCP4725(int16);
extern void SetFreescaleOffset(void);
extern void ReadFreescaleBaro(void);
extern void GetFreescaleBaroAltitude(void);
extern boolean IsFreescaleBaroActive(void);
extern void InitFreescaleBarometer(void);

// Measurement Specialities Baro

#define MS5611_ID	0xee

extern void ReadMS5611Baro(void);
extern boolean IsMS5611BaroActive(void);	

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

extern const uint8 BaroError[];
extern int32 OriginBaroTemperature, OriginBaroPressure;
extern uint32 BaroPressure, BaroTemperature;
extern boolean AcquiringPressure;
extern int24 BaroRelAltitude;
extern i24u	BaroVal;
extern uint8 BaroType;
extern int16 AltitudeUpdateRate;
extern int16 BaroRetries;
extern int24 AltCF;
extern int16 TauCF;
extern int32 AltF[];

#ifdef SIMULATE
extern int24 FakeBaroRelAltitude;
#endif // SIMULATE

//______________________________________________________________________________________________

// compass.c

#define COMPASS_MAXDEV		30			// maximum yaw compensation of compass heading 
#define COMPASS_MIDDLE		10			// yaw stick neutral dead zone
#define COMPASS_TIME_MS		50			// 20Hz
#define COMPASS_UPDATE_HZ	(1000/COMPASS_TIME_MS)

#define COMPASS_MAX_SLEW	(12L*COMPASS_TIME_MS) //((TW0MILLIPI * COMPASS_TIME_MS)/500)

#define MAG_INIT_RETRIES	10

enum CompassTypes { HMC5883Magnetometer, HMC5843Magnetometer, HMC6352Compass, CompassUnknown };

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

// HMC58X3 Bosch Magnetometer

#define HMC58X3_3DOF    0x3C       
#define HMC58X3_9DOF 	0x1E

extern int16 GetHMC58X3Magnetometer(void);
extern void DoTestHMC58X3Magnetometer(void);
extern void CalibrateHMC58X3Magnetometer(void);
extern void InitHMC58X3Magnetometer(void);
extern boolean HMC58X3MagnetometerActive(void);
extern void ReadMagCalEE(void);
extern void WriteMagCalEE(void);

extern uint8 HMC58X3_ID;

// HMC6352 Bosch Compass

#define HMC6352_ID		0x42

extern int16 GetHMC6352Compass(void);
extern void DoTestHMC6352Compass(void);
extern void CalibrateHMC6352Compass(void);
extern void InitHMC6352Compass(void);
extern boolean HMC6352CompassActive(void);

extern i24u Compass;
extern int16 MagHeading, Heading, HeadingP, DesiredHeading, CompassOffset;
extern uint8 CompassType;
extern uint8 MagRetries;

//______________________________________________________________________________________________

// control.c

enum Attitudes { Roll, Pitch, Yaw };
enum Sensors {X, Y, Z};
enum Directions { LR, FB, DU }; // Roll, Pitch & Yaw

extern void DoAltitudeHold(void);
extern void UpdateAltitudeSource(void);
extern void AltitudeHold(void);

extern void DoAttitudeAngle(AxisStruct *);
extern void DoYawRate(void);
extern void DoOrientationTransform(void);
extern void GainSchedule(void);
extern void DoControl(void);
extern void InitControl(void);

extern uint8 PIDCyclemS, ServoInterval;
extern uint32 LastPIDUpdatemS;
extern uint32 CycleHist[];

extern AxisStruct A[3];
	
extern int16 CameraAngle[3];				
extern int24 OSO, OCO;
extern int16 Ylp;

extern int16 YawRateIntE;
extern int16 HoldYaw;

extern int16 ControlRoll, ControlPitch, CurrMaxRollPitch;

extern int16 AttitudeHoldResetCount;
extern int24 DesiredAltitude, Altitude, Altitudep; 
extern int16 AccAltComp, AltComp, BaroROC, BaroROCp, RangefinderROC, ROC, ROCIntE, MinROCCmpS;

extern int32 GS;

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

extern void UpdateField(void);
extern int32 ConvertGPSToM(int32);
extern int32 ConvertGPSTodM(int32);
extern int32 ConvertMToGPS(int32);
extern int24 ConvertInt(uint8, uint8);
extern int32 ConvertLatLonM(uint8, uint8);
extern int32 ConvertUTime(uint8, uint8);
extern void ParseGPRMCSentence(void);
extern void ParseGPGGASentence(void);
extern void SetGPSOrigin(void);
extern void ParseGPSSentence(void);
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
extern const uint8 NMEATags[MAX_NMEA_SENTENCES][5];

extern uint8 GPSPacketTag;

extern int32 GPSMissionTime, GPSStartTime;
extern int32 GPSLatitude, GPSLongitude;
extern int32 OriginLatitude, OriginLongitude;
extern int24 GPSAltitude, GPSRelAltitude, GPSOriginAltitude;
extern int32 DesiredLatitude, DesiredLongitude;
extern int32 LatitudeP, LongitudeP, HoldLatitude, HoldLongitude;
extern int16 GPSLongitudeCorrection;
extern int16 GPSHeading;
extern int16 GPSVel;
extern uint8 GPSNoOfSats;
extern uint8 GPSFix;
extern int16 GPSHDilute;
extern uint8 nll, cc, lo, hi;
extern boolean EmptyField;
extern int16 ValidGPSSentences;

#ifdef SIMULATE
extern int32 FakeGPSLongitude, FakeGPSLatitude;
#endif // SIMULATE

//______________________________________________________________________________________________

// gyro.c

enum GyroTypes { MLX90609Gyro, ADXRS150Gyro, IDG300Gyro, LY530Gyro, ADXRS300Gyro, 
		ITG3200Gyro, SFDOF6, SFDOF9, MPU6050, FreeIMU, Drotek, IRSensors, GyroUnknown }; 

extern void ShowGyroType(uint8);
extern void GetGyroValues(void);
extern void CalculateGyroRates(void);
extern void CheckGyroFault(uint8, uint8, uint8);
extern void ErectGyros(void);
extern void GyroTest(void);
extern void InitGyros(void);

#define INV_ID_3DOF 	0xD2
#define INV_ID_6DOF 	0xD0
#define INV_ID_MPU6050	MPU6050_ID

extern uint8 INV_ID, INVGyroAddress, INVAccAddress;

extern void InvenSenseViewRegisters(void);
extern void BlockReadInvensenseGyro(void);
extern void InitInvenSenseGyro(void);
extern boolean InvenSenseGyroActive(void);

extern uint8 GyroType;
extern int16 RawGyro[];
extern int32 NoAccCorr;

//______________________________________________________________________________________________

// irq.c

#define CONTROLS 			10

#define RxFilter			MediumFilterU
//#define RxFilter			SoftFilterU
//#define RxFilter			NoFilter

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

extern void SyncToTimer0AndDisableInterrupts(void);
extern void ReceivingGPSOnly(uint8);
extern void InitTimersAndInterrupts(void);
extern void ReceivingGPSOnly(uint8);
extern uint24 mSClock(void);

enum { StartTime, GeneralCountdown, LastPIDUpdate, UpdateTimeout, RCSignalTimeout, BeeperTimeout, ThrottleIdleTimeout, 
	FailsafeTimeout, AbortTimeout, NavStateTimeout, DescentUpdate, LastValidRx, LastGPS, AccTimeout, 
	GPSTimeout, RxFailsafeTimeout, StickChangeUpdate, LEDChaserUpdate, LastBattery, BatteryUpdate, 
  	TelemetryUpdate, NavActiveTime, BeeperUpdate, ArmedTimeout,
	ThrottleUpdate, BaroUpdate, CompassUpdate};

enum WaitStates { WaitSentinel, WaitTag, WaitBody, WaitCheckSum};

extern volatile uint24 mS[];
extern volatile uint24 PIDUpdate;

extern volatile near uint24 	MilliSec;
extern near i16u 	PPM[CONTROLS];
extern near uint8 	PPM_Index, NoOfControls;
extern near int24 	PrevEdge, CurrEdge;
extern near i16u 	Width, Timer0;
extern near int24 	PauseTime;
extern near uint8 	RxState;

extern near uint8 	ll, ss, tt, RxCh;
extern near uint8 	RxCheckSum, GPSCheckSumChar, GPSTxCheckSum;
extern near boolean WaitingForSync;

extern int8	SignalCount;
extern uint16 RCGlitches;

//______________________________________________________________________________________________

// inertial.c

extern void CompensateRollPitchGyros(void);
extern void CompensateYawGyro(void);
extern void DoAttitudeAngles(void);
extern void GetAttitude(void);

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
extern boolean ReadI2Ci16v(uint8, int16 *, uint8, boolean);
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

// The minimum value for PW width is 1 for the pulse generators

#define OUT_MAXIMUM			208	//210 222	//  to reduce Rx capture and servo pulse output interaction
#define OUT_NEUTRAL			111			//  1.503mS @ 105 16MHz

#define OUT_HOLGER_MAXIMUM	225
#define OUT_YGEI2C_MAXIMUM	240
#define OUT_X3D_MAXIMUM		200
#define OUT_LRC_MAXIMUM		200

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
extern void WriteT580ESC(uint8, uint8, uint8);
extern void WriteT580ESCs(int8,  uint8, uint8, uint8, uint8);
extern void T580ESCs(uint8, uint8, uint8, uint8);

enum PWTagsQuad {FrontC=0, LeftC, RightC, BackC, CamRollC, CamPitchC}; // order is important for X3D & Holger ESCs
enum PWTagsVT {FrontLeftC=0, FrontRightC};
enum PWTagsY6 {FrontTC=0, LeftTC, RightTC, FrontBC, LeftBC, RightBC };
enum PWTagsHexa {HFrontC=0, HLeftFrontC, HRightFrontC, HLeftBackC, HRightBackC, HBackC }; 
enum PWTagsAileron {ThrottleC=0, AileronC, ElevatorC, RudderC};
enum PWTagsElevon {RightElevonC=1, LeftElevonC=2};
enum PWTagsVTOL { RightPitchYawC=1, LeftPitchYawC=2, RollC=3 };
enum PWTags {K1=0, K2, K3, K4, K5, K6};

#if ( defined Y6COPTER ) | ( defined HEXACOPTER )
	#define NO_OF_I2C_ESCS 	6
#else
	#if ( defined TRICOPTER )
		#define NO_OF_I2C_ESCS 	2
	#else
		#define NO_OF_I2C_ESCS 	4
	#endif
#endif

extern int16 PW[6];
extern int16 PWSense[6];
extern int16 ESCI2CFail[NO_OF_I2C_ESCS];
extern int16 CurrThrottle;

extern near uint8 SHADOWB, PW0, PW1, PW2, PW4, PW5;
extern near int16 Rl, Pl, Yl;

extern int16 ESCMax;

//______________________________________________________________________________________________

// params.c

extern void ReadParametersEE(void);
extern void WriteParametersEE(uint8);
extern void UseDefaultParameters(void);
extern void UpdateParamSetChoice(void);
extern boolean ParameterSanityCheck(void);
extern void InitParameters(void);

enum TxRxTypes { 
	FutabaCh3, FutabaCh2, FutabaDM8, JRPPM, JRDM9, JRDXS12, 
	DX7AR7000, DX7AR6200, FutabaCh3_6_7, DX7AR6000, GraupnerMX16s, DX6iAR6200, FutabaCh3_R617FS, DX7aAR7000, ExternalDecoder, 
    FrSkyDJT_D8R, UnknownTxRx, CustomTxRx };
enum RCControls {ThrottleRC, RollRC, PitchRC, YawRC, RTHRC, CamPitchRC, NavGainRC, Ch8RC, Ch9RC, ChDumpRC}; 
enum ESCTypes { ESCPPM, ESCHolger, ESCX3D, ESCYGEI2C, ESCLRCI2C, ESCUnknown };
enum AFs { QuadAF, TriAF, VAF, Y6AF, HeliAF, ElevAF, AilAF, HexAF, VTOLAF, AFUnknown };

enum Params { // MAX 64
	RollKp, 			// 01
	RollKi,				// 02
	RollKd,				// 03
	NeutralRadius,		// 04 was HorizDampKp
	RollIntLimit,		// 05
	PitchKp,			// 06
	PitchKi,			// 07
	PitchKd,			// 08
	AltKp,				// 09
	PitchIntLimit,		// 10
	
	YawKp, 				// 11
	YawKi,				// 12
	AccTrack,			// 13
	YawLimit,			// 14
	YawIntLimit,		// 15
	ConfigBits,			// 16
	RxThrottleCh,		// 17 was TimeSlots
	LowVoltThres,		// 18
	CamRollKp,			// 19
	PercentCruiseThr,	// 20 
	
	BaroFilt,			// 21
	MiddleDU,			// 22
	PercentIdleThr,		// 23
	MiddleLR,			// 24
	MiddleFB,			// 25
	CamPitchKp,			// 26
	CompassKp,			// 27
	AltKi,				// 28c
	NavSlew,			// 29 was NavRadius
	NavKi,				// 30
	
	GSThrottle,			// 31
	Acro,				// 32
	NavRTHAlt,			// 33
	NavMagVar,			// 34
	SensorHint,			// 35
	ESCType,			// 36
	RxChannels,			// 37 was TxRxType
	RxRollCh,			// 38
	PercentNavSens6Ch,	// 39
	CamRollTrim,		// 40
	NavKd,				// 41
	RxPitchCh,			// 42
	RxYawCh,			// 43
	BaroScale,			// 44
	TelemetryType,		// 45
	MaxDescentRateDmpS,	// 46
	DescentDelayS,		// 47
	NavIntLimit,		// 48
	AltIntLimit,		// 49
	RxGearCh,			// 50 was GravComp
	RxAux1Ch,			// 51 was CompSteps
	ServoSense,			// 52
	CompassOffsetQtr,	// 53
	BatteryCapacity,	// 54
	RxAux2Ch,				// 55 was GyroYawType
	RxAux3Ch,				// 56 was AltKd
	Orient,				// 57
	NavYawLimit,		// 58
	Balance,			// 59
	RxAux4Ch,			// 60
	RollKp2,			// 61
	PitchKp2			// 62
	
	// 63 - 64 unused currently
	};

#define UsePositionHoldLock 0
#define UsePositionHoldLockMask 	0x01

#define UseRTHDescend 		1
#define	UseRTHDescendMask	0x02

#define TxMode2 			2
#define TxMode2Mask 		0x04

#define RxSerialPPM 		3
#define RxSerialPPMMask		0x08 

#define RFInches 		4
#define RFInchesMask		0x10

// bit 4 is pulse polarity for 3.15

#define UseFailsafe 			5
#define	UseFailsafeMask		0x20

#define UseAltControl 			6
#define	UseAltControlMask		0x40

// bit 7 unusable in UAVPSet

// In Servo Sense Byte
#define Polarity 			6
#define	PPMPolarityMask		0x40

extern const int8 DefaultParams[MAX_PARAMETERS][2];
extern const uint8 ESCLimits [];

extern int16 OSin[], OCos[];
extern int8 Orientation;

extern uint8 ParamSet;
extern boolean ParametersChanged, SaveAllowTurnToWP;
extern int8 P[];

extern uint8 UAVXAirframe;

//__________________________________________________________________________________________

// rangefinder.c

extern void GetRangefinderAltitude(void);
extern void InitRangefinder(void);

extern int16 RangefinderAltitude, RangefinderAltitudeP;

//__________________________________________________________________________________________

// rc.c

extern void DoRxPolarity(void);
extern void InitRC(void);
extern void MapRC(void);
extern void CheckSticksHaveChanged(void);
extern void UpdateControls(void);
extern void CaptureTrims(void);
extern void CheckThrottleMoved(void);

extern uint8 Map[], RMap[];
extern boolean PPMPosPolarity;
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

#define TX_BUFF_MASK	127
extern uint8 	TxCheckSum;
extern uint8x128Q 	TxQ;

//______________________________________________________________________________________________

// stats.c

extern void ZeroStats(void);
extern void ReadStatsEE(void);
extern void WriteStatsEE(void);
extern void ShowStats(void);

enum Statistics { 
	GPSAltitudeS, BaroRelAltitudeS, ESCI2CFailS, GPSMinSatsS, MinROCS, MaxROCS, GPSVelS,  
	AccFailS, CompassFailS, BaroFailS, GPSInvalidS, GPSMaxSatsS, NavValidS, 
	MinHDiluteS, MaxHDiluteS, RCGlitchesS, GPSBaroScaleS, GyroFailS, RCFailsafesS, 
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
extern void SensorTrace(void);

extern uint8 UAVXCurrPacketTag;

enum PacketTags {UnknownPacketTag = 0, LevPacketTag, NavPacketTag, MicropilotPacketTag, WayPacketTag, 
	AirframePacketTag, NavUpdatePacketTag, BasicPacketTag, RestartPacketTag, TrimblePacketTag, 
	MessagePacketTag, EnvironmentPacketTag, BeaconPacketTag, UAVXFlightPacketTag, 
	UAVXNavPacketTag, UAVXStatsPacketTag, UAVXControlPacketTag, UAVXParamPacketTag, UAVXMinPacketTag, 
	UAVXArmParamPacketTag, UAVStickPacketTag, 
	FrSkyPacketTag = 99 };

enum TelemetryTypes { NoTelemetry, GPSTelemetry, UAVXTelemetry, UAVXControlTelemetry, UAVXMinTelemetry, ArduStationTelemetry, CustomTelemetry };

//______________________________________________________________________________________________

// temperature.c

#define TMP100_ID		0x96 

extern void GetTemperature(void);
extern void InitTemperature(void);

extern int16 AmbientTemperature;

//______________________________________________________________________________________________

// tests.c

extern void DoLEDs(void);
extern void ReceiverTest(void);
extern void BatteryTest(void);

//______________________________________________________________________________________________

// utils.c

#define BATTERY_UPDATE_MS	1000

extern void LightsAndSirens(void);
extern void InitPorts(void);
extern void InitPortsAndUSART(void);
extern void InitMisc(void);
extern void Delay1mS(int16);
extern void Delay100mSWithOutput(int16);
extern void DoBeep100mSWithOutput(uint8, uint8);
extern void DoStartingBeepsWithOutput(uint8);
extern int32 SlewLimit(int32, int32, int32);
extern int32 ProcLimit(int32, int32, int32);
extern int16 DecayX(int16, int16);
//extern void LPFilter16(int16*, i32u*, int16);
//extern void LPFilter24(int24* i, i32u* iF, int16 FilterA);
extern void CheckBatteries(void);
extern void CheckAlarms(void);
extern int32 Abs(int32);
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
extern void BatteryTest(void);

//______________________________________________________________________________________________

// Sanity checks

#if RC_MINIMUM >= RC_MAXIMUM
#error RC_MINIMUM < RC_MAXIMUM!
#endif
#if (RC_MAXIMUM < RC_NEUTRAL)
#error RC_MAXIMUM < RC_NEUTRAL !
#endif

#if (( defined TRICOPTER + defined QUADROCOPTER + defined VTCOPTER + defined Y6COPTER + defined HEXACOPTER + defined HELICOPTER + defined AILERON + defined ELEVON  + defined VTOL ) != 1)
#error None or more than one aircraft configuration defined !
#endif




