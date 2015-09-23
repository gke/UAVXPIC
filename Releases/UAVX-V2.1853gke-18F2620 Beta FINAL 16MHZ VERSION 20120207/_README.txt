=========================================================================

  UAVX - Experimental Quadrocopter Flight Software

  Based partially on UAVP (c) 2007 Ing. Wolfgang Mahringer

  Rewritten as UAVX (c) 2008 Prof. Greg Egan

=========================================================================

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

=========================================================================
  Reference Sites                           
=========================================================================

The original project may be found at:

	http://www.uavp.org 

The variants of UAVP to which this readme applies are hosted
at http:

	http://code.google.com/p/uavp-mods/ 

The related RCGroups thread is at:

	http://www.rcgroups.com/forums/showthread.php?t=811550

Disclaimer: The project at the Google Site is not endorsed by 
nor is it part of the official UAVP project.

=========================================================================
 The GNU GENERAL PUBLIC LICENSE             
=========================================================================

This software is released under GNU GPL license.
The GPL license is contained in the files
gpl-en.txt (in English) and gpl-de.txt (in German).

=========================================================================
 How to setup the Compilation Software      
=========================================================================

    * This version is for the 18F2620 PIC
    * Obtain MPLab and the C18 compiler from Microchip 

=========================================================================
 Transferring Firmware to the PIC           
=========================================================================

    * Use a PIC programmer to burn a version
      to the PIC - this will include the bootloader.
    * Once the bootloader is loaded once UAVPSet 
      may be used to load future versions of the
      flight and TestSoftware

=========================================================================
 VERSION HISTORY                            
=========================================================================

The first part of the version history relates to the original UAVP software
written by Ing. Wolfgang Mahringer as is retained for context.

Some source files in this directory are derived from the the originals 
including compatibility with UAVPSet the PC based GUI.  The general 
control flow departs significantly from the orginal and should not be
confused with UAVP. It is expected that remaining elements of the 
original UAVP will be replaced over time. 


15.3.2007 V3.05/V3.10 released by Wolferl
First public released version.
- Reworked RX stick reading (does no longer influence PID controllers)
- Camera servos working. Use RX channel 6 für elevation control.
- "Model lost" beeper works now ONLY when ufo switch (K8) is ON (switch 
  open)
- Bootloader
- low throttle limit to prevent motor shutoffs while in flight
- spectrum DX transmitters are now supported.
  set the synchronisation gap to 5 ms or more.
- NEW: Holgers brushless ESCs are now supported and tested!

19.4.2007 V3.06/V3.11 released by Wolferl
- Bugfix: spurios beeps in flight fixed
- NEW: Holgers brushless ESCs are now supported and tested!

13.5.2007 V3.12 released by Wolferl
- Low voltage detection on the 3.1 PCB can ONLY be positive
- Version split vor ADXRS150 and ADXRS300 gyros
- Bugfix: only 3.1 PCB: beeper did not stop beeping when
  turning off the ufo switch
- Blue LED now flashes when doing M commands (setting params)
- Red LED now flashes if batt voltage is low,
  nice for those ufos with no beeper connected.
- Diff-factors on Roll and Nick are now pure differential factors!
  New P-factor = Old P-factor + old D-factor!
- compass sensor supported (on black PCB only)

04.07.2007 V3.07/V3.13 released by Wolferl
- 3.1 PCB and IDG300: Roll and Nick gyro is now measured with
  3.6 volts reference. When upgrading from a previous version,
  divide roll and nick proportional factors by 1.3
- Compass sensor HMC6352 fully supported
- flip problem fixed (limiting both motors now)
- smoothing of roll and nick stick signals implemented
- camera servos now selectable 0 or 45 degrees

30.12.2007 V308/V3.14 released by Wolferl
- Ufo now continues flight for 3200 cycles (30 sec at timing 6)
  if RX signal is lost
- quick beeping when signal is lost
- fixed DEBUG_MOTORS with I2C ESCs
- DEBUG_SENSORS for visualizing sensor data via UAVPset
- X flight mode now mixed correctly
  mounting change no longer needed
- Baro controlled flight altitude
  new register 4 (TempComp), 9 (BaroProp), 28 (BaroDiff)

02.01.2008 V3.15beta1 released by Wolferl
- AUX1 to AUX3 now do a running light, and show baro lock

04.02.2008 v3.15beta2 released by Wolferl
- fixed an overflow in baro lock (diff param)
- if ch7 on switch-on is > 30 use ch7 as cam roll trim

27.07-2008 v3.15 released by Wolferl
- better Acc-sensor read (saved 300us cycle time)
- Cam servos are active even with 5-ch receivers
- better baro sensor algorithm (since 3.15beta5)
- RX-yaw channel is now filtered like roll and nick channels
- optimized interrupt routine
- A/D conversion now uses more A/D setup time to
  yield better, more accurate results

19.11.2008 V3.15m3 Greg Egan
- added non-latching battery alarm and support for 
  Spektrum AR7000 Rx
- removed redundant source from Interrupt routine
- rewrote makefiles (makeall)

22.11.2008 v3.15m3 Greg Egan
- added lookupup table based throttle curve and range
  checking on maximum throttle. Commissioned but
  abandoned in favour of Tx based throttle shaping.

27.11.2008 v3.15m3 Greg Egan
- DSM2 support for AR7000 Rx includes:
  * Spektrum DX7
  * Futaba 9C with Spektrum DM8
  * JR 9XII with DM9 module

13.12.2008 v3.15m3 Greg Egan
- Limited Impulse to minimum 4 as values of 1 through 
  UAVPSet were causing spontaneous motor intermittent
  start!

07.1.2009 v3.15m3 Greg Egan
- Reduced _Maximum output pulse width to prevent 
  minimum width Rx pulses from being missed with
  interrupts masked.

UAVX ********************************************************************

13.1.2009 UAVX Greg Egan
- Port of V3.15 to 18F2520 abandoned. Only a small part of
  the original V3.15 retained specifically compatability
  with UAVPSet and gyro compensation the latter to be 
  reformulated later. 

29.03.2009 UAVX Greg Egan
- Replaced old motor mixing scheme. Added battery alarm to
  TestSoftware. Improved vertical velocity damping scheme.

01.04.2009 UAVX Greg Egan
- Added single trace file under DEBUG_Sensors version.
  These are all 16 bit signed and may be displayed using the 
  hidden (bottom blank) option in the UAVPSet tools pulldown menu. 
  The trace values in order are:

	1  Heading Error
	2  Current Baro Pressure				
	3  Roll Rate
	4  Pitch Rate
	5  Yaw Error
	6  Roll Sum/Angle
	7  Pitch Sum/Angle
	8  Yaw Sum/Angle
	9  LR Acceleration
	10 FB Acceleration
	11 UD Acceleration
	12 Roll Correction
	13 Pitch Correction
	14 Desired Throttle
	15 Desired Roll 
	16 Desired Pitch
	17 Desired Yaw
	18 Motor Front 
	19 Motor Back
	20 Motor Left
	21 Motor Right
	22 Cam Roll Servo
	23 Cam Pitch Servo

  DEBUG_Motors removed.

03.04.2009 UAVX Greg Egan
- Added NMEA GPS test to test and flight software.
  Reduced baud rates to 9600b to make inter-character 
  delay greater than 1mS to avoid Rx capture clash.

25.04.2009 UAVX Greg Egan
- Cutover to UAVX from final version of UAVPm3.
- GPS based navigation included with station holding and return to home.

8.05.2009 UAVX Greg Egan
- Merged flight and test software for 18F2620 PIC.

13.07.2009 UAVX Greg Egan
- Installation of Tortoise SVN required for builds.

22.07.2009
- Complete rationalisation of compass yaw compensation.
  Parameters converted to vector.

15.08.2009
- Release of Experimental UAVX.

10.05.2010
- DEBUG_SENSORS revision to sensor trace order:

  * 1 Heading
  * 2 Baro Relative Altitude
  * 3 Rangefinder Altitude
  * 4 unused		
  * 5 Desired Throttle
  * 6 Desired Roll
  * 7 Desired Pitch
  * 8 Desired Yaw
  * 9 Roll Rate
  * 10 Pitch Rate
  * 11 Yaw Rate
  * 12 Roll Angle
  * 13 Pitch Angle
  * 14 Yaw Angle
  * 15 LR (X) Acceleration
  * 16 FB (Y) Acceleration
  * 17 DU (Z) Acceleration
  * 18 LR Compensation
  * 19 FB Compensation
  * 20 DU Compensation
  * 21 Alt Compensation

- Added revised Tricopter mixing to allow Y flight or one arm forward.
- Added Aileron and Elevon Mix.

19.06.2011
- Long time since an update of the README - time flies.
- V2 with re-formulated attitude control also for inclusion in UAVXArm.
- Improved communications with UAVXGS and UAVXNav.

=========================================================================
 SAFETY FIRST!                              
=========================================================================

If you work on your Quadrocopter, PLEASE TAKE CARE!

Nowadays, extremely powerful motors and accumulators
can pose a serious threat to your safety.

Safety measures:

- Remove the propellers before you test parameter or firmware modifications
- Work with a good bench power supply instead of an accupack.
  A short circuit will then not be able to destroy your hardware too easily
- Your ufo mainboards electronics can be damaged by 
  electrostatic discharge sparks. Make sure you are doing any
  work at the electronics in an ESD safe workplace.
- When using your transmitter, be sure to use the correct model memory on it.
- Be sure to have no other transmitters on your channel on air.

The authors will NOT BE RESPONSABLE FOR ANY ACCIDENTS OR
INJURIES IN ANY WAY!

PLEASE do yourself a favour and get an insurance which covers risks by doing 
model flight!

Imagine what happens if you only damage someone else's car or something....


=========================================================================
 Questions?                                
=========================================================================

BUT PLEASE, BEFORE YOU ASK, READ THE MANUAL. It is all in there, really! 
The manual may be found at the original project site.

Also, if you have some hardware related questions, please download the 
datasheets for the chips and read them thoroughly!

The variants of UAVP to which this readme applies are hosted
at:
	 http://code.google.com/p/uavp-mods/

There is an active RCGroups thread at:

	http://www.rcgroups.com/forums/showthread.php?t=811550

If you have any questions on the Official UAVP Project, please go to the forum at:
 
	http://forum.uavp.ch 

or the original project site at:

	http://www.uavp.org 

However, a registration is required to gain access to the forum.


Make sure you have the most recent version of the firmware!

