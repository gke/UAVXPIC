### Introduction ###

**If you have the UAVXArm32F4  version you should read UAVXArm32F4Startup. This Wiki is for the version of UAVX which uses the original UAVP board based on PIC microprocessors.** This Wiki however useful information and is well worth reading through even if you are using the new board.

UAVX using the original UAVP board was released for Beta testing on the 15 August 2009 Australian EST. The normal caveats apply in that use of UAVX and these instructions is entirely at your own risk - see the GNU License.

This is intended to be a brief introduction to the steps you may wish to take when first using UAVX. Internally UAVX is completely different to UAVP flight software.  Despite the internal differences a significant amount of effort has made to render the transition from UAVP to UAVX as painless as possible by preserving the validity of most parameter values and a familiar user interface. UAVPSet (V3.0 on-wards also in this repository) has been extended to include the new features supported by UAVX.

You should NOT use GPS for your first test flights of UAVX regardless of how tempting it may be to do so. UAVX only requires GPS for navigation functions and may be flown without GPS if desired.

### Assumptions ###

It is assumed you:
  * Are trained in aircraft safety by a qualified **FAI** affiliated trainer.
  * You have read and understood the UAVP manual as it applies to safety and setting up a UAVP quadrocopter.
  * Are familiar with the use of UAVPSet to setup parameters for UAVP, UAVX's predecessor.
  * You have previously flown your quadrocopter/tricopter using UAVP flight software. As this is less likely now you need to do some reading of the UAVX thread and ask questions.
  * You have read and understood the contents of this document.
  * **You will read and comply with all WARNINGS and CAUTIONS for Safety**

### Notes/FAQs ###

  * Is there a Pre-flight Safety Checklist: There **IS** a safety checklist which you should use [UAVX Flight Checklist](http://code.google.com/p/uavp-mods/wiki/UAVPandUAVXFlightChecklist)

### Obtaining UAVX ###
  * How do get UAVX: The latest version of UAVX may be obtained under the downloads tab on the uavp-mods pages or you may obtain it preloaded from quadroufo.com.

  * Which version of UAVX do I Use: See File Versions below.

  * I have my own upgrade raw 18F2620 PIC: UAVX has its own bootloader which is not compatible with the original UAVP bootloader. You will need to use a PIC programmer to write a copy of UAVX to the PIC.

  * Can I just load the bootloader onto the PIC first: The bootloader is integrated with all versions of UAVX. Once initially loaded using a programmer onto the PIC, the bootloader will not allow itself to be overwritten but can be used to load later versions of UAVX using UAVPSet and a normal serial cable.

  * Can the bootloader for the 16MHz version of UAVX load the 40MHz version of UAVX: Unfortunately no. You will have to use a PIC programmer to write the 40MHz version for the first time.

  * What modifications do I need to make to the board for 40MHz: Replace the original 16Mhz crystal with a 10MHz version.

### Board Diagram ###
  * **Note 1:** The flight controller board requires 12v power to the board and at least the K1 ESC hooked to it and also powered.

  * Board connections:

http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a3561705-34-UAVP%20top%20board%20jesolins.jpg?d=1288234002

### Props ###
http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a4655671-204-ccwtractor-cwpusher_jesolins.JPG

  * **Tricopter:**
  * Use tri firmware.
  * You must jumper the middle +5v pins from K3 to K4 to power the servo.
  * Use either all standard CCW or CW props on all axis, or CW and CCW on either left and right and either CCW or CW on the tail/front.
  * Adjust tail motor radius and lipo for CG and flight performance.
  * Front of board is oriented to K1.
  * For servo on back arm or tail use UAVPset FWD (x7.5 Deg) = 24.
  * For servo on back arm or front use UAVPset FWD (x7.5 Deg) = 0.
  * Currently I2c ESCs can not be used for the tricopter configuration.
  * For the tricopter firmware, you must solder a bridge/wire between the K3 and K4 middle 5v pins to provide power to the mechanical servo.  You should remove this bridge if you decide to use the board for any other firmware that does not use the K4 for a mechanical servo.
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_tri_options_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_tri_options_jesolins.png)

  * **VT-Copter:**
  * Use VT firmware
  * Use either all standard CCW or CW props on all axis, or CW and CCW on either left and right and either CCW or CW on the tail/front.
  * Experiment with smaller motors on V-tail in relation to the front motors, motor angles, and prop sizes.
  * Adjust tail motor radius and lipo for CG and flight performance.
  * Front of board is oriented to K1.
  * For servo on back arm or tail use UAVPset FWD (x7.5 Deg) = 24.
  * For servo on back arm or front use UAVPset FWD (x7.5 Deg) = 0.
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_VT-Copter_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_VT-Copter_jesolins.png)

  * **Quadrocopter:**
  * Use quad firmware.
  * Use standard CCW tractor props on the front and rear arms and CW pusher props on the left and right arms.
  * Front of board is oriented to K1.
  * For +-mode the UAVPset FWD (x7.5 Deg) = 0.
  * For X-mode the UAVPset FWD (x7.5 Deg) = 6. Use = -6 for CW or +6 for CCW.
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_quad_options_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_quad_options_jesolins.png)

  * **Hexacopter:**
  * Use tri firmware.
  * You must jumper the middle +5v pins from K3 to K4 to power the servo.
  * Use the same CW or CCW props on each axis.
  * You can use y-connectors and either one or two servo to tilt one or two motors.
  * For servo on back arms or tail use UAVPset FWD (x7.5 Deg) = 20.
  * For servo on back arms or front use UAVPset FWD (x7.5 Deg) = 4.
  * Can be configured in a flat/star or Y-coax model.
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_hexa_options_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_hexa_options_jesolins.png)

  * **Octocopter:**
  * Use quad firmware.
  * Use standard CCW tractor props on the front and rear arms and CW pusher props on the left and right arms.
  * Use a y-cable to connect a second ESC and motor to each servo axis for redundancy.
  * Either orient the board's front between the two axis motors and use "0" for the UAVPset FWD offset, or use the UAVPset FWD (x7.5 Deg) offset of 3.  Use = -3 for CW or +3 for CCW.
  * Can be configured in a flat/star or coax model.
  * The motor/prop rotation is the same on each axis
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options1_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options1_jesolins.png)
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options2_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options2_jesolins.png)
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options3_jesolins.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/1/UAVX_octo_options3_jesolins.png)

  * **Notes:**
  * The flight controller board requires 12v power to the board and at least the K1 ESC hooked to it and also powered.
  * K1, K2, K3, K4 refers to motor connection.
  * The control board "front" must ALWAYS point TOWARD the K1 motor(s).
  * Motors are always in the same relative position which is K2, K1, K3, K4 with K4 being replaced by the steering servo for tricopters.
  * The UAVPset FWD (x7.5 Deg) value sets the forward flight orientation CW relative to the motor, or pair of motors, connected to K1 in increments of 7.5 deg. For CW use +x. for CCW use -x.
  * For a quad UAVPset FWD (x7.5 Deg) = 0.
  * For a quad X-mode UAVPset FWD (x7.5 Deg) K1 CCW = 6.
  * For a tri UAVPset FWD (x7.5 Deg) = 24.
  * For a hexa UAVPset FWD (x7.5 Deg) K1 CCW = 8.
  * For an octo UAVPset FWD (x7.5 Deg) = If FCB front is facing K1(R) CCW = 3.
    * Suggestion is to mount the octo FCB so the front is between the two K1 arms and use a "0" FWD offset.
  * You DO NOT have to adjust the compass offset as this is always relative to K1.
  * Tricopter:  The UAVPset FWD (x7.5 Deg) value of 24 (180 deg) is Y mode with steering servo motor arm to the rear and a value of 0 is steering arm forward.
  * For the Tricopter I2C is not currently supported. The yaw servo is updated at the same rate as the motors and so greater Pulse Cycle times may be required.  Different modes may require changes to P, I and D parameters.
  * For the tricopter firmware, you must solder a bridge/wire between the K3 and K4 middle 5v pins to provide power to the mechanical servo.  You should remove this bridge if you decide to use the board for any other firmware that does not use the K4 for a mechanical servo.

### Tx/Rx ###
**WARNING: Nothing will work properly unless your UAVX is successfully setup for your TX/Rx set and in fact it may be**dangerous**- (see Tx/Rx Setup below).**

**WARNING: Do not power the Rx separately using one of the motor ESC's.  Always power it from the filtered power from the flight controller board, or if desired a separate lipo that is grounded to the board, or a separate BEC.  An ESC BEC improperly used that directly powers the Rx can have negative effects on the Rx and board's performance and decrease PPM signal quality and model safety.**

**WARNING: In order for the UAVX built in failsafe to work properly, you must pay attention to how your particular Rx failsafe operates and set it up so that it works safely with the UAVX.  See the Tx/Rx WIKI http://code.google.com/p/uavp-mods/wiki/UAVXFailsafe and setup instructions for failsafe in the Tx/Rx setup paragraph below!**


**Note 1:** UAVX firmware V2.xxx and UAVXArm use a more direct scheme for setting up the Tx/Rx. You should read [http://code.google.com/p/uavp-mods/wiki/TxRxChannelAssignments TxRxChannelAssignments}

**Note 2:** Also read the Tx/Rx Setup paragraph.

  * The following list of UAVX v1.xxxx Tx/Rx combinations are know to work using the UAVPset v4.xx odd channel selection and can be selected in the pull-down menu:
    * Futaba Thr3/ Turnigy 9X - Aileron, Elevator, Gear, Aux2 (also Turnigy 9X)
    * Futaba Thr2 - Aileron, Rudder, Gear, Aux2
    * Futaba 9C DM8/AR7000 - Throttle, Aileron, Rudder, Aux2
    * JR XP8103/[R700](https://code.google.com/p/uavp-mods/source/detail?r=700) PPM - Throttle, Elevator, Gear, Aux2
    * JR 9XII  DM9/AR7000 - Throttle, Elevator, Gear, Aux2
    * JR DSX12/AR7000 - Aileron, Rudder, Gear, Aux2
    * Spektrum DX7/AR7000 - Aileron, Rudder, Gear, Aux2
    * Spektrum DX7/AR6200 - Throttle, Elevator, Gear (Mix Rudder to Aux1)
    * Futaba Thr3 Sw 6&7 - Aileron, Elevator, Gear, Aux2
    * Spektrum DX7/AR6000 - Throttle, Elevator, Gear, Aux2
    * Graupner MX16S - Throttle, Elevator, Gear, Aux2
    * Spektrum DX6i/AR6200 - Aileron, Elevator, Throttle (6ch version only)
    * Futaba Thr3/R617FS (6ch only) - Aileron, Throttle, Ch5 (6ch version only)
    * Spektrum DX7a/AR7000 - Throttle, Elevator, Gear, Aux2
    * External Decoder (Fut Th3 6/7 Sw)
    * FrSky DJT/D8R-SP (Th1 Comp.)
    * NOT SET YET - You have not set the Tx/Rx combination you are using
    * Custom - for self compiled firmware channel order

  * For UAVX v1.xxxx and UAVPset v4.xx, click once on the Tx/Rx box and look at the bottom UAVPset information window. Find the one that matches your Rx connection to the UAVP board (you may need to scroll down) and select it in the Tx/Rx pull-down box. You may also need to select Mode 2 if your throttle is on the left hand side of your Tx.

  * If your Tx/Rx combination is not currently supported then you should check the following site [Logic Analyzer Displays of PPM Rx Channels](http://www.rcgroups.com/forums/showthread.php?t=1171547). The Rx signal decoding used by UAVX/UAVP requires that the pulses emitted by the Rx do not overlap and there are no gaps between the pulses.

  * If your Tx/Rx is not listed, or when scoping the Rx signals in the [Logic Analyzer Displays of PPM Rx Channels](http://www.rcgroups.com/forums/showthread.php?t=1171547), then you can always use a separate PPM adapter board and then select the "all channels" box in the UAVPset.  When using a PPM adapter such as the $25 DIYDrones PPM adapter, you can use any Tx/Rx combination.

  * For UAVX v2.xxxx and UAVPset v4.xx see **Note 1** in this paragraph.

  * I have a Futaba and only Ch6 is connected to a Pot so how do I swap Ch6 and Ch7: Try the "Custom" Rx/Tx config in the UAVPSet pulldown. UAVX can map any input sequence order to any other order provided that the Rx pulses do not overlap and are emitted by the Rx with no gaps. You need to change the **custom** setting definition in the source program to achieve this (uavx.c).

  * What does the Tx Mode checkbox do: This is used to tell the stick programming software (see below) which channels to use. It **does not swap** where your throttle etc. is on your Tx. Set it and do a Write if you are using a Tx with the throttle on the left hand.

  * I have a Rx which uses a single composite PPM connection to the UAVP: Select the Rx to Quad "All Channels" checkbox in UAVPSet and do a Write.

  * I have a 6 Channel Receiver: Use the versions with 6CH in the file name. You will need to set the GPS Navigation sensitivity using UAVPSet and will not be able to change it when in flight. You may have to mix Aux1 to Rudder to obtain Yaw control. Be sure to ground the remaining UAVX Rx wire to a ground on your Rx.

  * UAVX firmware V2.xxx and UAVXArm use a more direct scheme for setting up the Tx/Rx. You should read [http://code.google.com/p/uavp-mods/wiki/TxRxChannelAssignments TxRxChannelAssignments}

### Lights and Sirens! ###
  * All of the LEDs are on: Your PIC has no parameters loaded yet. Use Tools->Test Software->D command in UAVPSet to load a default set of parameters.

  * I have a solid Green LED and a solid Yellow LED - the Red LED is flashing in time with a very annoying beeper noise when the quadrocopter is disarmed:  You have the RTH switch activated. Switch it off, as you do not want the quadrocopter to go to full throttle and climb to the RTH altitude **through you** when you bend over it to arm!

  * I have a solid Red LED and a solid Yellow LED: There is no Rx signal. Did you turn on your Tx? (see Tx/Rx Setup below).

  * I saw a brief flash on the Yellow LED when I switched on: This was the accelerometer test. If you did not see this your accelerometers were offline or not installed.

  * I have a solid Green LED and a solid Yellow LED: You probably have your throttle open - close it before arming.

  * I have a solid Green LED, solid Yellow LED and flashing Red LED after I armed: You throttle was open when you armed - close the throttle, disarm and re-arm.

  * I have a solid Green LED and a flickering Yellow LED: Good you are ready to arm.

  * I have a solid Green LED but the Red LED flashed in a regular way several times when I armed just before the starting beeps: This is the Gyro calibration being performed. **Do not move the quadrocopter** while this is happening. It does not need to be perfectly level just stationary.

  * I have solid Green and Red LEDs and just heard three short beeps and one long beep: The quadrocopter is ready to fly but has not detected a GPS unit as being connected. It may be your GPS is not set to the correct configuration (See Compass FAQs).

  * I have solid Green and Red LEDs and a flashing Blue LED: The GPS is online and is still acquiring a valid position fix. You can take off but all GPS navigation support will be disabled.

  * The Blue LED is flashing but both the Green and Red LEDs are still on solid after a long time: Yes it does seem like a long time when all we want to do is just fly!  It can take 2-3 minutes especially if your GPS does not have battery backup or if it is a new flying location.

  * I have solid Green and a flashing Blue LED: You are ready to fly as the GPS has recorded your origin location.

  * From UAVX v1.1247 onward the aircraft will not arm unless the accelerometer is ONLINE. The LED indication for failed Acc is both Yellow and Red LEDs flashing.

#### Special Alarms In Flight ####
  * When the barometer is holding altitude, the onboard LEDs will flash in sequence.  You will also hear the beeper make a scratchy sound.  You can also hook up the AUX LEDs to the appropriate K13 pins to use as additional LED hover indicators.  Pin8 is positive(anode) and pins 1-7 are negative(cathode). To use this you must use an appropriate ballast resistor for the LEDs type and color you choose.  Caution: Do not exceed 500ma total!

  * I occasionally hear a single beep particularly when I let the sticks go back to neutral: This indicates that the aircraft has captured a new desired GPS position hold point.

  * I hear the beeper sounding once every second: This is the flight battery alarm. You should LAND immediately.

  * I hear the beeper sounding twice every second and the aircraft is coming down really quickly: This is a **failsafe alarm** and UAVX has lost the RC comms link and is doing an auto-land (See UAVXFailsafe Wiki).

  * My aircraft is sitting on the ground and has started making beep noises every six seconds: the aircraft thinks it is unloved and lost and starts the **lost model alarm** after sitting around for 2-3 minutes or more likely you forgot to disarm the aircraft and make it safe after landing.

  * I don't seem to be able to find my aircraft but I can hear a beep noise off in the trees or the briar patch every six seconds: Yes that is the **lost model alarm** you hear coming from the direction where it crashed (See UAVXFailsafe Wiki). It starts automatically after a delay of a few minutes.


### Setting Parameters ###
  * Setting Default Parameter Values: Use Tools->Test Software->"Shift D" command in UAVPSet. The D command is sent directly to UAVX which uses the defaults stored within UAVX. You must do a Read from UAVX after you have done this to see the parameter values which have been used. To go to param 2 you must use the pitch/roll stick to select it first, use either upper right for RTH altitude off, or lower left for RTH altitude on, then do a read.

  * Can I use my old UAVX parameter files: Unfortunately there are new parameters still being added so when there is a change to the UAVPSet release you should expect the parameters to have also changed.  Take a note of your particular settings (screen dump is fastest) and then  load the defaults using the D command (see above). This will ensure the additional parameters of UAVX are given sensible values. You can then update the parameters with your favorite settings and save them to a file. The download notes tell you which versions of UAVX, UAVPSet are cross compatible.

  * Setting your own Parameters: There is a large amount of information on parameter values on the threads. Both parameter sets must be programmed - you switch between parameter sets using stick programming (See below). Parameters go ORANGE when changed in UAVPSet and should go GREEN after you do a Write. If they go RED then the Write was unsuccessful or the values were rejected as invalid. You should always do your read after writing parameters to double check that UAVX has received them.

### ESCs ###
  * Also read the ESC Setup paragraph.

  * YGE I2C ESCs: The YGE i2C ESC must be programmed to respond to particular addresses on the I2C bus. Use the Tools->Test Software->Y command and follow the instructions.

  * I am using I2C ESCs (Holger, X3D, YGE) and the motors don't start: Make sure you ground the signal lead of the ESC's PPM connection otherwise the electrical noise may cause the ESC to believe it is talking to a PPM signal rather than a digital I2C signal.

  * My motors are making a periodic beeping sound: You may have loaded the Debug SENSORS version of the UAVX and not disconnected your motors. This version of UAVX does not generate signals to control the motors and so your ESCs are not detecting a valid signal. You may have the wrong ESC selected in your configuration.

### Compass ###
  * Is a Compass required: It is possible to fly a quadrocopter manually without a compass. You must have a compass for GPS navigation.

  * Do I need to calibrate the Compass: You must calibrate the compass using Tools->Test Software->K. Follow the instructions carefully.

  * Compass orientation: The software expects the compass orientation to be the same as in the manual. A correction is made by the software such that a heading of zero is obtained when the forward arm points True North. The Compass may be mounted elsewhere on the aircraft as it is the orientation that is important. The orientation was not important for non-GPS versions which used changes in direction not the absolute direction. Be aware that the North arrow on the Sparkfun boards may be incorrect!

  * Are there any adjustments for setting North: The Magnetic variation must be set correctly using UAVPSet. Look it up for your location. **Check that the front motor arm is pointing to True North** when the heading reads zero using Tools->Test Software->C (Compass test).

  * The Compass seems to be offset by 180deg: Some earlier compasses had an offset of 90deg instead of the 270deg for more recent ones. Versions from 1.1045 and up have a setting in the UAVPset instead of separate firmware to correct for this offset. The values are 1=90, 2=180, 3=270.  This only needs to be set once.  The setting can be viewed by running the testfirmware and selecting the Tools/Testfirmware/Setup.  Doing the compass calibration and a compass check of all cardinal directions with North off the nose will tell you that the offset you have set is correct.  Most newer HMC6352 compass' sold have a 270 degree offset and that is the default.

### Barometer ###
  * Is a Barometers required: It is possible to fly a quadrocopter manually without a barometer but you will not have any automatic hover hold. You must have a barometer for GPS navigation as it is required for altitude hold(see below).

  * The altitude seems to drift a lot when I look at it in Tools->Test Software->H barometer test: If it is cold then the barometer electronics, and other electronics including the gyros,  may take several minutes to reach a reasonably stable operating temperature. The origin altitude is recomputed just before flight so the main thing is to see a stable altitude in the barometer test when you do several readings.

  * What is the recommended Barometer: The BMP085 is recommended but UAVX can use the SMD500. UAVX automatically senses which barometer you are using.

  * Do I need to calibrate the Barometer: No this is done automatically each flight.

  * Does wind or light effect the Barometer: Yes it does. You need to wrap the Barometer in foam which does not have closed cells (the air has to get through not surprisingly) or use some other light and wind masking arrangement of your choosing. You must avoid strong light falling directly on the sensor inlet hole or water droplets from rain blocking this hole.

  * When the barometer is holding altitude, the onboard LEDs will flash in sequence.  You will also hear the beeper make a scratchy sound.  You can also hook up the AUX LEDs to the appropriate K13 pins to use as additional LED hover indicators.  Pin8 is positive(anode) and pins 1-7 are negative(cathode). To use this you must use an appropriate ballast resistor for the LEDs type and color you choose.  Caution: Do not exceed 500ma total!

### Accelerometers ###
  * Is an Accelerometer required: While it is just possible to fly a quadrocopter without accelerometers you must have them for GPS navigation.

  * From UAVX v1.1247 onward the aircraft will not arm unless the accelerometer is ONLINE. The LED indication for failed Acc is both Yellow and Red LEDs flashing.

  * Do I need to calibrate the Accelerometers: YES and it is **EXTREMELY** important to do this correctly as the quadrocopter's ability to achieve and maintain level flight depends **ABSOLUTELY** on your accuracy. Make sure the the motor axes are parallel (all pointing in the same direction) and that the quadrocoptor is **perfectly level** (at right angles to the motor shafts which is the plane or planes of the propellers) because this this is the way it will fly relative to the Center of the Earth. Use a spirit level to ensure this and once it is level make sure you **do not move the quadrocopter**. Select the Accelerometer Neutrals Icon in UAVPset and then click the **Set** button. The numbers will then appear in the relevant offset boxes in UAVPSet. Do NOT change the vertical axis offset to zero as was the case for UAVP. Do the initial calibration of the neutral testing the values at least 3 ties to make sure they are not varying more than a point or two due to some movement of the quad.  You should only need to do this initial accelerometer calibration once as the offsets will not change unless you crash or move the accelerometer board relative to the rest of the quadrocopter. Make a note of the offsets so you can restore them should you accidentally overwrite the values in the field. Note that from UAVX V1.1247 and up the UAVX will not arm unless the accelerometer is on-line and with no faults.

  * It is recommended to do the initial accelerometer calibration by placing the model in a large bowl that has been first checked and shimmed for level edges and then resting the models arms on the edges of that bowl and then calibrating the accelerometer. Using just your landing gear during the accelerometer calibration might not be perfectly level with the prop plane and it will skew level off a bit causing drift.  It is the prop plane is what you are actually trying to level with the accelerometer calibration. Turn the model on the bowl and test the accelerometer calibration numbers several times to make sure they remain within 1-2 points of being the same before writing them to the UAVX. This shows that it is properly leveled.

  * Generally the pitch and roll "I" will be most effective for AVP at about half the "P" value.   So a quick test is if you can get it to level nicely with 10-20 seconds of hands off flight with zeros in the pitch and roll "I". After that then add the pitch and roll "I" value back in after properly calibrating the accelerometer. That should get you a good self-level where you should be able to hold it in place easily with very minor stick inputs. A blip of the sticks with the correct amount of pitch and roll "I" will cause it to immediately come back to level. Higher angles than 10 degrees will take up to several seconds to come back to level unless you limit your rates on those channels. This is so the accelerometer influence does not adversely affect fast forward flight. So with the proper neutral stability and setup you should have longer hands off performance needing only minor stick corrections without GPS.

  * Do the Accelerometer offsets need to be zero: No but you should mechanically align the Accelerometer as close as possible to achieving this. The pitch accelerometer axis can be gently tilted in hair like movements forward for decreasing and back for increasing the test number to achieve a zero reading.  Then it is suggested to permanently bridge the accelerometer to keep it from vibrating.

  * Is fastening the Accelerometer firmly important: The Accelerometer needs to be fastened firmly (glued or mechanically fixed) to the main board so that it does not move. If you do use glue make it is temperature stable as if it expands or contracts then the offsets may change.

### Rangefinder ###

  * How do I connect the rangefinder: The rangefinder uses Pin 5 on the PIC. Pin 5 may be connected to a 3.3v-5v supply on the original UAVP boards sold prior to 2009.  On the boards www.quadroufo.com has sold since 2009, there is a rangefinder connector at K18 which on the older boards was for the IDG300 gyro.  Simply remove the jumper and connect the rangefinder positive, negative and signal wires to the proper pins: pin 1, Inboard is ground; pin 2 is +5v; pin 3 closest to the K18 marking is rangefinder signal input.  For the original older UAVP boards simply cut the track and connect the signal line of the rangefinder to the pin.  See the WIKI UAVXRangefindersetup for more information: http://code.google.com/p/uavp-mods/wiki/UAVXRangefindersetup

  * What if do not want to connect the rangefinder: If you have made the wiring modification but wish to remove the rangefinder you need to connect Pin 5 to 5V.  For the flight controller boards sold after 2009, simply attach the jumper to the 2 pins closest to the K18 marking.

  * How does UAVX know to use the rangefinder: The signal on Pin 5 is checked when UAVX first starts. Connecting Pin 5 to 3.3V or higher tells UAVX that there is no rangefinder as the voltage should be much lower when the aircraft is on the ground.

  * When is the rangefinder being used: UAVX detects that the rangefinder reading is within range when the aircraft falls below the best range of the rangefinder and uses that altitude value instead.  There is some overlap to allow UAVX to decide when to switch back to the barometer for altitude readings. The Rangefinder operates from zero to about 20ft/3M.

### GPS ###
**Warning: Do not use GPS position hold or return to home while flying indoors when flying multicopters! For the UAVX when flying indoors, turn the ch7 GPS sensitivity knob to below 20% to turn off the GPS.  The GPS signal indoors is not reliable without an expensive GPS signal repeater. Using GPS indoors can result in random position data that can cause your model to move suddenly and unexpectedly. Indoor flight use of GPS is not approved and can be dangerous!**

  * Should I connect both the GPS Rx and Tx line: No you should connect only the GPS Tx signal. If you connected the GPS Rx line the signals that UAVX sends to UAVPSet will also be received by the GPS and may change the GPS settings.

  * What Settings should I use with my GPS: Your GPS should preferably have the ability to update at 5Hz. Lower update rates will work but not as well. If you use only $GPGGA then you may use the 5Hz update rate. If you are using $GPGGA, $GPRMC and $GPGSV set the update at 3-4Hz maximum because of the additional processing load; yes it takes processing time to throw away stuff! MiniGPS download: http://www.dpcav.com/data_sheets/gps_mini.zip Read [GpsNavigation](http://code.google.com/p/uavp-mods/wiki/GpsNavigation) and [Programming GPS's 101](http://www.rcgroups.com/forums/showpost.php?p=13927790&postcount=1070).

http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a3536269-132-minigps_uavx_setup_jesolins.png?d=1287141967

  * Can I connect my GPS directly to the UAVP Board: The PIC requires 5V TTL signaling and so you will need a 3.3V to 5V level converter for most GPS units which use 3.3V signaling even if the say they are compatible with 5V.

  * Can I use the UAVP board to Connect my GPS to my PC: Yes. You remove the PIC and swap the Tx and Rx leads of the level translator you have used for your GPS. This is a last option as it is better to have a dedicated connection to your PC. There are numerous TTL to USB adapters available.

  * What else do I need to consider: The GPS connection to the quadrocopter must occur at the same time as the quadrocopter is armed and disconnected as the quadrocopter is disarmed. You will need a dual switch for this.

  * The required switch is **Double Pole-Double Throw (DPDT) ON-NONE-ON** (That means a two position switch) Example or similar: http://www.mouser.com/ProductDetail/...2fGbGLJhevQ%3d and http://www.nkkswitches.com/pdf/MtogglesBushing.pdf

  * DPDT ON-NONE-ON: There are two independent SPDT in one switch. It allows true bypass switching (6 pins).  The UAVX requires that the GPS signal be set via the MiniGPS application to 9600bps, GGA, WAAS enabled and 3-5Hz (4hz recommended).  The DPDT switch disarms the UAVX by shorting K8 pins 1 and 2 or 1 and 3 (pins 2 and 3 share continuity)which allows the UAVPset to communicate at 38400bps.

http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a3580453-31-UAVX%20DPST%20switch%20jesolins.jpg?d=1289087212

  * UAVP board GPS pins:
http://static.rcgroups.com/forums/attachments/1/4/0/5/4/1/a2740282-155-UAVX%20GPS%20connections%20for%20flight%20jesolins.jpg?d=1252275212

  * More info on wiring it with GPS here: http://www.rcgroups.com/forums/showt...6#post12936706 UAVX GPS Wiring 101] and [UAVX GPS programming 101](http://www.rcgroups.com/forums/showpost.php?p=13927790&postcount=1070)

  * **GPS Initial Flight Testing:** Please start out slowly!  First test the GPS using the simulator hex and UAVXNav.  When you are totally familiar with the functions and features proceed to an outdoor hand test to make sure it tilts in the proper directions when position hold and RTH and waypoints if used are engaged. The ch7 POTI must be above 10% for altitude hold and above 20% to engage GPS. To initially test your GPS do a walk-around hand test using below hover power and see how it responds with your ch7 sensitivity set between 12 and 1 O'Clock as you are walking around with it after it takes its first home fix.  Note: You must apply some hover power after the first GPS fix and wait about 15 seconds for the navigation feature to activate.  You will hear a confirming beep.  Vary your distance about 10-20M from the home position.  Do be very careful with the spinning props!  Do the same with the RTH and waypoints to gain confidence.  For hand testing the RTH and waypoints use the upper left while disarmed stick movement to disable the altitude climb feature for RTH.  I always recommend using an old lipo hooked up to the UAVX when you get to your flying area while it is in open space for 5-15 minutes to get a good initial GPS fix.  This will also give the GPS time to update its satellite ephemeris data http://www.how-gps-works.com/glossary/ephemeris-data.shtml if it has not been powered on within a few days. Of course then put on a new lipo after that to begin flying.


  * **Note:** The GPS from www.quadroufo.com does not require a level converter and has provisions for USB serial TTL communication to a PC.  See here for more info: http://www.quadroufo.com/download/gps-uavx.zip You must also install a 3v battery backup for changes to the GPS setup to hold.  Use the MiniGPS application to setup the GPS. Here: http://www.dpcav.com/data_sheets/gps_mini.zip

### PC Cable ###
  * A simple 3-wire PC serial cable connected to the flight controller K7 pins is required to communicate with the UAVX and the UAVPset, UAVXGS and UAVXNav.

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_DB9_cable_info_jesolins.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_DB9_cable_info_jesolins.jpg)

  * A cable can be made by the user or by special request from www.quadroufo.com

  * Most PC's do not have a serial communications port.  An inexpensive USB to serial adapter is suggested: http://www.rcgroups.com/forums/showpost.php?p=16881303&postcount=4978

  * Make sure your PC's serial port is properly setup: http://www.rcgroups.com/forums/showpost.php?p=16881303&postcount=4978

  * You can test you cable by doing a loopback test: http://www.rcgroups.com/forums/showthread.php?t=1093510&page=433#post17890496

  * This can also be done wirelessly.  See the WIKI for some wireless setup possibilities. http://code.google.com/p/uavp-mods/wiki/WirelessSerial


### UAVPset General ###

**Note 1:** UAVPset v4.xx supports UAVX v1.xxxx.  UAVPset v6.xx supports UAVX v2.xxxx.

**Note 2:** The instructions for the Rx channel setup for UAVX V2 using UAVPset v6.xx is in the WIKI: http://code.google.com/p/uavp-mods/wiki/TxRxChannelAssignments

  * Check your PC's baud rate for the com port you are using for the UAVPset. If it is at the default of 9600bps for that com port, then set it to 115.2Kbps. Go to: My Computer/Properties/Hardware/Device Manager/Ports, then "your com device name"/Port settings and set Bits per second at 115200, 8, none, 1, and flow control none. This is your PC's com port setting. Note: If your com port is set to 9600bps there will be connection and parameter write problems. Only set the UAVPset com port to the one your PC is using and leave the rest of the UAVPset com port settings at their defaults. See the PC cable build links in the RCGroups UAVX #2 post:

  * Download and install the latest UAVPset and UAVX firmware from: http://code.google.com/p/uavp-mods/downloads/list. The Beta firmware has been flight tested.  The Alpha UAVX firmware is in the process of being flight tested and may contain bugs so if interested in helping with flight testing, then take care!

  * Download the latest Beta UAVX v1.xxxx and save to a directory you can find again. Unzip the files. Then unzip the folder with the 16Mhz firmware. Using the latest UAVPset v4.xx. Either press the burn icon, or go to the menu Commands/Burn PIC and load the test quad hex. Use the hex file with 16 in its name if you have not changed the UAVP board's original 16MHz crystal and 40 if you changed the board's crystal to a 10MHz.

  * You must first load the testfirmware and use the UAVPset to do the initial setup. Make absolutely sure you select the firmware for your set that you will be loading. Pay attention to the number 16 and 40 in the firmware name. The "16"Mhz is the firmware for a board that has not had the original 16Mhz crystal replaced with a 10MHz crystal which is necessary to use the "40"Mhz firmware. So, for example, if you are loading a quad firmware and will use a 7 channel Tx/Rx, you will load: UAVX-V1.xxxgke-18F2620-16-TEST.hex for the test firmware and UAVX-V1.xxxgke-18F2620-16-QUAD.hex for the flight firmware. Then open UAVPset and the menu Tools/Testfirmware/Rx test to get all the channels to pass with low or no glitch count. You must first set your Tx/Rx set type and mode and Rx to either "odd channels" for connecting the odd Rx channels per the order in the info window of the UAVPset for your model of Tx/Rx, or use the "All channels" for a Rx that outputs a standard PPM composite signal or a PPM adapter board that can do so.  With a PPM adapter board, you might have to rearrange the signal pins from th Rx to the adapter board to get the UAVPset Tx/Rx test graphic to move the correct sliders for the proper channel order for yours Tx/Rx.  After you select the correct values make sure you do a write with the UAVPset. Follow the instructions in the Tx/Rx setup to complete the initial setup.

  * After loading the testfirmware or a updated flight firmware, go to the UAVPset/Tools/Testfirmware and do a "Shift D" to load the default parameters.  Then do a read to check parm 1 values and then put the Tx pitch and roll stick to the bottom right to select parm 2 and do a read to check that those parameters are all populated.


  * **Notes**
    * The flight controller board requires 12v power to the board and at least the K1 ESC hooked to it and also powered.
    * When does UAVX respond to UAVPSet commands: Unlike UAVP, UAVX talks to UAVPSet **ONLY** when the quadrocopter is disarmed.
    * The procedure to change values in the UAVPset is to first select the parm table you want to change with the Tx sticks, then do a read on the UAVPset, make your changes, do a write, then again a read to confirm the changes.
    * Do I need to do a Write after I change any of the box values in UAVPSet: Yes.
    * To go to parm 2 you must use the pitch/roll stick to select it first, use either upper right for RTH altitude off, or lower left for RTH altitude on, then do a read.
    * Always do a read after you do a write for parameter changes.
    * Are there any Test Tools: Use Tools->Test Software. Their purpose is fairly obvious and as we seem to act like engineers (usually a good thing) and learning by doing, without reading, there seems little point to writing a description of what each does. Play with each test as they are all harmless except Compass Calibration. The tools are also accessible using HyperTerm (@38.4KBaud) which is required for the GPS test.
    * How do I know if UAVX understands what is connected and whether it is working: Use the Tools->Test Software->S command and check the setup information displayed.
    * The arrangement of the screen seems incorrect for my language: You will need to use the English version until the other translations are completed. Use the Configuration->Miscellaneous pulldown and select English under language.
    * I am using a Notebook and UAVPSet does not fit on the screen: You may need to change the screen resolution and pan so the UAVPSet window fits.
    * For the ASUS eee set the display to 1024x768.

### File Versions ###
**Note:** For UAVX v1.xxxx use UAVPset v4.xx. For UAVX v2.xxxx use UAVPset v6.xx.

**Which hex file do I use: Tags within the UAVX hex file names tells you what it contains:**

  * 16: 16Mhz firmware for the original boards with a 16Mhz crystal installed.
  * 40: 40Mhz firmware for boards that have replaced the 16Mhz crystal with a 10MHz crystal. Using this firmware requires a PIC programmer for the first time load. **Warning: Do not install this 40Mhz firmware if you have the 16MHz crystal.  If you do you will then need a separate PIC programmer to reprogram it for the 16MHz firmware!**
  * QUAD: four motor quadrocopter.
  * TRI: three motor tricopter.
  * VT: four motor tricopter with two V apposed steering motors.
  * HELI: for helicopter IMU usage.
  * 6CH: for use with 6ch Rx. The navigation sensitivity (below) must be set using UAVPSet rather than the Ch7 pot.
  * TEST: for UAVX v1.1049 and above to load for the initial setup for Tx/Rx and compass and any follow on testing. Also for use with the sensor trace (see SensorTest Wiki) to debug sensors and for getting graphs of the sensor behavior. The motors are disabled for this version for safety reasons but you can get plots of the computed motor throttle values and your stick inputs as well as the main sensor values.
  * SIMULATOR: from R9xx onwards (See UAVXSimulator Wiki).

### ESC Setup ###
**WARNING: Do not power the Rx separately using one of the motor ESC's.  Always power it from the filtered power from the flight controller board, or if desired a separate lipo that is grounded to the board, or a separate BEC.  An ESC BEC improperly used that directly powers the Rx can have negative effects on the Rx and board's performance and decrease PPM signal quality and model safety.**

**Note 1:** This checklist only needs to be accomplished for new ESC's
during the initial setup.

**Note 2:** Some ESC's do not have the ability to calibrate the throttle
range.  Some ESC's automatically calibrate the throttle range.  It is
not recommended to use the ESC's that automatically adjust the throttle
range as this is not compatible with multicopter boards.

**Note 3:** Many of these ESC's can be modified for I2c communications or fast PWM
which the UAVP board can also use if desired.  If you have the skills
and are interested in doing this please see the ESC conversion
paragraph in the [Quadrocopter and Tricopter Mega Link Index](http://www.rcgroups.com/forums/showthread.php?t=1097355)

**Note 4:** The flight controller board requires 12v power to the board and at least the K1 ESC hooked to it and also powered.



**1. Recommended ESC's:**

  * While the original quadrocopters used ESCs around 10A. UAVX and other similar controllers update the throttle setting 4 or more times faster than with a regular aircraft. This can lead to peak currents much higher than the average motor current.  As there is very little cost or weight penalty it is worth considering higher current ESCs up to 30A even if your motors will only draw under 10A in hover.
  * [Quadroufo.com](http://www.quadroufo.com) Standard 30A ESCs (note that if you requested from Quadroufo to have the fast PWM modified firmware installed on the ESC's, then they do not require throttle calibration)
  * HKSS
  * Holger I2c
  * Mystery
  * RCTimer
  * TowerPro
  * Turnigy Plush
  * X-BLC I2c
  * YGE for I2c or PWM
  * Modified standard to I2c and to fast PWM ESCs: http://www.rcgroups.com/forums/showthread.php?t=766589

**2. Program the 4 ESCs with these settings if available:**

  * Brake OFF
  * Battery type: NiMh (yes even if you use LiPo)
  * Startup: Normal
  * Cut Off Type: soft cut
  * Cut Off Voltage: Low (or even better turn the ESC LVC completely off is that option is available)
  * Timing Mode: Low to Middle
  * Music: your preference if available
  * Governor Mode: Off

**3. Calibrate each ESC for your radio:**

  * See Note 2 above
  * Disconnect all ESCs servo cables from the board
  * Connect ESCs one after the other directly to Rx you will be using throttle channel
  * Move Tx throttle stick to the full up position
  * Turn TX on
  * Power the ESC wait for initialization beeps + 2 beeps
  * After 2 beeps, move throttle stick to minimum
  * Wait for beeps (generally 3 + 1 or 1 + 1)
  * Unpower ESC and disconnect
  * Repeat until all 4 are calibrated and you're done

  * ESC servo connections: (see Board Diagram above and note the S, +, - connections)
    * K1 ESC front motor
    * K2 ESC left motor
    * K3 ESC right motor
    * K4 ESC rear Motor
    * K5 camera gyro stabilized servo pitch
    * K6 camera gyro stabilized servo roll

**4. I2c ESC Setup:**

**Note 1:** This paragraph is only important if you plan to use I²C ESC's.

**Note 2:** The I²C controllers do not work with a PPM pulse length, but with a I²C data interface. All four controllers must be connected to one bus, which is comprised of two servo outputs (K2 and K3). Using I2c ESCs will provide faster ESC responses and give better stability performance on multicopters that are smaller than approximately 30cm in diameter.

**Note 3:** Some I²C controllers do not offer a 5 volts BEC source. The quadroufo flight controllers since 2009 include a separate onboard 5v power source.  All older flight controller boards must have a separate BEC added by the user if the ESC, such as the Holger ESCs, do not provide one.

  * To enable communication on the I²C bus, a clock and a data line is needed. The data line is pin 1 of connector K2, the clock line is pin1 of K3. To make it work properly, two additional pull-up 4k7 SMT 0805 size resistors must be mounted to the traces on the pads near the PIC pin22 and pin23. These are located on the bottom of the UAVX board sold by quadroufo since 2009 and are approximately between the markings of [R26](https://code.google.com/p/uavp-mods/source/detail?r=26) and C1.  For the UAVP boards older than 2009, please see the original UAVP manual in the UAVX Google Code downloads area.

  * To set the I²C addresses, use the UAVX testfirmware and the UAVPset/Tools/Test software/I2c setup tool.

  * Select I2c ESCs in the UAVPset and write the values to the UAVX.



### Tx/Rx Setup ###

**WARNING:**  **As it was with the UAVP, a proper Tx/Rx setup is extremely important for both safety and proper flight operation.  Always setup your Tx/Rx with the quadrocopter tied or weighted down and or props removed or motors disabled.  It is strongly advised that you remove props or disable the motors by removing one or two of the three wires before proceeding with the Tx/Rx setup!**

**WARNING: Do not power the Rx separately using one of the motor ESC's.  Always power it from the filtered power from the flight controller board, or if desired a separate lipo that is grounded to the board, or a separate BEC.  An ESC BEC improperly used that directly powers the Rx can have negative effects on the Rx and board's performance and decrease PPM signal quality and model safety.**

**WARNING: It is a requirement that your particular Rx's failsafe is properly set up so that the UAVX will remain safe (See UAVXFailsafes Wiki).  If the Rx's failsafe is not properly set up then on loss of signal the multicopter could potentially and dangerously go to full throttle.**

**Note:** If using a 6ch Tx, and only the odd channels 1, 3 and 5 on your Rx, then you should ground the remaining UAVX Rx wire to an Rx ground pin.

The setup procedure:

  * First make sure that the UAVP is not armed! Set your Tx to its default values to start fresh.  Set your Tx back to the defaults by using the Tx menu to do that to make sure you do not have any wrong mixing, trim, subtrim, exponentials or EPAs set to start your setup per the UAVPstartup instructions.  You must use ACRO mode on your Tx.  Set your trims and sub-trims to zero, make sure you have no mixing or throttle curves set.  Using UAVPset V3.0 or above pull down menus select the correct Tx/Rx combination and Rx to Quad and TX mode for your kit.  Write this to the UAVP. Note that for UAVX firmware v1.1049 and above, you must first load the correct TEST firmware for your model and do the initial compass and Tx/Rx setup and make sure all your Rx channels pass the test.  See above in the **File Versions** paragraph for help in identifying the proper firmware you must use.

  * Go to the Tool->Test software and do the Rx test and see that all channels pass.  If not then the Tx EPAs (End Point Adjustment) must be adjusted here first.  Note that the Rx Test displays the channels in the order they are received by UAVX. Each channel is marked with the control function corresponding to the Tx/Rx combination you have selected. You should expect the channels corresponding to pitch, roll and yaw to have a value of approximately 0%. If you do not see this pattern then you may have the wrong Tx/Rx combination selected. For most cases, simply setting channel 5 lower EPA to 80% and upper EPA to 75% will get a pass on all channels that the Tx/Rx combination allows, i.e., either 5 or 7.  The next channels to adjust if you still do not get a pass are setting channels 6 and 7 to 75% upper and lower.  Note that the first indication of all channels passing while you adjust your EPA's will be the illumination of the UAVP board's green LED.  If you still cannot get the channels to pass, then post in the forum for additional help. Here: http://www.rcgroups.com/forums/showthread.php?t=1093510

  * After you get the channels to pass, go to the Tx/Rx graphic. Now set the channels which are represented in the graphic as "Roll, Pitch, Yaw" for EPA's of about 90 upper and -90 for the lower values **_using the values on the graphic_**. Try to make the EPA for "CamTrim" symmetrical around 0% as it is the Flap trim on some Txs and may have a limited range. On some Tx's you can extend the ch6 range in the Tx menu in order to get a full range for ch6.  Set channels "Throttle, NavS" for lower EPA of 4  and upper EPA of 95. Set "RTH" for lower 10 and upper 90 that you set up using the EPA's. Now set your sub-trims to make the graphic read 4% for "Throttle,  and 5% for NavS" and 0% for channels "Roll, Pitch, Yaw, CamTrim". **NOTE**: The RTH, CamTrim and NavS - correspond to Ch 5,6 and 7 in the UAVPSet.

  * When setup correctly your NEUTRAL, LOWER and UPPER UAVPset Tx/Rx graphic should look like the following captures +/- a value of 1:

  * **NEUTRAL**
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_neutral_TxRx_graphic_jesolins.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_neutral_TxRx_graphic_jesolins.jpg)
  * **LOW**
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_low_TxRx_graphic_jesolins.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_low_TxRx_graphic_jesolins.jpg)
  * **HIGH**
![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_high_TxRx_graphic_jesolins.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVX_high_TxRx_graphic_jesolins.jpg)

  * Test your set still having the quadrocopter disarmed then cycling the sticks and channel 5, 6 and 7 pots/switches through the full combination of ranges.  The graphic should respond correctly and not freeze or disappear.  If it does, then the EPA's and sub-trims are still not correct, so go back and check the values. Adjust if necessary up or down 2-5 on upper and lower and or sub-trims to see if you can get the graphic to respond correctly as these values could vary a bit with different models of Tx/Rx.

  * Click on your Rx graphic and do the following checks:
    * Throttle: slider is full down.
    * Roll: When given Tx roll stick left input the slider goes down (-)
    * Pitch: When given Tx forward stick input the slider goes down (-)
    * Yaw: When given Tx yaw right stick input the slider goes down (-)
    * RTH/PH: When the Tx switch is away from you the slider goes down (-PH\*must be in PH before you will be allowed to arm!)
    * Cam/POF: When Tx slider or variable pot is ccw the slider goes down (-)
    * NavS: When Tx variable pot is ccw the slider goes down (-baro/GPS off)

  * If any Tx stick or switch movements are not as above, then make them so with your Tx channel reverse setting. Re-check all the channels upper and lower EPAs after you have done this and make any corrections as required.

**Caution! After properly setting up the Tx/Rx if any changes are made to the EPAs, Subtrims, channel reversal, or adding a throttle curve using Tx mixing always do a check of the Tx/Rx graphic and do a stir the sticks, switches and pots test before flying to make sure the board's green LED stays on!**

**Caution! Insure the servo connectors are on the board in the correct order and on the correct pins. The signal pin is on the inside of the board and the ground is to the outside of the board. If these are wrong the model will flip!**


  * When you get the graphic to respond, now it the time to very carefully check the Tx/Rx and motor responses.  Either weight or tie or have someone tightly hold down the quad.  Do the first check with the motor wires disconnected or the props off!  Arm the quadrocopter. Move all the sticks and switches through their ranges and confirm that the Green LED remains on.  **Do not run the throttle at max for any more than a split-second as damage to the ESC's or motor's could result!**  If you got this far, then the UAVX is properly setup for the Tx/Rx.

  * If your Tx/Rx supports it you should consider changing the Pitch and Roll stick sensitivity/rates on your Tx (not UAVX) to around 70% (or less) with an exponential to -40%. This will make the quadrocopter much more controllable. The Yaw Tx control is usually fine for most without any Tx exponential or rate adjustments. You may wish to adjust the Tx exponential or rate to increase the yaw speed with positive values.

  * Now that you have gotten this far with the testfirmware it is time to load the flight firmware version you will need for your particular model.  See "File Versions" definitions above.  Since you now have already with the previous testfirmware setup procedures loaded your "Shift D" initial values and setup your Tx, Rx and ESCs, compass and accelerometer, you will only have to double-check that all the values on both parameter tables are still proper and in-place.  Keep safety in mind!  With the flight firmware installed, the motors can start if the throttle is up.  Take care and leave the props off until you are satisfied that the Tx/Rx is properly setup by doing yet another Tx/Rx graphic check and a good SSSPT check to make sure green LED always stays on.

  * **Set your Rx failsafe!**  The UAVX has a failsafe built-in that will go to hover throttle and then land if the Rx signals are lost for 2 seconds when GPS is not installed.  If GPS is installed and Rx signal loss is detected, then at 6 seconds it will RTH and auto-land.  Most modern Rx's have their own failsafe.  If possible, then disable the Rx failsafe so that the UAVX can use its built-in failsafe.  If you can not disable the Rx failsafe, then set the Rx failsafe's throttle to just below what it would take to hover with a full lipo and set ch5 to the high/RTH position.  You should then test the Rx failsafe by holding the model down securely while hooked up to the UAVPset with the Tx/Rx graphic displayed. With the UAVX disarmed, turn off your Tx and note how the Tx/Rx graphic sliders change.  What you want to see is the throttle going to just below hover power and the ch5 going to the high/RTH position with the pitch, roll and yaw going to the neutral positions.  Some adjust your particular Rx failsafe positions must be done for this to work properly and safely!  If the Rx failsafe inputs bad pitch and roll or yaw angles on loss of Tx signals, then the outcome would not be successful.  See your Rx instructions on how to set its failsafe and make sure that it is done correctly! **Read the UAVXFailsafe Wiki**.

  * Conduct a proper range test with the quadrocopter oriented in all quadrants before flight. **Fly safe!**

### Stick Based Configuration ###

When the quadrocopter is disarmed the Tx sticks may be used to access some parameters in the field without the use of a PC. It is important that the Tx Mode checkbox is set correctly for your Tx with UAVPSet. The Tx Mode is ONLY used by the stick programming function and **does not** swap the controls about on your Tx.

  * Pitch Stick: The "corners" are used with left side being parameter set 1 and right side 2. If the bottom corners are used RTH altitude hold is enabled.  If the top corners are used then the Pilot controls RTH altitude. You should see the Blue LED flash and hear short beeps for the parameter set chosen followed by a slightly longer beep if RTH altitude hold is enabled.

  * Throttle Stick: Only the bottom corners are used. Bottom left disables Turn to Heading (TTH) and so the Quadrocopter uses a combination of pitch and roll to move toward the Origin; Bottom right enables TTH where the Quadrocopter turns and pitches directly toward the origin.

You will only hear beeps if the setting you have selected is different from the previous setting.

RTH altitude hold is **enabled** and TTH **disabled** when you power up the quad. These are the safest values for use with Rx failsafes. Unlike the main parameter sets any changes you make through stick programming are not stored permanently. They ARE retained if you arm/disarm but are lost when you disconnect power.

TTH is not active within the closing distance, or radius, of the Origin nor is it active when simply holding a position. The Pilot still has control of a yaw offset and thus can control the direction of camera gaze regardless of whether TTH is active or not although if it is active then there will be a small "resistance" to yaw commands.

Tx (Mode 2) labeled with Stick and channel options:

![http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a4097162-82-UAVX%20Tx%20Stick%20Selections_1.161%20and%20higher%20jesolins.jpg](http://static.rcgroups.net/forums/attachments/1/4/0/5/4/1/a4097162-82-UAVX%20Tx%20Stick%20Selections_1.161%20and%20higher%20jesolins.jpg)

### Pre-Flight Safety Checklist ###
  * Please review the checklist found in our WIKI and note all warnings prior to flight: [UAVX Flight Checklist](http://code.google.com/p/uavp-mods/wiki/UAVPandUAVXFlightChecklist)

### Stage 1: without GPS ###

Most of this will be familiar to you as the procedure is nearly identical to that you will have used with UAVP. If you have a GPS DISCONNECT it.

You must have installed the requisite 18F2620 PIC with associated bootloader.

Using UAVPSet V3.0 or later:
  * Load the flight software into your quadrocopter (there are many fewer HEX files to choose from and the one you need should be self evident). See the **File Versions** paragraph to help you identify the correct hex file to load for your model.

  * Select UAVX in the pulldown.

  * Initialize your flight parameters from a saved UAVP V3.15 file or using the Tools->Test software->D command in UAVPSet. If you choose to use the D command you need to do a read params to update the UAVPSet screen. See the FAQs above.

  * Select the Tx/Rx, Gyro and ESCs you are using from the appropriate pull-downs and write the config to parameter set #1. Be aware that the Gear switch is no longer used to switch between parameter set 1 & 2. This is done using stick programming (see the  gpsNavigation page ) once UAVX is successfully talking to your Tx/Rx.

  * Just as for UAVP use Tools->Test software to ensure your Rx signals are being received correctly. Verify this in the Rx Graphic which now displays in % units. **Read and accomplish the TX/RX setup instructions above.**

  * Save your parameters to a file. UAVX will accept V3.15 parameter files but files written by UAVX are not compatible with V3.15.

  * Initialize the accelerometer neutrals as for UAVP.

  * Set up parameter set 2. You will see that many of the parameters are not displayed on the Parameter Set 2 page as these are now common to both parameter sets.

  * While you steady your nerves go into the Tools menu and check out some of the many tests available because at about this point you will realize you have not calibrated your compass, checked your baro etc.! The now inbuilt test software has some sanity checks on voltages etc and may detect problems with sensors. In particular look at the Setup display which now has an extended summary of what UAVX "thinks" you have connected in the way of sensors etc. and whether they may be working or not.

  * Arm the quadrocopter and observe that the Red LED will blink several times at about one second intervals as the Gyros are initialized - this is the first difference from UAVP. The quadrocopter MUST NOT be moved while the Red LED is blinking otherwise you may have a large drift offset! The quadrocopter does not need to be level - just stationary. Once the gyros are initialized you will should hear 3 short beeps and one long beep at which point the throttle is finally armed.

  * Ensure that the response of the quadrocopter is acceptable using the normal pre-flight checks. You will see that UAVPSet no longer communicates with UAVX once the quadrocopter is armed. If you need to modify parameters you will need to disarm.

  * Fly (at your own risk as always) and tune the various flight parameters as required.

### Stage 2: with GPS ###

This is new territory and you will need to do some homework for your location particularly the Magnetic Variation. Read the GPS Navigation page carefully.

**Warning: Do not use GPS position hold or return to home while indoors when flying multicopters! For the UAVX when flying indoors, turn the ch7 GPS sensitivity knob to below 20% to turn off the GPS.  The GPS signal indoors is not reliable without an expensive GPS signal repeater. Using GPS indoors can result in random position data that can cause your model to move suddenly and unexpectedly. Indoor flight use of GPS is not approved and can be dangerous!**

**WARNING** - If you are using a 6 channel Rx then you do not have control over the GPS gain (see below) - do not proceed if you are unsure. You may still choose to fly with the GPS disconnected.

Using the UAVX capable version of UAVPSet:
  * Connect your GPS.

  * Set the GPS related parameters (notes below to follow).

  * Turn the Pot/Knob connected to Channel 7 to minimum or below 20%. This will disable GPS support and in firmwares since UAVX v1.1174 when below 10% it will also turn off the baro hover/altitude hold.

  * Select off/minimum using your Gear switch on Channel 5 (this turns RTH off). If you fail to do this a beeper alarm will sound.

  * Disable RTH altitude hold (see stick programming). This means that the normal hover mechanism is used whereby the quadrocopter will assume you are hovering after 3 seconds and use the baro and accelerometer to attempt to maintain altitude.

  * The quadrocopter can return home maintaining its current heading, pitching and/or rolling as required to move toward the Origin, or it can turn toward the Origin using pitch control only. This may also be set with Stick Programming.

  * Arm the quadrocopter and after the initial blinking Red LED as the Gyros are initialized you should see the Blue LED blinking at the update rate of your GPS. At this point the Red LED will stay lit until UAVX is happy with the GPS accuracy. This may take anything up to 5 minutes particularly on the first flight in a new location. Once the Red LED goes out a satisfactory "Origin" location has been recorded. Currently the flight software does not prevent you from taking off before the Red LED goes out but be aware the Origin value may be somewhere (anywhere) along your flight path. Be patient and wait.

  * **GPS Initial Flight Testing:** Please start out slowly!  First test the GPS using the simulator hex and UAVXNav.  When you are totally familiar with the functions and features proceed to an outdoor hand test to make sure it tilts in the proper directions when position hold and RTH and waypoints if used are engaged. The ch7 POTI must be above 10% for altitude hold and above 20% to engage GPS. To initially test your GPS do a walk-around hand test using below hover power and see how it responds with your ch7 sensitivity set between 12 and 1 O'Clock as you are walking around with it after it takes its first home fix.  Note: You must apply some hover power after the first GPS fix and wait about 15 seconds for the navigation feature to activate.  You will hear a confirming beep.  Vary your distance about 10-20M from the home position.  Do be very careful with the spinning props!  Vary your distance about 10-20M from the home position.  Do the same with the RTH and waypoints to gain confidence.  For hand testing the RTH and waypoints use the upper left while disarmed stick movement to disable the altitude climb feature for RTH.  I always recommend using an old lipo hooked up to the UAVX when you get to your flying area while it is in open space for 5-15 minutes to get a good initial GPS fix.  This will also give the GPS time to update its satellite ephemeris data http://www.how-gps-works.com/glossary/ephemeris-data.shtml if it has not been powered on within a few days. Of course then put on a new lipo after that to begin flying.

  * Takeoff and fly to a safe height and away from you 30M/100'. Check to see that you have a good hover. This initial hover is **important** as it and subsequent hovers adaptively establish the hover throttle setting used by the RTH altitude hold function. You can read what hover throttle has been used after a flight by reading the parameters. Other statistics are tracked and may be accessed using Tools->Test Software->X. Please note that the GPS software is not active until 30 seconds after takeoff.

  * Engage RTH using the Gear switch. You should see no change in the behavior of the quadrocopter. It is important that you HAVE disabled the RTH altitude hold (see above) otherwise you will have no throttle control.

  * Slowly increase the setting of the Channel 7 Pot and you should see the quadrocopter start to tilt toward the Origin position (where the GPS location was acquired after arming). You should still have control of pitch, roll, yaw and throttle and can overcome the corrections being applied by the GPS. If Channel 7 is below around 20% GPS assistance is disabled.

  * Try switching off RTH and you should come to a halt at the current position. The GPS at this point is attempting to maintain the current position but again can be overcome using pitch and roll corrections.  There will be a temptation to "help" the GPS using pitch and roll but be aware that if you move either the pitch and roll sticks by more than about 5% then the desired location will be updated to a new position in the direction of what you perceive to be a drift which is not what you want! You may change Yaw without triggering a position update - useful for pointing cameras.

  * Keep playing until you are familiar with what is happening.

### Stage 3: RTH with Altitude Hold ###

Up to this point moving the control sticks will overcome any corrections the GPS may be exerting. This stage is a lot different as you are handing over throttle control completely to the navigation software.

The steps are:
  * Set the Barometer as your Altitude source using UAVPSet.

  * Check the navigation parameters are as recommended in the defaults.

  * Select RTH altitude hold using stick programming.

  * Use the GPS gain setting (Channel 7) you found you were happy with in Stage 2.

  * As in Stage 2 fly to a safe altitude and distance.

  * Engage RTH. At this point you have NO THROTTLE CONTROL AT ALL as this is being used by the RTH altitude hold software. Leave the throttle where it is or increase it by about 10%. Do not close throttle or when you disengage RTH, the quadrocopter will fall like a brick.  If the RTH altitude you have set is much higher than when you engaged RTH then the quadrocopter will climb quite rapidly and your natural instinct will be to close the throttle - don't do it. If all is well you should see the quadrocopter climb/descend to the RTH altitude you have selected and start heading back to the Origin. If you are unhappy disengage RTH.

  * As in Stage 2 keep playing - safely.

### Stage 4: RTH with Altitude Hold and Auto Descend ###

Start with an RTH altitude setting which is reasonably high (say 30M) to allow you time to respond.

If you enable RTH Descend in UAVPSet then thirty seconds after you have arrived within the selected radius of the Origin position with RTH still engaged the quadrocopter should start to descend at around 1 Meter/Sec slowing in the last 15M.  Disengaging RTH will cause the quadrocopter to revert to GPS assisted position hold but you must control the altitude manually. This is the reason that moving the throttle up 10% after engaging RTH may be a good strategy to have as it will arrest the fall if you disengage and hopefully give you enough time to re-establish hover.

The auto descend feature is the logical extension of using RTH primarily as failsafe support. We suggest RTH altitude enabled (the default) be adopted with you Rx failsafes set appropriately to advantage of this.

There are now provisions on the latest UAVX boards and in the firmware and UAVPset to add an ultrasonic rangefinder, a Maxbotix XL-EZ3 to make the auto descent and altitude hovers below 20 feet more precise.

### Camera Pitch and Roll ###

The UAVP also has servo connections for channel 6 and 7 to control gyro assisted pitch and roll.  Channel 6 is for pitch and 7 is for roll.  You can adjust responsiveness by setting values in the fields in the UAVPset.  In the UAVPset under the roll and pitch columns the cam roll adjusts the servo response to the model.  A value of zero is off.  The cam trim adjusts how far it will be allowed to correct for roll.  The cam pitch value adjusts the servo response to pitch.  A value of zero is no camera pitch gyro compensation.  You can still adjust the cam pitch tilt with your ch 6 variable poti.  These values must be initially setup in a hand test with the UAVX in the armed position so take extreme care and disable the motors by either removing the motors servo connections or two of the three motor wires.
Note that the cam servos are powered by the ESC's from K3 for servo power to K6 and K4 for servo power to K5.  Note also that the servo connectors are not numbered sequentially so they are K1, K2, K3, K4, **K6 and K5**.

**Warning: Do not attempt to provide power to the servos from the Rx pins by using an unused channel.  There is not enough power provided by the board's 5v source to power the servos.  A digital servo will have higher power requirements and if it is directly connected to the Rx will cause the board to lose enough power to function and result in a crash. Do not power any servo directly from the Rx if the Rx is powered from the board as it normally is.**

### Optional LED connection ###
For LEDs the K15 is for an always on LED and the K13 connections are duplicates of the LEDs on board and some auxiliary pins.

#### LED K13 connector: ####
  * pin1= Aux2 outside edge
  * pin2= blue repeater
  * pin3= red repeater
  * pin4= green repeater
  * pin5= Aux1
  * pin6= yellow repeater
  * pin7= Aux3
  * pin8= +5v inside edge

If you want to use additional LEDs, you can use connector K13. There are 7 power LED outputs available. You do have to calculate the value for the current-limiting resistor for each LED (check LED nominal current and battery voltage). The positive leads (anodes) of all LEDs are common to Pin 8 of K13. The negative leads (cathodes) of each LED are connected to pins 1 to 7 respectively, the signal running through the individual LED’s current limiter resistors. In addition, any type of LED can be used to denote the "front“ of the quadrocopter. Connect this LED to K15.
A current limiting resistor ([R16](https://code.google.com/p/uavp-mods/source/detail?r=16)) is already installed (100kohms). The LED is fed through the BEC in all of the options, so note that the total current **draw must not exceed 500ma!** If you require more current then go directly either from your lipo with a separate BEC using the proper voltage and current-limiting resistor for your color and type of LED, or direct from one of your ESC BECs.



### Other Stuff ###

There are many other things we can try/abandon including:

  * Ground proximity using a simple switch to kill throttle (its in the software) or an ultrasonic rangefinder. Not useful for tree encounters or landing upside down.

  * Altitude and distance from Origin containment i.e. not too high and not too far.

  * Camera Point of focus

  * Polar steering mode

  * etc....

### Suggested Navigation Parameters ###

Use the UAVPSet Tools->Test software->D defaults for now.

An option is to use Parameter set #1 for still conditions and #2 for windy conditions. We expect the more successful combinations of navigation related parameters to evolve over time. More of the internal software controls are likely to be accessible through UAVPSet as we gain experience.

### WARNING: electrically powered aircraft ###

#### It is very important that you follow the normal safety procedures for electrically powered aircraft with propeller(s). ####
  * You **MUST** check that the radio control link is functioning correctly before you apply power to the motor(s).
  * You **MUST** conduct a radio range check before every flight.
  * The aircraft **MUST** be restrained when power is applied and you **MUST** be clear of the propeller(s) at all times.
#### The motors(s) can start immediately in all electric aircraft when the battery is connected and you should **NEVER** rely on arming switches which are fitted to many aircraft. ####
#### If your **Tx is OFF** or the aircraft is out of radio range PPM receivers can, and do, receive radio noise which they interpret as valid signals; this includes throttle commands. ####
#### PCM receivers with incorrectly set failsafes can also apply **full throttle** or maintain current throttle which may not be zero if the transmitter signal is lost. Make sure that you read and fully understand the UAVX Failsafe WIKI and the proper Tx/Rx setup procedures in this document. ####
#### This does not apply just to UAVP/UAVX and other quadrocopters. It applies to ALL electric aircraft. ####