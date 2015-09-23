### Introduction ###

You must first load the DEBUG or Testfirmware versions of the software. These versions have been modified so the motors should not start for safety reasons. You are advised however to disable the motors by removing the propellors or disconnecting two motor wires on each motor.

Once you have done this:
  * Launch UAVPSet.
  * Select Tools->Sensor Test Graphic. In the older versions of UAVPSet go to the unnamed (blank) option also in the Tools pulldown menu.
  * ARM the quadrocopter.
  * Click on the Connect/Start menu item.
  * Advance the throttle and the traces should progressively appear. **WARNING:** If the motors start you have loaded the wrong software (see above).

Once you have successfully obtained traces you may modify which traces to display:

  * Stop the trace by selecting STOP in the menu.
  * Select Configuration. In each row of numbers **Input** is the number corresponding to the sensor or other value you wish to observe. **Divisor** is used to scale the trace down if it is too large. **Offset** to position the trace within the particular pane you may find values of 50, 100 and 150 useful. **Bit** should be 16 in all cases.
  * Close the configuration window.
  * Resume as detailed above.

![http://uavp-mods.googlecode.com/files/sensor%20screen%20and%20mapping.jpg](http://uavp-mods.googlecode.com/files/sensor%20screen%20and%20mapping.jpg)

The above picture is a sample of the output that you may see:
  * The first trace is the absolute heading reading from the compass sensor (if installed)
  * The second two relate to the barometer.
  * The middle set of traces are the accelerometer outputs.
  * The bottom set of traces are the gyro outputs.

### Trace numbers ###

UAVX V1.1020 Onwards (all 16 bits)

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

Pre UAVX V1.1020

  * 1  Heading Error
  * 2  Current Baro Pressure
  * 3  Roll Rate
  * 4  Pitch Rate
  * 5  Yaw Error
  * 6  Roll Sum/Angle
  * 7  Pitch Sum/Angle
  * 8  Yaw Sum/Angle
  * 9  LR (X) Acceleration
  * 10 FB (Y) Acceleration
  * 11 DU (Z) Acceleration
  * 12 Roll Correction
  * 13 Pitch Correction
  * 14 Desired Throttle
  * 15 Desired Roll
  * 16 Desired Pitch
  * 17 Desired Yaw
  * 18 Motor Front
  * 19 Motor Back
  * 20 Motor Left
  * 21 Motor Right
  * 22 Cam Roll Servo
  * 23 Cam Pitch Servo

This information was compiled from:

http://www.rcgroups.com/forums/showpost.php?p=10797029&postcount=1387

http://www.rcgroups.com/forums/showpost.php?p=10766063&postcount=1371