# Setting up the XL- MaxSonar®- EZ3™ (MB1230) on the UAVX #

(5 February 2011 Jesolins)


The addition of a rangefinder allows much better control of the multicopter’s close to ground hovers and smoother control in the final descend when using the UAVX auto-landing feature. Using GPS as an altitude source alone tends to be unreliable unless the satellite constellation is very good. The barometer can give good altitude resolution, but not a precise as the ultrasonic rangefinder which operates only below about 7M.  Therefore installing a rangefinder gives the UAVX a better auto-land capability than using GPS or the barometer alone.

The rangefinder is capable of measuring 0 to about 7M or 21ft in altitude.  The UAVX code transitions to the barometer for altitude control and measurement at approximately that point.

Although there are several choices of rangefinder to experiment with, we used the Maxbotix XL-EZ3 for its narrower beam width and higher power and resolution.


The range finder is low current draw.
V+
Operates on 3.3V - 5V. The average (and peak) current draw for 3.3V
operation is 2.1mA (50mA peak) and 5V operation is 3.4mA (100mA
peak) respectively. Peak current is used during sonar pulse transmit.

A supply of 5V yields ~4.9mV/cm., and 3.3V yields
~3.2mV/cm. Hardware limits the maximum reported range on this output to ~700 cm at 5V and ~600 cm at 3.3V. The output is buffered and corresponds to the most recent range data.
More Maxbotix rangefinder info:

Datasheet: http://www.maxbotix.com/MB1230__XL-MaxSonar-EZ3.html

FAQ:  http://www.maxbotix.com/MaxSonar-EZ1__FAQ.html

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/EZ3.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/EZ3.png)


Mount the sensor to point straight down.  It is best to mount it near the center of the multicopter to minimize range change altitude calculations as a result of the swinging of the arms.


The original version of the UAVP board requires that a trace that supplies 5v from the PIC pin5 to a via to be cut on the bottom of the FCB:

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/UAVX_PIN5_ON_PIC.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/UAVX_PIN5_ON_PIC.jpg)

This trace must be jumpered/shorted again if you decide to remove the rangefinder and use only the barometer.  The K18 on the original FCB was used for a dual axis IDG gyro.  You can use the ground and 5v Vcc from the via where you cut the trace to power the Maxbotix EZ3.  A wire soldered to the PIC pin #5 is then connected to the rangefinder Analog Pin #5.  See the rangefinder photo below for the correct pins to use from the Maxbotix rangefinder.




The newer board from www.quadroufo.com has the original K18 that was used for the 3v IDG dual axis gyro modified for the rangefinder.


![https://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/UAVP_FCB_new_top_RF_K18_pop_jesolins.jpg](https://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/UAVP_FCB_new_top_RF_K18_pop_jesolins.jpg)

pin 1, Inboard is ground
pin 2 is +5v
pin 3 is rangefinder signal input

You must replace the jumper to short the V+ and S pins if you remove the rangefinder and want to use only the barometer.  See the rangefinder photo below for the correct pins to use from the Maxbotix rangefinder.
The range finder suggested has 7 pins.
pin 1 is open
pin 2 is PW (not used)
pin 3 is Analog to PIC pin5 (connect to new FCB K18 pin3)
pin 4 is RS232 Rx (not used)
pin 5 is Rs232 Tx (not used)
pin 6 is 3.3 or 5V (new FCB K18 pin2)
pin 7 is ground (connect to new FCB K18 pin1)


Use these connections from the EZ3 to the UAVX:

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/EZ3_pins_to_UAVX.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/rangefinder/EZ3_pins_to_UAVX.png)







