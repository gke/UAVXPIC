## Introduction ##

The 40MHz version of UAVX on the 18F2620 is almost as far is it goes for our "Legacy" UAVP board.

Going from 16MHz to 40MHz using a simple 10MHz crystal upgrade at the cost of a dollar or so is the cheapest performance upgrade in town. It allows us to go from reading the sensors and updating the motor commands 125 times a second or 125Hz up to 250Hz. That with a workover of the control code gave much crisper control response.

If you fly UAVX with a span of around 600mm or greater, as most of us do, then 250Hz should be satisfactory for FPV type work.

### 40MHz Versions of UAVX ###

The 40MHz version of UAVX is now the default and has been shown to give much better performance.

### WARNING - READ THIS OR YOU WILL LOSE COMMUNICATION WITH YOUR UAVP BOARD ###

Do not load load versions of UAVX with "40" in the name if you are still using the original 16Mhz crystal. If you do **YOU WILL NOT BE ABLE TO COMMUNICATE WITH UAVX**.  If you make this mistake you will require a PIC programmer to reload A 16Mhz version of UAVX onto the PIC.

You **must** do the first 40Mhz UAVX firmware load with a PIC programmer because this loads a different boot sector code which requires the PIC programmer. So **do not** do the first load of the 40Mhz firmware via the UAVPset with or without the crystal change. This will stop further communication with the PIC until you properly program it initially with a PIC programmer.


### Using the 40MHz Version ###

In order to use the 40Mhz UAVX firmware, you MUST do two things:

  1. remove the 16MHz crystal and replace it with a 10MHz crystal and
  1. use a PIC programmer to do the first load of a 40MHz version onto your PIC ( or obtain one ready programmed from www.quadroufo.com ).

CAUTION: The PIC's pins are somewhat delicate.  Take care to make sure they are lined up properly and gently press them into place in the socket.  See [PIC Replacement 101](http://www.rcgroups.com/forums/showpost.php?p=12924177&postcount=96)

After this you will be able to use UAVPSet as before for updating UAVX but again only those hex files with 40 in the name. The 40MHz versions, while not extensively tested, are likely to give better performance particularly for smaller (decreased motor spacing) more agile quads.

See the "File Versions" paragraph in the UAVXStartup document in the WIKI for further details on the naming of the UAVX hex files.

### Where to Get 40MHz Crystals ###

The required crystal can be added to the board if ordered from www.quadroufo.com for about 50 cents.

The crystal is one similar to: http://www.mouser.com/Search/ProductDetail.aspx?R=ABL-10.000MHZ-B2virtualkey52750000virtualkey815-ABL-10-B2

## Fast PWM ESCs ##

The PIC processor spends half its time every control update to generate the 1 to 2 millisecond wide pulses for PWM ESCs.

As was stated here, way way back, off the shelf PWM ESCs were geared to how throttles are used on conventional aircraft which means relatively slow changes. This meant that many of them had and still have filters to slow down the rate at which the throttle can be opened or closed regardless of how fast you move the stick.  This is EXACTLY what we do not want for multicopters. The filters also limited the peak current applied to the motors allowing lower current ESCs. Some ESCs had settings to increase the filtering to protect shock loads on gear boxes.

So most of the research now a few years later out there goes into so called Fast PWM ESCs which run at a fraction of a Hz below 500Hz. Just increasing the update rate tricked the ESC filters allowing more rapid changes. We are now up at a fraction under 500Hz update rate on really cheap ESCs $10-20 for 30A ESCs is normal. Having the 30A headroom can be important even if our motors are only drawing around 10-15A.

We are at down 250Hz so what can we do if bigger is better is what turns us on?

## I2C ESCs ##

I2C ESCs can be updated in a fraction of a millisecond and so run immediately at 500Hz this being automatic with the current UAVX when I2C ESCs are selected.

### I Don't Have I2C ESCs ###

The simple trick is to have a translator that takes the I2C signals generated by UAVX and convert them to Fast PWM. Look up I2C to PWM converters. There is at least one out there that has 8 PWM outputs ;).

## Other Limits - the venerable HMC6352 ##

Most of the I2C sensors we currently use 400KHz communication rates. Note the "K" in front of the Hz so this is 400000Hz and is not to be confused with the control update rate.

One device we use however is the HMC6352 Compass and it can only communicate at 100KHz. Having code in UAVX which has to switch between these two rates is a little complicated and in practice means that the 400KHz devices are being run at under 300KHz chewing up time.
HMC5883L magnetometer communicates at 400KHz and if used allows us to run close to 400KHz with simpler code.  The magnetometer also, in theory, allows us to compensate for the aircraft not being level.

Bottom line consider using a HMC5883L and select the 40MHz hexfiles without I2C100KHz in their name. It is worth considering this even if you continue to use PWM ESCs directly.

## Tuning Parameters ##

Not much too this. The PID parameters need to have the I parameter halved as an initial starting point if we go from PWM (250Hz) to I2C (500Hz) including I2C to PWM through a converter.

If the I parameter is not halved you will get oscillation caused by the control code driving you back towards level so hard it overshoots.

Remember you need to set ILimit at around the current default or higher if you want strong positive restoration of level flight when you let go the sticks.