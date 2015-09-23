## Introduction ##

UAVXArm32F4 board may be used without sensors as a UAVP adapter. The Arm board acts as a processor upgrade for the UAVP board and uses all the original sensors which are the analog gyros, LISL accelerometer, HMC6352 compass and BMP085 barometer.

#### UAVXArm32F4 Board ####

If you look carefully you can see pins marked + down the left side of the board being those that plug into some of the sockets in the old PIC connector. The block of 2x4 pins towards the top right of the board connect to the "K" motor servo pins to stabilise the board.  You may wish to put standoffs on the board to further stabilise it.

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVXArm32F4Connections.png](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVXArm32F4Connections.png)

#### Procedure ####

Remove both the chips (PIC and TPIC LED driver) from the UAVP board.

The Baro and Compass which you have cut from the UAVP board will plug into the I2C 4 pin connector on the top of the Arm board.

The Compass and Baro are stacked together and then soldered to the UAVP board. Carefully cut the wires as close to the UAVP board as possible as you will need to preserve enough lead length as possible.  You could try unsoldering the wires from the UAVP board but that risks damage.

The Arm board comes with pins that connect to the PIC socket and some of the motor servo connectors. These pins are very fine and fragile so be very careful that they are lined up properly before plugging the board in fully.

http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVXArm32F4_Adapter.JPG

Both the UAVP board and the Arm board need to be connected to the main battery.

None of the connectors on the original UAVP board are used but leave them intact.




..... more to follow.