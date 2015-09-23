## Introduction ##

Firstly it is a **better investment** to buy the new UAVXArm32F4 board than buy one of the I2C IMU boards. The difference in cost is quite small and so if you don't already have one of the IMU boards consider buying the UAVXArm32F4 board instead.  If you do have a FreeIMU board with the same sensors as those by used by the Arm board then consider getting the UAVXArm32F4 without sensors.

However some of you will have become quite fond of your UAVP boards given their history being amongst the very first multicopter flight controllers.  The 18F2620 PIC extended its life quite a bit and the latest I2C sensors or the UAVXArm32F4 daughter board are the final chapter.

I for one will be keeping my UAVP boards for a while yet although the UAVXArm32F4 boards are my frontline flight controllers.

### I2C Sensor Boards ###

There are a number of single board sensor combinations or IMUs that are compatible with the UAVP board. One such combination, the FreeIMU, uses the same set of sensors as UAVXArm32F4.

The images below show the expected orientation of these IMU cards that the firmware expects. The UAVP board does not need to be modified other than probably adding a socket to the I2C pins previously used by the baro and compass.

If analog gyros are still fitted to the board these may be used by selecting the specific analog gyro in UAVPSet. UAVX will (or should) detect the rest of the sensors on the IMU board and use them.

#### FreeIMU ####

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_FreeIMU.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_FreeIMU.jpg)

#### FreeIMU Clone ####

The pins do not follow the pattern commonly used so alternate pins need to be swapped. It is possible to move the board 0.1" towards the PIC clearing the K pins.

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_FreeIMUClone.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_FreeIMUClone.jpg)

#### Drotek ####

To reduce code size the Drotek is not currently in the formware but it is easy to select with the define in uavx.h or ask me to do a "special".

The Drotek overlaps the K pins and so longer pins as for the old compass and baro may be necessary or alternatively use a stacking connector.

![http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_Drotek.jpg](http://uavp-mods.googlecode.com/svn/branches/uavx_graphics/UAVP_Drotek.jpg)

_G.K. Egan 2013_