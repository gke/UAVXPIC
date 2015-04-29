@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Batch compiles various possibilities of the UFO software for the Microchip 18F2xxx series
rem
rem Greg Egan 2008-2010
rem
rem Uses: makeallhelper.bat and makeclean.bat
rem
rem Clock rate CLOCK_16MHZ (Only 16MHZ available for UAVP version)
rem Type of PIC processor 18F2620 only
rem SIMULATE to generate a simple flight simulator (no dynamics) for use with UAVXGS - no motors.
rem using UAVPSet (blank option in menu below testsoftware).
rem Configuration TRICOPTER for 3 motors and QUAD for 4.
rem TESTING for checking out sensors Motors are disabled for safety reasons.
rem UAVPBLACK original UAVP board
rem EXPERIMENTAL - USE WITH GREAT CAUTION - EXPERIMENTAL CONTROLS SEVERAL OTHER
rem DEFINES CONTAINED IN THE BLOCK TOWARDS THE TOP OF UAVX.H

rem Add/Delete required combinations to these sets
set I2C=I2C100KHZ I2C400KHZ
set DBG=NODEBUG
rem set CFG=QUADROCOPTER QUADROCOPTERX VTOL HEXACOPTER HEXACOPTERX TRICOPTER VTCOPTER Y6COPTER HELICOPTER
set CFG=QUADROCOPTER QUADROCOPTERX
set GPS=USE_UBLOX_BIN USE_NMEA

rem Personal choice
rem set I2C=I2C100KHZ I2C400KHZ
rem set CFG=QUADROCOPTERX VTOL AILERON ELEVON
rem set GPS=USE_UBLOX_BIN USE_NMEA

rem Delete working files
call makeclean.bat

rem Requires Tortoise SVN 
call makerev.bat

del *.HEX

echo Starting makeall uavp > gen.lst
echo Starting makeall uavp > log.lst

for %%x in (%I2C%) do for %%c in (%CFG%) do for %%g in (%GPS%) do for %%d in (%DBG%) do call makeallhelper.bat %%x %%c %%g %%d

set DBG=TESTING SIMULATE 
set CFG=QUADROCOPTER
set GPS=USE_NMEA

for %%x in (%I2C%) do for %%c in (%CFG%) do for %%g in (%GPS%) do for %%d in (%DBG%) do call makeallhelper.bat %%x %%c %%g %%d 


