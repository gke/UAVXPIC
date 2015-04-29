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
set CLOCK=CLOCK_16MHZ CLOCK_40MHZ
set PROC=18F2620
set DBG=NO_DEBUG
rem set RX=RX7CH
rem set CFG=QUADROCOPTER TRICOPTER HEXACOPTER HELICOPTER AILERON ELEVON
set EXP=NO_EXP

rem Personal choice
set CLOCK=CLOCK_16MHZ
set PROC=18F2620
set DBG=NO_DEBUG
set RX=RX7CH
set CFG=QUADROCOPTER	
set EXP=NO_EXP EXPERIMENTAL
set BRD=UAVPBLACK

rem Delete working files
call makeclean.bat

rem Requires Tortoise SVN 
call makerev.bat

rem del *.HEX

echo Starting makeall uavp > gen.lst
echo Starting makeall uavp > log.lst

for %%x in (%CLOCK%) do for %%p in (%PROC%) do for %%d in (%DBG%) do for %%r in (%RX%) do for %%c in (%CFG%) do for %%e in (%EXP%) do for %%b in (%BRD%) do call makeallhelper.bat %%x %%p %%d %%r %%c %%e %%b 

set PROC=18F2620
set DBG=TESTING 
set RX=RX7CH
set CFG=QUADROCOPTER
set BRD=UAVPBLACK

for %%x in (%CLOCK%) do for %%p in (%PROC%) do for %%d in (%DBG%) do for %%r in (%RX%) do for %%c in (%CFG%) do for %%e in (%EXP%) do for %%b in (%BRD%) do call makeallhelper.bat %%x %%p %%d %%r %%c %%e %%b 
