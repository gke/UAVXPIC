@echo off

SETLOCAL ENABLEDELAYEDEXPANSION

rem Helper script for makeall.bat
rem =======================================================
rem parameters passed are:

set I2C=%1
set CFG=%2
set GPS=%3
set DBG=%4

for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set year=%%c
for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set month=%%a
for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set day=%%b
set TODAY=%year%%month%%day%

for /f "tokens=1 delims=: " %%h in ('time /T') do set hour=%%h
for /f "tokens=2 delims=: " %%m in ('time /T') do set minutes=%%m
for /f "tokens=3 delims=: " %%a in ('time /T') do set ampm=%%a
set NOW=%hour%%minutes%%ampm%

set CSRC=leds stats eeprom math params adc uavx irq inertial menu control compass baro tests serial rc utils gps rangefinder telemetry autonomous i2c outputs
set ASRC=bootl18f

rem Set all the name tokens for the HEX files
set G=
set E=
set D=
set T=
set R=
set G=
set I=
set L=
set X=-40

if "%I2C%" == "I2C100KHZ"     		set I=-I2C100KHZ

if "%DBG%" == "TESTING"     		set D=-TEST
if "%DBG%" == "SIMULATE"     		set D=-SIMULATOR
if "%CFG%" == "QUADROCOPTER"        set C=-QUAD
if "%CFG%" == "QUADROCOPTERX"        set C=-QUADX
if "%CFG%" == "HEXACOPTER"			set C=-HEXA
if "%CFG%" == "HEXACOPTERX"			set C=-HEXAX
if "%CFG%" == "TRICOPTER"           set C=-TRI
if "%CFG%" == "HELICOPTER"			set C=-HELI
if "%CFG%" == "VTCOPTER"			set C=-VT
if "%CFG%" == "Y6COPTER"			set C=-Y6
if "%CFG%" == "AILERON"				set C=-AILERON
if "%CFG%" == "ELEVON"				set C=-ELEVON
if "%CFG%" == "VTOL"				set C=-VTOL

if "%GPS%" == "USE_UBLOX_BIN"			set G=-UBX

if "%DBG%" == "TESTING"				set C=

set CC="C:\MCC18\bin\mcc18" 
rem removed integer promotions set CCMD=  -Oi -w1 -Opa- -DBATCHMODE
set CCMD=  -w3 -Opa- -DBATCHMODE

set ACMD=/q /dCLOCK_40MHZ /p18F2620 %%i.asm /l%%i.lst /e%%i.err /o%%i.o
set AEXE="C:\MCC18\mpasm\mpasmwin.exe"

set LCMD=/p18F2620 /l"C:\MCC18\lib" /k"C:\MCC18\lkr"
set LEXE="C:\MCC18\bin\mplink.exe"

rem Build the list of expected object files
set F=
for %%i in ( %CSRC% ) do set F=!F! %%i.o
for %%i in ( %ASRC% ) do set F=!F! %%i.o

for %%i in ( %CSRC% ) do %CC% -p=18F2620 /i"C:\MCC18\h" %%i.c -fo=%%i.o %CCMD%  -DCLOCK_40MHZ -D%I2C% -D%CFG% -D%DBG% -D%GPS% >> log.lst

for %%i in ( %ASRC% ) do %AEXE%  %ACMD% >> log.lst

%LEXE% %LCMD% %F% /u_CRUNTIME /z__MPLAB_BUILD=1 /W /o UAVX%L%-V2.$WCREV$gke-18F2620-40%I%%G%%C%%D%%T%.hex >> log.lst 


if %ERRORLEVEL% == 1 goto FAILED

echo compiled - UAVX%L%-V2.$WCREV$gke-18F2620-40%I%%G%%C%%D%%T%.hex
echo compiled - UAVX%L%-V2.$WCREV$gke-18F2620-40%I%%G%%C%%D%%T%.hex >> gen.lst
call makeclean.bat
goto FINISH

:FAILED
echo failed - UAVX%L%-V2.$WCREV$gke-%E%18F2620-40%I%%G%%C%%D%%T%.hex
echo failed - UAVX%L%-V2.$WCREV$gke-%E%18F2620-40%I%%G%%C%%D%%T%.hex >> gen.lst
rem don't delete working files

:FINISH















