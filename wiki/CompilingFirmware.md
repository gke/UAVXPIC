The zip archives in the downloads area contain a full set of flight and test HEX files. The following processes are only necessary if you are doing your own development work.

## Installing the UAVP/UAVX firmware compilers (see below for UAVPset) ##
First you need to install the compilers required for generating the flight and test HEX files. You only need to do this once.

  * Download the latest version of MPLab and the C18 compiler and install them (http://www.microchip.com).
  * Download and install Tortoise SVN ( http://tortoisesvn.net/ ).
  * For the superseded 16F876 versions of UAVP download the cc5x compiler (http://www.bknd.com/cc5xfree.exe). Run the installer to install cc5x into c:\program files\microchip\cc5x.
  * Download and install the current version of UAVPSet ( http://code.google.com/p/uavp-mods/downloads/list).
The latest version of UAVPSet supports V3.15 UAVP and UAVX.

## Obtaining the UAVP/UAVX files ##
  * Download the appropriate zip archive for your particular PIC processor ( http://code.google.com/p/uavp-mods/downloads/list) and unzip it.
  * Run the makeall.bat file within the directory created.
  * Done! Select the appropriate HEX file and download it to your quadrocopter using UAVPSet.

## Using MPLab ##
  * All of the revisions on this site now contain an MPLab project workspace file (mcw). For the older 16F876 PICs the compiler is used is cc5x which does not readily support simulation within MPLab.
  * With the exception of sensor inputs full simulation can be performed within MPLab when using the C18 compiler.

## Compiling UAVPset firmware ##

  * Download and install the Microsoft Visual Express Studio for C#: http://www.microsoft.com/visualstudio/en-us/products/2010-editions/visual-csharp-express
  * Obtain the UAVPset source code from ( http://code.google.com/p/uavp-mods/downloads/list) and unzip it.
  * Open the .sln to edit the UAVPset software.



See the relevant README files.