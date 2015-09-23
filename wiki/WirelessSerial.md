There are at several options that you can use to get rid of the serial cable on the UAVX. The second involves using an XBee module instead of a bluetooth adapter.

The first is documented here:

http://uavp.ch/forum/viewtopic.php?t=1179&highlight=bluetooth

There are many differrent types of bluetooth modules to choose from.  The ranges and setup varies with each.  The BlueSmurf Gold from Sparkfun has had successful setups.  If you have others that work then let us know and we will post your details here.

The wiring to the UAVX is different depending on the bluetooth or XBee or Zigbee or other Tx/Rx you use.

For a serial connection the use of the K7 Tx, Rx and Gnd is used, An in most cases would require a 232 or TTL level converter to work.  The Tx modules must be set to read and send at 9600bps.  The UAVPset Telemetry pull down should be normally be UAVX and written to the UAVX.

For the TTL with non-inverted signals the GPS K19 connector is used.

Take care with the Tx voltage requirements.  The board can supply a total of 5v 1A of which 100ma is used by the devices on the board.  The 3v board power is only 100ma of which 50 is used so it is not recommended to use that as a Vcc for a Tx.  Instead use a separate 5v BEC or the ESC BEC from K2 to K4 for the older UAVP board or K1 to K4 for the newer UAVP board from http://www.quadroufo.com.  If 3v is required then use diodes or another regulator to lower the 5v from the ESC BECs.

This WIKI info will be expanded by thread contributors on detailed setup solutions for each type of telemetry Tx installed.

1. Greg Swiss FireFly Buletooth install: http://www.rcgroups.com/forums/showpost.php?p=17476351&postcount=6032

Well, the How-to I promised:

I bought the Bluetooth Serial Adapter from Sparcfun called
Bluetooth Modem - Roving Networks RS232


From hardware point of view I connected the UAVX as proposed to the serial
port and powered the bluetooth adapter (FireFly) with 5v from my 5v-source.


1) Making a Bluetooth Connection
By default, the FireFly acts as a slave and the PC is the master.
Connecting to the FireFly is done through the Bluetooth device manager
The FireFly must be discoverable by simply turning it on. The Green LED should be blinking. On your PC open the Bluetooth device manager and click on “Add” a new device. The Bluetooth device manager is located in the bottom right corner of your screen in the taskbar. The Bluetooth device manager will display a list of all the Bluetooth devices that are discoverable. The FireFly will be displayed as “FireFly-XXXX” where XXX is
the last 4 digits of the MAC address.

2) Pairing
Next you must pair with the device by double clicking on FireFly-XXXX in the list. Select “Enter the device’s pairing code” option from the list. Enter the default pin code of 1234. Once the Bluetooth device manager completes you will see a message to the effect, “Bluetooth device installed on COMX” where COMX is unique to your machine. In some cases the Bluetooth device manager will create two COM ports, in this case you only want to use the COM port labeled “outgoing”. this done only once!!!

3) Connecting
To establish a Bluetooth connection, open up the COM port assigned to the device from UAVPset

The default configuration for the FireFly is perfect for UAVPset
• Bluetooth slave mode
• Bluetooth pin code 1234
• Serial port 115K baud rate, 8 bits, NP, 1 stop bit
• Serial port flow control disabled
• Low power mode off

If you want to change the default, you can do it via remote configuration over bluetooth. Remote configuration can only occur if the bootup configuration timer (default 60 seconds) has no expired. Use the free TeraTerm program from our website(www.rovingnetworks.com/support/teraterm.zip). Getting into command mode, launch TeraTerm and make sure that the default settings are selected (Serial port COMX, 115,200Kbps, 8 bits, No Parity, 1 stop bit). You can change these settings by clicking on Setup Serial Port within TeraTerm.

Enter d for getting the current settings
Enter...
- SU,9600 sets Uart Baudrate to 9600
- SN,myname sets Bluetooth name to “myname”
- SA,1 enables secure authentication
- SP,secret sets security pincode to “secret”
- SF,1 restores all values to factory defaults
- R,1 reboots the module

I used only SU and SN commands for doing some tests..
I have not yet used it for UAVXGS.... I need to know what baudrate should be used...

The advantage:
- It is very small (I disposed the rs232 connector and the cover for weight-savings)
- It is configurable via bluetooth
- Range should be 300ft or 100m (not tested yet)
- Super flexible for setting UAVPset parameters and doing compass-calibration without cables

Under sparkfun you get background information:
firefly-ref-guide.pdf etc...
