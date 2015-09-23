### Introduction ###

UAVX PIC V2 and UAVXArm use a direct channel assignment scheme. This Wiki gives the current known channel orders.

For the initial setup **DISARM** the aircraft and disconnect your Rx. This will prevent the stick programming feature from flipping between parameter sets - specifically from parameter set #1 to parameter #2 where the Tx/Rx configuration is not available. This problem will be familiar to Mode 2 folk from the earlier Tx/Rx scheme - a bit of a Catch 22.

Make sure the Rx Graphic in UAVPSet shows all of your channels working as you expect **BEFORE ARMING** as having the throttle assigned incorrectly can be **DANGEROUS**.

### Tx/Rx Channel Ordering ###

There are many different orderings of the Tx/Rx channels and each new Manufacturer seems to have yet another scheme. So to make sense of these different orders we need to convert them to a single reference ordering which for UAVX is:

  * 1: Throttle
  * 2: Aileron
  * 3: Elevator
  * 4: Rudder
  * 5: Gear
  * 6: Aux1
  * 7: Aux2
  * etc.

Using a Futaba example:

|Tx/Rx|Time Order|Select|Connect|PPM Polarity|
|:----|:---------|:-----|:------|:-----------|
|Futaba DM8 & AR7000 	|R,E,A,1,T,G,2|5,3,2,1,6,4,7|R,A,T,2 |Pos         |

The channels in the **select** column are what you would normally enter in order in the numeric boxes for the Rx channels. The first 4 numbers in almost all cases correspond to the primary flight controls. So the 5th channel is Throttle, the 3rd is Aileron, the 2nd is Elevator etc.

It is possible to change this order but usually you would only change the order of the last three channels to put the RTH/Nav, Sensitivity and Camera trim on more appropriate Tx controls. For example selecting waypoint navigation means the RTH/Nav switch needs to be in the centre position.

It is important to understand that when we refer to the odd channel scheme it means the odd numbered channel in the time ordering **NOT** the number of the channel on the Rx.

### UAVP Parallel Decoder ###

For the Parallel Decoder the Rx must send out its control pulses in order with no overlaps. The **time order** column is the order in which the signals appear (the first letter of the channel name is used with 1&2 meaning Aux1 and Aux2). The **connect** column is gives the signals which **MUST** be used for parallel connections from the Rx to the UAVP board. The order of the channels does not matter but if you use any other channels it will simply not work. You must specify an ODD number of channels for this decoder even if your Rx has an EVEN number of channels - usually one less.

### External Decoder ###

There are a number of external decoders which may be connected to any of the UAVX Rx input signal pins. These decoders produce composite PPM or a "packet" of channel values. In this case you can connect the output of the decoder to any of the UAVP signal inputs.

The Neg./Pos. indicates the polarity of signals for composite PPM input.

### More Than 7 Channels ###

It is relatively easy to add additional channels to the UAVP board and most external decoders have more. UAVX does not currently require additional channels but has provision for 9 channels in the V2 release.

### Known Tx/Rx Channel Orderings ( Max 7Ch ) ###

|Tx/Rx|Time Order|Select|Connect|PPM Polarity|
|:----|:---------|:-----|:------|:-----------|
|Futaba Th 3 	|A,E,T,R,G,1,2|3,1,2,4,5,6,7|A,T,G,2 |Neg         |
|Futaba Th 2 	|A,T,R,E,G,1,2|2,1,4,3,5,6,7|A,R,G,2 |Neg         |
|Futaba DM8 & AR7000 	|R,E,A,1,T,G,2|5,3,2,1,6,4,7|R,A,T,2 |Pos         |
|JR PPM 	|T,A,E,R,G,1,2|1,2,3,4,5,6,7|T,E,G,2 |Pos         |
|JR DM9 & AR7000 	|R,E,A,1,T,G,2|5,3,2,1,6,4,7|R,A,T,2 |Pos         |
|JR DSX12 & AR7000 	|A,1,G,E,2,T,R|6,1,4,7,3,2,5|A,G,2,R |Pos         |
|Spektrum DX7 & AR7000 	|A,1,G,E,2,T,R|6,1,4,7,3,2,5|A,G,2,R |Pos         |
|Spektrum DX7 & AR6200 	|A,1,G,E,T,R,2|5,1,4,6,3,2,7|A,G,T,2 |Pos         |
|Futaba Th 2 Swap 6&7 	|A,E,T,R,G,2,1|3,1,2,4,5,7,6|A,T,G,1 |Neg         |
|Spektrum DX7 & AR6000 	|T,A,E,R,G,1,2|1,2,3,4,5,6,7|T,E,G,2 |Pos         |
|Graupner MX16s 	|T,A,E,R,G,1,2|1,2,3,4,5,6,7|T,E,G,2 |Pos         |
|Spektrum DX6i & AR6200 	|A,G,E,R,T,1,2|5,1,3,4,2,6,7|A,E,T,2 |Pos         |
|Futaba Th 3 & R617FS 	|A,E,T,R,G,1,2|3,1,2,4,5,6,7|A,T,G,2 |Pos         |
|Spektrum DX7a & AR7000 	|A,1,E,R,T,G,2|5,1,3,4,6,2,7|A,E,T,2 |Pos         |
|External Decoder 	|A,E,T,R,G,2,1|3,1,2,4,5,7,6|A,T,G,1 |Neg         |
|FrSky DJT & D8R-SP Composite 	|T,A,E,R,G,1,2|1,2,3,4,5,6,7|T,E,G,2 |Pos         |
|Turnigy 9X 	        |A,E,T,R,G,1,2|2,1,3,4,5,6,7|1,2,5,7 |Pos         |