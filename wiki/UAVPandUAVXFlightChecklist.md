# UAVP and UAVX Flight Checklist--THINK SAFETY-SAFETY-SAFETY!!! #

**Safety checklist:**

1. Do not fly over/near (within 50ft) people. Consider even farther safety distances for quads over 1kg or 35oz.

2. Winds over 15-20 knots make flying difficult at best, so consider increasing safety distance.

3. Stay several feet away from walls and ceilings when flying indoors as the prop currents and vacuum can cause loss of control and crash damage.

4. Check frame for CF/Aluminum/wood cracks and all mounts are fully inserted and fit tight and for any loose screws.  Make sure motors/props are perfectly level with each other and the PCB.

5. Check props for stress cracks and replace if necessary.  Check O-rings for stress cracks if prop-savers are used.  Check prop security and tightness on the motors.

6. Check all wires and connectors and antennas on Tx and Rxs for proper contact and security.

7. Check Lipos for any damage and if found safely dispose them.


**Pre-Flight Checklist:**

1. Confirm proper firmware load using UAVPset.  Confirm that parameter sets 1 and 2 are BOTH properly loaded.
For UAVX confirm channel 5 switch is in the manual/hover position. This appears as a slider tab at the low position on UAVPset V3.0 graphic.  The UAVX will beep and not arm if the channel 5 switch is in the RTH (tab high in UAVPset) position.

2. Tx/Rx check: If 72mhz, 35mhz, i.e. FM, then insure same channel on Tx and Rx on UAVP. If 2.4Ghz then insure proper TX/RX binding

3. **Disarmed:** UAVP/UAVX Green LED and flickering Yellow LED will be on.

4. **Armed:** UAVPgkeware/UAVX Green LED will be on when armed (UAVP-Wolf).  UAVPgkeware/UAVX: Red will blink as gyros initialize then go solid if GPS has not taken its first position fix. It is IMPORTANT that the quadrocopter be level to within less than 10 degrees before arming.

5. After gyro initialization (blinking Red LED) the Red LED and Green LED must be on solid. There will then be three short beeps and one long beep at which time the throttle is active. If GPS is installed the Blue LED will be blinking.  The Red LED will go out when UAVX has taken a good GPS fix.  This can be from seconds to minutes depending on how recent the GPS was powered on in the same location.

6. Do a full Stir the Stick controls, Switches, and Pots Test (SSSPT) while the model is disarmed.   **Warning: The Green LED should remain solid and not blink at all!!**

7. Do a Tx/Rx proper range check in all quadrants.

8. Insure that the Tx and multicopter batteries are fully charged.


**Flight checklist:**

1. Tx on, throttle down, then power-on the UAVX with arming switch in **Disarmed** position.

**WARNING!:  Never power on the quad before the Tx.  If the Tx is off and the quad’s arming switch is accidentally left in the armed position the motors may start at high speed when using some Rx's and cause serious injury!**

2. **Disarmed:** Select a parameter using the pitch/roll sticks(UAVX): **Parm 1 with RTH altitude control = lower left = default**, Parm 1 with no RTH altitude control = upper left, Parm2 with RTH altitude control = lower right, Parm 2 no altitude control = upper right.

3. **Disarmed:** Select yaw turn on/off for RTH using the throttle stick(UAVX).  **Throttle stick to left = RTH yaw turn off = default**, Throttle stick to right = RTH yaw turn on.  After RTH is engaged, if the UAVX is outside of the NAV radius set in UAVPset v3.0, the quad will yaw turn to the home position.

4. **Armed:**  Stand behind the UAVP, slightly increase throttle to get props spinning, insure proper nick, roll and yaw stick response. Again it is IMPORTANT that the quadrocopter be level to within less than 10 degrees before arming.

5. Bump throttle to 50% to get a low hover, note and accomplish any trimming required to get a stable hover.  It is strongly suggestd to land and trim until you get a perfectly vertical lift-off.

6. Fly! Keep several feet from walls and ceilings, or trees and buildings when outside as there will be a tendency for the quad to get sucked into them by the air currents it generates.

7. Land as soon as the lipo voltage alarm sounds.  Recommended setting is to 48=10.43v in order to allow at least a 1 minute pad for completing a landing with reserve power. The red LED will blink and the beeper will sound steadily when the lipo is below the minimum set voltage.  **Warning!:  If you used the auto land feature make certain that you put the throttle to minimum immediately after landing.  If you forget to do so and then put the RTH ch5 switch in PH, the model will immediately launch!**

8. **Disarm** then remove power from UAVX, then turn Tx off.  **Warning!:  Never turn the Tx off first!**

9. Do a post flight inspection!  After landing and removing power, check the temperature of all motors and ESCs.  Higher temps on one or more indicate possible neutral stability or motor or ESC issues.  Correct them before the next fight.

Have fun and safe flying!
Cheers,
Jim