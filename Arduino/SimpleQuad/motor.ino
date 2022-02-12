/*
 * Source: https://technicaladventure.blogspot.com/2012/09/quadcopter-stabilization-control-system.html
 * 
 * Plus (+) Configuration Control

In this configuration the control as follows:
// Pitch Control
M1 = M1 + RX(Elevator);
M4 = M4 - RX(Elevator);
// Roll Control
M2 = M2 + RX(Aileron);
M3 = M3 - RX(Aileron);

This is very simple control and is found in all quadcopter code. 
Sure there are some checks here and there to avoid motor stopping or saturating.  
But the flying logic is always as above. There is also a scaling factor that is 
used to determine the sensitivity of the sticks and a Divide factor that limits 
the value range.

Stabilization System

Now let us see the stabilization system. The idea is simple, if the quad is 
falling to the right then speed up the right motor and slow the left one with 
the same amount and vice versa. Also if the quad is falling down from the front 
arm then speed up the M1 motor and slow down  M4. The rule is written as follows.

M1 = M1 + PitchAmount  *  PitchGain
M4 = M4 -  PitchAmount  *  PitchGain
M2 = M2 + RollAmount   *   RollGain
M3 = M3  - RollAmount   *   RollGain

This is the basic rule that should be found in all quadcopter programs. 
The main difference between the different firmware approaches is how to calculate 
PitchAmount and RollAmount. This can simply be read from a Gyro sensor and multiply
it by a constant factor to adjust the range, or we can use PID approach or even Kalman 
filter and combine Gyro with Acc sensors to get the exact degree.
The Gain factor is used to determine the sensitivity. Other checks such as trimming 
are common in quadcopter code.

Do we need Different Stabilization for X-Configuration?

Well in almost all quadcopter programs you will find that stabilization and control 
calculation follows the same concept. i.e. if we use X-Quadcopter then we add the PitchAmount
to both motors M1 & M2. Also RollAmount is added to motors M1 & M3. This is valid approach 
but in fact it is not necessary at all, also there is a major drawback here.
It is not necessary because the stabilization control of Plus configuration can 
stabilize quadcopter if there is a simultaneous picth & roll forces, that means if the 
Control System is Plus of X the Stabilization System can stabilize both using the same 
PLUS configuration.
The main drawback is that Stabilization control requires the orient the control board so
that its onboard gyro and acc are in the right directions, so you cannot switch between PLUS 
configuration to X-Configuration without reorient the board. So if we can keep the Stabilization
System and only change the Control System we don’t need to reorient the board as Control System 
has nothing to do with the board sensors it only change the signal values sent to different motors.
 */
