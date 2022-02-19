
/* Based on MPU6050 Basic Example with IMU
  by: Kris Winer
  date: May 10, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.

  Demonstrate  MPU-6050 basic functionality including initialization, accelerometer trimming, sleep mode functionality as well as
  parameterizing the register addresses. Added display functions to allow display to on breadboard monitor.
  No DMP use. We just want to get out the accelerations, temperature, and gyro readings.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors worked for me. They should be on the breakout
  board.

  Hardware setup:
  MPU6050 Breakout --------- Arduino
  3.3V --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

  Note: The MPU6050 is an I2C sensor and uses the Arduino Wire library.
  Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
  We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
  We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/

// definitions
#define blinkPin 13  // Blink LED on Teensy or Pro Mini when updating

//#define quaternions // debuggin
//#define rollpitchyaw
#define ESC_OUT

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#include <Wire.h>
#include <PID_v1.h>
#include "MPU6050.h"

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion (move to IMU) when ready
float yaw = 0.0f;                    // store roll, pitch and yaw in radians

//Define Variables we'll be connecting to
// Roll
double setRoll, inRoll, rollOut; 
double kpRoll=1.3, kiRoll=0.04, kdRoll=18.0;
double roll;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,P_ON_M, DIRECT);
PID quadRoll(&roll, &rollOut, &setRoll, kpRoll, kiRoll, kdRoll, P_ON_M, REVERSE); 

// Pitch
double setPitch, inPitch, pitchOut;
double kpPitch=1.3, kiPitch=0.04, kdPitch=18.0;
double pitch;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID quadPitch(&pitch, &pitchOut, &setPitch, kpPitch, kiPitch, kdPitch,P_ON_M, REVERSE);


void setup()
{
  Wire.begin();
  Serial.begin(115200); 
  
  quadRoll.SetSampleTime(10);
  quadRoll.SetOutputLimits(-400, 400);
  quadRoll.SetMode(AUTOMATIC);

  quadPitch.SetSampleTime(10);
  quadPitch.SetOutputLimits(-400, 400);
  quadPitch.SetMode(AUTOMATIC);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);
  initSensors(); 


  setRoll  = 0;
  setPitch = 0; 
}

void loop()
{
  
  imuCalc(); // calculate roll,pitch and yaw 
  quadRoll.Compute();
  quadPitch.Compute();
  
#ifdef quaternions
   Serial.print(q[0]);
   Serial.print("," );
   Serial.print(q[1]);
   Serial.print(","); 
   Serial.print(q[2]);
   Serial.print(","); 
   Serial.print(q[3]);
#endif
#ifdef rollpitchyaw       
   Serial.print((roll));
   Serial.print(", ");
   Serial.print((pitch));
   Serial.print(", ");
   Serial.print((yaw));
#endif
#ifdef ESC_OUT
     Serial.print(-int((rollOut-pitchOut)/2));
     Serial.print(",");
     Serial.print(+int((rollOut-pitchOut)/2));
     Serial.print(",");
     Serial.print(-int((rollOut+pitchOut)/2));
     Serial.print(",");
     Serial.print(+int((rollOut+pitchOut)/2));
#endif
    Serial.println();
    
    }
