/* MPU6050 Basic Example with IMU
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

//#define quaternions // debugging
#define rollpitchyaw

#include <Wire.h>
#include "MPU6050.h"

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion (move to IMU) when ready
float roll, pitch, yaw = 0.0f;                    // store roll, pitch and yaw in radians


void setup()
{
  Wire.begin();
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);
  initSensors(); 
  
}

void loop()
{
  
  imuCalc(); // calculate roll,pitch and yaw 

 
#ifdef quaternions
   Serial.print(q[0]);
   Serial.print("," );
   Serial.print(q[1]);
   Serial.print(","); 
   Serial.print(q[2]);
   Serial.print(","); 
   Serial.println(q[3]);
#endif
#ifdef rollpitchyaw       
    Serial.print(degrees(roll));
    Serial.print(", ");
    Serial.print(degrees(pitch));
    Serial.print(", ");
    Serial.println(degrees(yaw));
#endif

    }
