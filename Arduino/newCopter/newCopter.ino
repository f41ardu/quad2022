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

#include <Wire.h>
#include "MPU6050.h"

MPU6050lib mpu;

float aRes, gRes; // scale resolutions per LSB for the sensors
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
#define blinkPin 13  // Blink LED on Teensy or Pro Mini when updating
boolean blinkOn = false;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gyrox, gyroy, gyroz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f;                              // integration interval for both filter schemes

uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval

void setup()
{
  Wire.begin();
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);
  initSensors(); 
  
}

void loop()
{
  
  imuCalc(); 
    /*
        Serial.print("ax = "); Serial.print((int)1000*ax);
        Serial.print(" ay = "); Serial.print((int)1000*ay);
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");

        Serial.print("gyrox = "); Serial.print( gyrox, 1);
        Serial.print(" gyroy = "); Serial.print( gyroy, 1);
        Serial.print(" gyroz = "); Serial.print( gyroz, 1); Serial.println(" deg/s");
*/
     /*   Serial.print(q[0]);
        Serial.print("," );
        Serial.print(q[1]);
        Serial.print(","); 
        Serial.print(q[2]);
        Serial.print(","); 
        Serial.println(q[3]);
        */
        Serial.print(roll);
        Serial.print(", ");
        Serial.print(pitch);
        Serial.print(", ");
        Serial.println(yaw);
    
   
   
  
}
