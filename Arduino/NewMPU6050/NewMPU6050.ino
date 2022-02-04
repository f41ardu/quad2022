// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
// accl and gyro values
int16_t ax, ay, az;
int16_t gx, gy, gz;
// offset 
int16_t axos, ayos, azos;
int16_t gxos, gyos, gzos;
// accel and gyro scale
float ares, gres;
// The Quaternion 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
// parameters for 6 DoF sensor fusion calculations
const float pi_180 = PI / 180.0f;
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = -PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;  
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;  // used to calculate integration interval
uint32_t delta = 0; 
uint32_t loop_timer = 0; 
uint32_t printTimer = 0; 
// sampling parameter
int freq = 250; // Hz
uint32_t cyclus = (uint32_t)1e6/freq; // in uS

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
// #define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define OUTPUT_QUATERNIONS


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
   // Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

/*
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03
*/
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    ares = MPU6050_getAres(MPU6050_GYRO_FS_500); // freq necessary ????      
/*
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03
*/
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    gres = MPU6050_getAres(MPU6050_ACCEL_FS_8);
/*    
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06
*/
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

    // verify connection
 //   Serial.println("Testing device connections...");
 //   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

   // Serial.println("Get current internal sensor offsets...");
/*    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    Serial.println("Set new internal sensor offsets");
    accelgyro.setXGyroOffset(70);
    accelgyro.setYGyroOffset(-16);
    accelgyro.setZGyroOffset(-15);
    accelgyro.setXAccelOffset(-13);
    accelgyro.setYAccelOffset(-393);
    accelgyro.setZAccelOffset(1285);
20:24:33.355 -> Data is printed as: acelX acelY acelZ giroX giroY giroZ
20:24:33.355 -> Check that your sensor readings are close to 0 0 16384 0 0 0
20:24:33.355 -> Sensor readings with offsets: -2  -3  16387 1 -1  0
20:24:33.355 -> Your offsets: 41  -74 1224  174 -17 -15
 
 */
    accelgyro.setXGyroOffset(174);
    accelgyro.setYGyroOffset(-17);
    accelgyro.setZGyroOffset(-15);
    accelgyro.setXAccelOffset(41);
    accelgyro.setYAccelOffset(-74);
    accelgyro.setZAccelOffset(1224);
    mpuCalibration();
 /*
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    Serial.println("Fine Calibration MPU devices");
     
     Serial.print("a/g:\t");
        Serial.print(axos); Serial.print("\t");
        Serial.print(ayos); Serial.print("\t");
        Serial.print(azos); Serial.print("\t");
        Serial.print(gxos); Serial.print("\t");
        Serial.print(gyos); Serial.print("\t");
        Serial.println(gzos);
    Serial.println("Calibration MPU devices done");
*/
    //delay(5000);
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    lastUpdate = firstUpdate = micros();
}

void loop() {


 if(lastUpdate - firstUpdate > 10000000uL) {
      beta = 0.041; // decrease filter gain after stabilized 0.041

      zeta = 0.015; // increase gyro bias drift gain after stabilized 0.015  
 }

 if((micros() - loop_timer) > cyclus) {                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax = ax - axos;
    ay = ay - ayos;
    az = az - azos;
    gx = gx - gxos;
    gy = gy - gyos;
    gz = gz - gzos;
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    
    //Now = micros();
    Now = loop_timer = micros() ;//Reset the loop timer
    delta = Now - lastUpdate;
    deltat = delta * 1e-6; // set integration time by time (im seconds) elapsed since last filter update
    
    
    MadgwickQuaternionUpdate(ax/ares, ay/ares, az/ares, gx/gres, gy/gres, gz/gres); 
    lastUpdate = Now;
   if (millis() - printTimer > 5000) {
      #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ares*ax); Serial.print("\t");
        Serial.print(ares*ay); Serial.print("\t");
        Serial.print(ares*az); Serial.print("\t");
        Serial.print(gres*gx); Serial.print("\t");
        Serial.print(gres*gy); Serial.print("\t");
        Serial.println(gres*gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    #ifdef OUTPUT_QUATERNIONS   
// use myMPU6050_quaternion
  Serial.print(q[0],5);
  Serial.print(",");
  Serial.print(q[1],5);
  Serial.print(","); 
  Serial.print(q[2],5);
  Serial.print(","); 
  Serial.print(q[3],5);
  Serial.print(",");
  Serial.println(deltat,5);
  printTimer = millis();
   }
  
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
} 
 
}
