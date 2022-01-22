// quad2022 edition

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

//---------------------------------------------------------------------------------------------------
// Definitions
#define max_samples 500
#define sampleFreq  250.0f    // sample frequency in Hz
#define betaDef   0.1f    // 2 * proportional gain

#define quaternions
//#define rollpitchyaw

#include<Wire.h>
#include <math.h>
#include "MadgwickAHRS.h"

//---------------------------------------------------------------------------------------------------
// Variable definitions
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 
float roll = 0, pitch = 0, yaw = 0; 
// ----------------------- MPU raw variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int16_t gyro_raw[3] = {0, 0, 0};
// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int16_t acc_raw[3] = {0 ,0 ,0};
// The RAW values got from accelerometer
int16_t Temp = 0; 
int16_t mx,my,mz; //triple axis data


// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};
long acc_offset[3]  = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
// Init flag set to TRUE after IMU calibration
boolean initialized = false;
// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;  
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float gres = 500.0 / 32768.0;
float ares = 8.0 / 32768.0;
const float pi_180 = PI / 180.0f;
// 
void setup(){
  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.
  //setupMPURegister(); 
  setupHMCRegister();
  
  Serial.begin(38400);
}

void loop(){

#ifdef quaternions   
// use myMPU6050_quaternion
  getQuaternions();
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(","); 
  Serial.print(q[2]);
  Serial.print(","); 
  Serial.println(q[3]);

#endif

#ifdef rollpitchyaw
// use myMPUDemo
  getQuaternions();
  quaternionToRollPitchYaw();  
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(","); 
  Serial.println(yaw);
#endif 
}





 