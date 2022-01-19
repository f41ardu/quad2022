// quad2022 edition

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define FREQ        250   // Sampling frequency
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq  250.0f    // sample frequency in Hz
#define betaDef   0.1f    // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 



#include<Wire.h>
#include <math.h>

// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int16_t gyro_raw[3] = {0,0,0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};
long acc_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
 int16_t acc_raw[3] = {0 ,0 ,0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0,0,0};

// Total 3D acceleration vector in m/s²
long acc_total_vector;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};


// MPU Temperature
int Temp; 
// Init flag set to TRUE after first loop
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
  setupMPURegister(); 
  calibrateMpu6050(); 
  
  Serial.begin(38400);
}

void loop(){

  readSensor();
 
 //     if(lastUpdate - firstUpdate > 10000000uL) {
 //       beta = 0.041; // decrease filter gain after stabilized
 //       zeta = 0.015; // increase gyro bias drift gain after stabilized
 //     }
  // Pass gyro rate as rad/s
  calculateGyroAngles(); 
  
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  
   MadgwickQuaternionUpdate(ares*acc_raw[X], ares*acc_raw[Y], ares*acc_raw[Z], gres*gyro_raw[X] *  pi_180, gres*gyro_raw[Y] * pi_180, gres*gyro_raw[Z] * pi_180);
  //Serial.print("Temp = "); Serial.print(Temp/340.00+36.53);
  /* 
  
  Serial.print("AccX = "); Serial.print(acc_raw[X]);
  Serial.print(" || RAccY = "); Serial.print(acc_raw[Y]);
  Serial.print(" || RAccZ = "); Serial.print(acc_raw[Z]);
  Serial.print(" || RGyroX = "); Serial.print(gyro_raw[X]);
  Serial.print(" || RGyroY = "); Serial.print(gyro_raw[Y]);
  Serial.print(" || RGyroZ = "); Serial.print(gyro_raw[Z]);
  
  */
  
/*  Serial.print(" || AccX = "); Serial.print(acc_angle[X]);
  Serial.print(" || AccY = "); Serial.print(acc_angle[Y]);
//  Serial.print(" || GyroX = "); Serial.print(gyro_angle[X]);
//  Serial.print(" || GyroY = "); Serial.print(gyro_angle[Y]);
  Serial.print(" || am YAW = "); Serial.println(angular_motions[YAW]);
  /*
  Serial.print(" || ROLL = "); Serial.print(measures[ROLL]);
  Serial.print(" || PITCH = "); Serial.print(measures[PITCH]);
  Serial.print(" || YAW = "); Serial.print(measures[YAW]);
  Serial.print(" || am ROLL = "); Serial.print(angular_motions[ROLL]);
  Serial.print(" || am PITCH = "); Serial.print(angular_motions[PITCH]);
  Serial.print(" || am YAW = "); Serial.println(angular_motions[YAW]);
  */
/*
  Serial.print(acc_angle[X]);
  Serial.print(","); 
  Serial.print(acc_angle[Y]);
  Serial.print(",");
  Serial.print(angular_motions[YAW]);
  Serial.print("\n");
*/
/*
  Serial.print(measures[ROLL]);
  Serial.print(",");
  Serial.print(measures[PITCH]);
  Serial.print(","); 
  Serial.println(measures[YAW]);
*/
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(","); 
  Serial.print(q[2]);
  Serial.print(","); 
  Serial.println(q[3]);
  
  //delay(100);
}





 
