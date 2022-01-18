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

float beta = betaDef;                // 2 * proportional gain (Kp)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame



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

// 
void setup(){
  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.
  setupMPURegister(); 
  calibrateMpu6050(); 
  
  Serial.begin(115200);
}

void loop(){

  readSensor();
  calculateAngles(); 
  updateIMU(acc_raw[X], acc_raw[Y], acc_raw[Z],gyro_raw[X] * PI / 180.0f, gyro_raw[Y] * PI / 180.0f, gyro_raw[Z] * PI / 180.0f);
  
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
  Serial.print(q0);
  Serial.print(",");
  Serial.print(q1);
  Serial.print(","); 
  Serial.print(q2);
  Serial.print(","); 
  Serial.println(q3);
  
  //delay(100);
}





 
