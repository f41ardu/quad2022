
// quad2022 edition v.02
// see also https://medium.com/@kavindugimhanzoysa/lets-work-with-mpu6050-gy-521-part1-6db0d47a35e6
#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
//#define FREQ        250   // Sampling frequency
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet
//#define SCALE_ACC  1
//#define SCALE_GYRO 65.5

// output 
#define ALL
//#define MEASURES
//#define ANGULAR
//#define ANGULAR

#include <Wire.h>

// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int16_t gyro_raw[3] = {0,0,0};
// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
 int16_t acc_raw[3] = {0 ,0 ,0};

// Average gyro offsets of each axis in that order: X, Y, Z
int16_t gyro_offset[3] = {0, 0, 0};
// Average acc offsets of each axis in that order: X, Y, Z
int16_t acc_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

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

// integration timer 
int  lastUpdate = 0; 
int  Freq, Now;

// MPU Temperature
int Temp; // not used 
 
// Init flag set to TRUE after first loop
boolean initialized = false;
 
void setup(){
  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.
  setupMPURegister(); 
  calibrateMpu6050(); 
  lastUpdate = micros();  
  Serial.begin(115200);
}

void loop(){

  readSensor();
  Now = micros() ;//Reset the loop timer
  Freq = 1.e6/(Now - lastUpdate);
    
  calculateAngles(); 
  #ifdef ALL
    Serial.print(measures[ROLL]);
    Serial.print(",");
    Serial.print(measures[PITCH]);
    Serial.print(","); 
    Serial.print(measures[YAW]);
    Serial.print(","); 
    Serial.print(angular_motions[ROLL]);
    Serial.print(",");
    Serial.print(angular_motions[PITCH]);
    Serial.print(","); 
    Serial.print(angular_motions[YAW]);
  #endif

  #ifdef MEASURES
    Serial.print(measures[ROLL]);
    Serial.print(",");
    Serial.print(measures[PITCH]);
    Serial.print(","); 
    Serial.print(measures[YAW]);
 #endif
 #ifdef ANGULAR 
    Serial.print(angular_motions[ROLL]);
    Serial.print(",");
    Serial.print(angular_motions[PITCH]);
    Serial.print(","); 
    Serial.print(angular_motions[YAW])
  #endif
  #ifdef GYROANGLE
    Serial.print(gyro_angle[X]);
    Serial.print(",");
    Serial.print(gyro_angle[Y]);
    Serial.print(","); 
    Serial.print(gyro_angle[Z]);
  #endif
  
    Serial.print("\n");
  lastUpdate = Now;
  //delay(100);
}


 
