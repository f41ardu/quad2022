
// quad2022 edition v.02
// see also https://medium.com/@kavindugimhanzoysa/lets-work-with-mpu6050-gy-521-part1-6db0d47a35e6


// output 
#define ALL
//#define MEASURES
//#define ANGULAR
//#define ANGULAR

#include <Wire.h>
#include "PID_v1.h"
#include "definitions.h"
#include "mpu.h"

//Define Variables we'll be connecting to
// Roll
double setRoll, inRoll, rollOut;
double kpRoll=0.5, kiRoll=0.01, kdRoll=1;
double roll;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID quadRoll(&roll, &rollOut, &setRoll, kpRoll, kiRoll, kdRoll,REVERSE); 

// Pitch
double setPitch, inPitch, pitchOut;
double kpPitch=0.5, kiPitch=0.01, kdPitch=1;
double pitch;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID quadPitch(&pitch, &pitchOut, &setPitch, kpPitch, kiPitch, kdPitch,REVERSE);


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
  quadRoll.SetSampleTime(1);
  quadRoll.SetOutputLimits(-500, 500);
  quadRoll.SetMode(AUTOMATIC);

  quadPitch.SetSampleTime(1);
  quadPitch.SetOutputLimits(-500, 500);
  quadPitch.SetMode(AUTOMATIC);

  setRoll = 0;
  setPitch =0; 

}

void loop(){
    readSensor();
  Now = micros() ;//Reset the loop timer
  Freq = 1.e6/(Now - lastUpdate);
    
  calculateAngles();
  roll = measures[ROLL]; 
  quadRoll.Compute();
  pitch = measures[PITCH]; 
  quadPitch.Compute();
  #ifdef ALL
    //Serial.print(roll);
    //Serial.print(",");
    /*Serial.print(measures[PITCH]);
    Serial.print(","); 
    Serial.print(measures[YAW]);
    Serial.print(","); 
    Serial.print(angular_motions[ROLL]);
    Serial.print(",");
    Serial.print(angular_motions[PITCH]);
    Serial.print(","); 
    Serial.print(angular_motions[YAW]);
    Serial.print(",");
    */ 
    Serial.print(rollOut);
    Serial.print(",");
    Serial.print(pitchOut);
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
    Serial.print(angular_motions[YAW]);
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


 
