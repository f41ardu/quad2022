
// quad2022 edition v.02
// see also https://medium.com/@kavindugimhanzoysa/lets-work-with-mpu6050-gy-521-part1-6db0d47a35e6


// output 
#define ALL
//#define MEASURES
//#define ANGULAR
//#define GYROANGLE

#include <Wire.h>
#include <PID_v1.h>
#include "definitions.h"
#include "mpu.h"

//Define Variables we'll be connecting to
// Roll
double setRoll, inRoll, rollOut;
double kpRoll=1.3, kiRoll=0.04, kdRoll=3.0;
double roll;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,P_ON_M, DIRECT);
PID quadRoll(&roll, &rollOut, &setRoll, kpRoll, kiRoll, kdRoll, P_ON_M, REVERSE); 

// Pitch
double setPitch, inPitch, pitchOut;
double kpPitch=1.3, kiPitch=0.04, kdPitch=3.;
double pitch;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID quadPitch(&pitch, &pitchOut, &setPitch, kpPitch, kiPitch, kdPitch,P_ON_M, REVERSE);

// Angular_Motion Yaw
double setAYaw, inAYaw,AYawOut;
double kpAYaw=4.0, kiAYaw=0.02, kdAYaw=0.0;
double aYaw;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID quadAYaw(&aYaw, &AYawOut, &setAYaw, kpAYaw, kiAYaw, kdAYaw,P_ON_M, DIRECT);


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
  quadRoll.SetOutputLimits(-400, 400);
  quadRoll.SetMode(AUTOMATIC);

  quadPitch.SetSampleTime(1);
  quadPitch.SetOutputLimits(-400, 400);
  quadPitch.SetMode(AUTOMATIC);

  quadAYaw.SetSampleTime(1);
  quadAYaw.SetOutputLimits(-50,50);
  quadAYaw.SetMode(AUTOMATIC);

// receiver

  setRoll  = 0;
  setPitch = 0;
  setAYaw  = 0;

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
  aYaw = measures[YAW]; 
  quadAYaw.Compute();
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
    /*
     *  // Calculate pulse duration for each ESC
       pulse_length_esc1 = throttle - int(roll_pid) - int(pitch_pid) + int(yaw_pid);
       pulse_length_esc2 = throttle + int(roll_pid) - int(pitch_pid) - int(yaw_pid);
       pulse_length_esc3 = throttle - int(roll_pid) + int(pitch_pid) - int(yaw_pid);
       pulse_length_esc4 = throttle + int(roll_pid) + int(pitch_pid) + int(yaw_pid);
     */
    Serial.print(1500-(rollOut-pitchOut)/2 + AYawOut);
    Serial.print(",");
    Serial.print(1500+(rollOut-pitchOut)/2 - AYawOut);
    Serial.print(",");
    Serial.print(1500-(rollOut+pitchOut)/2 - AYawOut);
    Serial.print(",");
    Serial.print(1500+(rollOut+pitchOut)/2 + AYawOut);
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


 
