// quad2022 edition v.00
// see also https://medium.com/@kavindugimhanzoysa/lets-work-with-mpu6050-gy-521-part1-6db0d47a35e6

// output 
// #define ALL 
#define THROTTLEOUT
 
#include <Wire.h>
#include "servo2.h"
#include "droneflightcontroller.h"

void setup(){
  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.
  setupMPURegister(); 
  calibrateMpu6050(); 
  configureChannelMapping();
  // Configure interrupts for receiver
  //PCICR  |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
  //PCMSK0 |= (1 << PCINT0); // Set PCINT0 (digital input 8) to trigger an interrupt on state change
  //PCMSK0 |= (1 << PCINT1); // Set PCINT1 (digital input 9) to trigger an interrupt on state change
  //PCMSK0 |= (1 << PCINT2); // Set PCINT2 (digital input 10)to trigger an interrupt on state change
  //PCMSK0 |= (1 << PCINT3); // Set PCINT3 (digital input 11)to trigger an interrupt on state change
  
  lastUpdate = micros();  
  Serial.begin(115200);
   
}

void loop(){
  
   // 1. First, read raw values from MPU-6050
  readSensor();
  // Measure update freqncy 
  Now = micros() ;//Reset the loop timer
  Freq = 250; //1.e6/(Now - lastUpdate);
  // 2. Calculate angles from gyro & accelerometer's values
  calculateAngles(); 

  // 3. Calculate set points of PID controller
    calculateSetPoints();

    // 4. Calculate errors comparing angular motions to set points
    calculateErrors();

     // 4. Calculate errors comparing angular motions to set points
    calculateErrors();
/*
    if (isStarted()) {
        // 5. Calculate motors speed with PID controller
        pidController();

        compensateBatteryDrop();
    }
*/
    // 6. Apply motors speed
     pidController();
    applyMotorSpeed();

  
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
    Serial.println(angular_motions[YAW]);
  #endif
  #ifdef THROTTLEOUT
    Serial.print(pulse_length_esc1);
    Serial.print(",");
    Serial.print(pulse_length_esc2);
    Serial.print(","); 
    Serial.print(pulse_length_esc3);
    Serial.print(","); 
    Serial.println(pulse_length_esc4);
  #endif
  pulse_length[mode_mapping[YAW]] = 1500; 
  pulse_length[mode_mapping[PITCH]] =1500; 
  pulse_length[mode_mapping[ROLL]] = 1500; 
  pulse_length[mode_mapping[THROTTLE]] = 1300; 
  status = STARTED;
  lastUpdate = Now;
  //delay(100);
}
