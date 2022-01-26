// #define MPU6050_CFG_DLPF_CFG_BIT    2
// #define MPU6050_CFG_DLPF_CFG_LENGTH 3

// Options:
/*
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03  // ****
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06
*/
#define MPU6050_DLPF_REG            0x1A
#define MPU6050_DLPF_BW_42          0x03  // 

void setupMPURegister() {
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission(true);              // End the transmission

    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission(true);              // End the transmission

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission(true);              // End the transmission

    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(#define MPU6050_DLPF_REG);                    // Request the CONFIG register
    Wire.write(MPU6050_DLPF_BW_42);      // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission(true);              // End the transmission
    
}

void readSensor() {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS,14,true);
  // Wait until all the bytes are received
  while(Wire.available() < 14);
  acc_raw[X]=Wire.read()<<8|Wire.read();
  acc_raw[Y]=Wire.read()<<8|Wire.read();
  acc_raw[Z]=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  gyro_raw[X]=Wire.read()<<8|Wire.read();
  gyro_raw[Y]=Wire.read()<<8|Wire.read();
  gyro_raw[Z]=Wire.read()<<8|Wire.read();

  if(initialized != true) {
     for (int i = 0; i < max_samples; i++) {
        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];
        // Just wait a bit before next loop
        delay(3);
    } 
    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
    initialized = true;   
  } else {   // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];
  } 
}

void getQuaternions() {
  readSensor();
  if(lastUpdate - firstUpdate > 10000000uL) {
      beta = 0.041; // decrease filter gain after stabilized 0.041
      zeta = 0.015; // increase gyro bias drift gain after stabilized 0.015
      digitalWrite(LED_BUILTIN, HIGH); 
   }
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  MadgwickQuaternionUpdate(ares*acc_raw[X], ares*acc_raw[Y], ares*acc_raw[Z], gres*gyro_raw[X] * pi_180, gres*gyro_raw[Y] * pi_180, gres*gyro_raw[Z] * pi_180); 
}

/*

// Define Tait-Bryan angles.
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;
 */
