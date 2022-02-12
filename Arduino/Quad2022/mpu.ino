// quad2022 edition v.02
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
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission(true);              // End the transmission
}

void readSensor() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS,14,true);
  // Wait until all the bytes are received
  while(Wire.available() < 14);
  acc_raw[X]=(Wire.read()<<8|Wire.read()); 
  acc_raw[Y]=(Wire.read()<<8|Wire.read()); 
  acc_raw[Z]=(Wire.read()<<8|Wire.read()); 
  Temp=Wire.read()<<8|Wire.read();
  gyro_raw[X]=(Wire.read()<<8|Wire.read()); 
  gyro_raw[Y]=(Wire.read()<<8|Wire.read()); 
  gyro_raw[Z]=(Wire.read()<<8|Wire.read());
}

/**
 * Calibrate MPU6050: take 2000 samples to calculate average offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 *
 * This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.
 *
 * This function might take ~2sec for 2000 samples.
 */
void calibrateMpu6050() {
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++) {
        readSensor();

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];
        acc_offset[X] += acc_raw[X];
        acc_offset[Y] += acc_raw[Y];
        acc_offset[Z] += acc_raw[Z];
     
        // Generate low throttle pulse to init ESC and prevent them beeping (need to be replaced in case of brushed engines)
        PORTD |= B11110000;      // Set pins #4 #5 #6 #7 HIGH
        delayMicroseconds(1000); // Wait 1000µs
        PORTD &= B00001111;      // Then set LOW

        // Just wait a bit before next loop
        delay(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;

    acc_offset[X] /= max_samples;
    acc_offset[Y] /= max_samples;
    acc_offset[Z] /= max_samples;

}
