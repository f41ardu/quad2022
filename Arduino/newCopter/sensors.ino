void initSensors() {
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");

    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    //    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    //    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
      Serial.println("Pass Selftest!");

      mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      Serial.println("MPU6050 bias");
      Serial.println(" x\t  y\t  z  ");
      Serial.print((int)(1000 * accelBias[0])); Serial.print('\t');
      Serial.print((int)(1000 * accelBias[1])); Serial.print('\t');
      Serial.print((int)(1000 * accelBias[2]));
      Serial.println(" mg");

      Serial.print(gyroBias[0], 1); Serial.print('\t');
      Serial.print(gyroBias[1], 1); Serial.print('\t');
      Serial.print(gyroBias[2], 1);
      Serial.println(" o/s");


      mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while (1) ; // Loop forever if communication doesn't happen
    }
  }
}

void imuCalc() {
  // If data ready bit set, all data registers have new data
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
    mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    aRes = mpu.getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;
    az = (float)accelCount[2] * aRes;

    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
    gRes = mpu.getGres();

    // Calculate the gyro value into actual degrees per second
    gyrox = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gyroy = (float)gyroCount[1] * gRes;
    gyroz = (float)gyroCount[2] * gRes;

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  //    if(lastUpdate - firstUpdate > 10000000uL) {
  //      beta = 0.041; // decrease filter gain after stabilized
  //      zeta = 0.015; // increase gyro bias drift gain after stabilized
  //    }
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gyrox * PI / 180.0f, gyroy * PI / 180.0f, gyroz * PI / 180.0f);
}
