// quad2022 edition v.02
/**
 * Calculate pitch & roll angles using only the gyro.
 */


void calculateGyroAngles() {
    // Subtract offsets

    // Your offsets: 41  -74 1224  174 -17 -15
//    mpu.setXGyroOffset(174);
//    mpu.setYGyroOffset(-17);
//    mpu.setZGyroOffset(-15);
//    mpu.setXAccelOffset(41);
//    mpu.setYAccelOffset(-74);
//    mpu.setZAccelOffset(1224);
    gyro_raw[X] = gyro_raw[X] + gyro_offset[X] + (  174);
    gyro_raw[Y] = gyro_raw[Y] + gyro_offset[Y] + (  -17); 
    gyro_raw[Z] = gyro_raw[Z] + gyro_offset[Z] + (  -15);

    acc_raw[X] = acc_raw[X] + acc_offset[X] + (  41);
    acc_raw[Y] = acc_raw[Y] + acc_offset[Y] + ( -74);
    acc_raw[Z] = acc_raw[Z] + acc_offset[Z] + (1224);
 
    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (Freq * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (Freq * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (Freq * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (Freq * SSF_GYRO * 180)));
}

/**
 * Calculate real angles from gyro and accelerometer's values
 */
void calculateAngles() {
    
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
        // Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();
        initialized = true;
    }
    // To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL]  = measures[ROLL]  * 0.98 + gyro_angle[X] * 0.02;
    measures[PITCH] = measures[PITCH] * 0.98 + gyro_angle[Y] * 0.02;
    measures[YAW]   = measures[YAW]    * 0.98 +  (-gyro_raw[Z] *0.02) / SSF_GYRO; // Store the angular motion for this axis

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO ;
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
    
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles() {
      
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}
