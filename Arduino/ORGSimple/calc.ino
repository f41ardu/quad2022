// quad2022 edition v.01
/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles() {
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    acc_raw[X] -= acc_offset[X];
    acc_raw[Y] -= acc_offset[Y];
    acc_raw[Z] -= acc_offset[Z];
 
    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / Freq);
    gyro_angle[Y] += (-gyro_raw[Y] / Freq); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (Freq * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (Freq * 180)));
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
    measures[ROLL]  = measures[ROLL]  * 0.98 + gyro_angle[X] * 0.2;
    measures[PITCH] = measures[PITCH] * 0.98 + gyro_angle[Y] * 0.2;
    measures[YAW]   = -gyro_raw[Z]; // Store the angular motion for this axis

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X];
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y];
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z];
    
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
