

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles() {
   
 
    // Angle calculation using integration
    gyro_angle[X] += (gyrox *deltat );
    gyro_angle[Y] += (-gyroy *deltat); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyroz * (PI / (deltat * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyroz * (PI / (deltat * 180)));
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
    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW]   = -gyroz*deltat; // Store the angular motion for this axis

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyrox;
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyroy;
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyroz;
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles() {
      
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(ax) < acc_total_vector) {
        acc_angle[X] = asin((float)ay / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(ay) < acc_total_vector) {
        acc_angle[Y] = asin((float)ax / acc_total_vector) * (180 / PI);
    }
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}
