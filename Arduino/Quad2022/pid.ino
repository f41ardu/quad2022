/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Keep values in acceptable range
    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}


/**
 * Calculate PID set points on axis YAW, PITCH, ROLL
 */
void calculateSetPoints() {
    pid_set_points[YAW]   = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
}

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle         Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

/**
 * Reset all PID controller's variables.
 */
void resetPidController() {
    errors[YAW]   = 0;
    errors[PITCH] = 0;
    errors[ROLL]  = 0;

    error_sum[YAW]   = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL]  = 0;

    previous_error[YAW]   = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL]  = 0;
}


/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 */
void pidController() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_length[mode_mapping[THROTTLE]];

    // Initialize motor commands with throttle
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

    // Do not calculate anything if throttle is 0
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }

    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, minPulse, maxPulse);
    pulse_length_esc2 = minMax(pulse_length_esc2, minPulse, maxPulse);
    pulse_length_esc3 = minMax(pulse_length_esc3, minPulse, maxPulse);
    pulse_length_esc4 = minMax(pulse_length_esc4, minPulse, maxPulse);
}
