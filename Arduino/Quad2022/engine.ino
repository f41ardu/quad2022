/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.
 *
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
 *
 * @see https:// www.arduino.cc/en/Reference/PortManipulation
 */
void applyMotorSpeed() {
    
        mEsc1 = pulse_length_esc1; // Set motor 1 
        mEsc2 = pulse_length_esc2; // Set motor 2 
        mEsc3 = pulse_length_esc3; // Set motor 3
        mEsc3 = pulse_length_esc4; // Set motor 4

}

/**
 * Reset motors' pulse length to 1000µs to totally stop them.
 */
void stopAll() {
    mEsc1 = minPulse; // 1000 we use minPulse from serve2.h  
    mEsc2 = minPulse;
    mEsc3 = minPulse;
    mEsc4 = minPulse;
}
