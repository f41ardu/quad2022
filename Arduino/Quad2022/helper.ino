/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 *
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

/**
 * Return whether the quadcopter is started.
 * To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
 * To stop the quadcopter move the left stick in bottom right corner.
 *
 * @return bool
 */
bool isStarted() {
    // When left stick is moved in the bottom left corner
    if (status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTING;
    }

    // When left stick is moved back in the center position
    if (status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTED;

        // Reset PID controller's variables to prevent bump start
        resetPidController();

        resetGyroAngles();
    }

    // When left stick is moved in the bottom right corner
    if (status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STOPPED;
        // Make sure to always stop motors when status is STOPPED
        stopAll();
    }

    return status == STARTED;
}


/**
 * Compensate battery drop applying a coefficient on output values
 */
void compensateBatteryDrop() {
    if (isBatteryConnected()) {
        mEsc1 = mEsc1 + (unsigned) mEsc1 * ((1240 - battery_voltage) / (float) 3500); // check if parameter 1240 and 3500 fit
        mEsc2 = mEsc2 + (unsigned) mEsc2 * ((1240 - battery_voltage) / (float) 3500);
        mEsc3 = mEsc3 + (unsigned) mEsc3 * ((1240 - battery_voltage) / (float) 3500);
        mEsc4 = mEsc4 + (unsigned) mEsc4 * ((1240 - battery_voltage) / (float) 3500);
    }
}

/**
 * Read battery voltage & return whether the battery seems connected
 *
 * @return boolean
 */
bool isBatteryConnected() {
    // Reduce noise with a low-pass filter (10Hz cutoff frequency)
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less convenient but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 * @see https://www.firediy.fr/article/utiliser-sa-radiocommande-avec-un-arduino-drone-ch-6
 */
ISR(PCINT0_vect) {
        current_time = micros();

        // Channel 1 -------------------------------------------------
        if (PINB & B00000001) {                                        // Is input 8 high ?
            if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL1] = HIGH;                       // Save current state
                timer[CHANNEL1] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL1] == HIGH) {                 // Input 8 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL1] = LOW;                            // Save current state
            pulse_length[CHANNEL1] = current_time - timer[CHANNEL1];   // Calculate pulse duration & save it
        }

        // Channel 2 -------------------------------------------------
        if (PINB & B00000010) {                                        // Is input 9 high ?
            if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL2] = HIGH;                       // Save current state
                timer[CHANNEL2] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL2] == HIGH) {                 // Input 9 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL2] = LOW;                            // Save current state
            pulse_length[CHANNEL2] = current_time - timer[CHANNEL2];   // Calculate pulse duration & save it
        }

        // Channel 3 -------------------------------------------------
        if (PINB & B00000100) {                                        // Is input 10 high ?
            if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL3] = HIGH;                       // Save current state
                timer[CHANNEL3] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL3] == HIGH) {                 // Input 10 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL3] = LOW;                            // Save current state
            pulse_length[CHANNEL3] = current_time - timer[CHANNEL3];   // Calculate pulse duration & save it
        }

        // Channel 4 -------------------------------------------------
        if (PINB & B00001000) {                                        // Is input 11 high ?
            if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL4] = HIGH;                       // Save current state
                timer[CHANNEL4] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL4] == HIGH) {                 // Input 11 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL4] = LOW;                            // Save current state
            pulse_length[CHANNEL4] = current_time - timer[CHANNEL4];   // Calculate pulse duration & save it
        }
}
