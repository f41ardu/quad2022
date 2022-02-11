// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int16_t gyro_raw[3] = {0,0,0};
// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
 int16_t acc_raw[3] = {0 ,0 ,0};

// Average gyro offsets of each axis in that order: X, Y, Z
int16_t gyro_offset[3] = {0, 0, 0};
// Average acc offsets of each axis in that order: X, Y, Z
int16_t acc_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0,0,0};

// Total 3D acceleration vector in m/s²
long acc_total_vector;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
double measures[3] = {0, 0, 0};
