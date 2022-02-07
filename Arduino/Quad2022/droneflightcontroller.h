// ------------------- Define some constants for convenience -----------------
// RC Channel
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

// RC Mapping
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

// axis
#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

// MPU 
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define SCALE_ACC  8182 
#define SCALE_GYRO 65.5

#define STOPPED  0
#define STARTING 1
#define STARTED  2
// ---------------- Receiver variables ---------------------------------------
// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];
// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = {0,0,0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = {0 ,0 ,0};


// Average acc offsets of each axis in that order: X, Y, Z
long acc_offset[3] = {0, 0, 0};

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
float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
// unsigned int  period; // Sampling period
// unsigned long loop_timer;
// unsigned long difference;

// integration timer 
int  lastUpdate = 0; 
int  Freq, Now;
// MPU Temperature
int Temp; // not used 

// Set pins for brushed engines #4 #5 #6 #7
tservo mEsc1(4,true),mEsc2(5,true), mEsc3(6,true),mEsc4(7,true); // will be renamed later 

unsigned long pulse_length_esc1 = minPulse,
        pulse_length_esc2 = minPulse,
        pulse_length_esc3 = minPulse,
        pulse_length_esc4 = minPulse;

// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll
// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 *   - 0 : stopped
 *   - 1 : starting
 *   - 2 : started
 *
 * @var int
 */
int status = STOPPED;
// ---------------------------------------------------------------------------
int battery_voltage;
// ---------------------------------------------------------------------------
