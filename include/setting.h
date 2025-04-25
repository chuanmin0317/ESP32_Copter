/*
    motor1  motor4
      \\      //
       \\    //
left  [ ESP32 ] right
       //    \\
      //      \\
    motor2  motor3
*/

// config
#define SERIAL_BAUD_RATE 115200
#define MAIN_LOOP_FREQUENCY_HZ 250
#define MAIN_LOOP_DELAY_US (1000000 / MAIN_LOOP_FREQUENCY_HZ)

// PID Settings
#define PID_ROLL_KP 0.0f
#define PID_ROLL_KI 0.0f
#define PID_ROLL_KD 0.0f
#define PID_PITCH_KP 0.0f
#define PID_PITCH_KI 0.0f
#define PID_PITCH_KD 0.0f
#define PID_YAW_KP 0.0f
#define PID_YAW_KI 0.0f
#define PID_YAW_KD 0.0f
#define PID_RATE_ROLL_KP 0.0f
#define PID_RATE_ROLL_KI 0.0f
#define PID_RATE_ROLL_KD 0.0f
#define PID_RATE_PITCH_KP 0.0f
#define PID_RATE_PITCH_KI 0.0f
#define PID_RATE_PITCH_KD 0.0f
#define PID_RATE_YAW_KP 0.0f
#define PID_RATE_YAW_KI 0.0f
#define PID_RATE_YAW_KD 0.0f
#define PID_INTEGRAL_LIMIT 0.0f
#define PID_RATE_INTEGRAL_LIMIT 0.0f
#define PID_OUTPUT_LIMIT 0.0f

// RC Settings
#define RC_MAP_INPUT_MAX 65535
#define RC_MAP_OUTPUT_MAX 2000
#define RC_MAP_OUTPUT_MIN 1000
#define RC_ROLL_RATE_SCALING 0.1f
#define RC_PITCH_RATE_SCALING 0.1f
#define RC_YAW_RATE_SCALING 0.15f
#define RC_THROTTLE_ARM_THRESHOLD 1030 // Value above min throttle (1000)

// Motor Settings
#define MOTOR1_PIN 13
#define MOTOR2_PIN 14
#define MOTOR3_PIN 33
#define MOTOR4_PIN 34
#define MOTOR_PWM_FREQUENCY 5000
#define MOTOR_PWM_RESOLUTION 10 // 10 bits resolution (0-1023)
#define MOTOR_PWM_MAX ((1 << MOTOR_PWM_RESOLUTION) - 1) // 1023
#define MOTOR_OUTPUT_MAX MOTOR_PWM_MAX
#define MOTOR_OUTPUT_MIN 0

// Sensor Settings
#define SDA_PIN 21
#define SCL_PIN 26
#define MPU9250_ADDRESS 0x68
#define CALIBRATION_PROMPT_TIMEOUT_MS 3000

