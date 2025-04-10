#include "motor_driver.h"
#include <Arduino.h> // Required for ledcSetup, ledcAttachPin, ledcWrite

MotorDriver::MotorDriver()
{
}

void MotorDriver::begin()
{
    Serial.println("Initializing Motor PWM channels...");

    // Channel 0 for Motor 1
    ledcSetup(0, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION); 
    ledcAttachPin(MOTOR1_PIN, 0);                            
    Serial.printf("Motor 1 (Pin %d) on PWM Channel 0\n", MOTOR1_PIN);

    // Channel 1 for Motor 2
    ledcSetup(1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(MOTOR2_PIN, 1);
    Serial.printf("Motor 2 (Pin %d) on PWM Channel 1\n", MOTOR2_PIN);

    // Channel 2 for Motor 3
    ledcSetup(2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(MOTOR3_PIN, 2); 
    Serial.printf("Motor 3 (Pin %d) on PWM Channel 2\n", MOTOR3_PIN);

    // Channel 3 for Motor 4
    ledcSetup(3, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(MOTOR4_PIN, 3);
    Serial.printf("Motor 4 (Pin %d) on PWM Channel 3\n", MOTOR4_PIN);

    // Ensure motors start off (write 0 PWM)
    DroneTypes::MotorCommands zero_commands = {0, 0, 0, 0}; // Use DroneTypes:: if needed
    writeOutputs(zero_commands);
}

// Optional: Implement limit function if needed
// int16_t MotorDriver::limit(int16_t value, int16_t max_val, int16_t min_val) {
//     if (value > max_val) return max_val;
//     if (value < min_val) return min_val;
//     return value;
// }

void MotorDriver::writeOutputs(const DroneTypes::MotorCommands &commands)
{ 
    // Assuming commands.motorX values are already in the correct PWM range (e.g., 0-1023)
    // Add limiting here as a safety net if desired:
    // uint16_t pwm1 = limit(commands.motor1, MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN); // Use constants
    // uint16_t pwm2 = limit(commands.motor2, MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN);
    // uint16_t pwm3 = limit(commands.motor3, MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN);
    // uint16_t pwm4 = limit(commands.motor4, MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN);
    // ledcWrite(0, pwm1);
    // ledcWrite(1, pwm2);
    // ledcWrite(2, pwm3);
    // ledcWrite(3, pwm4);

    // Directly write without extra limiting (assuming input is already correct):
    ledcWrite(0, commands.motor1);
    ledcWrite(1, commands.motor2);
    ledcWrite(2, commands.motor3);
    ledcWrite(3, commands.motor4);
}