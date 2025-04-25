#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "common_types.h" // For MotorCommands struct
#include "setting.h"       // For motor pins and PWM settings

// Assuming you might use the DroneTypes namespace from common_types.h
// using namespace DroneTypes;
// Or qualify types with DroneTypes:: below

class MotorDriver {
public:
    MotorDriver(); // Constructor

    /**
     * @brief Initializes the LEDC PWM channels for all motors.
     * Call this once in setup().
     */
    void begin();

    /**
     * @brief Writes the given commands (PWM duty cycle values) to the motors.
     * Assumes the values in 'commands' are already scaled to the correct PWM range
     * (e.g., 0-1023 for 10-bit resolution). Clamping/limiting should ideally
     * happen before calling this, but a safety limit can be added here.
     * @param commands A struct containing the PWM values for motor1 to motor4.
     */
    void writeOutputs(const DroneTypes::MotorCommands& commands); // Use DroneTypes:: if needed

private:
    // Optional: Helper for limiting values (could be public or static too)
    // int16_t limit(int16_t value, int16_t max_val, int16_t min_val);
};

#endif // MOTOR_DRIVER_H