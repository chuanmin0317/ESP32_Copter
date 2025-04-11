// src/pid.h
#ifndef PID_CONTROLLER_H // Changed include guard name
#define PID_CONTROLLER_H

#include <stdint.h> // Keep for standard types if needed, though float is used mainly
#include <math.h>   // For fabs() if used for limits, or remove if not needed

class PIDController {
public:
    /**
     * @brief Constructor. Initializes PID controller with gains and limits.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param integral_limit Absolute limit for the integral term.
     * @param output_limit Absolute limit for the controller output.
     */
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                  float integral_limit = 400.0f, float output_limit = 400.0f);

    /**
     * @brief Sets or updates the PID controller's configuration.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param integral_limit Absolute limit for the integral term.
     * @param output_limit Absolute limit for the controller output.
     */
    void setConfig(float kp, float ki, float kd, float integral_limit, float output_limit);

    /**
     * @brief Updates the PID controller calculation.
     * @param setpoint The desired value (target).
     * @param measurement The current measured value.
     * @param dt Time delta since the last update, in seconds.
     * @return The calculated control output.
     */
    float update(float setpoint, float measurement, float dt);

    /**
     * @brief Resets the controller's internal state (integral and previous error).
     */
    void reset();

    /**
     * @brief Gets the last calculated output value.
     * @return The last output value.
     */
    float getOutput() const;

    /**
     * @brief Gets the current integral term value (for debugging/tuning).
     * @return The current integral value.
     */
    float getIntegral() const;

private:
    // Configuration Parameters
    float kp_;
    float ki_;
    float kd_;
    float integral_limit_;
    float output_limit_;

    // Internal State
    float integral_;
    float prev_error_;
    float output_;
    // Note: We don't strictly need to store setpoint/measurement as members
    // unless needed for other logic, they are passed to update().
};

#endif // PID_CONTROLLER_H