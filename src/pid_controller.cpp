// src/pid.cpp
#include "pid_controller.h"
#include <Arduino.h> // For constrain() or manual clamping

// Constructor implementation
PIDController::PIDController(float kp, float ki, float kd,
                             float integral_limit, float output_limit)
{
    setConfig(kp, ki, kd, integral_limit, output_limit);
    reset(); // Ensure clean state on creation
}

// Configuration implementation
void PIDController::setConfig(float kp, float ki, float kd, float integral_limit, float output_limit)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integral_limit_ = fabsf(integral_limit); // Ensure limit is positive
    output_limit_ = fabsf(output_limit);     // Ensure limit is positive
}

// Update implementation
float PIDController::update(float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
    {
        // Avoid division by zero or negative time steps
        return output_; // Return previous output
    }

    float error = setpoint - measurement;

    // Proportional term
    float p_term = kp_ * error;

    // Integral term with anti-windup (clamping)
    integral_ += error * dt;
    // Clamp integral term using limits
    integral_ = constrain(integral_, -integral_limit_, integral_limit_);

    float i_term = ki_ * integral_;

    // Derivative term (consider derivative on measurement later to avoid kick)
    // Basic derivative on error:
    float derivative = (error - prev_error_) / dt;
    float d_term = kd_ * derivative;

    // Calculate final output
    output_ = p_term + i_term + d_term;

    // Clamp output using limits
    output_ = constrain(output_, -output_limit_, output_limit_);

    // Store error for next derivative calculation
    prev_error_ = error;

    return output_;
}

// Reset implementation
void PIDController::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    output_ = 0.0f; // Reset output as well
}

// getOutput implementation
float PIDController::getOutput() const
{
    return output_;
}

// getIntegral implementation
float PIDController::getIntegral() const
{
    return integral_;
}

float PIDController::getKp() const
{
    return kp_;
}

float PIDController::getKi() const
{
    return ki_;
}

float PIDController::getKd() const
{
    return kd_;
}

float PIDController::getIntegralLimit() const
{
    return integral_limit_;
}

float PIDController::getOutputLimit() const
{
    return output_limit_;
}