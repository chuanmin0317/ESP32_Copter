#include "pid_controller_set.h"
#include <Arduino.h>

PIDControllerSet::PIDControllerSet()
{
}

void PIDControllerSet::initialize()
{
    Serial.println("Initializing PID Controllers...");

    // Rate Loops
    pidRateRoll_.setConfig(PID_RATE_ROLL_KP, PID_RATE_ROLL_KI, PID_RATE_ROLL_KD,
                           PID_RATE_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidRatePitch_.setConfig(PID_RATE_PITCH_KP, PID_RATE_PITCH_KI, PID_RATE_PITCH_KD,
                            PID_RATE_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidRateYaw_.setConfig(PID_RATE_YAW_KP, PID_RATE_YAW_KI, PID_RATE_YAW_KD,
                          PID_RATE_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    
    // Angle Loops
    pidRoll_.setConfig(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
                       PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidPitch_.setConfig(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
                        PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidYaw_.setConfig(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
                      PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);

    reset(); // Reset all PIDs to ensure clean state
    Serial.println("PID Controllers Initialized.");
}

void PIDControllerSet::update(const DroneTypes::ControlSetpoint& setpoints,
                            const DroneTypes::RawMPUData& mpu_data,
                            const DroneTypes::Attitude& attitude,
                            float dt)
{
    // 1. Extract Setpoints and Measurements
    // Setpoints
    float rollAngleSetpoint = setpoints.roll_angle_setpoint; 
    float pitchAngleSetpoint = setpoints.pitch_angle_setpoint; 
    float yawRateSetpoint = setpoints.yaw_rate_setpoint;

    // Measurements
    float rollAngleMeas = attitude.roll;
    float pitchAngleMeas = attitude.pitch;

    // Angular Rate Measurements
    float rollRateMeas = mpu_data.gyroX; 
    float pitchRateMeas = mpu_data.gyroY; 
    float yawRateMeas = mpu_data.gyroZ; 

    // 2. Calculate PID

    // Roll Cascode Control
    pidRoll_.update(rollAngleSetpoint, rollAngleMeas, dt);
    float rollRateSetpoint = pidRoll_.getOutput();
    pidRateRoll_.update(rollRateSetpoint, rollRateMeas, dt);

    // Pitch Cascode Control
    pidPitch_.update(pitchAngleSetpoint, pitchAngleMeas, dt);
    float pitchRateSetpoint = pidPitch_.getOutput();
    pidRatePitch_.update(pitchRateSetpoint, pitchRateMeas, dt);

    // Yaw Rate Control
    pidRateYaw_.update(yawRateSetpoint, yawRateMeas, dt);
}

void PIDControllerSet::reset()
{
    pidRateRoll_.reset();
    pidRatePitch_.reset();
    pidRateYaw_.reset();
    pidRoll_.reset();
    pidPitch_.reset();
    pidYaw_.reset();
    Serial.println("PID states reset.");
}

// Getters for outputs
float PIDControllerSet::getRateRollOutput() const
{
    return pidRateRoll_.getOutput();
}

float PIDControllerSet::getRatePitchOutput() const
{
    return pidRatePitch_.getOutput();
}

float PIDControllerSet::getRateYawOutput() const
{
    return pidRateYaw_.getOutput();
}