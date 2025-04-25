#include "pid_controller_set.h"
#include <Arduino.h>

const char *NVS_PID_NAMESPACE = "pid-settings";
const char *KEY_PID_RATE_R_KP = "prr_kp";
const char *KEY_PID_RATE_R_KI = "prr_ki";
const char *KEY_PID_RATE_R_KD = "prr_kd";
const char *KEY_PID_RATE_P_KP = "prp_kp";
const char *KEY_PID_RATE_P_KI = "prp_ki";
const char *KEY_PID_RATE_P_KD = "prp_kd";
const char *KEY_PID_RATE_Y_KP = "pry_kp";
const char *KEY_PID_RATE_Y_KI = "pry_ki";
const char *KEY_PID_RATE_Y_KD = "pry_kd";
const char *KEY_PID_ANG_R_KP = "par_kp";
const char *KEY_PID_ANG_R_KI = "par_ki";
const char *KEY_PID_ANG_R_KD = "par_kd";
const char *KEY_PID_ANG_P_KP = "pap_kp";
const char *KEY_PID_ANG_P_KI = "pap_ki";
const char *KEY_PID_ANG_P_KD = "pap_kd";
const char *KEY_PID_ANG_Y_KP = "pay_kp";
const char *KEY_PID_ANG_Y_KI = "pay_ki";
const char *KEY_PID_ANG_Y_KD = "pay_kd";

PIDControllerSet::PIDControllerSet()
{
}

const PIDController *PIDControllerSet::getControllerByLoop(PIDLoop loop) const
{
    switch (loop)
    {
    case PIDLoop::RATE_ROLL:
        return &pidRateRoll_;
    case PIDLoop::RATE_PITCH:
        return &pidRatePitch_;
    case PIDLoop::RATE_YAW:
        return &pidRateYaw_;
    case PIDLoop::ANGLE_ROLL:
        return &pidRoll_;
    case PIDLoop::ANGLE_PITCH:
        return &pidPitch_;
    case PIDLoop::ANGLE_YAW:
        return &pidYaw_;
    default:
        return nullptr; // Invalid loop
    }
}

void PIDControllerSet::loadPIDConfig(PIDLoop loop, const char *kp_key, float default_kp,
                                     const char *ki_key, float default_ki,
                                     const char *kd_key, float default_kd)
{
    PIDController *controller = getControllerByLoop(loop);
    if (!controller)
        return;

    float kp = preferences_.getFloat(kp_key, default_kp);
    float ki = preferences_.getFloat(ki_key, default_ki);
    float kd = preferences_.getFloat(kd_key, default_kd);
    float int_limit = PID_INTEGRAL_LIMIT;
    float out_limit = PID_OUTPUT_LIMIT;

    controller->setConfig(kp, ki, kd, int_limit, out_limit);
}

void PIDControllerSet::initialize()
{
    Serial.println("Initializing PID Controllers...");
    if (!preferences_.begin(NVS_PID_NAMESPACE, true))
    {
        Serial.println("Failed to open PID NVS namespace (read-only). Using default values.");
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
    }
    else
    {
        loadPIDConfig(PIDLoop::RATE_ROLL, KEY_PID_RATE_R_KP, PID_RATE_ROLL_KP, KEY_PID_RATE_R_KI, PID_RATE_ROLL_KI, KEY_PID_RATE_R_KD, PID_RATE_ROLL_KD);
        loadPIDConfig(PIDLoop::RATE_PITCH, KEY_PID_RATE_P_KP, PID_RATE_PITCH_KP, KEY_PID_RATE_P_KI, PID_RATE_PITCH_KI, KEY_PID_RATE_P_KD, PID_RATE_PITCH_KD);
        loadPIDConfig(PIDLoop::RATE_YAW, KEY_PID_RATE_Y_KP, PID_RATE_YAW_KP, KEY_PID_RATE_Y_KI, PID_RATE_YAW_KI, KEY_PID_RATE_Y_KD, PID_RATE_YAW_KD);
        loadPIDConfig(PIDLoop::ANGLE_ROLL, KEY_PID_ANG_R_KP, PID_ROLL_KP, KEY_PID_ANG_R_KI, PID_ROLL_KI, KEY_PID_ANG_R_KD, PID_ROLL_KD);
        loadPIDConfig(PIDLoop::ANGLE_PITCH, KEY_PID_ANG_P_KP, PID_PITCH_KP, KEY_PID_ANG_P_KI, PID_PITCH_KI, KEY_PID_ANG_P_KD, PID_PITCH_KD);
        loadPIDConfig(PIDLoop::ANGLE_YAW, KEY_PID_ANG_Y_KP, PID_YAW_KP, KEY_PID_ANG_Y_KI, PID_YAW_KI, KEY_PID_ANG_Y_KD, PID_YAW_KD);
        preferences_.end(); // 關閉 NVS
        Serial.println("PID gains loaded from NVS or defaults.");
    }

    reset(); // Reset all PIDs to ensure clean state
    Serial.println("PID Controllers Initialized.");
}

void PIDControllerSet::update(const DroneTypes::ControlSetpoint &setpoints,
                              const DroneTypes::RawMPUData &mpu_data,
                              const DroneTypes::Attitude &attitude,
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
    // Serial.println("PID states reset.");
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

bool PIDControllerSet::saveSettingsToNVS()
{
    Serial.println("Saving PID settings to NVS...");
    if (!preferences_.begin(NVS_PID_NAMESPACE, false))
    {
        Serial.println(" Failed to open PID NVS namespace (read-write). Cannot save settings.");
        return false;
    }

    bool success = true;

    success &= (preferences_.putFloat(KEY_PID_RATE_R_KP, pidRateRoll_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_R_KI, pidRateRoll_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_R_KD, pidRateRoll_.getKd()) > 0);

    success &= (preferences_.putFloat(KEY_PID_RATE_P_KP, pidRatePitch_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_P_KI, pidRatePitch_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_P_KD, pidRatePitch_.getKd()) > 0);

    success &= (preferences_.putFloat(KEY_PID_RATE_Y_KP, pidRateYaw_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_Y_KI, pidRateYaw_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_RATE_Y_KD, pidRateYaw_.getKd()) > 0);

    success &= (preferences_.putFloat(KEY_PID_ANG_R_KP, pidRoll_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_R_KI, pidRoll_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_R_KD, pidRoll_.getKd()) > 0);

    success &= (preferences_.putFloat(KEY_PID_ANG_P_KP, pidPitch_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_P_KI, pidPitch_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_P_KD, pidPitch_.getKd()) > 0);

    success &= (preferences_.putFloat(KEY_PID_ANG_Y_KP, pidYaw_.getKp()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_Y_KI, pidYaw_.getKi()) > 0);
    success &= (preferences_.putFloat(KEY_PID_ANG_Y_KD, pidYaw_.getKd()) > 0);

    preferences_.end();

    if (success)
    {
        Serial.println("PID settings saved successfully.");
    }
    else
    {
        Serial.println("ERROR: Failed to save one or more PID settings!");
    }
    return success;
}

bool PIDControllerSet::updateSingleGainSet(PIDLoop loop, float kp, float ki, float kd)
{
    PIDController *controller = getControllerByLoop(loop);
    if (!controller)
    {
        Serial.printf("ERROR: Invalid PID loop identifier (%d) in updateSingleGainSet.\n", (int)loop);
        return false;
    }

    // Get current limits before setting new config
    float current_int_limit = controller->getIntegralLimit();
    float current_out_limit = controller->getOutputLimit();

    // Update the controller's config with new gains and existing limits
    controller->setConfig(kp, ki, kd, current_int_limit, current_out_limit);

    Serial.printf("PID Loop %d gains updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", (int)loop, kp, ki, kd);
    return true;
}

bool PIDControllerSet::getSingleConfig(PIDLoop loop, float &kp, float &ki, float &kd, float &int_limit, float &out_limit) const
{
    const PIDController *controller = getControllerByLoop(loop);
    if (!controller)
    {
        Serial.printf("ERROR: Invalid PID loop identifier (%d) in getSingleConfig.\n", (int)loop);
        return false;
    }

    kp = controller->getKp();
    ki = controller->getKi();
    kd = controller->getKd();
    int_limit = controller->getIntegralLimit();
    out_limit = controller->getOutputLimit();
    return true;
}