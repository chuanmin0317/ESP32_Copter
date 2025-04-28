#include "flight_controller.h"
#include "imu_handler.h"
#include "remote_handler.h"
#include "pid_controller_set.h"
#include "motor_driver.h"
#include "setting.h"
#include <Arduino.h>

FlightController::FlightController(IMUHandler &imus, RemoteHandler &remote,
                                   PIDControllerSet &pids, MotorDriver &motors)
    : imu_handler_(imus), remote_handler_(remote),
      pid_controllers_(pids), motor_driver_(motors), armed_(false)
{
}

void FlightController::initialize()
{
    Serial.println("Initializing Flight Controller...");
    armed_ = false; // Ensure disarmed state on init
    // Initialize flight mode if added later
    Serial.println("Flight Controller Initialized.");
}

bool FlightController::isArmed() const
{
    return armed_;
}

// Private Helper Functions

bool FlightController::handleFailsafe(const DroneTypes::RemoteData &remote_data)
{
    if (!remote_data.is_connected)
    {
        if (armed_)
        {
            Serial.println("FAILSAFE: Remote disconnected!");
        }
        armed_ = false;
        pid_controllers_.reset();
        DroneTypes::MotorCommands zero_commands = {0, 0, 0, 0};
        motor_driver_.writeOutputs(zero_commands); // Stop motors
        return true;
    }
    return false;
}

void FlightController::handleDisarmed()
{
    // Ensure motors are off and PIDs are reset when disarmed
    pid_controllers_.reset();
    DroneTypes::MotorCommands zero_commands = {0, 0, 0, 0};
    motor_driver_.writeOutputs(zero_commands);
}

void FlightController::checkArmingConditions(const DroneTypes::RemoteData &remote_data)
{
    // Arming Logic:
    // 1. Must be connected.
    // 2. Throttle must be low to enable arming sequence.
    // 3. Specific stick command to arm
    // 4. Throttle low always disarms

    if (!remote_data.is_connected)
    {
        if (armed_)
        {
            Serial.println("Disarming: Remote disconnected.");
            armed_ = false;
        }
        return;
    }

    if (remote_data.setpoint.throttle_value < RC_THROTTLE_ARM_THRESHOLD)
    {
        if (armed_)
        {
            Serial.println("Disarming: Throttle low.");
            armed_ = false;
        }
    }
    else
    {
        if (!armed_)
        {
            Serial.println("Arming Sequence Met (Throttle raised).");
            armed_ = true;
            pid_controllers_.reset();
        }
    }
}

void FlightController::calculateMotorCommands(const DroneTypes::ControlSetpoint &setpoints,
                                              DroneTypes::MotorCommands &commands_out)
{
    // Gets PID outputs
    float roll_output = pid_controllers_.getRateRollOutput();
    float pitch_output = pid_controllers_.getRatePitchOutput();
    float yaw_output = pid_controllers_.getRateYawOutput();

    // get base throttle (already in 1000-2000 range)
    float throttle_base = (float)setpoints.throttle_value;

    // Convert throttle (1000-2000) to PWM base (0-1023)
    float throttle_pwm_base = map(throttle_base, RC_MAP_OUTPUT_MIN, RC_MAP_OUTPUT_MAX, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    // Ensure throttle is within limits
    throttle_pwm_base = constrain(throttle_pwm_base, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);

    // Calculate individual motor outputs
    float motor1_f = throttle_pwm_base + roll_output + pitch_output - yaw_output; // Front-Left (CW)
    float motor2_f = throttle_pwm_base + roll_output - pitch_output + yaw_output; // Rear-Left (CCW)
    float motor3_f = throttle_pwm_base - roll_output - pitch_output - yaw_output; // Rear-Right (CW)
    float motor4_f = throttle_pwm_base - roll_output + pitch_output + yaw_output; // Front-Right (CCW)

    // Limit motor outputs to valid PWM range and convert to integer type
    commands_out.motor1 = (uint16_t)constrain(motor1_f, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    commands_out.motor2 = (uint16_t)constrain(motor2_f, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    commands_out.motor3 = (uint16_t)constrain(motor3_f, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    commands_out.motor4 = (uint16_t)constrain(motor4_f, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
}

// Main Control Cycle
void FlightController::runControlCycle(float dt, const DroneTypes::SensorData &sensor_data, DroneTypes::RemoteData &remote_data)
{
    // 1. Handle Failsafe
    if (handleFailsafe(remote_data))
    {
        return;
    }

    // 2. Get Inputs
    DroneTypes::Attitude attitude = imu_handler_.getAttitude();
    DroneTypes::RawMPUData mpu_data = imu_handler_.getRawMPUData();
    DroneTypes::ControlSetpoint setpoints = remote_handler_.getSetpoint();

    // 3. Check Arming State
    checkArmingConditions(remote_data);

    // 4. Handle Disarmed State
    if (!armed_)
    {
        handleDisarmed();
        return; // If disarmed, stop further processing
    }

    // if Armed, Proceed with Control

    // 5. Update PID Controllers
    pid_controllers_.update(remote_data.setpoint, sensor_data.raw_mpu, sensor_data.attitude, dt);

    // 6. Calculate Motor Outputs
    DroneTypes::MotorCommands motor_commands;
    calculateMotorCommands(remote_data.setpoint, motor_commands);

    // 7. Write Outputs to Motors
    motor_driver_.writeOutputs(motor_commands);
}