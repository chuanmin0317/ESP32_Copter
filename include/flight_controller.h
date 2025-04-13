#ifndef FlightController_H
#define FlightController_H

#include "common_types.h"

class IMUHandler;
class RemoteHandler;
class PIDControllerSet;
class MotorDriver;

class FlightController
{
public:
    /**
     * @brief Constructor with Dependency Injection.
     * @param imus Pointer to the IMU handler object.
     * @param remote Pointer to the Remote handler object.
     * @param pids Pointer to the PID Controller set object.
     * @param motors Pointer to the Motor driver object.
     */
 FlightController(IMUHandler &imus, RemoteHandler &remote,
                      PIDControllerSet &pids, MotorDriver &motors);

    /**
     * @brief Initializes the flight controller.
     * Call this once in setup().
     */
    void initialize();

    /**
     * @brief Executes one cycle of the main flight control loop.
     * Reads inputs, runs calculations, and sends commands to motors.
     * Call this periodically from the main loop or a dedicated RTOS task.
     * @param dt Time delta since the last cycle, in seconds.
     */
    void runControlCycle(float dt, const DroneTypes::SensorData &sensor_data, DroneTypes::RemoteData &remote_data); 

    /**
     * @brief Gets the current arming state of the drone.
     * @return true if armed, false otherwise.
     */
    bool isArmed() const;

private:
    IMUHandler &imu_handler_;
    RemoteHandler &remote_handler_;
    PIDControllerSet &pid_controllers_;
    MotorDriver &motor_driver_;

    bool armed_;
    // Add FlightMode state here later if needed

    /**
     * @brief Checks remote connection and handles failsafe condition.
     * @return true if failsafe is active, false otherwise.
     */
    bool handleFailsafe(const DroneTypes::RemoteData& remote_data);

    /**
     * @brief Handles actions needed when the drone is disarmed
     */
    void handleDisarmed();

    /**
     * @brief Checks conditions for arming or disarming based on remote input.
     * Updates the internal armed_ state.
     * @param setpoints Current control setpoints from the remote.
     */
    void checkArmingConditions(const DroneTypes::RemoteData& remote_data);

    /**
     * @brief Calculates the individual motor commands based on PID outputs and throttle.
     * Implements the motor mixing algorithm.
     * @param setpoints Current control setpoints (primarily for throttle).
     * @param commands_out [Output] The calculated motor commands.
     */
    void calculateMotorCommands(const DroneTypes::ControlSetpoint &setpoints,
                                DroneTypes::MotorCommands &commands_out);
};

#endif // FlightController_H