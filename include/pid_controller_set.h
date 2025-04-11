#ifndef PID_CONTROLLER_SET_H
#define PID_CONTROLLER_SET_H

#include "pid_controller.h"
#include "common_types.h"
#include "setting.h"

class PIDControllerSet
{
public:
    PIDControllerSet(); // Constructor

    /**
     * @brief Initializes the PID controllers for roll, pitch, and yaw.
     * Call this once in setup().
     */
    void initialize();

    /**
     * @brief Updates the state of all PID controllers.
     * Calculates PID outputs based on the provided setpoints, sensor data,
     * and time delta dt. Implements internal cascade control logic for
     * roll and pitch.
     * @param setpoints The control setpoints (from the remote controller).
     * @param mpu_data Raw MPU data (primarily for gyroscope rates).
     * @param attitude The current estimated attitude (angles).
     * @param dt Time delta since the last update, in seconds.
     */
    void update(const DroneTypes::ControlSetpoint &setpoint,
                const DroneTypes::RawMPUData &mpu_data,
                const DroneTypes::Attitude &attitude,
                float dt);

    /**
     * @brief Resets the internal state (integral term, previous error) of all PIDs.
     * Typically called during Disarm or when Failsafe is triggered.
     */
    void reset();

    /** @brief Gets the output of the Roll Rate PID controller. */
    float getRateRollOutput() const;

    /** @brief Gets the output of the Pitch Rate PID controller. */
    float getRatePitchOutput() const;

    /** @brief Gets the output of the Yaw Rate PID controller. */
    float getRateYawOutput() const;

private:
    PIDController pidRateRoll_;
    PIDController pidRatePitch_;
    PIDController pidRateYaw_;

    PIDController pidRoll_;
    PIDController pidPitch_;
    PIDController pidYaw_;
};

#endif // PID_CONTROLLER_SET_H