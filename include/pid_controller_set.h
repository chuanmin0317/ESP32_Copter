#ifndef PID_CONTROLLER_SET_H
#define PID_CONTROLLER_SET_H

#include <Preferences.h>
#include "pid_controller.h"
#include "common_types.h"
#include "setting.h"

enum class PIDLoop : int
{
    RATE_ROLL = 0,
    RATE_PITCH,
    RATE_YAW,
    ANGLE_ROLL,
    ANGLE_PITCH,
    ANGLE_YAW,
    COUNT
};
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

    // Methods for Web Tuning
    bool saveSettingsToNVS();

    /**
     * @brief Updates the gains (Kp, Ki, Kd) for a specific PID loop at runtime.
     * IMPORTANT: Call this only when the drone is DISARMED for thread safety.
     * @param loop The PIDLoop identifier for the target loop.
     * @param kp New Kp value.
     * @param ki New Ki value.
     * @param kd New Kd value.
     * @return true if update was successful (valid loop identifier), false otherwise.
     */
    bool updateSingleGainSet(PIDLoop loop, float kp, float ki, float kd);

    /**
     * @brief Gets the current configuration (gains and limits) of a specific PID loop.
     * @param loop The PIDLoop identifier for the target loop.
     * @param kp [Output] Current Kp value.
     * @param ki [Output] Current Ki value.
     * @param kd [Output] Current Kd value.
     * @param int_limit [Output] Current Integral limit.
     * @param out_limit [Output] Current Output limit.
     * @return true if loopIdentifier is valid, false otherwise.
     */
    bool getSingleConfig(PIDLoop loop, float &kp, float &ki, float &kd, float &int_limit, float &out_limit) const;

private:
    PIDController pidRateRoll_;
    PIDController pidRatePitch_;
    PIDController pidRateYaw_;

    PIDController pidRoll_;
    PIDController pidPitch_;
    PIDController pidYaw_;

    Preferences preferences_;

    const PIDController *getControllerByLoop(PIDLoop loop) const;

    void loadPIDConfig(PIDLoop loop, const char *kp_key, float default_kp,
                       const char *ki_key, float default_ki,
                       const char *kd_key, float default_kd);
};

#endif // PID_CONTROLLER_SET_H