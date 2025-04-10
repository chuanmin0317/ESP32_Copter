#ifndef REMOTE_HANDLER_H
#define REMOTE_HANDLER_H

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "common_types.h"
#include "setting.h"

class RemoteHandler
{
public:
    RemoteHandler(); // Constructor

    /**
     * @brief Initializes the Xbox controller.
     * Call this once in setup(). Assumes the I2C bus is already initialized.
     * @return true if initialization is successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Updates the controller state.
     * Call this periodically in the main loop.
     * @return true if update is successful, false otherwise.
     */
    bool update();

    /**
     * @brief Checks if the controller is connected.
     * @return true if connected, false otherwise.
     */
    bool isConnected() const;

    /**
     * @brief Gets the latest calculated controller setpoints.
     * @return ControllerSetpoint.
     */
    DroneTypes::ControlSetpoint getSetpoint() const;

    /**
     * @brief Gets the latest raw mapped input values (1000-2000 range).
     * Useful for debugging
     * @return RemoteInputRaw struct.
     */
    DroneTypes::RemoteInputRaw getRawInput() const;

private:
    XboxSeriesXControllerESP32_asukiaaa::Core controller_lib_;

    // Internal state variables
    DroneTypes::ControlSetpoint current_setpoint_;
    DroneTypes::RemoteInputRaw current_raw_input_;
    bool is_connected_;

    /**
     * @brief Maps raw joystick values (0-65535) to a standard range (e.g., 1000-2000).
     * @param input Raw joystick value.
     * @return Mapped value.
     */
    uint16_t mapValue(int input);

    /**
     * @brief Calculates control setpoints based on the current raw input.
     */
    void calculateSetpoints();
};

#endif // REMOTE_HANDLER_H