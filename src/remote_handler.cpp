#include "remote_handler.h"

RemoteHandler::RemoteHandler() : connected_(false)
{
    // Set default safe setpoints
    current_setpoint_ = {0.0f, 0.0f, 0.0f, RC_MAP_OUTPUT_MIN}; // Use constant from setting.h
    current_raw_input_ = {
        (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2, // Center roll
        (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2, // Center pitch
        (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2, // Center yaw
        RC_MAP_OUTPUT_MIN                            // Min throttle
    };
}

bool RemoteHandler::begin()
{
    controller_lib_.begin();
    Serial.println("Xbox Controller Library Initialized.");
    
    return true;
}

// Maps raw joystick value (0-65535 assumed) to standard RC range
uint16_t RemoteHandler::mapValue(int input)
{
    return RC_MAP_OUTPUT_MAX - ((long)input * (RC_MAP_OUTPUT_MAX - RC_MAP_OUTPUT_MIN) / RC_MAP_INPUT_MAX);
}

// Calculates setpoints based on raw mapped input
void RemoteHandler::calculateSetpoints()
{
    const int16_t midpoint = (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2;

    current_setpoint_.roll_angle_setpoint = (float)(midpoint - current_raw_input_.roll_raw) * RC_ROLL_RATE_SCALING;    // Or angle scaling
    current_setpoint_.pitch_angle_setpoint = (float)(midpoint - current_raw_input_.pitch_raw) * RC_PITCH_RATE_SCALING; // Or angle scaling (Check inversion?)
    current_setpoint_.yaw_rate_setpoint = (float)(current_raw_input_.yaw_raw - midpoint) * RC_YAW_RATE_SCALING;

    // Throttle is usually used directly from the mapped value
    current_setpoint_.throttle_value = current_raw_input_.throttle_raw;

    // --- Add Deadzone Logic (Recommended) ---
    // Example for roll:
    // const int16_t deadzone = 20; // Define in config.h: RC_STICK_DEADZONE
    // if (abs(current_raw_input_.roll_raw - midpoint) < deadzone) {
    //     current_setpoint_.roll_angle_setpoint = 0.0f;
    // }
    // Apply similarly to pitch and yaw.
    // --- End Deadzone Logic ---
}

void RemoteHandler::update()
{
    controller_lib_.onLoop(); // Essential for library processing (Bluetooth)

    connected_ = controller_lib_.isConnected();

    if (connected_)
    {
        if (controller_lib_.isWaitingForFirstNotification())
        {
            // Still waiting for the first data packet
            // Maybe set setpoints to zero/safe values
        }
        else
        {
            // Read raw joystick values (0-65535 range from library)
            int joyLHori = controller_lib_.xboxNotif.joyLHori;
            int joyLVert = controller_lib_.xboxNotif.joyLVert; // Throttle? Check axis assignment
            int joyRHori = controller_lib_.xboxNotif.joyRHori; // Roll?
            int joyRVert = controller_lib_.xboxNotif.joyRVert; // Pitch?

            // Map raw values to 1000-2000 range (or your defined MIN/MAX)
            // Double check which stick maps to which function in your setup!
            current_raw_input_.throttle_raw = mapValue(joyLVert); // Assuming Left Stick Vertical is Throttle
            current_raw_input_.yaw_raw = mapValue(joyLHori);      // Assuming Left Stick Horizontal is Yaw
            current_raw_input_.pitch_raw = mapValue(joyRVert);    // Assuming Right Stick Vertical is Pitch (Check Inversion)
            current_raw_input_.roll_raw = mapValue(joyRHori);     // Assuming Right Stick Horizontal is Roll

            // Calculate the final control setpoints based on mapped values
            calculateSetpoints();
        }
    }
    else
    {
        // Not connected: Reset setpoints to safe values (e.g., zero rates, min throttle)
        current_setpoint_ = {0.0f, 0.0f, 0.0f, RC_MAP_OUTPUT_MIN};
        current_raw_input_ = {
            (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2,
            (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2,
            (RC_MAP_OUTPUT_MAX + RC_MAP_OUTPUT_MIN) / 2,
            RC_MAP_OUTPUT_MIN};
    }
}

// --- Getter Implementations ---
bool RemoteHandler::isConnected() const
{
    return connected_;
}

DroneTypes::ControlSetpoint RemoteHandler::getSetpoint() const
{ 
    return current_setpoint_;
}

DroneTypes::RemoteInputRaw RemoteHandler::getRawInput() const
{ 
    return current_raw_input_;
}