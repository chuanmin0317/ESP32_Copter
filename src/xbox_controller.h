#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <cmath>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

class Controller
{
public:
    void begin()
    {
        xboxController.begin();
    }

    void read_controller()
    {
        if (!xboxController.isConnected())
        {
            Serial.println("Xbox Controller not connected.");
            return;
        }
        receiveValue[0] = xboxController.xboxNotif.joyLVert;
        receiveValue[1] = xboxController.xboxNotif.joyLHori;
        receiveValue[2] = xboxController.xboxNotif.joyRVert;
        receiveValue[3] = xboxController.xboxNotif.joyRHori;
    }

    void calculate_Throttle()
    {
        Throttle = abs(1 - receiveValue[1] / joystickMax);
    }

    float getThrottle()
    {
        return Throttle;
    }

    void unlock()
    {
        if (Throttle == 0 && xboxController.xboxNotif.btnXbox == 1)
        {
            Serial.println("Drone unlocked!");
            lock = false;
        }

        // 初始化
        Throttle = 0;
    }

    bool getLock()
    {
        return lock;
    }

private:
    int ChannelNumber = 4;
    float receiveValue[4] = {0, 0, 0, 0};
    float Throttle;
    bool lock = true;
    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
};

#endif