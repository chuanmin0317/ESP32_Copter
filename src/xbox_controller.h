#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <cmath>

class Controller
{
public:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    void onLoop();

    void begin();

    bool isConnected();

    bool isWaitingForFirstNotification();

    int getCountFailedConnection();

    void read_controller();

    void calculate_Throttle();

    float getThrottle();

    void unlock();

    bool getLock();

private:
    int ChannelNumber = 4;
    float receiveValue[4] = {0, 0, 0, 0};
    float Throttle;
    bool lock = true;
    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
};

#endif