#include "xbox_controller.h"

void Controller::begin()
{
    xboxController.begin();
}

void Controller::read_controller()
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

void Controller::calculate_Throttle()
{
    Throttle = abs(1 - receiveValue[1] / joystickMax);
}

float Controller::getThrottle()
{
    return Throttle;
}

void Controller::unlock()
{
    if (Throttle == 0 && xboxController.xboxNotif.btnXbox == 1)
    {
        Serial.println("Drone unlocked!");
        lock = false;
    }

    // 初始化
    Throttle = 0;
}

bool Controller::getLock()
{
    return lock;
}