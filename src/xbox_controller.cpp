#include "xbox_controller.h"

void Controller::begin()
{
    xboxController.begin();
}

void Controller::onLoop()
{
    xboxController.onLoop();
}

bool Controller::isConnected()
{
    return xboxController.isConnected();
}

bool Controller::isWaitingForFirstNotification()
{
    return xboxController.isWaitingForFirstNotification();
}

int Controller::getCountFailedConnection()
{
    return xboxController.getCountFailedConnection();
}

void Controller::read_controller()
{

    receiveValue[0] = xboxController.xboxNotif.joyLHori;
    receiveValue[1] = xboxController.xboxNotif.joyLVert;
    receiveValue[2] = xboxController.xboxNotif.joyRHori;
    receiveValue[3] = xboxController.xboxNotif.joyRVert;
}

void Controller::calculate_Throttle()
{
    Throttle = abs(1 - receiveValue[1] / joystickMax);
    Serial.println(Throttle);
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
    else
    {
        Serial.println("Lock!");
        return;
    }

    // 初始化
    Throttle = 0;
}

bool Controller::getLock()
{
    return lock;
}