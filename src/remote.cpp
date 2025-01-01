#include "remote.h"
#include "MyCommon.h"

void ControllerInit()
{
    xboxController.begin();
}

uint16_t mapValue(int input)
{
    return 2000 - (input * 1000) / 65535;
}

void RC_Analyse()
{
    Controller.roll = mapValue(xboxController.xboxNotif.joyRHori);
    Controller.pitch = mapValue(xboxController.xboxNotif.joyRVert);
    Controller.yaw = mapValue(xboxController.xboxNotif.joyLHori);
    Controller.throttle = mapValue(xboxController.xboxNotif.joyLVert);

    pidRoll.desired = (Controller.roll - 1500) * 0.1;
    pidPitch.desired = (Controller.pitch - 1500) * 0.1;
    pidYaw.desired = (Controller.yaw - 1500) * 0.15;
}
