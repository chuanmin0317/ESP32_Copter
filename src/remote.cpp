#include "remote.h"
#include "MyCommon.h"

extern XboxSeriesXControllerESP32_asukiaaa::Core controller;

void ControllerInit()
{
    controller.begin();
}

uint16_t mapValue(int input)
{
    return 2000 - (input * 1000) / 65535;
}

void RC_Analyse()
{
    Controller.roll = mapValue(controller.xboxNotif.joyRHori);
    Controller.pitch = mapValue(controller.xboxNotif.joyRVert);
    Controller.yaw = mapValue(controller.xboxNotif.joyLHori);
    Controller.throttle = mapValue(controller.xboxNotif.joyLVert);

    pidRoll.desired = (Controller.roll - 1500) * 0.1;
    pidPitch.desired = (Controller.pitch - 1500) * 0.1;
    pidYaw.desired = (Controller.yaw - 1500) * 0.15;
}
