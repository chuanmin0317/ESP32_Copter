#include "MyCommon.h"

void setup()
{
    Serial.begin(115200);
    ALL_Init();

    Timer = micros();
}

void loop()
{
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        if (xboxController.isWaitingForFirstNotification())
        {
            Serial.println("waiting for first notification");
        }
        else
        {
            GetImuData(&Mpu9250, &Angle);
            KalmanAngle();
            RC_Analyse();
            PidControl(0.004);
            MotorControl();
        }
    }
    else
    {
        Serial.println("not connected");
        if (xboxController.getCountFailedConnection() > 2)
        {
            ESP.restart();
        }
    }

    while (micros() - Timer < (4000))
        ;

    Timer = micros();
}