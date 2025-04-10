#include "MyCommon.h"
#include "imu_handler.h"

XboxSeriesXControllerESP32_asukiaaa::Core controller;
MPU9250 mpu9250;

void setup()
{
    Serial.begin(115200);
    ALL_Init();

    Timer = micros();
}

void loop()
{
    GetImuData(&Mpu9250, &Angle);
    controller.onLoop();
    if (controller.isConnected())
    {
        if (controller.isWaitingForFirstNotification())
        {
            Serial.println("waiting for first notification");
        }
        else
        {
            // KalmanAngle();
            RC_Analyse();
            PidControl(0.004);
            MotorControl();
        }
    }
    else
    {
        Serial.println("not connected");
        if (controller.getCountFailedConnection() > 2)
        {
            ESP.restart();
        }
    }

    while (micros() - Timer < (4000))
    {
    }

    Timer = micros();
}