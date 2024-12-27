#include "sensors.h"
#include <Wire.h>
#include <EEPROM.h>

bool SensorManger::begin(int sda, int scl, uint8_t address)
{
    Wire.begin(sda, scl);
    delay(2000);

    if (!mpu9250.setup(address))
    {
        return false;
    }

    
    initialized = true;
    return true;
}

