#include <Arduino.h>
#include <Wire.h>

#include "setting.h"
#include "common_types.h"
#include "imu_handler.h"
#include "remote_handler.h"
#include "motor_driver.h"
#include "pid_controller_set.h"
#include "flight_controller.h"

IMUHandler imu_handler;
RemoteHandler remote_handler;
MotorDriver motor_driver;
PIDControllerSet pid_controllers;

FlightController flight_controller(imu_handler, remote_handler, pid_controllers, motor_driver);

uint32_t loop_start_time = 0;
float dt = 0.0f; // Time delta in seconds

void setup(){
    Serial.begin(SERIAL_BAUD_RATE);
    while(!Serial);
    Serial.println();
    Serial.println("============================");
    Serial.println(" ESP32 Drone Booting Up! ");
    Serial.println("============================");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.println("I2C Initialized.");
    delay(100);

    Serial.println("Initializing Drone Components...");

    if (!imu_handler.begin()) {
        Serial.println("IMU Initialization Failed!");
        while (1) { delay(1000); }; // Halt if IMU fails
    }

    if (!remote_handler.begin()) {
        Serial.println("INFO: Remote Handler Initialized (Waiting for connection...)");
    }

    motor_driver.begin(); // Initialize motor driver

    pid_controllers.initialize(); // Initialize PID controllers

    flight_controller.initialize(); // Initialize flight controller

    Serial.println("----------------------------");
    Serial.println(" All Systems Initialized.");
    Serial.println(" Waiting for RC Connection...");
    Serial.println("----------------------------");

    loop_start_time = micros();
}

void loop(){
    uint32_t now = micros();
    dt = (now - loop_start_time) * 1e-6f;
    loop_start_time = now;

    remote_handler.update(); // Update remote handler state
    imu_handler.update(); // Update IMU data

    flight_controller.runControlCycle(dt); // Run control cycle

    while (micros() - loop_start_time < MAIN_LOOP_DELAY_US) {
        yield();
    }
}