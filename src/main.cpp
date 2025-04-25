#include <Arduino.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "setting.h"
#include "common_types.h"
#include "imu_handler.h"
#include "remote_handler.h"
#include "motor_driver.h"
#include "pid_controller_set.h"
#include "flight_controller.h"
#include "web_server_manager.h"

// Task Definitions
#define IMU_TASK_STACK_SIZE 4096
#define IMU_TASK_PRIORITY 5
#define IMU_TASK_CORE 1
#define IMU_TASK_DELAY_MS 4 // 250Hz

#define REMOTE_TASK_STACK_SIZE 4096
#define REMOTE_TASK_PRIORITY 4
#define REMOTE_TASK_CORE 0
#define REMOTE_TASK_DELAY_MS 20 // 50Hz

#define FLIGHTCONTROL_TASK_STACK_SIZE 8196
#define FLIGHTCONTROL_TASK_PRIORITY 5
#define FLIGHTCONTROL_TASK_CORE 1
#define FLIGHTCONTROL_TASK_DELAY_MS 4 // 250Hz

#define STATUS_TASK_STACK_SIZE 3072
#define STATUS_TASK_PRIORITY 2
#define STATUS_TASK_CORE 0
#define STATUS_TASK_DELAY_MS 100 // 10Hz

// Global Objects Instances
IMUHandler imu_handler;
RemoteHandler remote_handler;
MotorDriver motor_driver;
PIDControllerSet pid_controllers;
FlightController flight_controller(imu_handler, remote_handler, pid_controllers, motor_driver);
WebServerManager web_server_manager(pid_controllers, flight_controller);

// FreeRTOS Handles
QueueHandle_t imuDataQueue = NULL;
QueueHandle_t remoteDataQueue = NULL;

TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t remoteTaskHandle = NULL;
TaskHandle_t flightControlTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;

// Task Function Prototypes
void imuTask(void *pvParameters);
void remoteTask(void *pvParameters);
void flightControlTask(void *pvParameters);
void statusTask(void *pvParameters);

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial)
        ;
    Serial.println();
    Serial.println("============================");
    Serial.println(" ESP32 Drone Booting Up! ");
    Serial.println("============================");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.println("I2C Initialized.");
    delay(100);

    Serial.println("Initializing Drone Components...");

    if (!imu_handler.begin())
    {
        Serial.println("IMU Initialization Failed!");
        while (1)
        {
            delay(1000);
        }; // Halt if IMU fails
    }

    if (!remote_handler.begin())
    {
        Serial.println("INFO: Remote Handler Initialized (Waiting for connection...)");
    }

    motor_driver.begin(); // Initialize motor driver

    pid_controllers.initialize(); // Initialize PID controllers

    flight_controller.initialize(); // Initialize flight controller

    web_server_manager.begin(); // Initialize web server
    
    Serial.println("Creating FreeRTOS Queues...");
    imuDataQueue = xQueueCreate(1, sizeof(DroneTypes::SensorData));
    remoteDataQueue = xQueueCreate(1, sizeof(DroneTypes::RemoteData));

    if (imuDataQueue == NULL || remoteDataQueue == NULL)
    {
        Serial.println("FATAL: Failed to create Queues!");
        while (1)
        {
            delay(1000);
        }; // Halt if queue creation fails
    }
    Serial.println("FreeRTOS Queues Created.");

    Serial.println("Creating FreeRTOS Tasks...");

    xTaskCreatePinnedToCore(imuTask, "IMU Task", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &imuTaskHandle, IMU_TASK_CORE);
    xTaskCreatePinnedToCore(remoteTask, "Remote Task", REMOTE_TASK_STACK_SIZE, NULL, REMOTE_TASK_PRIORITY, &remoteTaskHandle, REMOTE_TASK_CORE);
    xTaskCreatePinnedToCore(flightControlTask, "Flight Control Task", FLIGHTCONTROL_TASK_STACK_SIZE, NULL, FLIGHTCONTROL_TASK_PRIORITY, &flightControlTaskHandle, FLIGHTCONTROL_TASK_CORE);
    xTaskCreatePinnedToCore(statusTask, "Status Task", STATUS_TASK_STACK_SIZE, NULL, STATUS_TASK_PRIORITY, &statusTaskHandle, STATUS_TASK_CORE);

    Serial.println("FreeRTOS Tasks Created.");
    Serial.println("----------------------------");
    Serial.println(" Scheduler Started. ");
    Serial.println("----------------------------");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief Task for reading IMU data periodically.
 */
void imuTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_DELAY_MS); // 500Hz

    Serial.println("IMU Task Started.");

    for (;;)
    {
        // 1. Update IMU Handler (reads sensor)
        imu_handler.update();

        // 2. If new data is ready, package and send it via Queue
        if (imu_handler.isDataReady())
        {
            DroneTypes::SensorData sensor_data; // Use DroneTypes:: if needed
            sensor_data.attitude = imu_handler.getAttitude();
            sensor_data.raw_mpu = imu_handler.getRawMPUData();

            xQueueOverwrite(imuDataQueue, &sensor_data);
        }

        // 3. Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Task for handling remote controller input periodically.
 */
void remoteTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(REMOTE_TASK_DELAY_MS); // 50Hz

    Serial.println("Remote Task Started.");

    for (;;)
    {
        // 1. Update Remote Handler (reads controller input)
        remote_handler.update();

        // 2. Package remote data and send it via Queue
        DroneTypes::RemoteData remote_data;
        remote_data.is_connected = remote_handler.isConnected();
        remote_data.setpoint = remote_handler.getSetpoint();
        xQueueOverwrite(remoteDataQueue, &remote_data);

        // 3. Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Task for running the main flight control loop.
 */
void flightControlTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(FLIGHTCONTROL_TASK_DELAY_MS); // 250Hz
    float dt = (float)FLIGHTCONTROL_TASK_DELAY_MS / 1000.0f;                  // Convert to seconds

    DroneTypes::SensorData current_sensor_data;
    DroneTypes::RemoteData current_remote_data;

    current_remote_data.is_connected = false;
    current_remote_data.setpoint = {0.0f, 0.0f, 0.0f, RC_MAP_OUTPUT_MIN}; // Use constant from setting.h

    Serial.println("Flight Control Task Started.");

    for (;;)
    {
        // 1. Wait for new data from Queues
        if (xQueueReceive(imuDataQueue, &current_sensor_data, portMAX_DELAY) == pdPASS)
        {
        }
        else
        {
            Serial.println("IMU Data Queue Receive Failed!");
        }

        // 2. Check for new Remote data
        xQueueReceive(remoteDataQueue, &current_remote_data, (TickType_t)0);

        // 3. Run the flight controller's main logic cycle
        flight_controller.runControlCycle(dt, current_sensor_data, current_remote_data);

        // 4. Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Task for printing status information periodically.
 */
void statusTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(STATUS_TASK_DELAY_MS); // 4Hz

    Serial.println("Status Task Started.");

    for (;;)
    {

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}