#ifndef WEB_SERVER_MANAGER_H
#define WEB_SERVER_MANAGER_H

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

class PIDControllerSet;
class FlightController;
enum class PIDLoop : int;

class WebServerManager
{
public:

    WebServerManager(PIDControllerSet &pid_controllers, FlightController &flight_controller);

    /**
     * @brief Initializes WiFi (AP mode) and starts the web server with defined routes.
     * Call this once in setup().
     */
    void begin();

private:
    AsyncWebServer server_;
    PIDControllerSet &pids_;
    FlightController &fc_;

    void handleRoot(AsyncWebServerRequest *request);
    void handleGetPIDs(AsyncWebServerRequest *request);

    void handleSetPIDsBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
    void handleSavePIDs(AsyncWebServerRequest *request);
    void handleNotFound(AsyncWebServerRequest *request);

    void setupWiFiAp();
};

#endif