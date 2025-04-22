#ifndef WEB_SERVER_MANAGER_H
#define WEB_SERVER_MANAGER_H

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "common_types.h"

class PIDControllerSet; // Forward declaration
class FlightController; // Forward declaration

class WebServerManager
{
public:
    /**
     * @brief Constructor for the WebServerManager class.
     * @param pids Reference to the PIDControllerSet instance.
     * @param fc Reference to the FlightController instance.
     */
    WebServerManager(PIDControllerSet &pids, FlightController &fc);

    /**
     * @brief Initializes WiFi (AP mode) and starts the web server with defined routes.
     * Call this once in setup().
     */
    void begin(); // Start the web server

private:
    AsyncWebServer server_;  
    PIDControllerSet &pids_; 
    FlightController &fc_;   

    // Web server routes
    void setupRoutes(); // Setup web server routes

    // Request Handler Methods
    void handleRoot(AsyncWebServerRequest *request);    
    void handleGetPIDs(AsyncWebServerRequest *request); 

    void handleSetPIDsBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
    void handleSavePIDs(AsyncWebServerRequest *request); 
    void handleNotFound(AsyncWebServerRequest *request); 

    void setupWiFiAP(); // Setup WiFi in AP mode
};

#endif // WEB_SERVER_MANAGER_H