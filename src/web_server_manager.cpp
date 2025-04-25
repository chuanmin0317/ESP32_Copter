#include "web_server_manager.h"
#include "web_page_content.h"
#include "pid_controller_set.h"
#include "flight_controller.h"
#include "setting.h"
#include <WiFi.h>
#include <functional>

WebServerManager::WebServerManager(PIDControllerSet &pids, FlightController &fc)
    : server_(80), pids_(pids), fc_(fc) {}

void WebServerManager::setupWiFiAp()
{
    Serial.println("\nSetting up WiFi Access Point...");
    const char *ssid = WIFI_SSID;
    const char *password = WIFI_PASSWORD;

    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.println("WiFi Access Point setup complete.");
}

void WebServerManager::begin()
{
    setupWiFiAp();

    Serial.println("Setting up Web Server Handlers...");

    server_.on("/", HTTP_GET, std::bind(&WebServerManager::handleRoot, this, std::placeholders::_1));

    server_.on("/api/pids", HTTP_GET, std::bind(&WebServerManager::handleGetPIDs, this, std::placeholders::_1));

    server_.on("/api/pids/save", HTTP_POST, std::bind(&WebServerManager::handleSavePIDs, this, std::placeholders::_1));

    server_.on("/api/pids", HTTP_POST, [](AsyncWebServerRequest *request) {

    },
               NULL, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
               { handleSetPIDsBody(request, data, len, index, total); });

    server_.onNotFound(std::bind(&WebServerManager::handleNotFound, this, std::placeholders::_1));

    server_.begin();
    Serial.println("Web Server Started.");
}

void WebServerManager::handleRoot(AsyncWebServerRequest *request)
{
    request->send(200, "text/html", HTML_CONTENT);
}

void WebServerManager::handleGetPIDs(AsyncWebServerRequest *request)
{
    Serial.println("[Web] GET /api/pids request");
    // Create a JSON document
    const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(int(PIDLoop::COUNT)) + int(PIDLoop::COUNT) * JSON_OBJECT_SIZE(5) + 300; // Estimate size
    DynamicJsonDocument jsonDoc(capacity);

    JsonObject pidsObject = jsonDoc.createNestedObject("pids");

    // Iterate through all PID loops and get their config
    for (int i = 0; i < (int)PIDLoop::COUNT; ++i)
    {
        PIDLoop currentLoop = static_cast<PIDLoop>(i);
        float kp, ki, kd, int_limit, out_limit;
        if (pids_.getSingleConfig(currentLoop, kp, ki, kd, int_limit, out_limit))
        {
            JsonObject loopObject = pidsObject.createNestedObject(String(i)); // Use index as key
            loopObject["kp"] = kp;
            loopObject["ki"] = ki;
            loopObject["kd"] = kd;
            loopObject["int_limit"] = int_limit;
            loopObject["out_limit"] = out_limit;
        }
    }

    // Serialize JSON to string
    String jsonResponse;
    serializeJson(jsonDoc, jsonResponse);

    request->send(200, "application/json", jsonResponse);
}

void WebServerManager::handleSetPIDsBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == 0 && len == total)
    { // Assume data comes in one chunk
        Serial.println("[Web] POST /api/pids request body received");

        if (fc_.isArmed())
        {
            Serial.println("  ERROR: Cannot set PIDs while armed!");
            request->send(403, "application/json", "{\"error\":\"Cannot set PIDs while armed\"}");
            return;
        }

        const size_t capacity = JSON_OBJECT_SIZE(4) + 100;
        DynamicJsonDocument jsonDoc(capacity);
        DeserializationError error = deserializeJson(jsonDoc, (const char *)data, len);

        if (error)
        {
            Serial.print("  ERROR: deserializeJson() failed: ");
            Serial.println(error.c_str());
            request->send(400, "application/json", "{\"error\":\"Invalid JSON format\"}");
            return;
        }

        if (!jsonDoc.containsKey("loop") || !jsonDoc.containsKey("kp") || !jsonDoc.containsKey("ki") || !jsonDoc.containsKey("kd"))
        {
            Serial.println("  ERROR: Missing required fields in JSON (loop, kp, ki, kd)");
            request->send(400, "application/json", "{\"error\":\"Missing required fields\"}");
            return;
        }

        int loopId = jsonDoc["loop"];
        float kp = jsonDoc["kp"];
        float ki = jsonDoc["ki"];
        float kd = jsonDoc["kd"];

        if (loopId < 0 || loopId >= (int)PIDLoop::COUNT)
        {
            Serial.printf("  ERROR: Invalid loop identifier: %d\n", loopId);
            request->send(400, "application/json", "{\"error\":\"Invalid loop identifier\"}");
            return;
        }

        PIDLoop targetLoop = static_cast<PIDLoop>(loopId);

        if (pids_.updateSingleGainSet(targetLoop, kp, ki, kd))
        {
            Serial.printf("  Successfully updated PID loop %d\n", loopId);
            request->send(200, "application/json", "{\"success\":true}");
        }
        else
        {
            Serial.printf("  ERROR: Failed to update PID loop %d\n", loopId);
            request->send(500, "application/json", "{\"error\":\"Internal error updating PID\"}");
        }
    }
    // Add chunk handling here if needed for larger request bodies
}

void WebServerManager::handleSavePIDs(AsyncWebServerRequest *request)
{
    Serial.println("[Web] POST /api/pids/save request");

    if (fc_.isArmed())
    {
        Serial.println("  ERROR: Cannot save PIDs while armed!");
        request->send(403, "application/json", "{\"error\":\"Cannot save PIDs while armed\"}");
        return;
    }

    if (pids_.saveSettingsToNVS())
    {
        Serial.println("  Successfully saved PID settings to NVS.");
        request->send(200, "application/json", "{\"success\":true}");
    }
    else
    {
        Serial.println("  ERROR: Failed to save PID settings to NVS.");
        request->send(500, "application/json", "{\"error\":\"Failed to save settings to NVS\"}");
    }
}

void WebServerManager::handleNotFound(AsyncWebServerRequest *request)
{
    Serial.printf("[Web] Not Found: %s\n", request->url().c_str());
    request->send(404, "text/plain", "Page Not Found");
}