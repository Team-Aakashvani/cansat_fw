/**
 * @file wifi_ota_manager.cpp
 * @brief Implements ESP32 Wi-Fi Access Point, ArduinoOTA updates, and TCP MSP/Terminal telemetry server.
 * @author Wastl Kraus
 * @license MIT
 */

#include "wifi_ota_manager.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

#define WIFI_AP_SSID      "Flight32_AP"
#define WIFI_AP_PASSWORD  "flight32"
#define TCP_TELEMETRY_PORT 2323

static WiFiServer* _tcpServer = nullptr;
static WiFiClient _tcpClient;
static bool _wifi_initialized = false;

void wifi_ota_init()
{
    if (_wifi_initialized) return;

    // Standard ESP32 Access Point mode with low RF TX power (5 dBm = ~80mA instead of 500mA!) to prevent USB voltage drop/brownout
    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_5dBm);
    bool ap_ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

    if (ap_ok)
    {
        pinMode(2, OUTPUT);
        digitalWrite(2, HIGH); // Light up onboard status LED to indicate Flight32_AP is active!
        Serial.printf("[WIFI] Access Point started: SSID='%s', Pass='%s', IP=%s\n",
                      WIFI_AP_SSID, WIFI_AP_PASSWORD, WiFi.softAPIP().toString().c_str());
    }
    else
    {
        Serial.println("[WIFI-ERROR] Failed to start Wi-Fi Access Point!");
    }

    // Initialize ArduinoOTA
    ArduinoOTA.setHostname("flight32");
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] Starting update: " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Update Complete! Rebooting ESP32...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA-ERROR] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("[OTA] ArduinoOTA Server started on hostname 'flight32' (192.168.4.1)");

    // Start TCP Telemetry Server on Port 2323
    _tcpServer = new WiFiServer(TCP_TELEMETRY_PORT);
    _tcpServer->begin();
    _tcpServer->setNoDelay(true); // Disable Nagle's algorithm for instant MSP packet streaming
    Serial.printf("[WIFI] TCP Telemetry & Terminal Server listening on port %d\n", TCP_TELEMETRY_PORT);

    _wifi_initialized = true;
}

void wifi_ota_handle()
{
    if (!_wifi_initialized || !_tcpServer) return;

    // Service ArduinoOTA update requests
    ArduinoOTA.handle();

    // Check for new incoming TCP clients
    if (_tcpServer->hasClient())
    {
        if (!_tcpClient || !_tcpClient.connected())
        {
            _tcpClient = _tcpServer->available();
            _tcpClient.setNoDelay(true);
            Serial.printf("[WIFI] New TCP telemetry client connected: %s\n", _tcpClient.remoteIP().toString().c_str());
        }
        else
        {
            // Reject secondary simultaneous connections
            WiFiClient rejectClient = _tcpServer->available();
            rejectClient.stop();
        }
    }
}

int wifi_ota_available()
{
    if (_tcpClient && _tcpClient.connected())
    {
        return _tcpClient.available();
    }
    return 0;
}

int wifi_ota_read()
{
    if (_tcpClient && _tcpClient.connected())
    {
        return _tcpClient.read();
    }
    return -1;
}

size_t wifi_ota_write(const uint8_t *buf, size_t len)
{
    if (_tcpClient && _tcpClient.connected() && len > 0)
    {
        return _tcpClient.write(buf, len);
    }
    return 0;
}

bool wifi_ota_is_client_connected()
{
    return (_tcpClient && _tcpClient.connected());
}

void wifi_ota_stop()
{
    if (_tcpClient)
    {
        _tcpClient.stop();
    }
    if (_tcpServer)
    {
        _tcpServer->stop();
        delete _tcpServer;
        _tcpServer = nullptr;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _wifi_initialized = false;
    digitalWrite(2, LOW);
    Serial.println("[WIFI] Access Point stopped.");
}

bool wifi_ota_is_initialized()
{
    return _wifi_initialized;
}
