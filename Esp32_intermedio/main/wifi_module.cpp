#include "wifi_module.h"
#include "config.h"
#include <Arduino.h>

void WiFiModule::begin() {
    Serial.begin(115200);
    Serial.print("Connecting to Wi-Fi...");
    connect();
}

void WiFiModule::connect() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println(" Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

bool WiFiModule::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

IPAddress WiFiModule::getLocalIP() {
    return WiFi.localIP();
}
