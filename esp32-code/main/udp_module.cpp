// udp_module.cpp
#include "udp_module.h"
#include "config.h"
#include <Arduino.h>

UDPModule::UDPModule() : lastPacketTime(0) {}

void UDPModule::begin(int port) {
    udp.begin(port);
    Serial.printf("UDP listening on port %d\n", port);
}

bool UDPModule::parseIncoming(char* buffer, int maxLen) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buffer, maxLen - 1);
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate
            lastPacketTime = millis();
            return true;
        }
    }
    return false;
}

void UDPModule::sendMessage(const char* ip, int port, const char* message) {
    udp.beginPacket(ip, port);
    udp.write((const uint8_t*)message, strlen(message));
    udp.endPacket();
}

unsigned long UDPModule::getLastPacketTime() {
    return lastPacketTime;
}
