// udp_module.h
#ifndef UDP_MODULE_H
#define UDP_MODULE_H

#include <WiFiUDP.h>

class UDPModule {
public:
    UDPModule();
    void begin(int port);
    bool parseIncoming(char* buffer, int maxLen);
    void sendMessage(const char* ip, int port, const char* message);
    unsigned long getLastPacketTime();
private:
    WiFiUDP udp;
    unsigned long lastPacketTime;
};

#endif // UDP_MODULE_H
