#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include <WiFi.h>

class WiFiModule {
public:
    void begin();
    bool isConnected();
    IPAddress getLocalIP();
private:
    void connect();
};

#endif // WIFI_MODULE_H
