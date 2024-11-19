// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi credentials
extern const char* SSID;
extern const char* PASSWORD;

// UDP communication setup
extern const char* PC_IP;
extern const int UDP_PORT_RECEIVE;
extern const int UDP_PORT_SEND;

// Connection timeout (milliseconds)
const unsigned long CONNECTION_TIMEOUT = 2000; // 2 seconds

// Wheel parameters
#define NUM_WHEELS 4

// Motor control pins (Adjust these pins according to your hardware setup)
extern const int motor_pins[NUM_WHEELS][2];

// LED pin for connection status (optional)
extern const int STATUS_LED_PIN;

#endif // CONFIG_H
