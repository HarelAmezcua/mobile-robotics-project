// config.cpp
#include "config.h"

// Define Wi-Fi credentials
const char* SSID = "iCUCEI";
const char* PASSWORD = "";

// Define UDP communication setup
const char* PC_IP = "10.214.127.133";
const int UDP_PORT_RECEIVE = 12345;
const int UDP_PORT_SEND = 12346;

// Define motor control pins
const int motor_pins[NUM_WHEELS][2] = {
    {5, 6},    // Wheel 0: IN1, IN2
    {7, 8},    // Wheel 1: IN1, IN2
    {9, 10},   // Wheel 2: IN1, IN2
    {11, 12}   // Wheel 3: IN1, IN2
};

// Define LED pin
const int STATUS_LED_PIN = 13;
