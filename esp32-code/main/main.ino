#include <Arduino.h>
#include "variables.h"
#include "wifi_setup.h"
#include "motor_control.h"
#include "encoder.h"
#include "control.h"

// Global variables
uint32_t lastTime = 0; // Initialize lastTime to 0 outside the loop


void setup() {
    Serial.begin(115200);

    // Set up Wi-Fi
    setupWiFi();

    // Start UDP
    udp.begin(udp_port_receive);

    // Set up motor pins
    pinMode(motor1_pin1, OUTPUT); pinMode(motor1_pin2, OUTPUT); pinMode(enable1_pin, OUTPUT);
    pinMode(motor2_pin1, OUTPUT); pinMode(motor2_pin2, OUTPUT); pinMode(enable2_pin, OUTPUT);
    pinMode(motor3_pin1, OUTPUT); pinMode(motor3_pin2, OUTPUT); pinMode(enable3_pin, OUTPUT);
    pinMode(motor4_pin1, OUTPUT); pinMode(motor4_pin2, OUTPUT); pinMode(enable4_pin, OUTPUT);

    // Set up encoders
    setupEncoders();

    startTime = millis();  // Store start time
}

void loop() {
    static uint32_t nextWakeTime = 0;    // Track the next wake time in microseconds
    const uint32_t loopInterval = 1000; // Desired interval in microseconds (1ms)

    uint32_t currentTime = micros();    // Current time in microseconds
    uint32_t dt = currentTime - lastTime; // Calculate the time since the last loop
    lastTime = currentTime;             // Update lastTime for the next iteration

    if (nextWakeTime == 0) {
        nextWakeTime = currentTime + loopInterval; // Initialize on the first run
    }

    // Print the elapsed time (dt)
    //Serial.print("dt: ");
    //Serial.print(dt);
    //Serial.println(" us");

    // Control loop logic can go here
    controlLoop(dt); // Pass the measured dt to the control logic

    // Calculate the time left until the next wake time
    int32_t timeToSleep = nextWakeTime - micros();

    if (timeToSleep > 0) {
        delayMicroseconds(timeToSleep); // Sleep for the remaining time
    }

    // Update the next wake time for the next iteration
    nextWakeTime += loopInterval;

    // Adjust for large overruns (catch up)
    if ((int32_t)(micros() - nextWakeTime) > 0) {
        nextWakeTime = micros() + loopInterval;
    }
}