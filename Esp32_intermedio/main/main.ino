#include <Arduino.h>
#include "config.h"
#include "wifi_module.h"
#include "udp_module.h"
#include "odometry.h"
#include "control.h"

// Instantiate modules
WiFiModule wifi;
UDPModule udp;
Odometry odometry;
Control control;

// Wheel speed variables
float wheel_speeds[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};

// Position and velocity variables
float position_x = 0.0;
float position_y = 0.0;
float local_velocity_v = 0.0;
float local_velocity_omega = 0.0;

// Timing
unsigned long previousMillis = 0;
const unsigned long loopInterval = 1; // 10ms for 100Hz

// LED status (optional)
void setupLED() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
}

void indicateConnectionStatus(bool connected) {
    digitalWrite(STATUS_LED_PIN, connected ? HIGH : LOW);
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000); // Allow time for serial monitor to initialize

    // Initialize LED (optional)
    setupLED();

    // Initialize Wi-Fi
    wifi.begin();
    indicateConnectionStatus(true);

    // Start UDP
    udp.begin(UDP_PORT_RECEIVE);

    // Initialize Control
    control.stop();
}

void loop() {
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= loopInterval) {
        previousMillis = currentMillis;

        // Check for incoming UDP packets
        char incomingPacket[255];
        if(udp.parseIncoming(incomingPacket, sizeof(incomingPacket))) {
            Serial.print("Received: ");
            Serial.println(incomingPacket);

            // Parse the wheel speeds
            int parsed = sscanf(incomingPacket, "%f,%f,%f,%f", 
                                &wheel_speeds[0], 
                                &wheel_speeds[1], 
                                &wheel_speeds[2], 
                                &wheel_speeds[3]);
            if(parsed == NUM_WHEELS) {
                // Apply control
                control.applyWheelSpeeds(wheel_speeds);

                // Optionally, print the wheel speeds
                Serial.printf("Wheel Speeds: %.2f, %.2f, %.2f, %.2f\n", 
                              wheel_speeds[0], 
                              wheel_speeds[1], 
                              wheel_speeds[2], 
                              wheel_speeds[3]);
            }
            else {
                Serial.println("Error parsing wheel speeds.");
            }
        }

        // Check for connection timeout
        if(currentMillis - udp.getLastPacketTime() > CONNECTION_TIMEOUT) {
            Serial.println("Connection lost! Stopping robot.");
            control.stop();
            indicateConnectionStatus(false); // Turn off LED

            // Signal computer about connection loss
            const char* lostConnectionMsg = "Connection lost!";
            udp.sendMessage(PC_IP, UDP_PORT_SEND, lostConnectionMsg);
        }
        else {
            indicateConnectionStatus(true); // Ensure LED is on

            // Update odometry
            odometry.update(wheel_speeds, loopInterval);

            // Get updated position and velocity
            position_x = odometry.getX();
            position_y = odometry.getY();
            local_velocity_v = odometry.getV();
            local_velocity_omega = odometry.getOmega();

            // Prepare the message to send back
            char outgoingMessage[255];
            sprintf(outgoingMessage, "%.2f,%.2f,%.2f,%.2f", 
                    position_x, 
                    position_y, 
                    local_velocity_v, 
                    local_velocity_omega);

            // Send the position and velocity data to the PC
            udp.sendMessage(PC_IP, UDP_PORT_SEND, outgoingMessage);
        }
    }
}
