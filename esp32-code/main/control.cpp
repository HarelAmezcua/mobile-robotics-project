#include "control.h"
#include "variables.h"
#include "motor_control.h"
#include <Arduino.h>
#include <WiFiUDP.h>

// Integral state variables
float integralX = 0.0f;
float integralY = 0.0f;
float integralO = 0.0f;

// Integral gain
const float kix = 0.01f; // Adjust as needed
const float kiy = 0.01f; // Adjust as needed
const float kio = 0.01f; // Adjust as needed

void controlLoop(uint32_t dt) {
  if (isInfoFirstReceived) {
    // Compute errors
    float errorX = desiredPosition[0] - position[0];
    float errorY = desiredPosition[1] - position[1];
    float errorO = desiredPosition[2] - position[2];

    // Update integral terms (considering the time step dt)
    integralX += errorX * (1 / 1000.0f); // dt is in milliseconds
    integralY += errorY * (1 / 1000.0f);
    integralO += errorO * (1 / 1000.0f);

    // Compute control actions with integral terms
    float ux = kx * errorX + kix * integralX;
    float uy = ky * errorY + kiy * integralY;
    float uo = ko * errorO + kio * integralO;

    float alpha = theta + PI / 4;
    // Control Action
    float v1 = sqrt(2) * sin(alpha) * ux - sqrt(2) * cos(alpha) * uy - (L + l) * uo;
    float v2 = sqrt(2) * cos(alpha) * ux + sqrt(2) * sin(alpha) * uy + (L + l) * uo;
    float v3 = sqrt(2) * cos(alpha) * ux + sqrt(2) * sin(alpha) * uy - (L + l) * uo;
    float v4 = sqrt(2) * sin(alpha) * ux - sqrt(2) * cos(alpha) * uy + (L + l) * uo;

    motor_control(motor1_pin1, motor1_pin2, enable1_pin, v1);
    motor_control(motor2_pin1, motor2_pin2, enable2_pin, v2);
    motor_control(motor3_pin1, motor3_pin2, enable3_pin, v3);
    motor_control(motor4_pin1, motor4_pin2, enable4_pin, v4);

    // Print the parsed positions
    Serial.printf("Position: %f, %f, %f\n", position[0], position[1], position[2]);
  } else {
    motor_control(motor1_pin1, motor1_pin2, enable1_pin, 0);
    motor_control(motor2_pin1, motor2_pin2, enable2_pin, 0);
    motor_control(motor3_pin1, motor3_pin2, enable3_pin, 0);
    motor_control(motor4_pin1, motor4_pin2, enable4_pin, 0);
  }
  receiveUdpPacket();
}


void receiveUdpPacket() {
    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {  // Only proceed if a packet is available
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = '\0'; // Null-terminate the string
        }
        //Serial.print("Received: ");
        //Serial.println(incomingPacket);

        // Parse the desired positions
        sscanf(incomingPacket, "%f,%f,%f,%f", &position[0], &position[1], &position[2],&pcIteration);


        isInfoFirstReceived = true;
    }
    // If no packet is available, the function exits immediately
}


/*
void controlLoop() {
    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = '\0'; // Null-terminate the string
        }
        Serial.print("Received: ");
        Serial.println(incomingPacket);

        // Parse the desired positions
        sscanf(incomingPacket, "%f,%f,%f", &position[0], &position[1], &position[2]);

        // Print the parsed positions
        Serial.printf("Position: %f, %f, %f\n", position[0], position[1], position[2]);
    }

    float ux = kx * (position[0] - position_x);
    float uy = ky * (position[1] - position_y);
    float uo = ko * (position[2] - position_theta);

    // Control code and motor movement
    float t = millis() / 1000.0;
    dt = t - t_old;
    t_old = t;

    int c1 = position1, c2 = position2, c3 = position3, c4 = position4;

    float delta_c1 = c1 - c1_old, delta_c2 = c2 - c2_old, delta_c3 = c3 - c3_old, delta_c4 = c4 - c4_old;

    c1_old = c1; c2_old = c2; c3_old = c3; c4_old = c4;

    float d1 = 2 * PIp * R * (delta_c1 / N);
    float d2 = 2 * PIp * R * (delta_c2 / N);
    float d3 = 2 * PIp * R * (delta_c3 / N);
    float d4 = 2 * PIp * R * (delta_c4 / N);

    float alpha = theta + PI / 4;
    float dx = (1.0 / 4) * sqrt(2) * ((d1 + d4) * sin(alpha) + (d2 + d3) * cos(alpha));
    float dy = (1.0 / 4) * sqrt(2) * ((d2 + d3) * sin(alpha) - (d1 + d4) * cos(alpha));
    float dtheta = ((d2 + d4 - d1 - d3) / (4 * (L + l)));

    // Update position and orientation
    position_x += dx;
    position_y += dy;
    position_theta += dtheta;

    // Prepare the message to send back
    char outgoingMessage[255];
    sprintf(outgoingMessage, "%.2f,%.2f,%.2f", position_x, position_y, position_theta);

    // Send the position and orientation data to the PC
    udp.beginPacket(pc_ip, udp_port_send);
    udp.write((const uint8_t*)outgoingMessage, strlen(outgoingMessage));
    udp.endPacket();

    // Control Action
    float v1 = sqrt(2) * sin(alpha) * ux - sqrt(2) * cos(alpha) * uy - (L + l) * uo;
    float v2 = sqrt(2) * cos(alpha) * ux + sqrt(2) * sin(alpha) * uy + (L + l) * uo;
    float v3 = sqrt(2) * cos(alpha) * ux + sqrt(2) * sin(alpha) * uy - (L + l) * uo;
    float v4 = sqrt(2) * sin(alpha) * ux - sqrt(2) * cos(alpha) * uy + (L + l) * uo;

    motor_control(motor1_pin1, motor1_pin2, enable1_pin, v1);
    motor_control(motor2_pin1, motor2_pin2, enable2_pin, v2);
    motor_control(motor3_pin1, motor3_pin2, enable3_pin, v3);
    motor_control(motor4_pin1, motor4_pin2, enable4_pin, v4);
}*/
