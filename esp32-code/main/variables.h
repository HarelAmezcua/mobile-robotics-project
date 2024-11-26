#ifndef VARIABLES_H
#define VARIABLES_H

#include <WiFi.h>
#include <WiFiUDP.h>

// Wi-Fi credentials
extern const char* ssid;
extern const char* password;

// UDP communication setup
extern WiFiUDP udp;
extern const char* pc_ip; // PC IP address
extern const int udp_port_receive;
extern const int udp_port_send;

// Wheel speed variables
extern float position[3];
extern float pcIteration;

// Position and velocity variables
extern float position_x;
extern float position_y;
extern float position_theta;

extern int pwm_value;

// Desired control gains
extern float kx;
extern float ky;
extern float ko;

// Robot constants
extern const float N;
extern const float R;
extern const float L;
extern const float l;
extern const float PIp;

// Position and encoder variables
extern float x, y, theta, theta_old;
extern int position1, position2, position3, position4;
extern int c1_old, c2_old, c3_old, c4_old;
extern unsigned long t_old;
extern float dt;

// Motor pin definitions
extern const int motor1_pin1, motor1_pin2, enable1_pin;
extern const int motor2_pin1, motor2_pin2, enable2_pin;
extern const int motor3_pin1, motor3_pin2, enable3_pin;
extern const int motor4_pin1, motor4_pin2, enable4_pin;

// Encoder pin definitions
extern const int encoder1_A, encoder1_B;
extern const int encoder2_A, encoder2_B;
extern const int encoder3_A, encoder3_B;
extern const int encoder4_A, encoder4_B;

extern unsigned long startTime; // Variable to store start time

#endif
