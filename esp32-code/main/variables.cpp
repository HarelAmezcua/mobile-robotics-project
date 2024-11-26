#include "variables.h"

// Wi-Fi credentials
const char* ssid = "iCUCEI";
const char* password = "";

// UDP communication setup
WiFiUDP udp;
const char* pc_ip = "10.214.118.186"; // PC IP address
const int udp_port_receive = 12345;
const int udp_port_send = 12346;

// Wheel speed variables
float position[3] = {0.0, 0.0, 0.0};
float desiredPosition[3] =  {0.25, 0.25, 0.0};
float pcIteration = 0;
bool isInfoFirstReceived = false;

// Position and velocity variables
float position_x = 0.0;
float position_y = 0.0;
float position_theta = 0.0;

int pwm_value;

// Desired control gains
float kx = 1.2;
float ky = 3;
float ko = 1;

// Robot constants
const float N = 80.0;
const float R = 0.04;
const float L = 0.15;
const float l = 0.185;
const float PIp = 3.1415;

// Position and encoder variables
float x = 0, y = 0, theta = 0, theta_old = 0;
int position1 = 1, position2 = 1, position3 = 1, position4 = 1;
int c1_old = 0, c2_old = 0, c3_old = 0, c4_old = 0;
unsigned long t_old = 0;
float dt = 0;

// Motor pin definitions
const int motor1_pin1 = 2, motor1_pin2 = 4, enable1_pin = 15;
const int motor2_pin1 = 12, motor2_pin2 = 14, enable2_pin = 13;
const int motor3_pin1 = 21, motor3_pin2 = 3, enable3_pin = 19;
const int motor4_pin1 = 33, motor4_pin2 = 32, enable4_pin = 25;

// Encoder pin definitions
const int encoder1_A = 5, encoder1_B = 18;
const int encoder2_A = 27, encoder2_B = 26;
const int encoder3_A = 22, encoder3_B = 23;
const int encoder4_A = 35, encoder4_B = 34;

unsigned long startTime; // Variable to store start time
