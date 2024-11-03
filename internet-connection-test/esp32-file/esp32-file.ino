#include <WiFi.h>
#include <WiFiUDP.h>

// Wi-Fi credentials
const char* ssid = "HARELPC 0764";
const char* password = "66,Fh172";

// UDP communication setup
WiFiUDP udp;
const char* pc_ip = "192.168.1.38"; // PC IP address
const int udp_port_receive = 12345;
const int udp_port_send = 12346;

// Wheel speed variables
float wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

// Position and velocity variables (example)
float position_x = 0.0;
float position_y = 0.0;
float local_velocity_v = 0.0;
float local_velocity_omega = 0.0;

// Function to set up Wi-Fi
void setupWiFi() {
  Serial.begin(115200);
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set up Wi-Fi
  setupWiFi();

  // Start UDP
  udp.begin(udp_port_receive);
}

void loop() {
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

    // Parse the wheel speeds
    sscanf(incomingPacket, "%f,%f,%f,%f", &wheel_speeds[0], &wheel_speeds[1], &wheel_speeds[2], &wheel_speeds[3]);

    // Example: Print the parsed wheel speeds
    Serial.printf("Wheel Speeds: %.2f, %.2f, %.2f, %.2f\n", wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
  }

  // Example: Update position and velocity
  position_x += 0.01; // Dummy update
  position_y += 0.01; // Dummy update
  local_velocity_v = 0.5; // Dummy velocity
  local_velocity_omega = 0.1; // Dummy angular velocity

  // Prepare the message to send back
  char outgoingMessage[255];
  sprintf(outgoingMessage, "%.2f,%.2f,%.2f,%.2f", position_x, position_y, local_velocity_v, local_velocity_omega);

  // Send the position and velocity data to the PC
  udp.beginPacket(pc_ip, udp_port_send);
  udp.write((const uint8_t*)outgoingMessage, strlen(outgoingMessage));
  udp.endPacket();

  // 100Hz loop (10ms delay)
  delay(10);
}
