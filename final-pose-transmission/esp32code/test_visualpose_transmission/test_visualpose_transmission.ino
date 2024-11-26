#include <WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi credentials
const char* ssid = "iCUCEI";
const char* password = "";

// UDP settings
WiFiUDP udp;
const int udpPortReceive = 12345;  // Port to receive handshake and pose data
const int udpPortSend = 12346;     // Port to send ACK to PC

char incomingPacket[255];          // Buffer for incoming packets
String receivedPose = "";          // Store the received pose data

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  // Wait for Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  // Display ESP32 IP address
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Begin listening for UDP messages
  udp.begin(udpPortReceive);
  Serial.printf("Listening on UDP port %d\n", udpPortReceive);
}

void loop() {
  // Check for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-terminate the string
    }

    String packet = String(incomingPacket);
    Serial.printf("Received: %s from %s:%d\n", packet.c_str(),
                  udp.remoteIP().toString().c_str(), udp.remotePort());

    if (packet == "Hello") {
      // Respond to handshake
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("ACK");
      udp.endPacket();
      Serial.println("Sent: ACK");
    } else {
      // Handle pose data
      receivedPose = packet;
      Serial.printf("Received pose: %s\n", receivedPose.c_str());
    }
  }
}
