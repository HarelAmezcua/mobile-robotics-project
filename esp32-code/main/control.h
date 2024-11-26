#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h> // Include for uint32_t and other Arduino utilities

void controlLoop(uint32_t dt);
void receiveUdpPacket();

#endif
