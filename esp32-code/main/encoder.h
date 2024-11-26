#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>  // Add this line to include Arduino.h

void setupEncoders();
void IRAM_ATTR encoder1_callback();
void IRAM_ATTR encoder2_callback();
void IRAM_ATTR encoder3_callback();
void IRAM_ATTR encoder4_callback();

#endif
