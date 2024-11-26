#include "encoder.h"
#include <Arduino.h>
#include "variables.h"

void setupEncoders() {
    pinMode(encoder1_A, INPUT); pinMode(encoder1_B, INPUT);
    pinMode(encoder2_A, INPUT); pinMode(encoder2_B, INPUT);
    pinMode(encoder3_A, INPUT); pinMode(encoder3_B, INPUT);
    pinMode(encoder4_A, INPUT); pinMode(encoder4_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoder1_A), encoder1_callback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2_A), encoder2_callback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder3_A), encoder3_callback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder4_A), encoder4_callback, CHANGE);
}

void IRAM_ATTR encoder1_callback() {
    if (digitalRead(encoder1_A) == digitalRead(encoder1_B)) position1++;
    else position1--;
}

void IRAM_ATTR encoder2_callback() {
    if (digitalRead(encoder2_A) == digitalRead(encoder2_B)) position2++;
    else position2--;
}

void IRAM_ATTR encoder3_callback() {
    if (digitalRead(encoder3_A) == digitalRead(encoder3_B)) position3++;
    else position3--;
}

void IRAM_ATTR encoder4_callback() {
    if (digitalRead(encoder4_A) == digitalRead(encoder4_B)) position4++;
    else position4--;
}