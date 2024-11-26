// control.cpp
#include "control.h"
#include <Arduino.h>

Control::Control() {
    // Initialize motor pins
    for(int i = 0; i < NUM_WHEELS; i++) {
        pinMode(motor_pins[i][0], OUTPUT);
        pinMode(motor_pins[i][1], OUTPUT);
        setMotorSpeed(i, 0.0); // Initialize motors to stopped
    }
}

void Control::applyWheelSpeeds(float wheel_speeds[]) {
    for(int i = 0; i < NUM_WHEELS; i++) {
        setMotorSpeed(i, wheel_speeds[i]);
    }
}

void Control::setMotorSpeed(int wheel, float speed) {
    // Assuming speed range is -1.0 to 1.0
    // Map to PWM 0-255
    int pwm = abs(speed) * 255;
    pwm = constrain(pwm, 0, 255);

    if(speed > 0) {
        analogWrite(motor_pins[wheel][0], pwm);
        analogWrite(motor_pins[wheel][1], 0);
    }
    else {
        analogWrite(motor_pins[wheel][0], 0);
        analogWrite(motor_pins[wheel][1], pwm);
    }
}

void Control::stop() {
    for(int i = 0; i < NUM_WHEELS; i++) {
        analogWrite(motor_pins[i][0], 0);
        analogWrite(motor_pins[i][1], 0);
    }
}
