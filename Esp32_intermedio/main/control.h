// control.h
#ifndef CONTROL_H
#define CONTROL_H

#include "config.h" // To access NUM_WHEELS

class Control {
public:
    Control();
    void applyWheelSpeeds(float wheel_speeds[]);
    void stop();
private:
    void setMotorSpeed(int wheel, float speed);
};

#endif // CONTROL_H
