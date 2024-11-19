// odometry.h
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "config.h" // To access NUM_WHEELS

class Odometry {
public:
    Odometry();
    void update(float wheel_speeds[], unsigned long deltaTime);
    float getX();
    float getY();
    float getV();
    float getOmega();
private:
    float x;
    float y;
    float v;
    float omega;
};

#endif // ODOMETRY_H
