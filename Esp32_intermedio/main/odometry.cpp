// odometry.cpp
#include "odometry.h"
#include <math.h>

Odometry::Odometry() : x(0.0), y(0.0), v(0.0), omega(0.0) {}

void Odometry::update(float wheel_speeds[], unsigned long deltaTime) {
    // Simplified odometry calculation    

    // Example: Average wheel speed as linear velocity
    v = 0.0;
    for(int i = 0; i < NUM_WHEELS; i++) { // NUM_WHEELS is now recognized
        v += wheel_speeds[i];
    }
    v /= NUM_WHEELS;

    // Example: Differential calculation for angular velocity
    omega = (wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] - wheel_speeds[3]) / NUM_WHEELS;

    // Update position based on velocity and deltaTime
    float delta_seconds = deltaTime / 1000.0;
    x += v * delta_seconds * cos(omega * delta_seconds);
    y += v * delta_seconds * sin(omega * delta_seconds);
}

float Odometry::getX() {
    return x;
}

float Odometry::getY() {
    return y;
}

float Odometry::getV() {
    return v;
}

float Odometry::getOmega() {
    return omega;
}
