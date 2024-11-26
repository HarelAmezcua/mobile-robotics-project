#include "motor_control.h"
#include <Arduino.h>
#include "variables.h"

void motor_control(int pin1, int pin2, int enable_pin, float speed_rad_s) {
    char direction;

    // Define max_speed_rad_s as the max rad/s corresponding to 100% PWM
    const float max_speed_rad_s = 10.0; // Adjust according to your motor's max speed in rad/s

    // Set direction and PWM based on speed_rad_s
    if (speed_rad_s > 0) {
        direction = 'f';
        pwm_value = (int)(min(speed_rad_s, max_speed_rad_s) / max_speed_rad_s * 255); // Proportional PWM
    } else if (speed_rad_s < 0) {
        direction = 'b';
        pwm_value = (int)(min(-speed_rad_s, max_speed_rad_s) / max_speed_rad_s * 255);
    } else {
        direction = 'f';
        pwm_value = 0; // Motor stopped
    }

    // Control the motor based on the direction and calculated PWM
    if (direction == 'f') {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else if (direction == 'b') {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    }
    analogWrite(enable_pin, pwm_value);
}