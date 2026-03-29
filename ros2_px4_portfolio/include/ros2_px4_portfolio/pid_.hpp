#pragma once
#include <cmath>
#include <stdint.h>

/*
Custom PID class for simple control of the UAV.
The timestamp can be expressed in any unit, but microsecond matches PX4 convention.
*/
class PID
{
public:
    PID(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;

        integral = 0.0;
    }

private:
    float kp, ki, kd, integral, prev_error;
    uint64_t prev_timestamp;

    // Computes the next value to send to the controlled system
    float compute(float error, uint64_t timestamp);

};