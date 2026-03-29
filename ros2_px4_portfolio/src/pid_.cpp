#include <ros2_px4_portfolio/pid_.hpp>

float PID::compute(float error, uint64_t timestamp)
{
    uint64_t dt = timestamp - prev_timestamp;
    integral += error * dt;
    error = kp * error + ki * integral + kd * (error - prev_error) / dt;
    prev_error = error;
    prev_timestamp = timestamp;
    return error;
}
