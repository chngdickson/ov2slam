#include "wheel_encoder.hpp"
#include <cmath>
#include <iostream>

double WheelEncoder::left_wheel_radius = 0.0;
double WheelEncoder::right_wheel_radius = 0.0;
double WheelEncoder::wheel_base = 0.0;
int WheelEncoder::encoder_resolution = 1;
bool WheelEncoder::use_wheel_enc = false;

void WheelEncoder::setParams(double left_wheel_radius, double right_wheel_radius, double wheel_base, int encoder_resolution, bool use_wheel_enc)
{
    WheelEncoder::left_wheel_radius = left_wheel_radius;
    WheelEncoder::right_wheel_radius = right_wheel_radius;
    WheelEncoder::wheel_base = wheel_base;
    WheelEncoder::encoder_resolution = encoder_resolution;
    WheelEncoder::use_wheel_enc = use_wheel_enc;
}

WheelEncoder::EncoderData WheelEncoder::getWheelTravel(long enc_left, long enc_right)
{
    double left_wheel_travel = (2.0 * M_PI * WheelEncoder::left_wheel_radius * enc_left) / WheelEncoder::encoder_resolution;
    double right_wheel_travel = (2.0 * M_PI * WheelEncoder::right_wheel_radius * enc_right) / WheelEncoder::encoder_resolution;

    EncoderData ret = {left_wheel_travel, right_wheel_travel};

    return ret;
}