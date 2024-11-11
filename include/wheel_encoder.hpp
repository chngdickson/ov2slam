#pragma once

class WheelEncoder {
private:
    static double left_wheel_radius;
    static double right_wheel_radius;
    static double wheel_base;
    static int encoder_resolution;
    static bool use_wheel_enc;
public:
    typedef struct EncoderData_t{
        double left_wheel_travel;
        double right_wheel_travel;
    } EncoderData;
    static void setParams(double left_wheel_radius, double right_wheel_radius, double wheel_base, int encoder_resolution, bool use_wheel_enc);
    static EncoderData getWheelTravel(long enc_left, long enc_right);
    static double getWheelBase() { return wheel_base; }
    static bool useWheelEncoder() { return use_wheel_enc; }
};