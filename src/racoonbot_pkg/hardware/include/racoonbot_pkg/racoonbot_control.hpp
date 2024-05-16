#ifndef DIFFDRIVE_RACOONBOT_CONTROL_HPP
#define DIFFDRIVE_RACOONBOT_CONTROL_HPP

#include <iostream>
#include <string>
#include <pigpio.h>
#include "rclcpp/rclcpp.hpp"


class RacoonBotControl
{
public:
    RacoonBotControl() : 
        left_motor_in1_(23), 
        left_motor_in2_(24), 
        right_motor_in1_(5), 
        right_motor_in2_(6),
        motor_en1_pwm_pin_(25), 
        motor_en2_pwm_pin_(26), 
        left_encoder_pin_(17), 
        right_encoder_pin_(27),
        resolution_(0), 
        wheel_radius_(0.0325), 
        wheel_span_(0.2), 
        left_encoder_counter_(0), 
        right_encoder_counter_(0),
        last_right_encoder_counter_value_(0),
        last_left_encoder_counter_value_(0),
        max_vel_rpm_(48),
        max_pwm_(255),
        left_forward_(true),
        right_forward_(true)
    {
    }

    ~RacoonBotControl()
    {
        gpioTerminate();
    }

    void activate()
    {
        if (gpioInitialise() < 0)
        {
            std::cerr << "PiGPIO initialization failed" << std::endl;
            return;
        }
        gpioInitialise()
        gpioSetMode(left_motor_in1_, PI_OUTPUT);
        gpioSetMode(left_motor_in2_, PI_OUTPUT);
        gpioSetMode(right_motor_in1_, PI_OUTPUT);
        gpioSetMode(right_motor_in2_, PI_OUTPUT);

        // Create PWM instances
        gpioSetMode(motor_en1_pwm_pin_, PI_OUTPUT);
        gpioSetMode(motor_en2_pwm_pin_, PI_OUTPUT);

        // Encoder pulse pins 
        gpioSetMode(left_encoder_pin_, PI_INPUT);
        gpioSetMode(right_encoder_pin_, PI_INPUT);

        gpioSetPullUpDown(left_encoder_pin_, PI_PUD_UP);
        gpioSetPullUpDown(right_encoder_pin_, PI_PUD_UP);

        gpioSetAlertFuncEx(left_encoder_pin_, pulseEx_(left_encoder_pin_), this);
        gpioSetAlertFuncEx(right_encoder_pin_, pulseEx_(right_encoder_pin_), this);
    }

    void deactivate()
    {
        gpioWrite(left_motor_in1_, 0);
        gpioWrite(left_motor_in2_, 0);
        gpioWrite(right_motor_in1_, 0);
        gpioWrite(right_motor_in2_, 0);
        gpioPWM(motor_en1_pwm_pin_, 0); 
        gpioPWM(motor_en2_pwm_pin_, 0);
        gpioTerminate();  
    }

    void write(const double rads, const std::string &source)
    {
        
        std::string motor = source;
        bool forward = (rads > 0);
        int pwm = convert_rads_pwm_(rads);
        pwm = (pwm > max_pwm_) ? max_pwm_ : pwm;

        
        if (motor == "left")
        {
            write_pwm_(motor, left_motor_in1_, left_motor_in2_, motor_en1_pwm_pin_, pwm, forward);
            left_forward_ = forward;
        }
        else if (motor == "right")
        {
            write_pwm_(motor, right_motor_in1_, right_motor_in2_, motor_en2_pwm_pin_, pwm, forward);
            right_forward_ = forward;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor values were not written!");
        }
    }

    void read(int &left_wheel_pos, int &right_wheel_pos, double &left_wheel_vel, double &right_wheel_vel, const double delta_seconds)
    {

        // left_vel = (2*3.14*wheel_radius_*(last_left_encoder_counter_value_ - left_encoder_counter_)/resolution_)/delta_seconds;
        // right_vel = (2*3.14*wheel_radius_*(last_right_encoder_counter_value_ - right_encoder_counter_)/resolution_)/delta_seconds;

        left_wheel_pos = left_encoder_counter_;
        right_wheel_pos = right_encoder_counter_;

        left_wheel_vel = (last_left_encoder_counter_value_ - left_encoder_counter_)/delta_seconds;
        right_wheel_vel = (last_right_encoder_counter_value_ - right_encoder_counter_)/delta_seconds;


    }

private:

    // Pins variables
    int left_motor_in1_;
    int left_motor_in2_;
    int right_motor_in1_;
    int right_motor_in2_;
    int motor_en1_pwm_pin_;
    int motor_en2_pwm_pin_;
    int left_encoder_pin_;
    int right_encoder_pin_;

    // Parameter variables
    int resolution_;
    double wheel_radius_;
    double wheel_span_;
    double max_vel_rpm_;
    double max_pwm_;

    // Auxiliary variables
    int last_left_encoder_counter_value_;
    int last_right_encoder_counter_value_;
    int left_encoder_counter_;
    int right_encoder_counter_;
    bool left_forward_;
    bool right_forward_;


    void write_pwm_(const std::string source, const int in1_pin, const int in2_pin, const int pwm_pin, const int pwm, const bool forward)
    {
        if (pwm != 0)
        {
            if (forward)
            {
                gpioWrite(in1_pin, 1);
                gpioWrite(in2_pin, 0);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "%s going forward", source);
            }
            else
            {
                gpioWrite(in1_pin, 0);
                gpioWrite(in2_pin, 1);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "%s going backward", source);
            }
        }
        else
        {
            gpioWrite(in1_pin, 0);
            gpioWrite(in2_pin, 0);
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "%s stopped", source);
        }
        gpioPWM(pwm_pin, pwm); // PWM duty cycle range is 0-255
    }

    int convert_rads_pwm_(double rads)
    {
        return std::abs(static_cast<int>(rads * (max_pwm_/(max_vel_rpm_ * 2 * 3.14 / 60))));
    }

    void pulseEx_(const int encoder_pin)
    {
        if(encoder_pin == left_encoder_pin_)
        {
            if(left_forward_== true)
            {
                left_encoder_counter_++;
            }
            else
            {
                left_encoder_counter_--;
            }

        }
        else if(encoder_pin == right_encoder_pin_)
        {
            if(right_forward_== true)
            {
                right_encoder_counter_++;
            }
            else
            {
                right_encoder_counter_--;
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Wrong encoder pin number!");
        }
    }
};

#endif
