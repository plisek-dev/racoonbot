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
        max_vel_rpm_(48),
        max_pwm_(255)
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

        gpioSetMode(left_motor_in1_, PI_OUTPUT);
        gpioSetMode(left_motor_in2_, PI_OUTPUT);
        gpioSetMode(right_motor_in1_, PI_OUTPUT);
        gpioSetMode(right_motor_in2_, PI_OUTPUT);

        // Create PWM instances
        gpioSetMode(motor_en1_pwm_pin_, PI_OUTPUT);
        gpioSetMode(motor_en2_pwm_pin_, PI_OUTPUT);

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

    int convert_rads_pwm(double rads)
    {
                return std::abs(static_cast<int>(rads * (max_pwm_/(max_vel_rpm_ * 2 * 3.14 / 60))));
    }


    void write(const double rads, const std::string &source)
    {
        int pwm;
        std::string motor = source;
        bool forward = rads > 0;

        pwm = convert_rads_pwm(rads);

        pwm = (pwm > max_pwm_) ? max_pwm_ : pwm;

        
        if (motor == "left")
        {
            if (pwm != 0)
            {
                if (forward)
                {
                    gpioWrite(left_motor_in1_, 1);
                    gpioWrite(left_motor_in2_, 0);
                    RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Left going forward");
                }
                else
                {
                    gpioWrite(left_motor_in1_, 0);
                    gpioWrite(left_motor_in2_, 1);
                    RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Left going backward");
                }
            }
            else
            {
                gpioWrite(left_motor_in1_, 0);
                gpioWrite(left_motor_in2_, 0);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Left stopped");
            }
            gpioPWM(motor_en1_pwm_pin_, pwm); // PWM duty cycle range is 0-1024
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "PWM LEFT: %d", pwm);

        }
        else if (motor == "right")
        {
            if (pwm != 0)
            {
                if (forward)
                {
                    gpioWrite(right_motor_in1_, 1);
                    gpioWrite(right_motor_in2_, 0);
                    RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Right going forward");
                }
                else
                {
                    gpioWrite(right_motor_in1_, 0);
                    gpioWrite(right_motor_in2_, 1);
                    RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Right going backward");
                }
            }
            else
            {
                gpioWrite(right_motor_in1_, 0);
                gpioWrite(right_motor_in2_, 0);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Right stopped");
            }
            gpioPWM(motor_en2_pwm_pin_, pwm); // PWM duty cycle range is 0-1024
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "PWM RIGHT: %d", pwm);

        }
        else
        {
              RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor values were not written!");
        }
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

    //Parameter variables
    int resolution_;
    double wheel_radius_;
    double wheel_span_;
    int left_encoder_counter_;
    int right_encoder_counter_;

    double max_vel_rpm_;
    double max_pwm_;
};

#endif
