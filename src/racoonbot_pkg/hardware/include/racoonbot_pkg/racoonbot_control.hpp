#ifndef DIFFDRIVE_RACOONBOT_CONTROL_HPP
#define DIFFDRIVE_RACOONBOT_CONTROL_HPP

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <wiringPi.h>
#include "rclcpp/rclcpp.hpp"

class RacoonBotControl
{
public:
    RacoonBotControl()
        : left_motor_in1_(23),
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
          right_forward_(true),
          stop_thread_(false)
    {
    }

    ~RacoonBotControl()
    {
        deactivate();
    }

    void activate()
    {
        if (wiringPiSetupGpio() == -1)
        {
            std::cerr << "WiringPi setup failed" << std::endl;
            return;
        }

        pinMode(left_motor_in1_, OUTPUT);
        pinMode(left_motor_in2_, OUTPUT);
        pinMode(right_motor_in1_, OUTPUT);
        pinMode(right_motor_in2_, OUTPUT);

        pinMode(motor_en1_pwm_pin_, PWM_OUTPUT);
        pinMode(motor_en2_pwm_pin_, PWM_OUTPUT);

        pinMode(left_encoder_pin_, INPUT);
        pinMode(right_encoder_pin_, INPUT);

        pullUpDnControl(left_encoder_pin_, PUD_UP);
        pullUpDnControl(right_encoder_pin_, PUD_UP);

        // Start encoder monitoring thread
        stop_thread_ = false;
        encoder_thread_ = std::thread(&RacoonBotControl::monitorEncoders, this);
    }

    void deactivate()
    {
        stop_thread_ = true;
        if (encoder_thread_.joinable())
        {
            encoder_thread_.join();
        }

        digitalWrite(left_motor_in1_, LOW);
        digitalWrite(left_motor_in2_, LOW);
        digitalWrite(right_motor_in1_, LOW);
        digitalWrite(right_motor_in2_, LOW);

        pwmWrite(motor_en1_pwm_pin_, 0);
        pwmWrite(motor_en2_pwm_pin_, 0);
    }

    void write(const double rads, const std::string &source)
    {
        bool forward = (rads > 0);
        int pwm = convert_rads_pwm_(rads);
        pwm = (pwm > max_pwm_) ? max_pwm_ : pwm;

        if (source == "left")
        {
            write_pwm_(left_motor_in1_, left_motor_in2_, motor_en1_pwm_pin_, pwm, forward);
            left_forward_ = forward;
        }
        else if (source == "right")
        {
            write_pwm_(right_motor_in1_, right_motor_in2_, motor_en2_pwm_pin_, pwm, forward);
            right_forward_ = forward;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor values were not written!");
        }
    }

    void read(int &left_wheel_pos, int &right_wheel_pos, double &left_wheel_vel, double &right_wheel_vel, const double delta_seconds)
    {
        left_wheel_pos = left_encoder_counter_;
        right_wheel_pos = right_encoder_counter_;

        left_wheel_vel = (last_left_encoder_counter_value_ - left_encoder_counter_) / delta_seconds;
        right_wheel_vel = (last_right_encoder_counter_value_ - right_encoder_counter_) / delta_seconds;
    }

private:
    int left_motor_in1_;
    int left_motor_in2_;
    int right_motor_in1_;
    int right_motor_in2_;
    int motor_en1_pwm_pin_;
    int motor_en2_pwm_pin_;
    int left_encoder_pin_;
    int right_encoder_pin_;

    int resolution_;
    double wheel_radius_;
    double wheel_span_;
    double max_vel_rpm_;
    double max_pwm_;

    int last_left_encoder_counter_value_;
    int last_right_encoder_counter_value_;
    std::atomic<int> left_encoder_counter_;
    std::atomic<int> right_encoder_counter_;
    bool left_forward_;
    bool right_forward_;

    std::thread encoder_thread_;
    std::atomic<bool> stop_thread_;

    void write_pwm_(const int in1_pin, const int in2_pin, const int pwm_pin, const int pwm, const bool forward)
    {
        if (pwm != 0)
        {
            if (forward)
            {
                digitalWrite(in1_pin, HIGH);
                digitalWrite(in2_pin, LOW);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor going forward");
            }
            else
            {
                digitalWrite(in1_pin, LOW);
                digitalWrite(in2_pin, HIGH);
                RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor going backward");
            }
        }
        else
        {
            digitalWrite(in1_pin, LOW);
            digitalWrite(in2_pin, LOW);
            RCLCPP_INFO(rclcpp::get_logger("RacoonBotSystemHardware"), "Motor stopped");
        }
        pwmWrite(pwm_pin, pwm);
    }

    int convert_rads_pwm_(double rads)
    {
        return std::abs(static_cast<int>(rads * (max_pwm_ / (max_vel_rpm_ * 2 * 3.14 / 60))));
    }

    void monitorEncoders()
    {
        while (!stop_thread_)
        {
            if (digitalRead(left_encoder_pin_) == LOW)
            {
                if (left_forward_)
                {
                    left_encoder_counter_++;
                }
                else
                {
                    left_encoder_counter_--;
                }
            }

            if (digitalRead(right_encoder_pin_) == LOW)
            {
                if (right_forward_)
                {
                    right_encoder_counter_++;
                }
                else
                {
                    right_encoder_counter_--;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

#endif
