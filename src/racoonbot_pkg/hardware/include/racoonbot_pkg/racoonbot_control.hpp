#ifndef DIFFDRIVE_RACOONBOT_CONTROL_HPP
#define DIFFDRIVE_RACOONBOT_CONTROL_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <WiringPi/wiringPi/wiringPi.h>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class RacoonBotControl
{
    public:
        RacoonBotControl() = default;

        ~RacoonBotControl(){
            // gpioTerminate();
        }

        void activate()
        {   
            // GPIO Activation
            
            wiringPiSetupGpio();
 
            pinMode(left_motor_in1_, OUTPUT);
            pinMode(left_motor_in2_, OUTPUT);
            pinMode(right_motor_in1_, OUTPUT);
            pinMode(right_motor_in2_, OUTPUT);

            // Create PWM instances
            pinMode(motor_en1_pwm_pin_, PWM_OUTPUT);
            pinMode(motor_en2_pwm_pin_, PWM_OUTPUT);

            // Create interrupts
            wiringPiISR(left_encoder_pin_, INT_EDGE_RISING, &RacoonBotControl::pulse_callback_wrapper_left);
            wiringPiISR(right_encoder_pin_, INT_EDGE_RISING, &RacoonBotControl::pulse_callback_wrapper_right);
        }

        void deactivete()
        {
            //GPIO Deactivation
        }

        void motor_speed_callback(const double duty_c, const std::string& source) {

            double duty_cycle;
            std::string motor = source;
            bool forward = duty_cycle > 0;

            duty_cycle = std::abs(duty_cycle) > 100 ? 100 : std::abs(duty_cycle);

            if (motor == "left") {
                if (duty_cycle != 0) {
                    if (forward) {
                        digitalWrite(left_motor_in1_, HIGH);
                        digitalWrite(left_motor_in2_, LOW);
                        std::cout << "Left going forward!" << std::endl;
                    } else {
                        digitalWrite(left_motor_in1_, LOW);
                        digitalWrite(left_motor_in2_, HIGH);
                        std::cout << "Left going backward!" << std::endl;
                    }
                } else {
                    digitalWrite(left_motor_in1_, LOW);
                    digitalWrite(left_motor_in2_, LOW);
                    std::cout << "Left stopped!" << std::endl;
                }
                pwmWrite(motor_en1_pwm_pin_, duty_cycle);
            } else if (motor == "right") {
                if (duty_cycle != 0) {
                    if (forward) {
                        digitalWrite(right_motor_in1_, HIGH);
                        digitalWrite(right_motor_in2_, LOW);
                        std::cout << "Right going forward!" << std::endl;
                    } else {
                        digitalWrite(right_motor_in1_, LOW);
                        digitalWrite(right_motor_in2_, HIGH);
                        std::cout << "Right going backward!" << std::endl;
                    }
                } else {
                    digitalWrite(right_motor_in1_, LOW);
                    digitalWrite(right_motor_in2_, LOW);
                    std::cout << "Right stopped!" << std::endl;
                }
                pwmWrite(motor_en2_pwm_pin_, duty_cycle);
            } else {
                std::cout << "Something went wrong!" << std::endl;
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
    double wheel_diameter_;
    double wheel_span_;
    int left_encoder_counter_;
    int right_encoder_counter_;

    // Wrapers for pulse callbacks functions
    static void pulse_callback_wrapper_left() {
        instance_->pulse_callback("left");
    }

    static void pulse_callback_wrapper_right() {
        instance_->pulse_callback("right");
    }

    void pulse_callback(const std::string& type) {
        if (type == "left") {
            left_encoder_counter_++;
        } else if (type == "right") {
            right_encoder_counter_++;
        } else {
            std::cout << "Wrong motor type!" << std::endl;
        }
    }
    
    // Static member to hold the instance
    static RacoonBotControl* instance_; 
};

#endif 