#include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <wiringPi.h>
#include <iostream>

int main(){

    // Pins variables
    int left_motor_in1_;
    int left_motor_in2_;
    int right_motor_in1_;
    int right_motor_in2_;
    int motor_en1_pwm_pin_;
    int motor_en2_pwm_pin_;
    int left_encoder_pin_;
    int right_encoder_pin_;
    
    left_motor_in1_ = 23;
    left_motor_in2_ = 24;
    right_motor_in1_ = 5;
    right_motor_in2_ = 6;
    motor_en1_pwm_pin_ = 25;
    motor_en2_pwm_pin_ = 26;
    left_encoder_pin_ = 17;
    right_encoder_pin_ = 27;

    //Parameter variables
    int resolution_;
    double wheel_diameter_;
    double wheel_span_;
    int left_encoder_counter_;
    int right_encoder_counter_;

    wiringPiSetupGpio();
 
    pinMode(left_motor_in1_, OUTPUT);
    pinMode(left_motor_in2_, OUTPUT);
    pinMode(right_motor_in1_, OUTPUT);
    pinMode(right_motor_in2_, OUTPUT);

    // Create PWM instances
    pinMode(motor_en1_pwm_pin_, PWM_OUTPUT);
    pinMode(motor_en2_pwm_pin_, PWM_OUTPUT);
    
    digitalWrite(left_motor_in1_, HIGH);
    digitalWrite(left_motor_in2_, LOW);

    while(1){
        digitalWrite(left_motor_in1_, HIGH);
        pwmWrite(motor_en1_pwm_pin_, 90);
        std::cout << "Writing!" << std::endl;
    }
                
}