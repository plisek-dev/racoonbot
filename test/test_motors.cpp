#include <pigpio.h>
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

    if (gpioInitialise() < 0)
    {
        std::cerr << "Error initializing PiGPIO" << std::endl;
        return 1;
    }

    gpioSetMode(left_motor_in1_, PI_OUTPUT);
    gpioSetMode(left_motor_in2_, PI_OUTPUT);
    gpioSetMode(right_motor_in1_, PI_OUTPUT);
    gpioSetMode(right_motor_in2_, PI_OUTPUT);

    // Create PWM instances
    gpioSetMode(motor_en1_pwm_pin_, PI_OUTPUT);
    gpioSetMode(motor_en2_pwm_pin_, PI_OUTPUT);

    gpioWrite(left_motor_in1_, 1);
    gpioWrite(left_motor_in2_, 0);
    float duty_cycle;

    std::cout << "Wpisz duty cycle: ";
    std::cin >> duty_cycle;

    while(1){
        gpioWrite(left_motor_in1_, 1);
        gpioPWM(motor_en1_pwm_pin_, duty_cycle);
        std::cout << "Writing!" << std::endl;
    }

    gpioTerminate();
    return 0;
}
