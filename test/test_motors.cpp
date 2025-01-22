#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

int main() {
    // Pins variables
    int left_motor_in1_;
    int left_motor_in2_;
    int right_motor_in1_;
    int right_motor_in2_;
    int motor_en1_pwm_pin_;
    int motor_en2_pwm_pin_;
    int left_encoder_pin_;
    int right_encoder_pin_;

    left_motor_in1_ = 4;  // WiringPi pin 4 = BCM GPIO 23
    left_motor_in2_ = 5;  // WiringPi pin 5 = BCM GPIO 24
    right_motor_in1_ = 21; // WiringPi pin 21 = BCM GPIO 5
    right_motor_in2_ = 22; // WiringPi pin 22 = BCM GPIO 6
    motor_en1_pwm_pin_ = 6;  // WiringPi pin 6 = BCM GPIO 25
    motor_en2_pwm_pin_ = 25; // WiringPi pin 25 = BCM GPIO 26
    left_encoder_pin_ = 0;  // WiringPi pin 0 = BCM GPIO 17
    right_encoder_pin_ = 2; // WiringPi pin 2 = BCM GPIO 27

    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Error initializing WiringPi" << std::endl;
        return 1;
    }

    // Set pin modes
    pinMode(left_motor_in1_, OUTPUT);
    pinMode(left_motor_in2_, OUTPUT);
    pinMode(right_motor_in1_, OUTPUT);
    pinMode(right_motor_in2_, OUTPUT);

    // Initialize software PWM
    if (softPwmCreate(motor_en1_pwm_pin_, 0, 100) != 0) {
        std::cerr << "Error initializing software PWM for motor_en1_pwm_pin_" << std::endl;
        return 1;
    }

    if (softPwmCreate(motor_en2_pwm_pin_, 0, 100) != 0) {
        std::cerr << "Error initializing software PWM for motor_en2_pwm_pin_" << std::endl;
        return 1;
    }

    float duty_cycle;

    std::cout << "Enter duty cycle (0-100): ";
    std::cin >> duty_cycle;

    if (duty_cycle < 0 || duty_cycle > 100) {
        std::cerr << "Duty cycle must be between 0 and 100." << std::endl;
        return 1;
    }

    while (1) {
        digitalWrite(left_motor_in1_, HIGH);
        digitalWrite(left_motor_in2_, LOW);
        softPwmWrite(motor_en1_pwm_pin_, duty_cycle);
        std::cout << "Writing!" << std::endl;
    }

    return 0;
}
