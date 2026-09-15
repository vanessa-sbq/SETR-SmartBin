#include <PiPCA9685/PCA9685.h>
#include "MotorControllere.h"
#include "MotorPins.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    try {
        /* PiPCA9685::PCA9685 pca{}; */

        
        
        PiPCA9685::PCA9685 pwm("/dev/i2c-1", 0x7f);
        pwm.set_pwm_freq(60.0);
        /* pwm.set_pwm(0,0,990);//20479);
        pwm.set_pwm(1,0,1000);//20479);
        pwm.set_pwm(2,0,990);//20479);
        pwm.set_pwm(3,0,1000);//20479); */

        pwm.set_pwm(0,0,20400);//20479);
        pwm.set_pwm(1,0,20479);//);
        pwm.set_pwm(2,0,20400);//20479);
        pwm.set_pwm(3,0,20479);//);
        MotorPins pins;
        MotorController motors("/dev/gpiochip0", pins/* , pwm */);

        uint16_t speed = 0xFFFF;
        std::cout << "ahead..." << std::endl;
        /* motors.change_speed(speed); */ 
        motors.go_ahead(speed);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        motors.stop_car();
        std::cout << "done" << std::endl;
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
