#include "Vision/PhotoeletricSensor.hpp"
#include <unistd.h>

Photoelctric::Photoelctric(
    frc::DigitalInput& Input,
    frc::DigitalOutput& Output){
        this->input = &Input; 
        this->output = &Output;
}

void Photoelctric::teachSensor(){
    std::cout << "Place sensor 12 mm from target" << std::endl;
    
    // std::this_thread::sleep_for (std::chrono::seconds(2));
    usleep(2000);

    this->output->Set(1);
    // std::this_thread::sleep_for (std::chrono::seconds(3));
    usleep(3500);
    this->output->Set(0);
    
    std::cout << "Remove the target" << std::endl;
    // std::this_thread::sleep_for (std::chrono::seconds(2));
    usleep(2000);

    this->output->Set(1);
    // std::this_thread::sleep_for (std::chrono::milliseconds(500)); 
    usleep(500);
    this->output->Set(0);

    std::cout << "wait 30 seconds and the programing is done" << std::endl;
}