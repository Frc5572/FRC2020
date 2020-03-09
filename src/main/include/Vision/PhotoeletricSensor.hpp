#ifndef PHOTOELCTRIC_HPP
#define PHOTOELCTRIC_HPP

#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

class Photoelctric{
    public:
    Photoelctric(
        frc::DigitalInput& Input,
        frc::DigitalOutput& Output
    );
    void teachSensor();

    private:
    frc::DigitalInput* input;
    frc::DigitalOutput* output;

};
#endif