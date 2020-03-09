#include "Auto/AutoDrip.hpp"



AutoDrip::AutoDrip(
    Hopper& hopper,
    DriveTrain& driveTrain,
    Shooter& shooter,
    VisionManager& visionManager)
{
    this->hopper = &hopper;
    this->driveTrain = &driveTrain;
    this->shooter = &shooter;
    this->visionManager = &visionManager;
}

void AutoDrip::ResetTimer(){
    m_timer.Reset();
    m_timer.Start();
}

void AutoDrip::BumperShot(){
    if(m_timer.Get() < 3){
    driveTrain->LeftMotors->Set(-.2);
    driveTrain->RightMotors->Set(.2);
    }
    // else if(){

    // }
}

void AutoDrip::InitiationShot(){

}


