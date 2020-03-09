#include "Movement/ClimbManager.hpp"

#include <frc/DoubleSolenoid.h>
ClimbManager::ClimbManager(
    frc::VictorSP& LeftClimb,
    frc::VictorSP& RightClimb,
    FRC5572Controller& Driver,
    frc::DoubleSolenoid& ClimbPistons
){
    this->leftClimb = &LeftClimb;
    this->rightClimb = &RightClimb;
    this->driver = &Driver;
    this->climbPistons = &ClimbPistons;
    //this->climbMotors = new frc::SpeedControllerGroup(LeftClimb, RightClimb);
    //this->leftClimb->SetInverted(true);
    //this->rightClimb->SetInverted(false);

}

void ClimbManager::ClimbPeriodic(){
    ClimbManager::Spin();
    ClimbManager::UpAndDown();
}

void ClimbManager::UpAndDown(){
    if(driver->LT() > 0){
        this->climbPistons->Set(frc::DoubleSolenoid::Value::kReverse); 
    }
    else if(driver->RT() > 0){
       this->climbPistons->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
        this->climbPistons->Set(frc::DoubleSolenoid::Value::kOff);
    }
    
}


void ClimbManager::Spin(){

    if(driver->RB()){
        leftClimb->Set(.4);
        rightClimb->Set(.4);
    }
    else if(driver->LB()){
        leftClimb->Set(-0.4);
        rightClimb->Set(-0.4);
    }
    else{
        leftClimb->Set(0.0);
        rightClimb->Set(0.0);
    }
}
