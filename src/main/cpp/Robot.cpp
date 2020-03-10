#include <iostream>
#include <string>

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

#include "rev/ColorSensorV3.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "cameraserver/CameraServer.h"

#include <frc/Timer.h>

void Robot::RobotInit(){
   
    m_timer.Start();
}

void Robot::RobotPeriodic(){ 
    LimeLight.Update();

    //driveTrain.Aim();

    driveTrain.Drive();

    shooter.Shots();
    
    climber.ClimbPeriodic();

    hopper.HopperPeriodic();
}

void Robot::AutonomousInit()     {
     m_timer.Reset();
     m_timer.Start();

    automovement = new AutoMovement{*driveTrain.LeftMotors, *driveTrain.RightMotors, ahrs, *BottomLeftMotorEncoder, *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() { 
    while (m_timer.Get() < 15)
    {
    automovement->TestDrive();
    continue;
    }
}

void Robot::TeleopInit(){
    shooter.InitPID();
}

void Robot::TeleopPeriodic(){

    //driveTrain.Aim();

    shooter.RunPID();

    driveTrain.Drive();

    shooter.Shots();
    
    climber.ClimbPeriodic();

    hopper.HopperPeriodic();
}

void Robot::TestInit(){
    
}

void Robot::TestPeriodic(){
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
