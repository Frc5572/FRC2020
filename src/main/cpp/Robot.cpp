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
   
}

void Robot::AutonomousInit()     {
    automovement = new AutoMovement{*driveTrain.LeftMotors, *driveTrain.RightMotors, ahrs, *driveTrain.MiddleLeftMotorEncoder, *driveTrain.BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() { 
    automovement->TestDrive();
    
}

void Robot::TeleopInit(){
    shooter.InitPID();
}

void Robot::TeleopPeriodic(){
     LimeLight.Update();

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
