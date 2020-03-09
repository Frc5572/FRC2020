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
}

void Robot::AutonomousPeriodic() { 
    while(m_timer.Get() < 2){
        m_leftShooter.Set(.88);
        m_rightShooter.Set(.88);
    }
    while(m_timer.Get() < 10 && m_timer.Get() > 2){
        shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
        //intake.Set(frc::DoubleSolenoid::Value::kForward);
        m_leftShooter.Set(.88);
        m_rightShooter.Set(.88);
        m_hopper.Set(.3);
        continue;
    }
    while(m_timer.Get() > 10 && m_timer.Get() < 11){
        //intake.Set(frc::DoubleSolenoid::Value::kReverse);
        shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
        m_leftShooter.Set(0);
        m_rightShooter.Set(0);
        m_hopper.Set(0.0);
        continue;
    }

    while(m_timer.Get() > 11 && m_timer.Get() < 12){
        m_rightBottomMotor.Set(.3);
        m_rightMiddleMotor.Set(.3);

        m_leftBottomMotor.Set(-.3);
        m_leftMiddleMotor.Set(-.3);
        continue;
    }
    
    // while(m_timer.Get() < 1.3){
    //     m_rightBottomMotor.Set(-.3);
    //     m_rightMiddleMotor.Set(-.3);

    //     m_leftBottomMotor.Set(.3);
    //     m_leftMiddleMotor.Set(.3);
    //     continue;
    // }

    // while(m_timer.Get() > 2 && m_timer.Get() < 4){
    //     m_leftShooter.Set(.68);
    //     m_rightShooter.Set(.68);
    //     continue;
    // }

    // while(m_timer.Get() > 4 && m_timer.Get() < 13){
    //     m_hopper.Set(.2);
    //     m_leftShooter.Set(.68);
    //     m_rightShooter.Set(.68);
    //     continue;
    // }

    
}

void Robot::TeleopInit(){
    
}

void Robot::TeleopPeriodic(){

}

void Robot::TestInit(){
    
}

void Robot::TestPeriodic(){
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
