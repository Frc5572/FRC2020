#include <iostream>
#include <string>

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

#include "rev/ColorSensorV3.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include <frc/Timer.h>

void Robot::RobotInit(){
   
    m_timer.Start();
}

void Robot::RobotPeriodic(){ 
   
}

void Robot::AutonomousInit()     {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();

    //automovement = new AutoMovement{*driveTrain.LeftMotors, *driveTrain.RightMotors, ahrs, *BottomLeftMotorEncoder, *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() { 
    // while (m_timer.Get() < 15)
    // {
    // automovement->TestDrive();
    // continue;
    // }
    while(m_timer.Get() < 6){
        shooter.AutoPID();
        m_leftShooter.Set(.72); 
        m_rightShooter.Set(.72); 
        //intake.Set(frc::DoubleSolenoid::Value::kForward);
        if (shooter.rpm >= shooter.SP)
        {
            m_hopper.Set(.3);
        }
        continue;
    }
    while(m_timer.Get() > 6 && m_timer.Get() < 11){
        //intake.Set(frc::DoubleSolenoid::Value::kReverse);
        shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
        m_leftShooter.Set(0);
        m_rightShooter.Set(0);
        m_hopper.Set(0.0);
        continue;
    }
    while(m_timer.Get() > 11 && m_timer.Get() < 12)
    {
        m_rightBottomMotor.Set(.3);
        m_rightMiddleMotor.Set(.3);

        m_leftBottomMotor.Set(-.3);
        m_leftMiddleMotor.Set(-.3);
        continue;
    }
    m_rightBottomMotor.Set(0);
    m_rightMiddleMotor.Set(0);

    m_leftBottomMotor.Set(0);
    m_leftMiddleMotor.Set(0);
    //dripping springs auto fixed below
//     while(m_timer.Get() < 2){
//         m_leftShooter.Set(.7);
//         m_rightShooter.Set(.7);
//     }
//     while(m_timer.Get() < 10 & m_timer.Get() > 2){
//         shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
//         //intake.Set(frc::DoubleSolenoid::Value::kForward);
//         m_leftShooter.Set(.7);
//         m_rightShooter.Set(.7);
//         m_hopper.Set(.3);
//         continue;
//     }
//     while(m_timer.Get() > 10 && m_timer.Get() < 11){
//         //intake.Set(frc::DoubleSolenoid::Value::kReverse);
//         shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
//         m_leftShooter.Set(0);
//         m_rightShooter.Set(0);
//         m_hopper.Set(0.0);
//         continue;
//     }

//     while(m_timer.Get() > 11 && m_timer.Get() < 11.5){
//         m_rightBottomMotor.Set(.3);
//         m_rightMiddleMotor.Set(.3);

//         m_leftBottomMotor.Set(-.3);
//         m_leftMiddleMotor.Set(-.3);
//         continue;
//     } 
//         m_rightBottomMotor.Set(0);
//         m_rightMiddleMotor.Set(0);

//         m_leftBottomMotor.Set(0);
//         m_leftMiddleMotor.Set(0);
// }
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
