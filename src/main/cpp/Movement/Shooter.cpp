#include "Movement/Shooter.hpp"
#define PID_TUNING

void Shooter::ResetPID(bool init = false)
{
    rev::CANPIDController m_pidController = leftMotor->GetPIDController();
    rev::CANPIDController m_pidController2 = rightMotor->GetPIDController();
    #ifdef PID_TUNING
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
    #endif
    if( init ){
        // set PID coefficients
        m_pidController.SetP(kP); m_pidController2.SetP(kP);
        m_pidController.SetI(kI); m_pidController2.SetI(kI);
        m_pidController.SetD(kD); m_pidController2.SetD(kD);
        m_pidController.SetIZone(kIz); m_pidController2.SetIZone(kIz);
        m_pidController.SetFF(kFF); m_pidController2.SetFF(kFF);
        m_pidController.SetOutputRange(kMinOutput, kMaxOutput); m_pidController2.SetOutputRange(kMinOutput, kMaxOutput);
    }
    #ifdef PID_TUNING
    else
    {
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.SetP(p); m_pidController2.SetP(p); kP = p; }
        if((i != kI)) { m_pidController.SetI(i); m_pidController2.SetI(i); kI = i; }
        if((d != kD)) { m_pidController.SetD(d); m_pidController2.SetD(d); kD = d; }
        if((iz != kIz)) { m_pidController.SetIZone(iz); m_pidController2.SetIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.SetFF(ff); m_pidController2.SetFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.SetOutputRange(min, max); m_pidController2.SetOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }
    }
    #endif
    
}


Shooter::Shooter(
    rev::CANSparkMax &LeftMotor,
    rev::CANSparkMax &RightMotor,
    frc::DoubleSolenoid &Hood,
    FRC5572Controller &Operator
    ){
    this->leftMotor = &LeftMotor;
    this->rightMotor = &RightMotor;
    this->Hood = &Hood;
    this->Operator = &Operator;
    shooterMotors = new frc::SpeedControllerGroup{ LeftMotor, RightMotor};
    leftMotorEncoder = new rev::CANEncoder{LeftMotor};
    rightMotorEncoder = new rev::CANEncoder{RightMotor};

}

void Shooter::Shot(){

    if(Operator->B()){
        Hood->Set(frc::DoubleSolenoid::Value::kForward); //do toggle
    }
    else{
      Hood->Set(frc::DoubleSolenoid::Value::kReverse);  
    }

    if(Tracked)
    {
        shooterMotors->Set(Operator->RT());
    }
    else
    {
        shooterMotors->Set(0); 
    }
}

void Shooter::Test(){
    if(Operator->LB()){
        //Hood->Set(frc::DoubleSolenoid::Value::kForward); 
        shooterMotors->Set(.65); 
    }
    else{
        shooterMotors->Set(0);
        //Hood->Set(frc::DoubleSolenoid::Value::kReverse); 
    }
}

void Shooter::TestRPM(){
    rpm = leftMotorEncoder->GetVelocity();
    std::cout << "RPM is: " << leftMotorEncoder->GetVelocity() << std::endl;
}

void Shooter::Shots(){

    if(this->Operator->POV() == 0 ){
        shooterMotors->Set(.65);
        Hood->Set(frc::DoubleSolenoid::Value::kReverse);    
    }
    else if(this->Operator->POV() == 90){
        shooterMotors->Set(.87);
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if(this->Operator->POV() == 270){
        shooterMotors->Set(.92); //small adjustment from .92 to .94
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else{
        shooterMotors->Set(0.0);
        Hood->Set(frc::DoubleSolenoid::Value::kReverse); 
    }
    leftRPM = leftMotorEncoder->GetVelocity();
    rightRPM = rightMotorEncoder->GetVelocity();
    rpm = ((leftRPM + rightRPM) / 2);

    frc::SmartDashboard::PutNumber("RPM", rpm);
    frc::SmartDashboard::PutNumber("Left RPM", leftRPM);
    frc::SmartDashboard::PutNumber("Right RPM", rightRPM);
}
