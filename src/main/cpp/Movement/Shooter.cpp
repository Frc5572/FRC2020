#include "Movement/Shooter.hpp"

void Shooter::InitPID(){
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    leftMotor->RestoreFactoryDefaults();
    rightMotor->RestoreFactoryDefaults();
    
    rightMotor->SetInverted(true);

    // set PID coefficients
    m_pidController->SetP(kP);
    m_pidController->SetI(kI);
    m_pidController->SetD(kD);
    m_pidController->SetIZone(kIz);
    m_pidController->SetFF(kFF);
    m_pidController->SetOutputRange(kMinOutput, kMaxOutput);

    m_pidController2->SetP(kP);
    m_pidController2->SetI(kI);
    m_pidController2->SetD(kD);
    m_pidController2->SetIZone(kIz);
    m_pidController2->SetFF(kFF);
    m_pidController2->SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("SetPoint", SetP);
    frc::SmartDashboard::PutNumber("Auto Selection", AutoSelection);
}
void Shooter::AutoPID()
{
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);
    double Auto = frc::SmartDashboard::GetNumber("Auto Selection", 0);
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController->SetP(p); m_pidController2->SetP(p); kP = p; }
        if((i != kI)) { m_pidController->SetI(i); m_pidController2->SetI(i); kI = i; }
        if((d != kD)) { m_pidController->SetD(d); m_pidController2->SetD(d); kD = d; }
        if((iz != kIz)) { m_pidController->SetIZone(iz); m_pidController2->SetIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController->SetFF(ff); m_pidController2->SetFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController->SetOutputRange(min, max); m_pidController2->SetOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

    if (Auto == 0)
    {
        SetPoint = 2700; //3375
        SP = SetPoint;
        Hood->Set(frc::DoubleSolenoid::Value::kForward);
        m_pidController->SetReference(SetPoint, rev::ControlType::kVelocity);
        leftRPM = leftMotorEncoder->GetVelocity();
        rightRPM = rightMotorEncoder->GetVelocity();
        rpm = ((leftRPM + rightRPM) / 2);   
    }

    else
    {
        SetPoint = 0;
    }
    
         

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", leftMotorEncoder->GetVelocity());

    frc::SmartDashboard::PutNumber("RPM", rpm );
    frc::SmartDashboard::PutNumber("Left RPM", leftRPM);
    frc::SmartDashboard::PutNumber("Right RPM", rightRPM);
    
}
void Shooter::RunPID()
{
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController->SetP(p); m_pidController2->SetP(p); kP = p; }
        if((i != kI)) { m_pidController->SetI(i); m_pidController2->SetI(i); kI = i; }
        if((d != kD)) { m_pidController->SetD(d); m_pidController2->SetD(d); kD = d; }
        if((iz != kIz)) { m_pidController->SetIZone(iz); m_pidController2->SetIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController->SetFF(ff); m_pidController2->SetFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController->SetOutputRange(min, max); m_pidController2->SetOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

    if(this->Operator->POV() == 0 ){ //bumber
        SetPoint = 2600;
        Hood->Set(frc::DoubleSolenoid::Value::kReverse);    
    }
    else if(this->Operator->POV() == 90){ //init
        SetPoint = 3375; //
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if(this->Operator->POV() == 270){ //trench
        SetPoint = 3800;
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if (Operator->POV() == 180) //colorwheel
    {
        SetPoint = 4500;
        Hood->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
        SetPoint = 0;
        Hood->Set(frc::DoubleSolenoid::Value::kReverse); 
    }

     m_pidController->SetReference(SetPoint, rev::ControlType::kVelocity);
     m_pidController2->SetReference(SetPoint, rev::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", leftMotorEncoder->GetVelocity());

    
    leftRPM = leftMotorEncoder->GetVelocity();
    rightRPM = rightMotorEncoder->GetVelocity();
    rpm = ((leftRPM + rightRPM) / 2);

    frc::SmartDashboard::PutNumber("RPM", rpm );
    frc::SmartDashboard::PutNumber("Left RPM", leftRPM);
    frc::SmartDashboard::PutNumber("Right RPM", rightRPM);

    }


Shooter::Shooter(
    rev::CANSparkMax &LeftMotor,
    rev::CANSparkMax &RightMotor,
    frc::DoubleSolenoid &Hood,
    FRC5572Controller &Operator
    ){

    m_pidController = new rev::CANPIDController{LeftMotor};
    m_pidController2 = new rev::CANPIDController{RightMotor};


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
    if(this->Operator->POV() == 0 ){
        shooterMotors->Set(.65);
        Hood->Set(frc::DoubleSolenoid::Value::kReverse);    
    }
    else if(this->Operator->POV() == 90){
        shooterMotors->Set(.78);
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if(this->Operator->POV() == 270){
        shooterMotors->Set(.90); //small adjustment from .92 to .94
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else{
        shooterMotors->Set(0.0);
        Hood->Set(frc::DoubleSolenoid::Value::kReverse); 
    }
        
}

void Shooter::Shots(){

    if(this->Operator->POV() == 0 ){
        shooterMotors->Set(.47);
        Hood->Set(frc::DoubleSolenoid::Value::kReverse);    
    }
    else if(this->Operator->POV() == 90){
        shooterMotors->Set(.6);
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if(this->Operator->POV() == 270){
        shooterMotors->Set(.7); //small adjustment from .92 to .94
        Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if (Operator->POV() == 180)
    {
        shooterMotors->Set(.85);
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
