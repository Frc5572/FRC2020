#ifndef CLIMB_MANAGER_HPP
#define CLIMB_MANAGER_HPP

#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
#include <frc/SpeedControllerGroup.h>
#include "Movement/ControllerManager.hpp"
#include <frc/VictorSP.h>

class ClimbManager {
 public:
  ClimbManager(
    frc::VictorSP& LeftClimb,
    frc::VictorSP& RightClimb,
    FRC5572Controller& Driver,
    frc::DoubleSolenoid& climbPistons
  );

  void ClimbPeriodic();
  void UpAndDown();
  void Spin();
  private:
  frc::SpeedControllerGroup* climbMotors;

  FRC5572Controller* driver;

  frc::VictorSP* leftClimb;
  frc::VictorSP* rightClimb;

  frc::DoubleSolenoid* climbPistons;

};
#endif