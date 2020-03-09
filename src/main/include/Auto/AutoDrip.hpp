#ifndef AUTODRIP_HPP
#define AUTODRIP_HPP

#include "Movement/DriveTrainManager.hpp"
#include "Movement/Hopper.hpp"
#include "Movement/Shooter.hpp"
#include "Vision/VisionManager.hpp"
#include <frc/Timer.h>

class AutoDrip{
    public:
    
    AutoDrip(
        Hopper& hopper,
        DriveTrain& driveTrain,
        Shooter& shooter,
        VisionManager& visionManager
    );

    void ResetTimer();
    void BumperShot();
    void InitiationShot();
    void AngledInitiationShot();

    Hopper* hopper;
    DriveTrain* driveTrain;
    Shooter* shooter;
    VisionManager* visionManager;
    double startTime; 
    frc::Timer m_timer;
};
#endif

/* 1. Bumper shot
    -> drive forward 
    -> shoot 4 balls
    -> drive back to line 
*/

/* 2. Initiation line shot
        -> shoot 4 balls


*/


