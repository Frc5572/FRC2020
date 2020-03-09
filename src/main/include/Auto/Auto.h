#include "rev/CANSparkMAX.h"
#include <frc/SpeedControllerGroup.h>
#include "AHRS.h"
#include "Auto/pathfinder.h"

class AutoMovement{
    public:
    void TestDrive();
    AutoMovement(frc::SpeedControllerGroup &leftMotors, frc::SpeedControllerGroup &rightMotors, AHRS &gyro, rev::CANEncoder &anyleftencoder, rev::CANEncoder &anyrightencoder);
    frc::SpeedControllerGroup *leftMotors; frc::SpeedControllerGroup *rightMotors; AHRS *gyro;
    EncoderFollower *leftfollower; EncoderFollower *rightfollower;
    rev::CANEncoder *leftencoder; rev::CANEncoder *rightencoder;
    Segment leftTrajectory; Segment rightTrajectory;
    double encoder_position, wheel_circumference, max_velocity, l, r, angle;
    int l_encoder_value, r_encoder_value, POINT_LENGTH, length;
};