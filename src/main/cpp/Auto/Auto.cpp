#include "Auto/Auto.h"
#include <frc/Timer.h>
#include <iostream>

AutoMovement::AutoMovement(frc::SpeedControllerGroup &leftMotors, frc::SpeedControllerGroup &rightMotors, AHRS &gyro, rev::CANEncoder &anyleftencoder, rev::CANEncoder &anyrightencoder)
{
POINT_LENGTH = 3;
Waypoint *points = new Waypoint[POINT_LENGTH];

Waypoint p1 = { 0, 0, d2r(0) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
Waypoint p2 = { .5, 0, d2r(0) };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
Waypoint p3 = {  1, 0, d2r(0) };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
points[0] = p1;
points[1] = p2;
points[2] = p3;
std::cout << "line 16" << std::endl;
TrajectoryCandidate candidate;
std::cout << "line 18" << std::endl;
// Prepare the Trajectory for Generation.
//
// Arguments: 
// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
//                      PATHFINDER_SAMPLES_LOW  (10 000)
//                      PATHFINDER_SAMPLES_FAST (1 000)
// Time Step:           0.001 Seconds
// Max Velocity:        15 m/s
// Max Acceleration:    10 m/s/s
// Max Jerk:            60 m/s/s/s
pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_LOW, 0.0008, 1.0, 1.0, 2.0, &candidate); //0.0005, 1.5 v, 2.0 a, 4.0 j
std::cout << "line 31" << std::endl;
length = candidate.length;

// Array of Segments (the trajectory points) to store the trajectory in
Segment *trajectory = new Segment[length];
std::cout << "line 36" << std::endl;
// Generate the trajectory
pathfinder_generate(&candidate, trajectory);
std::cout << "line 39" << std::endl;
Segment *leftTrajectory = new Segment[length];
Segment *rightTrajectory = new Segment[length];

// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.559;

// Generate the Left and Right trajectories of the wheelbase using the 
// originally generated trajectory
pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);
std::cout << "line 49" << std::endl;

leftfollower = new EncoderFollower{0.0, 0.0, 0.0, 0, 0};
rightfollower = new EncoderFollower{0.0, 0.0, 0.0, 0, 0};
// leftfollower->last_error = 0.0; leftfollower->segment = 0; leftfollower->finished = 0;     // Just in case!
// rightfollower->last_error = 0.0; rightfollower->segment = 0; rightfollower->finished = 0;     // Just in case!
std::cout << "line 53" << std::endl;

this->leftMotors = &leftMotors;
this->rightMotors = &rightMotors;
this->gyro = &gyro;
this->leftencoder = &anyleftencoder;
this->rightencoder = &anyrightencoder;
AutoMovement::gyro->ZeroYaw();
max_velocity = 2.5;
wheel_circumference = .47877887204060999;
std::cout << "line 65" << std::endl;
}

void AutoMovement::TestDrive()
{
    frc::Timer m_timer;
    m_timer.Start();
    l = 0; r = 0;
    leftencoder->SetPosition(0);
    rightencoder->SetPosition(0);
    gyro->ZeroYaw();
    std::cout << "l0 is: " << l << std::endl;
    std::cout << "r is: " << r << std::endl;
    std::cout << "44 " << leftencoder->GetPosition() << std::endl;
    std::cout << "44 " << rightencoder->GetPosition() << std::endl;
    EncoderConfig leftconfig = { leftencoder->GetPosition(), 10.5, wheel_circumference, 1.0, 0.0, 0.0, (1.0 / max_velocity), 0.0};  
    EncoderConfig rightconfig = { rightencoder->GetPosition(), 10.5, wheel_circumference, 1.0, 0.0, 0.0, (1.0 / max_velocity), 0.0};  
    std::cout << "I am here too" << std::endl;
    while (true){
    l = pathfinder_follow_encoder(leftconfig, leftfollower, &leftTrajectory, length, leftencoder->GetPosition());
    r = pathfinder_follow_encoder(rightconfig, rightfollower, &rightTrajectory, length, rightencoder->GetPosition());
    std::cout << "l1 is: " << l << std::endl;
    std::cout << "r is: " << r << std::endl;
    angle = gyro->GetAngle();
    std::cout << angle << std::endl;
    double desired_heading = leftfollower->heading;
    double differeince_angle = desired_heading - angle;
    std::cout << "l2 is: " << l << std::endl;
    std::cout << "r is: " << r << std::endl;
    differeince_angle = std::fmod(differeince_angle, 360.0);
    if (std::abs(differeince_angle) > 180.0) 
    {
        differeince_angle = (differeince_angle > 0) ? differeince_angle - 360 : differeince_angle + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * differeince_angle;
    std::cout << "l3 is: " << l << std::endl;
    std::cout << "r is: " << r << std::endl;
    std::cout << turn << std::endl;
    leftMotors->Set(l + turn);
    rightMotors->Set(r - turn);
    if (m_timer.Get() > 15)
    {
        break;
    }
    }
}
