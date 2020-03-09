#include "Auto/Auto.h"

AutoMovement::AutoMovement(frc::SpeedControllerGroup &leftMotors, frc::SpeedControllerGroup &rightMotors, AHRS &gyro, rev::CANEncoder &anyleftencoder, rev::CANEncoder &anyrightencoder)
{
POINT_LENGTH = 3;
Waypoint *points = new Waypoint[POINT_LENGTH];

Waypoint p1 = { 0, 0, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
Waypoint p2 = { .5, .5, d2r(45) };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
Waypoint p3 = {  1, 1, d2r(45) };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
points[0] = p1;
points[1] = p2;
points[2] = p3;

TrajectoryCandidate candidate;

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
pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.05, 1.5, 9.0, 20.0, &candidate);

length = candidate.length;

// Array of Segments (the trajectory points) to store the trajectory in
Segment *trajectory = new Segment[length];

// Generate the trajectory
pathfinder_generate(&candidate, trajectory);

Segment *leftTrajectory = new Segment[length];
Segment *rightTrajectory = new Segment[length];

// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.559;

// Generate the Left and Right trajectories of the wheelbase using the 
// originally generated trajectory
pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);


leftfollower->last_error = 0; leftfollower->segment = 0; leftfollower->finished = 0;     // Just in case!
rightfollower->last_error = 0; rightfollower->segment = 0; rightfollower->finished = 0;     // Just in case!


this->leftMotors = &leftMotors;
this->rightMotors = &rightMotors;
this->gyro = &gyro;
this->leftencoder = &anyleftencoder;
this->rightencoder = &anyrightencoder;
AutoMovement::gyro->ZeroYaw();
leftencoder->SetPositionConversionFactor(42);
rightencoder->SetPositionConversionFactor(42);
max_velocity = 1.5;
wheel_circumference = .47877887204060999;
}

void AutoMovement::TestDrive()
{
    while (true)
    {
    EncoderConfig leftconfig = { leftencoder->GetPosition(), 42, wheel_circumference, 1.0, 0.0, 0.2, 1.0 / max_velocity, 0.0};  
    EncoderConfig rightconfig = { rightencoder->GetPosition(), 42, wheel_circumference, 1.0, 0.0, 0.2, 1.0 / max_velocity, 0.0};  

    l = pathfinder_follow_encoder(leftconfig, leftfollower, &leftTrajectory, length, leftencoder->GetPosition());
    r = pathfinder_follow_encoder(rightconfig, rightfollower, &rightTrajectory, length, rightencoder->GetPosition());

    angle = gyro->GetAngle();
    double desired_heading = leftfollower->heading;
    double differeince_angle = desired_heading - angle;

    differeince_angle = std::fmod(differeince_angle, 360.0);
    if (std::abs(differeince_angle) > 180.0) 
    {
        differeince_angle = (differeince_angle > 0) ? differeince_angle - 360 : differeince_angle + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * differeince_angle;
    std::cout << "l is: " << l << std::endl;
    std::cout << "r is: " << r << std::endl;
    leftMotors->Set(l + turn);
    rightMotors->Set(r - turn);
    }
}
