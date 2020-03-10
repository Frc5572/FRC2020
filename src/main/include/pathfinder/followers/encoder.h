#ifndef PATHFINDER_FOL_ENCODER_H_DEF
#define PATHFINDER_FOL_ENCODER_H_DEF

typedef struct {
    double initial_position, ticks_per_revolution;
    double wheel_circumference;
    double kp, ki, kd, kv, ka;
} EncoderConfig;

typedef struct {
    double last_error, heading, output;
    int segment, finished;
} EncoderFollower;

double pathfinder_follow_encoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, double encoder_tick);

double pathfinder_follow_encoder2(EncoderConfig c, EncoderFollower *follower, Segment segment, int trajectory_length, double encoder_tick);

#endif