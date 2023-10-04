#include <iostream>

#include <Eigen/Dense>

class WalkingPatternGeneration
{
public:
    WalkingPatternGeneration() {};
    ~WalkingPatternGeneration() {};

    // ROBOT PARAMETERS
    double z0 = 0.825;              // height of COM
    double g = 9.81;                // gravity term
    double omega = sqrt(g / z0);    // natural frequency of the robot
    double leg_length = 0.5;        // length of leg ( prismatic length not included ) - need check again
    double foot_radius = 0.05;
    double lp = 0.2;                // length of pelvis ( check needed )

    // NOMINAL GAIT TIMING PARAMETER 
    double alpha1 = 1;
    double alpha2 = 5;
    double alpha3 = 1000;

    double min_L = -0.5; double max_L = 0.5;    // step length
    double min_W = -0.1; double max_W = 0.4;    // step width
    double min_T = 0.4; double max_T = 0.8;     // step time

    double B_l1, B_l2, B_l3;
    double B_u1, B_u2, B_u3;
    double Bl, Bu;

    double nominal_T = 0.0;
    double nominal_L = 0.0;
    double nominal_W = 0.0;
    double nominal_bx = 0.0;
    double nominal_by = 0.0;
    double nominal_tau = 0.0;

    void initialize_qp_param();
    void specifying_nominal_values();
    void update_qp_param();
    void online_foot_time_placement();
    void online_swing_foot_trajectory();
    void com_trajectory_generation();
};