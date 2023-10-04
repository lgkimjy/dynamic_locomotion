#include "WalkingPatternGeneration.h"

void WalkingPatternGeneration::initialize_qp_param()
{

}

void WalkingPatternGeneration::specifying_nominal_values()
{
    std::cout << "specifying_nominal_values" << std::endl;
}

void WalkingPatternGeneration::update_qp_param()
{

}

void WalkingPatternGeneration::online_foot_time_placement()
{
    // update_qp_param();

	// example.getPrimalSolution( xOpt );

    // u_x = xOpt[0];
    // u_y = xOpt[1];
    // t_step = (1 / omega) * log(xOpt[2]);
    // b_x = xOpt[3];
    // b_y = xOpt[4];

    // std::cout.precision(3);
    // std::cout << stateMachine << "     ";
    // std::cout << "u_x: " << std::fixed << u_x << ", u_y: " << std::fixed << u_y \
    //             << ", t_step: " << std::fixed << t_step \
    //             << ", b_x: " << std::fixed << b_x << ", b_y: " << std::fixed << b_y << std::endl;
}

void WalkingPatternGeneration::com_trajectory_generation()
{

}