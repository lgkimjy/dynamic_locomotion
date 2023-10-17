#include "WalkingPatternGeneration.h"

void WalkingPatternGeneration::initialize_qp_param()
{

}

void WalkingPatternGeneration::initialize_nominal_param()
{
    min_L = -0.5;   max_L = 0.5;    // step length
    min_W = -0.1;   max_W = 0.4;    // step width
    min_T = 0.2;    max_T = 0.8;    // step time
}

void WalkingPatternGeneration::specifying_nominal_values(Eigen::Vector3d des_vel, double stateMachine)
{
    // std::cout << "[specifying_nominal_values]" << std::endl;

    B_l3 = min_T;
    B_u3 = max_T;
    if (des_vel(0) != 0) {
        B_l1 = min_L / std::abs(des_vel(0)); 
        B_u1 = max_L / std::abs(des_vel(0)); 
    }
    if (des_vel(1) != 0) {
        B_l2 = min_W / std::abs(des_vel(1)); 
        B_u2 = max_W / std::abs(des_vel(1));
    }

    std::cout << B_l1 << ", " << B_l2 << ", " << B_l3 << std::endl;

    if( des_vel(0) != 0 && des_vel(1) != 0 ) {
        Bl = std::max(B_l1, std::max(B_l2, B_l3));
        Bu = std::min(B_u1, std::min(B_u2, B_u3));
    }
    else if ( des_vel(0) != 0) {
        Bl = std::max(B_l1, B_l3);
        Bu = std::min(B_u1, B_u3);
    }
    else if ( des_vel(1) != 0) {
        Bl = std::max(B_l1, B_l2);
        Bu = std::min(B_u1, B_u2);
    }
    else {
        Bl = B_l3;
        Bu = B_u3;
    }

    nominal_T = (Bl + Bu) / 2;
    nominal_L = des_vel(0) * nominal_T;
    nominal_W = des_vel(1) * nominal_T;
    // nominal_W = (stateMachine == LEFT_CONTACT) ? des_vel(1) * nominal_T - lp/2 \
    //                                            : des_vel(1) * nominal_T + lp/2;

    nominal_bx = nominal_L / ( std::exp(omega*nominal_T) - 1 );
    nominal_by = pow(-1, (double)stateMachine) * ( lp / (1 + std::exp(omega*nominal_T)) ) - ( nominal_W / (1-std::exp(omega*nominal_T)) );

    nominal_tau = exp(omega * nominal_T);
    t_step = (1 / omega) * log(nominal_tau);

    /* Nominal Variables for the walking */
    // std::cout << "-------------------------------------------------------------------------" << std::endl;
    printf("[specifying_nominal_values] desired x vel: %1.2f, desired y vel: %1.2f, Bl: %1.2f, Bu: %1.2f,\n", des_vel(0), des_vel(1), Bl, Bu);
    printf("[specifying_nominal_values] nominal_T: %1.2f, nominal_L: %1.2f, nominal_W: %1.2f, t_step: %1.2f, nominal_bx: %1.2f, nominal_by: %1.2f \n\n", \
                                                                    nominal_T, nominal_L, nominal_W, t_step, nominal_bx, nominal_by);
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
    com_dot_trajectory = -omega * (com_trajectory - dcm_trajectory);
    com_ddot_trajectory = com_dot_trajectory / control_period;
    com_trajectory = com_dot_trajectory * control_period + com_trajectory;
}