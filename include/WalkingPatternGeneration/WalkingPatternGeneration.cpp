#include "WalkingPatternGeneration.h"

void WalkingPatternGeneration::initialize_qp_param()
{
    alpha1 = 1;
    alpha2 = 5;
    alpha3 = 1000;

    int i, j;

    /* NOMINAL GAIT QP */
    /*     ( alpha1, 0.0, 0.0, 0.0, 0.0 )
     *     ( 0.0, alpha2, 0.0, 0.0, 0.0 )
     * H = ( 0.0, 0.0, alpha3, 0.0, 0.0 )
     *     ( 0.0, 0.0, 0.0, alpha2, 0.0 )
     *     ( 0.0, 0.0, 0.0, 0.0, alpha1 ) */
	for( i=0; i<5*5; ++i )
		H[i] = 0.0;
	H[0] = H[24] = alpha1;
	H[6] = H[18] = alpha2;
	H[12] = alpha3;

    /*     ( -2*nominal_L   * alpha1 )
     *     ( -2*nominal_W   * alpha2 )
     * k = ( -2*nominal_tau * alpha3 )
     *     ( -2*nominal_bx  * alpha2 )
     *     ( -2*nominal_by  * alpha1 ) */
    k[0] = -2 * nominal_L * alpha1;
    k[1] = -2 * nominal_W * alpha2;
    k[2] = -2 * nominal_tau * alpha3;
    k[3] = -2 * nominal_bx * alpha2;
    k[4] = -2 * nominal_by * alpha1;

    /*     ( 1.0, 0.0,                                  0.0, 0.0, 0.0 ) 
     *     (-1.0, 0.0,                                  0.0, 0.0, 0.0 )
     *     ( 0.0, 1.0,                                  0.0, 0.0, 0.0 )
     * A = ( 0.0,-1.0,                                  0.0, 0.0, 0.0 )
     *     ( 0.0, 0.0,                                  1.0, 0.0, 0.0 )
     *     ( 0.0, 0.0,                                 -1.0, 0.0, 0.0 )
     *     ( 1.0, 0.0, -(xi[0] - u_x)*exp(-1*omega*prevSim), 1.0, 0.0 )  
     *     ( 0.0, 1.0, -(xi[1] - u_y)*exp(-1*omega*prevSim), 0.0, 1.0 ) */
    for(i=0; i<5*8; i++)
        A[i] = 0.0;
    A[0] = A[11] = A[22] = A[30] = A[33] = A[36] = A[39] = 1.0;
    A[5] = A[16] = A[27] = -1.0;
    A[32] = -(xi[0] - u_x)*exp(-omega*t_moving);   // check needed (ux, prevSim)
    A[37] = -(xi[1] - u_y)*exp(-omega*t_moving);   // check needed (ux, prevSim)
    
    /*	     (     )
     *       (     )
     *       (     )
     * lbA = (     )
     *       (     )
     *       (     )
     *       ( u_x )       
     *       ( u_y ) */
    for(i=0; i<6; i++)
        lbA[i] = -std::numeric_limits<real_t>::infinity();
    lbA[6] = u_x;           // check needed (ux, prevSim)
    lbA[7] = u_y;           // check needed (ux, prevSim)

	/*       (  max_L            ) 
     *       ( -min_L            ) 
     *       (  max_W            ) 
     * ubA = ( -min_W            )
     *       (  exp(omega*max_T) )
     *       ( -exp(omega*min_T) )
     *       ( u_x               )              
     *       ( u_y               ) */
    ubA[0] = max_L;
    ubA[1] = -min_L;
    ubA[2] = max_W;
    ubA[3] = -min_W;
    ubA[4] = exp(omega*max_T);
    ubA[5] = -exp(omega*min_T);
    ubA[6] = u_x;           // check needed (ux, prevSim)
    ubA[7] = u_y;           // check needed (ux, prevSim)

    /* Setting up QProblem object. */
    example = { 5, 8 };
    // example_traj_z = { 10, 10 };
    /* # of Variables and # of Constraints */
    // std::cout << example.getNV() << " " << example.getNC() << std::endl;   

    // options.setToReliable();
    options.printLevel = PL_LOW;
	example.setOptions( options );
	// example_traj_z.setOptions( options );

	/* Solve first(initial) QP. */
	int_t nWSR = 100;
	example.init( H, k, A, lb, ub, lbA, ubA, nWSR);
	// example_traj_x.init( H_traj, k_traj, A_traj, lb, ub, lbA_traj_x, ubA_traj_x, nWSR_x);
	// example_traj_y.init( H_traj, k_traj, A_traj, lb, ub, lbA_traj_y, ubA_traj_y, nWSR_y);
	// example_traj_z.init( H, k, A, lb, ub, lbA, ubA, nWSR);
}

void WalkingPatternGeneration::initialize_nominal_param()
{
    min_L = -0.5;   max_L = 0.5;    // step length
    min_W = -0.1;   max_W = 0.4;    // step width
    min_T = 0.2;    max_T = 0.8;    // step time
}

void WalkingPatternGeneration::specifying_nominal_values(Eigen::Vector3d des_vel, double stateMachine)
{
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

    // int i, j;

    // k[0] = -2 * nominal_L;
    // k[1] = -2 * nominal_W;
    // k[2] = -2 * nominal_tau;
    // k[3] = -2 * nominal_bx;
    // k[4] = -2 * nominal_by;

    // for(i=0; i<5*8; i++)
    //     A[i] = 0.0;
    // A[0] = A[11] = A[22] = A[30] = A[33] = A[36] = A[39] = 1.0;
    // A[5] = A[16] = A[27] = -1.0;
    // // A[32] = -(xi[0] - u_x)*exp(-1*omega*t_moving);      // check needed (ux, prevSim)
    // // A[37] = -(xi[1] - u_y)*exp(-1*omega*t_moving);      // check needed (ux, prevSim)
    // A[32] = -(xi[0] - 0)*exp(-1*omega*t_moving);        // check needed (ux, prevSim)
    // A[37] = -(xi[1] - 0)*exp(-1*omega*t_moving);        // check needed (ux, prevSim)

    // for(i=0; i<6; i++)
    //     lbA[i] = -std::numeric_limits<real_t>::infinity();
    // // lbA[6] = u_x;       // check needed (ux, prevSim)                           
    // // lbA[7] = u_y;       // check needed (ux, prevSim)       // w.r.t global frame
    // lbA[6] = 0;         // check needed (ux, prevSim)                               
    // lbA[7] = 0;         // check needed (ux, prevSim)       // w.r.t local frame

    // ubA[0] = max_L;
    // ubA[1] = -min_L;
    // ubA[2] = max_W;
    // ubA[3] = -min_W;
    // ubA[4] = exp(omega*max_T);
    // ubA[5] = -exp(omega*min_T);
    // // ubA[6] = u_x;      // check needed (ux, prevSim)                           
    // // ubA[7] = u_y;      // check needed (ux, prevSim)       // w.r.t global frame
    // ubA[6] = 0;        // check needed (ux, prevSim)                               
    // ubA[7] = 0;        // check needed (ux, prevSim)       // w.r.t local frame
	
    // int_t nWSR = 100;
    // example.hotstart( H, k, A, lb, ub, lbA, ubA, nWSR);

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
    com_trajectory(2) = z0;
}