#include "Opt_Trajectory_EE.h"

OptTrajectoryEE::OptTrajectoryEE()
{
    mid_air_height = 0.1;
    n_var_x = 6;
    n_var_y = 6;
    n_var_z = 9;
    cost_x = 1e1;
    cost_y = 1e1;
    cost_z = 1e1;
    hess_regul = 1e-6;

    time_vec_x.resize(n_var_x); time_vec_x.setZero();
    time_vec_y.resize(n_var_y); time_vec_y.setZero();
    time_vec_z.resize(n_var_z); time_vec_z.setZero();

    initial_pos.setZero();
    current_pos.setZero();
    current_vel.setZero();
    current_acc.setZero();
    target_pos.setZero();

    initial_time = 0.0;
    current_time = 0.0;
    final_time = 0.0;
    last_end_time = 0.0;
    
    n_var = n_var_x + n_var_y + n_var_z;
    n_ineq = 2 * 10;
    n_eq = 5 + 5 + 6;

    traj_opt.resize(n_var); traj_opt.setZero();
    traj_opt_lb.resize(n_var); traj_opt_lb.setZero();
    traj_opt_ub.resize(n_var); traj_opt_ub.setZero();

    Q.resize(n_var, n_var); Q.setZero();
    q.resize(n_var); q.setZero();
    Q_regul = Eigen::MatrixXd::Identity(n_var, n_var) * hess_regul;

    A_eq.resize(n_eq, n_var);
    A_eq.setZero();
    b_eq.resize(n_eq);
    b_eq.setZero();

    A_ineq.resize(n_ineq, n_var);
    A_ineq.setZero();
    b_ineq.resize(n_ineq);
    b_ineq.setZero();
    traj_qp_solver.problem(n_var, n_eq, n_ineq);
}

OptTrajectoryEE::~OptTrajectoryEE() = default;

bool OptTrajectoryEE::compute(Eigen::Vector3d initial_pos, 
        Eigen::Vector3d current_pos, 
        Eigen::Vector3d current_vel, 
        Eigen::Vector3d current_acc, 
        Eigen::Vector3d target_pos, 
        double initial_time, double current_time, double final_time, 
        double last_end_time, 
        Eigen::Vector3d& solved_pos)
{
    double duration = (final_time - initial_time);
    double local_current_time = (current_time - initial_time) / duration;
    double local_end_time = 1.0;
    double mid_time = 0.5;

    return true;
}