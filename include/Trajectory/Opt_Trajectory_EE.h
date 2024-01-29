#include <Eigen/Dense>
#include <eigen-quadprog/QuadProg.h>

class OptTrajectoryEE
{
public:
    OptTrajectoryEE();
    ~OptTrajectoryEE();

    double mid_air_height;
    double n_var_x;
    double n_var_y;
    double n_var_z;
    double cost_x;
    double cost_y;
    double cost_z;
    double hess_regul;

    Eigen::VectorXd time_vec_x;
    Eigen::VectorXd time_vec_y;
    Eigen::VectorXd time_vec_z;

    Eigen::Vector3d initial_pos;
    Eigen::Vector3d current_pos;
    Eigen::Vector3d current_vel;
    Eigen::Vector3d current_acc;
    Eigen::Vector3d target_pos;

    Eigen::Vector3d prev_solved_pos;

    double initial_time;
    double current_time;
    double final_time;
    double last_end_time;

    // Quadratic Programming Variables
    Eigen::QuadProgDense traj_qp_solver;

    double n_var;
    double n_eq;
    double n_ineq;

    Eigen::VectorXd traj_opt;
    Eigen::VectorXd traj_opt_ub;
    Eigen::VectorXd traj_opt_lb;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd Q_regul;
    Eigen::VectorXd q;

    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;

    bool compute(Eigen::Vector3d initial_pos, Eigen::Vector3d current_pos, Eigen::Vector3d current_vel, Eigen::Vector3d current_acc, 
                    Eigen::Vector3d target_pos, double initial_time, double current_time, double final_time, double last_end_time, Eigen::Vector3d& solved_pos);
    void get_next_state();

private:

};