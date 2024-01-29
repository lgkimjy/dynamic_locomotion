#pragma once

#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <eigen-quadprog/QuadProg.h>
#include <yaml-cpp/yaml.h>
#include <pinocchio/spatial/se3.hpp>

#include "ARBMLlib/ARBML.h"
#include "Trajectory/polynomial_end_effector_trajectory.hpp"

#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define CYAN "\033[36m" /* Cyan */
#define MAGENTA "\033[35m" /* Magenta */

class ReactiveStepper
{
private:
public:
    // Robot Param
    double lp_;                 // pelvis length
    double z0_;                 // height of CoM
    double omega_;              // natural frequency of the robot
    double g_;                  // gravity term
    double foot_radius_;        // radius of foot
    double control_period_;     // control period

    // Nominal Step Param ( Step Timing Adaptation )
    double l_min_, l_max_, l_nom_;
    double w_min_, w_max_, w_nom_;
    double t_min_, t_max_, t_nom_;
    double tau_min_, tau_max_, tau_nom_;
    double bx_min_, bx_max_, bx_nom_;
    double by_max_in_, by_max_out_, by_nom_;
    
    // qp variables
    Eigen::QuadProgDense stepper_solver_;
    int nb_var_, nb_eq_, nb_ineq_;
    Eigen::VectorXd opt_;
    Eigen::VectorXd opt_lb_;
    Eigen::VectorXd opt_ub_;
    Eigen::VectorXd cost_weights_;
    Eigen::VectorXd slack_variables_;
    Eigen::Vector3d dcm_nominal_;
    Eigen::MatrixXd Q_;
    Eigen::VectorXd q_;
    Eigen::MatrixXd A_eq_;
    Eigen::VectorXd b_eq_;
    Eigen::MatrixXd A_ineq_;
    Eigen::VectorXd b_ineq_;

    // reactive stepper variables
    bool is_left_leg_in_contact_;
    double step_duration_;
    double time_from_last_step_touchdown_;
    Eigen::Vector3d v_des_;
    Eigen::Vector3d previous_support_foot_position_;
    Eigen::Vector3d current_support_foot_position_;
    Eigen::Vector3d next_support_foot_position_;
    Eigen::Vector3d v_des_local_;
    Eigen::Vector3d current_step_location_local_;
    Eigen::Vector3d dcm_local_;

    Eigen::Vector3d feasible_com_vel_;

    // Foot Trajectory
	PolynomialEndEffectorTrajectory trajectory_;
    double mid_air_foot_height_;
    double cost_x_, cost_y_, cost_z_;
    double hess_regul_;
    Eigen::Vector3d left_foot_position_;
    Eigen::Vector3d right_foot_position_;
    Eigen::Vector3d left_foot_velocity_;
    Eigen::Vector3d right_foot_velocity_;
    Eigen::Vector3d left_foot_acceleration_;
    Eigen::Vector3d right_foot_acceleration_;

    // Constructor & Destructor
    ReactiveStepper();
    ~ReactiveStepper() {};

    // User Interface
    void stand_still(double time, CARBML robot, double contactState,
        Eigen::Ref<const Eigen::Vector3d> left_foot_pos, Eigen::Ref<const Eigen::Vector3d> right_foot_pos);
    void walking(double time, CARBML robot, double contactState,
        Eigen::Ref<const Eigen::Vector3d> left_foot_pos, Eigen::Ref<const Eigen::Vector3d> right_foot_pos,
        Eigen::Ref<const Eigen::Vector3d> com_pos, Eigen::Ref<const Eigen::Vector3d> com_vel);

    // Internal Functions
    void initialize();
    void initializeRobotModel(double stride_length, double com_height, double foot_radius, double control_period);
    void initializeStepTiming();
    void initializeStepper(int nb_var, int nb_eq, int nb_ineq);
    void computeNominalStepValues(Eigen::Vector3d des_vel, bool contactState);
    // void computeReactiveStepper(double moving_time, bool contactState, Eigen::Affine3d local_frame, 
    //     Eigen::Ref<const Eigen::Vector3d> current_support_foot_position, 
    void computeReactiveStepper(double moving_time, bool contactState, pinocchio::SE3 local_frame, 
        Eigen::Ref<const Eigen::Vector3d> current_support_foot_position, 
        Eigen::Ref<const Eigen::Vector3d> com_pos, Eigen::Ref<const Eigen::Vector3d> com_vel);
    // void solve(Eigen::Affine3d local_frame);
    void solve(pinocchio::SE3 local_frame);

    // setter
    void set_desired_CoM_velocity(Eigen::Ref<const Eigen::Vector3d> des_vel) { v_des_ = des_vel; }
    
    // getter
    const double &get_step_duration() { return step_duration_; }
    const Eigen::Vector3d &get_desired_left_foot_postion()      { return left_foot_position_; }
    const Eigen::Vector3d &get_desired_left_foot_velocity()     { return left_foot_velocity_; }
    const Eigen::Vector3d &get_desired_left_foot_acceleration() { return left_foot_acceleration_; }
    const Eigen::Vector3d &get_desired_right_foot_position()     { return right_foot_position_; }
    const Eigen::Vector3d &get_desired_right_foot_velocity()     { return right_foot_velocity_; }
    const Eigen::Vector3d &get_desired_right_foot_acceleration() { return right_foot_acceleration_; }
    const Eigen::Vector3d &get_next_support_foot_position()      { return next_support_foot_position_; }
    const Eigen::Vector3d &get_current_support_foot_position()   { return current_support_foot_position_; }
    const Eigen::Vector3d &get_feasible_com_velocity()           { return feasible_com_vel_; }
};
