#include "reactive_stepper.hpp"

ReactiveStepper::ReactiveStepper()
{
    YAML::Node yaml_node;
    std::string path = CMAKE_SOURCE_DIR "/config/config_stepping.yaml";
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        // Reactive Footstep planning parameters
        lp_ = yaml_node["pelvis_length"].as<double>();
        z0_ = yaml_node["com_height"].as<double>();
        foot_radius_ = yaml_node["foot_radius"].as<double>();
        control_period_ = yaml_node["control_period"].as<double>();
        g_ = 9.81;
        omega_ = sqrt(g_ / z0_);

        l_min_ = yaml_node["l_min"].as<double>();
        l_max_ = yaml_node["l_max"].as<double>();
        w_min_ = yaml_node["w_min"].as<double>();
        w_max_ = yaml_node["w_max"].as<double>();
        t_min_ = yaml_node["t_min"].as<double>();
        t_max_ = yaml_node["t_max"].as<double>();
        t_nom_ = yaml_node["t_nom"].as<double>();

        cost_weights_.resize(9);
        for(int i=0; i<9; i++)
            cost_weights_(i) = yaml_node["cost_weights"][i].as<double>();

        // Swing Foot Trajectory Parameters
        mid_air_foot_height_ = yaml_node["swing_poly_traj"]["step_height"].as<double>();
        cost_x_ = yaml_node["swing_poly_traj"]["cost_x"].as<double>();
        cost_y_ = yaml_node["swing_poly_traj"]["cost_y"].as<double>();
        cost_z_ = yaml_node["swing_poly_traj"]["cost_z"].as<double>();
        hess_regul_ = yaml_node["swing_poly_traj"]["hess_regul"].as<double>();

        std::cout << GREEN << "Stepping Config YAML File Loaded" << RESET << std::endl;
    }
    catch(const std::exception& e) 
    {
        std::cerr << RED << "Fail to read Stepping Config YAML File" << RESET << std::endl;
        exit(0);
    }

    initialize();
    initializeStepTiming();
    initializeStepper(9, 2, 10);

	trajectory_.set_mid_air_height(mid_air_foot_height_);
    trajectory_.set_costs(cost_x_, cost_y_, cost_z_, hess_regul_);
}

void ReactiveStepper::initialize()
{
    // Scheduler
    is_left_leg_in_contact_ = false;

    // Step adjustmnet planner
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();

    // Swing
    left_foot_position_.setZero();
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();

    right_foot_position_.setZero();
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    
    feasible_com_vel_.setZero();
}

void ReactiveStepper::initializeStepTiming()
{
    tau_min_ = exp(omega_ * t_min_);
    tau_max_ = exp(omega_ * t_max_);
    bx_min_ = l_min_ / (tau_min_ - 1);
    bx_max_ = l_max_ / (tau_min_ - 1);
    by_max_in_ = lp_ / (1 + tau_min_) +
                 (w_min_ - w_max_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
    by_max_out_ = lp_ / (1 + tau_min_) +
                  (w_max_ - w_min_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
}

void ReactiveStepper::initializeStepper(int nb_var, int nb_eq, int nb_ineq)
{
    nb_var_ = nb_var;    // 9
    nb_eq_ = nb_eq;      // 2
    nb_ineq_ = nb_ineq;  // 10

    stepper_solver_.problem(nb_var_, nb_eq_, nb_ineq_);

    // psi are slack variables here.
    opt_.resize(nb_var_);
    opt_.setZero();
    opt_lb_.resize(nb_var_);
    opt_lb_.setZero();
    opt_ub_.resize(nb_var_);
    opt_ub_.setZero();
    slack_variables_.resize(4);
    dcm_nominal_.setZero();
    Q_.resize(nb_var_, nb_var_);
    Q_.setZero();
    Q_.diagonal() = cost_weights_;
    q_.resize(nb_var_);
    q_.setZero();

    A_eq_.resize(nb_eq_, nb_var_);
    A_eq_.setZero();
    b_eq_.resize(nb_eq_);
    b_eq_.setZero();

    A_ineq_.resize(nb_ineq_, nb_var_);
    //          ux    uy    tau   bx    by    psi0  psi1  psi2  psi3
    A_ineq_ <<  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 0
                0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 1
               -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 2
                0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 3
                0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 4
                0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 5
                0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,   // 6
                0.0,  0.0,  0.0, -1.0,  0.0,  1.0,  0.0,  0.0,  0.0,   // 7
                0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,   // 8
                0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  1.0;   // 9

    b_ineq_.resize(nb_ineq_);
    b_ineq_.setZero();
}

void ReactiveStepper::computeNominalStepValues(Eigen::Vector3d des_vel, bool contactState)
{
    const double contact_switcher = contactState ? 2.0 : 1.0;
    tau_nom_ = exp(omega_ * t_nom_);
    l_nom_ = des_vel(0) * t_nom_;
    w_nom_ = contactState ? des_vel(1) * t_nom_ - lp_
                          : des_vel(1) * t_nom_ + lp_;
    bx_nom_ = l_nom_ / (tau_nom_ - 1);
    by_nom_ = (pow(-1, contact_switcher) * (lp_ / (1 + tau_nom_))) -
              des_vel(1) * t_nom_ / (1 - tau_nom_);
}

void ReactiveStepper::computeReactiveStepper(double moving_time, bool contactState, pinocchio::SE3 local_frame, 
    Eigen::Ref<const Eigen::Vector3d> current_support_foot_position, 
    Eigen::Ref<const Eigen::Vector3d> com_pos, Eigen::Ref<const Eigen::Vector3d> com_vel)
{
    // world frame to local frame
    time_from_last_step_touchdown_ = moving_time;
    double ground_height = 0.0;

//     // Local frame parallel to the world frame and aligned with the base yaw.
//     world_M_local_.translation() << world_M_base.translation()(0), world_M_base.translation()(1), ground_height;
//     world_M_local_.rotation() = world_M_base.rotation();

    // Compute the DCM in the local frame.
    dcm_local_.head<2>() = com_vel.head<2>() / omega_ + com_pos.head<2>();
    dcm_local_(2) = ground_height;
    // dcm_local_ = local_frame.rotation().transpose()  * dcm_local_;
    dcm_local_ = local_frame.actInv(dcm_local_);

    // Express the desired velocity in the local frame.
    v_des_local_ = v_des_;
    v_des_local_ = local_frame.rotation().transpose() * v_des_local_;
    // v_des_local_ = local_frame.actInv(v_des_local_);

    // Nominal value tracked by the QP.
    computeNominalStepValues(v_des_local_, contactState);

    // Current step location in the local frame.
    const Eigen::Vector3d& tmp = current_support_foot_position;
    // current_step_location_local_ = local_frame.rotation().transpose() * tmp;
    current_step_location_local_ = local_frame.actInv(tmp);

    // DCM nominal
    dcm_nominal_ = (com_vel / omega_ + com_pos - current_support_foot_position) * tau_nom_;
    dcm_nominal_(2) = 0.0;

    // Quadratic cost matrix is constant
    Q_.diagonal() = cost_weights_;

    // Quadratic cost Vector
    q_(0) = -cost_weights_(0) * l_nom_;
    q_(1) = -cost_weights_(1) * w_nom_;
    q_(2) = -cost_weights_(2) * tau_nom_;
    q_(3) = -cost_weights_(3) * bx_nom_;
    q_(4) = -cost_weights_(4) * by_nom_;
    q_.tail<4>().setZero();

    // Inequality constraints
    double w_max_local, w_min_local, by_max, by_min;
    if (contactState)
    {
        w_max_local = -w_min_ - lp_;
        w_min_local = -w_max_ - lp_;
        by_max = -by_max_out_;  // by_max_in_;
        by_min = -by_max_in_;   // by_max_out_;
    }
    else
    {
        w_max_local = w_max_ + lp_;
        w_min_local = w_min_ + lp_;
        by_max = by_max_in_;   //-by_max_out_;
        by_min = by_max_out_;  //-by_max_in_;
    }
    tau_min_ = exp(omega_ * std::max(t_min_, time_from_last_step_touchdown_ - 0.0001));

    b_ineq_ << l_max_,                // 0
            w_max_local,           // 1
            -l_min_,                // 2
            -w_min_local,           // 3
            tau_max_,              // 4
            -tau_min_,              // 5
            bx_max_,               // 6
            -bx_min_,               // 7
            by_max,                // 8
            -by_min;                // 9

    // Equality constraints
    double tmp0 = (dcm_local_(0) - current_step_location_local_[0]) * exp(-1.0 * omega_ * time_from_last_step_touchdown_);
    double tmp1 = (dcm_local_(1) - current_step_location_local_[1]) * exp(-1.0 * omega_ * time_from_last_step_touchdown_);
    A_eq_ << 1.0, 0.0, -1.0 * tmp0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, -1.0 * tmp1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    b_eq_ << 0.0,
             0.0;
}

void ReactiveStepper::solve(pinocchio::SE3 local_frame)
{
    if(!stepper_solver_.solve(Q_, q_, A_eq_, b_eq_, A_ineq_, b_ineq_))
    {
        std::cout << RED << "Reactive Stepper Quadratic Programming Solve Error " << RESET << std::endl;
        exit(0);

        slack_variables_.setZero();

        step_duration_ = t_nom_;
        next_support_foot_position_ << l_nom_, w_nom_, 0.0;
        next_support_foot_position_ += current_step_location_local_;
        next_support_foot_position_ = local_frame.rotation() * next_support_foot_position_;
    }
    else
    {
        opt_ = stepper_solver_.result();
        next_support_foot_position_ = current_step_location_local_ + (Eigen::Vector3d() << opt_(0), opt_(1), 0.0).finished();
        // next_support_foot_position_ = local_frame.rotation() * next_support_foot_position_;
        next_support_foot_position_ = local_frame.act(next_support_foot_position_);
        step_duration_ = log(opt_(2)) / omega_;   // tau
        slack_variables_ = opt_.tail<4>();

        std::cout << "--------------------------------------------- [Reactive Stepper] ---------------------------------------------" << std::endl;
        std::cout << GREEN << "Reactive Stepper Optimal Solution solved" << RESET << std::endl;
        std::cout << "optimal footplacement: " << next_support_foot_position_.transpose() << std::endl;
        std::cout << "optimal step duration: " << step_duration_ << std::endl;
        std::cout << "optimal slack variables: " << slack_variables_.transpose() << std::endl;
    }
}

void ReactiveStepper::walking(double moving_time, CARBML robot, double contactState,
    Eigen::Ref<const Eigen::Vector3d> left_foot_pos, Eigen::Ref<const Eigen::Vector3d> right_foot_pos, 
    Eigen::Ref<const Eigen::Vector3d> com_position, Eigen::Ref<const Eigen::Vector3d> com_velocity)
{
    /////////////////////////////////////////////////////////////////
    // intialize member variable with provided (input) arguments
    // and convert world frame to local frame
    /////////////////////////////////////////////////////////////////
    if(contactState == 2) // left contact
    {
        is_left_leg_in_contact_ = true;
        current_support_foot_position_ = left_foot_pos;
        previous_support_foot_position_ = right_foot_pos;
        // next_support_foot_position_ = next_right_foot_pos;
    }
    else if(contactState == 1) // right contact
    {
        is_left_leg_in_contact_ = false;
        current_support_foot_position_ = right_foot_pos;
        previous_support_foot_position_ = left_foot_pos;
        // next_support_foot_position_ = next_left_foot_pos;
    }

    // Conversion from world frame to local frame
    Eigen::Vector3d support_foot;
    if(is_left_leg_in_contact_) support_foot << left_foot_pos(0), left_foot_pos(1), 0.0;
    else                        support_foot << right_foot_pos(0), right_foot_pos(1), 0.0;
    // support_foot << robot.p_CoM(0), robot.p_CoM(1), 0.0;
    Eigen::Matrix3d rotation;
    double yaw = std::atan2(robot.R_B(0, 1), robot.R_B(0, 0));
    std::cout << "yaw_degrees: " << yaw << std::endl;
    // rotation  << cos(yaw), -sin(yaw), 0,
    //              sin(yaw),  cos(yaw), 0,
    //                     0,         0, 1;
    // std::cout << rotation << std::endl;
    // Eigen::Affine3d local_frame;
    // local_frame.linear() = rotation;
    // local_frame.translation() = support_foot;
    // std::cout << local_frame.matrix() << std::endl;

    pinocchio::SE3 world_M_base(Eigen::AngleAxisd(yaw * M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix(), support_foot);        // world frame to base frame
    // pinocchio::SE3 world_M_base(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(), support_foot);        // world frame to base frame
    // pinocchio::SE3 world_M_base(Eigen::Matrix3d::Identity(), support_foot);        // world frame to base frame
    pinocchio::SE3 world_M_local;       // world frame to local frame
    world_M_local.translation() << world_M_base.translation()(0), world_M_base.translation()(1), 0.0;
    world_M_local.rotation() = world_M_base.rotation();
    // std::cout << "world_M_base: " << world_M_base << std::endl;
    // std::cout << "world_M_local: " << world_M_local << std::endl;

    /////////////////////////////////////////////////////////////////
    // Compute Nominal Step parameter and Optimal Footplacement and Timing
    // -> compute out the next step location and step duration
    // @output: next_support_foot_position_, step_duration_
    /////////////////////////////////////////////////////////////////
    computeReactiveStepper(moving_time, 
                        is_left_leg_in_contact_, 
                        world_M_local, 
                        current_support_foot_position_, 
                        com_position, 
                        com_velocity);
    solve(world_M_local);

    /////////////////////////////////////////////////////////////////
    // Compute Optimal Swing Trajectory
    // -> compute desired swing and support foot trajectory w.r.t global(world) frame
    // @input: previous_support_foot_position_, 
    //         next_support_foot_position_,
    //         current_support_foot_position_
    // @output: left_foot_position_, left_foot_velocity_, left_foot_acceleration_
    //          right_foot_position_, right_foot_velocity_, right_foot_acceleration_
    /////////////////////////////////////////////////////////////////
    double start_time = 0.0;            // step start time
    double end_time = step_duration_;   // step duration
    double current_time = moving_time;  // elpased step time from last step touchdown
    if(is_left_leg_in_contact_) // left foot in contact 
    {
        std::cout << "left foot in contact" << std::endl;
        std::cout << "previous support foot position : " << previous_support_foot_position_.transpose() << std::endl;
        std::cout << "next support foot position : " << next_support_foot_position_.transpose() << std::endl;
		if (current_time <= end_time - control_period_) 
        {
			trajectory_.compute(previous_support_foot_position_, 
                                right_foot_position_, right_foot_velocity_, right_foot_acceleration_, 
                                next_support_foot_position_, 
                                start_time, current_time, end_time);
        }
        trajectory_.get_next_state(current_time + control_period_, 
                                right_foot_position_, 
                                right_foot_velocity_, 
                                right_foot_acceleration_);
		
        // The current support foot does not move
        left_foot_position_ = current_support_foot_position_;
        left_foot_velocity_.setZero();
        left_foot_acceleration_.setZero();
	}
    else
    {
        std::cout << "right foot in contact" << std::endl;
        std::cout << "previous support foot position : " << previous_support_foot_position_.transpose() << std::endl;
        std::cout << "next support foot position : " << next_support_foot_position_.transpose() << std::endl;
		if (current_time <= end_time - control_period_) 
        {
            trajectory_.compute(previous_support_foot_position_, 
                                left_foot_position_, left_foot_velocity_, left_foot_acceleration_,
                                next_support_foot_position_, 
                                start_time, current_time, end_time);
        }
        trajectory_.get_next_state(current_time + control_period_, 
                            left_foot_position_, 
                            left_foot_velocity_, 
                            left_foot_acceleration_);
	
    	// The current support foot does not move
        right_foot_position_ = current_support_foot_position_;
        right_foot_velocity_.setZero();
        right_foot_acceleration_.setZero();
    }
    // com trajectory computation
    feasible_com_vel_ = (next_support_foot_position_ - previous_support_foot_position_) * 0.5;
    feasible_com_vel_(2) = 0.0;
}

void ReactiveStepper::stand_still(double time, CARBML robot, double contactState,
    Eigen::Ref<const Eigen::Vector3d> left_foot_pos, Eigen::Ref<const Eigen::Vector3d> right_foot_pos)
{
    // Extract the useful information.
    step_duration_ = 0.0;

    if(contactState == 1) 
    {
        previous_support_foot_position_ = left_foot_pos;
        current_support_foot_position_ = right_foot_pos;
        next_support_foot_position_ = right_foot_pos;
    }
    else if(contactState == 2) 
    {
        previous_support_foot_position_ = right_foot_pos;
        current_support_foot_position_ = left_foot_pos;
        next_support_foot_position_ = left_foot_pos;
    }

    // Feet do not move.
    left_foot_position_ = left_foot_pos;
    right_foot_position_ = right_foot_pos;

    left_foot_position_(2) = 0.0;
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    right_foot_position_(2) = 0.0;
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();

    // Compute the feasible velocity.
    feasible_com_vel_.setZero();
}