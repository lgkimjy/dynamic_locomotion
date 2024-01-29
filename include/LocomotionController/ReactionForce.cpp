#include "ReactionForce.hpp"

/**
 *	@brief compute centroidal dynamics using qp solver, w/ state of robot stance
 * 
 *  @param[in] robot : robot model
 *	@param[in] stateMachine : state of robot stance
 *  @param[in] desired_com_pos : desired com position
 *  @param[in] desired_com_vel : desired com velocity
 *  @param[in] desired_orien_B : desired orientation of base
 *  @param[in] desired_omega_B : desired angular velocity of base
 *  @param[in] p_EE : end-effector position
 * 
 *  @param[out] opt_rho : optimal rho for each reaction cone
 *  @param[out] opt_f : optimal reaction force for each contact
 * 
 *	@todo
 *		1. orientation error using exponential map representation must be re-checked.
 *		2. centroidal rotational inertia matrix must be re-checked.
 *		3. R_C, which is roation matrix is defined w/ identity matrix as assuming Contact frame equals to Inertial frame,
 *		but in future, it could need to be re-defined w.r.t Contact frame.
 */


CoMDynamics::CoMDynamics()
{
    YAML::Node yaml_node;
    std::string path = CMAKE_SOURCE_DIR "/config/config_centoridal.yaml";
	try
	{
        yaml_node = YAML::LoadFile(path.c_str());

        K_f_ = 	yaml_node["ContactWrench"]["k_f"].as<double>();
        K_mu_ = yaml_node["ContactWrench"]["k_mu"].as<double>();
		for(int i=0; i<3; i++) {
			Kp_p_(i, i) = yaml_node["CoMDynamics"]["CoM_translational_gains"]["kp"][i].as<double>();
			Kd_p_(i, i) = yaml_node["CoMDynamics"]["CoM_translational_gains"]["kd"][i].as<double>();
			Kp_omega_(i, i) = yaml_node["CoMDynamics"]["body_angular_acceleration_gains"]["kp"][i].as<double>();
			Kd_omega_(i, i) = yaml_node["CoMDynamics"]["body_angular_acceleration_gains"]["kd"][i].as<double>();
		}
		cost_ = yaml_node["CoMDynamics"]["qp_weight_gain"].as<double>();
		alpha_ = yaml_node["CoMDynamics"]["hess_regul"].as<double>();
    }
    catch(const std::exception& e) 
    {
        std::cerr << RED << "Fail to read Centroidal Dynamics Config YAML File" << RESET << std::endl;
        exit(0);
    }

	ContactWrench_.init(K_f_, K_mu_);
	setGain();
}

void CoMDynamics::setQPSize(int dim_G, int dim_CE, int dim_CI)
{
	// ( D:_dim_G->12 )  ( S:_dim_G->6 )
	// ( D:_dim_CE->12 ) ( S:_dim_CE->6 )
	// ( D:_dim_CI->0 ) ( S:_dim_CI->0 )
	_dim_G = dim_G;
	_dim_CE = dim_CE;
	_dim_CI = dim_CI;

	reaction_force_solver.problem(_dim_G, _dim_CE, _dim_CI);

	opt_rho = Eigen::VectorXd(_dim_G).setZero();

	G = Eigen::MatrixXd(_dim_G, _dim_G).setZero();
    S = Eigen::MatrixXd(_dim_G, _dim_G).setZero();

	g0 = Eigen::VectorXd(_dim_G).setZero();
	ce = Eigen::VectorXd(_dim_CE).setZero();
	ci = Eigen::VectorXd(_dim_CI).setZero();

	f = Eigen::VectorXd(_dim_G/2).setZero();
	b_d = Eigen::VectorXd(6).setZero();
	A = Eigen::MatrixXd(6, _dim_G/2).setZero();	// D: 6x6 or S: 6x3
	Ce = Eigen::MatrixXd(_dim_CE, _dim_G).setZero();
	Ci = Eigen::MatrixXd(_dim_CI, _dim_G).setZero();

	Ubar = Eigen::MatrixXd(_dim_G / 2, _dim_G).setZero(); 	// D: 6x12 or S: 3x6
	ContactWrench_.computeContactWrenchCone();
	for(int i=0; i < _dim_G / 6; i++) {
		Ubar.block(i*3, i*6, 3, 6) = ContactWrench_.W;
	}
	R_C = Eigen::MatrixXd(_dim_G / 2, _dim_G / 2).setZero();    // D: 6x6 or S: 3x3
	for(int i=0; i< _dim_G / 6; i++) {
		R_C.block(i*3, i*3, 3, 3) = Eigen::MatrixXd(3, 3).setIdentity();
	}
}

void CoMDynamics::setGain()
{
	S = cost_ * S.setIdentity();
}

void CoMDynamics::computeReactionForce(CARBML robot, Eigen::Vector3d desired_com_pos, Eigen::Vector3d desired_com_vel, 
	Eigen::Matrix3d desired_orien_B, Eigen::Vector3d desired_omega_B, std::vector<Eigen::Vector3d> p_EE)
{
	// Desired Centroidal Dynamics (PD control law)
    Eigen::Matrix3d errorMatrix = desired_orien_B * robot.R_B.transpose();
    Eigen::AngleAxisd errorAngleAxis(errorMatrix);
	
	pddot_c_d = Kp_p_ * (desired_com_pos - robot.p_CoM) + Kd_p_ * (desired_com_vel - robot.pdot_CoM);		// desired COM acceleration using PD Controller
	omegadot_b_d = Kp_omega_ * (errorAngleAxis.angle() * errorAngleAxis.axis()) + Kd_omega_ * (desired_omega_B - robot.omega_B);	// desired angular acceleration using PD Controller
	g_vec << 0.0, 0.0, robot.getGravityConst();

	// b_d.segment<3>(0) = (robot.getTotalMass() * (g_vec));					// total mass * (gravity) -> just for checking reaction ground force w/o desired COM acceleration
	b_d.segment(0, 3) = (robot.getTotalMass() * (pddot_c_d + g_vec));		// total mass * (CoM acceleration + gravity)
	b_d.segment(3, 3) = (robot.I_G_BCS[0] * omegadot_b_d);					// centroidal rotational inertia * body angular acceleration
	
	for(int i=0; i<p_EE.size(); i++) {
		A.block(0, i*3, 3, 3) = Eigen::Matrix3d::Identity();
		A.block(3, i*3, 3, 3) = Skew(p_EE[i] - robot.p_CoM);
	}

	G =	(A * R_C * Ubar).transpose() * S * (A * R_C * Ubar);
	G += alpha_ * Eigen::MatrixXd::Identity(_dim_G, _dim_G);		// w/o alpha, which is force nomralization factor, cholesky decomposition will be errroed in quadprogpp
	g0 = -1 * (A * R_C * Ubar).transpose() * S * b_d;

	//////* solve qp */
    if(!reaction_force_solver.solve(G, g0, Ce, ce, Ci, ci))
    {
        std::cout << RED << "CoM Dynamics solve error " << RESET << std::endl;
        exit(0);
    }
    else
    {
        std::cout << GREEN << "CoM Dynamics solved" << RESET << std::endl;
        opt_rho = reaction_force_solver.result();
    }
	f = R_C * Ubar * opt_rho;
}

void CoMDynamics::checkComputation(double contactState)
{
	std::cout << "[ CoMDynamics ] opt_rho:" << opt_rho.transpose().format(fmt) << std::endl;
	if(contactState == 1) {
		std::cout << "[ CoMDynamics ] r_foot reaction force: " << f.transpose().format(fmt) << std::endl;
	}
	else if(contactState == 2) {
		std::cout << "[ CoMDynamics ] l_foot reaction force: " << f.transpose().format(fmt) << std::endl;
	}
	else if(contactState == 3) {
		std::cout << "[ CoMDynamics ] l_foot reaction force: " << f.head(3).transpose().format(fmt) << std::endl;
		std::cout << "[ CoMDynamics ] r_foot reaction force: " << f.tail(3).transpose().format(fmt) << std::endl;
	}
}