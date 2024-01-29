#include "DynWBC.hpp"

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


DynWBC::DynWBC()
{
    _dim_G = 0;
    _dim_CE = 0;
    _dim_CI = 0;

	Sf.setZero();
	Sb.setZero();
	Sf.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
	Sb.block(6, 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);

    double K_f = 1.0;
	double K_mu = 1.0;
	ContactWrench.init(K_f, K_mu);

    parseYAML(CMAKE_SOURCE_DIR "/config/config.yaml");
}

DynWBC::~DynWBC()
{
}

void DynWBC::setQPSize(int dim_G, int dim_CE, int dim_CI)
{
    _dim_G = dim_G;
    _dim_CE = dim_CE;
    _dim_CI = dim_CI;

    dyn_solver.problem(_dim_G, _dim_CE, _dim_CI);

    opt_dyn = Eigen::VectorXd(dim_G).setZero();
    delta_qddot = Eigen::VectorXd(TOTAL_DOF).setZero();
    delta_rho = Eigen::VectorXd(dim_G - TOTAL_DOF).setZero();

    w0 = Eigen::VectorXd(_dim_G).setZero();
    ce_dyn = Eigen::VectorXd(_dim_CE).setZero();
    ci_dyn = Eigen::VectorXd(_dim_CI).setZero();

    W = Eigen::MatrixXd(_dim_G, _dim_G).setZero();
    W1 = Eigen::MatrixXd(TOTAL_DOF, TOTAL_DOF).setIdentity();
    W2 = Eigen::MatrixXd(_dim_G - TOTAL_DOF, _dim_G - TOTAL_DOF).setIdentity();
    
    Ce_dyn = Eigen::MatrixXd(_dim_CE, _dim_G).setZero();
    Ci_dyn = Eigen::MatrixXd(_dim_CI, _dim_G).setZero();

	Ubar = Eigen::MatrixXd(_dim_CI/2, _dim_CI).setZero(); // D: 6x12 or S: 3x6
    ContactWrench.computeContactWrenchCone();
    for(int i=0; i < _dim_CI / 6; i++) {
        Ubar.block(i*3, i*6, 3, 6) = ContactWrench.W;
    }
    // std::cout << Ubar << std::endl;
    R_C = Eigen::MatrixXd(_dim_CI/2, _dim_CI/2).setZero();    // D: 6x6 or S: 3x3
    for(int i=0; i< _dim_CI / 6; i++) {
        R_C.block(i*3, i*3, 3, 3) = Eigen::MatrixXd(3, 3).setIdentity();
    }
    // std::cout << R_C << std::endl;
}

void DynWBC::setGain(double a, double b)
{
    W1 = a * W1.setIdentity();
	W2 = b * W2.setIdentity();
}

void DynWBC::computeDynWBC(CARBML robot, Eigen::Matrix<double, TOTAL_DOF, 1> qddot_cmd, Eigen::VectorXd rho_qp, Eigen::MatrixXd J_contact, Eigen::MatrixXd Jdot_contact)
{
    //////* Cost Function */
    W.block(0, 0, TOTAL_DOF, TOTAL_DOF) = W1;
    W.block(TOTAL_DOF, TOTAL_DOF, _dim_CI, _dim_CI) = W2;
	W += 0.0001 * Eigen::MatrixXd::Identity(_dim_G, _dim_G);
	w0.setZero();
    // std::cout << W << std::endl;
    // std::cout << w0.transpose() << std::endl;

	//////* inequality constraints */
	Ci_dyn.block(0, TOTAL_DOF, _dim_CI, _dim_CI) = Eigen::MatrixXd(_dim_CI, _dim_CI).setIdentity();
	ci_dyn = rho_qp;
    // std::cout << Ci_dyn << std::endl;
    // std::cout << ci_dyn.transpose() << std::endl;

	//////* equality constraints */
    Ce_dyn.block(0, 0, TOTAL_DOF, TOTAL_DOF) = Sf * robot.M_mat;
	Ce_dyn.block(0, TOTAL_DOF, TOTAL_DOF, _dim_G - TOTAL_DOF) = - Sf * J_contact.transpose() * R_C * Ubar;
    Ce_dyn.block(TOTAL_DOF, 0, _dim_CE - TOTAL_DOF, TOTAL_DOF) = J_contact;
    ce_dyn.segment(0, TOTAL_DOF) = Sf * (robot.M_mat * qddot_cmd + robot.C_mat * robot.xidot + robot.g_vec - J_contact.transpose() * (R_C * Ubar * rho_qp));
    ce_dyn.segment(TOTAL_DOF, _dim_CE - TOTAL_DOF) = J_contact * qddot_cmd + Jdot_contact * robot.xidot;
    // std::cout << Ce_dyn << std::endl;
    // std::cout << ce_dyn.transpose() << std::endl;

	//////* solve qp */
    if(!dyn_solver.solve(W, w0, Ce_dyn, ce_dyn, Ci_dyn, ci_dyn))
    {
        std::cout << RED << "DynWBC solve error " << RESET << std::endl;
        exit(0);
    }
    else
    {
        // std::cout << "DynWBC solved" << std::endl;
        opt_dyn = dyn_solver.result();
        delta_qddot = opt_dyn.segment(0, TOTAL_DOF);
        delta_rho = opt_dyn.segment(TOTAL_DOF, _dim_G - TOTAL_DOF);
    }
}

void DynWBC::checkComputation()
{
	std::cout << "[ DynWBC ] " << "delta_qddot:" << delta_qddot.transpose().format(fmt) << std::endl;
	std::cout << "[ DynWBC ] " << " delta_rho :" << delta_rho.transpose().format(fmt) << std::endl;
}

bool DynWBC::parseYAML(std::string path)
{
    std::cout << path << std::endl;
    // YAML::Node gain = YAML::LoadFile("/home/jhk/catkin_ws/src/ARBML/src/Config/gain.yaml");
    // if(gain.IsNull())
    // {
    //     std::cout << RED << "gain.yaml is not loaded" << RESET << std::endl;
    //     return false;
    // }
    // else
    // {
    //     std::cout << GREEN << "gain.yaml is loaded" << RESET << std::endl;
    // }

    // double a = gain["a"].as<double>();
    // double b = gain["b"].as<double>();
    // setGain(a, b);
    return true;
}