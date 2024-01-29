#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <eigen-quadprog/QuadProg.h>

#include "ARBMLlib/ARBML.h"
#include "Contact/ContactWrenchCone.h"

#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define CYAN "\033[36m" /* Cyan */
#define MAGENTA "\033[35m" /* Magenta */

class CoMDynamics
{
private:
public:

	int _dim_G;
    int _dim_CE;
    int _dim_CI;

    Eigen::Matrix3d Kp_p_, Kd_p_, Kp_omega_, Kd_omega_;
	Eigen::Vector3d pddot_c_d;
	Eigen::Vector3d omegadot_b_d;
	Eigen::Vector3d g_vec;
	Eigen::Matrix3d R_C_left, R_C_right;
	CContactWrenchCone	ContactWrench_;
	double K_f_, K_mu_;

	Eigen::MatrixXd					R_C;
	Eigen::MatrixXd					A;
	Eigen::Matrix<double, 6, 1> 	b_d;
	Eigen::VectorXd					f;
	Eigen::MatrixXd     			Ubar;

    Eigen::QuadProgDense 		reaction_force_solver;

	double						alpha_;		// force normalization factor
	double						cost_;		// cost
	Eigen::MatrixXd 			S;			// Weighting matrix for CoM dynamics
	Eigen::VectorXd 			opt_rho;
	Eigen::MatrixXd 			G, Ce, Ci;
	Eigen::VectorXd 			g0, ce, ci;

    // functions
    CoMDynamics();
    ~CoMDynamics() {};

    void setGain();
	void setQPSize(int dim_G, int dim_CE, int dim_CI);
	void computeReactionForce(CARBML robot, Eigen::Vector3d desired_com_pos, Eigen::Vector3d desired_com_vel, 
					Eigen::Matrix3d desired_orien_B, Eigen::Vector3d desired_omega_B, std::vector<Eigen::Vector3d> p_EE);
	void checkComputation(double contactState);
};