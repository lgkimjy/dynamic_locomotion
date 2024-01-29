#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

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

class DynWBC
{
private:
public:
    
    int _dim_G;
    int _dim_CE;
    int _dim_CI;

    Eigen::QuadProgDense dyn_solver;

	Eigen::VectorXd			opt_dyn;
	Eigen::VectorXd 		delta_qddot;
	Eigen::VectorXd 		delta_rho;

    CContactWrenchCone		ContactWrench;
	Eigen::MatrixXd 		R_C;
	Eigen::Matrix3d 		tmp_R_C_left;
	Eigen::Matrix3d 		tmp_R_C_right;
	Eigen::MatrixXd     	Ubar;

	Eigen::Matrix<double, 12, 12> 	Sb;     // selection matrix 
	Eigen::Matrix<double, 12, 12> 	Sf;     // selection matrix
	Eigen::MatrixXd     			W1;
	Eigen::MatrixXd     			W2;
	Eigen::MatrixXd     			W;
	Eigen::VectorXd     			w0;
	Eigen::MatrixXd     			Ce_dyn;
	Eigen::VectorXd     			ce_dyn;
	Eigen::MatrixXd     			Ci_dyn;
	Eigen::VectorXd     			ci_dyn;


    // functions
    DynWBC();
    ~DynWBC();

	bool parseYAML(std::string path);
    void setQPSize(int dim_G, int dim_CE, int dim_CI);
    void setGain(double a, double b);
    void computeDynWBC(CARBML robot, Eigen::Matrix<double, TOTAL_DOF, 1> qddot_cmd, Eigen::VectorXd rho_qp, Eigen::MatrixXd J_contact, Eigen::MatrixXd Jdot_contact);
    void checkComputation();
};
