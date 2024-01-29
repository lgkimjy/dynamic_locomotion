#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "ARBMLlib/ARBML.h"
#include "task.hpp"

#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define CYAN "\033[36m" /* Cyan */
#define MAGENTA "\033[35m" /* Magenta */


class KinWBC
{
private:
public:

    int n_tasks;
	Eigen::MatrixXd 	J_c;            // Contact Jacobian
	Eigen::MatrixXd 	Jdot_c;         // Contact Jacobian derivative
	std::vector<Task> 	tasks;

    Eigen::VectorXd 	delta_q;
    Eigen::VectorXd 	qdot;
    Eigen::VectorXd 	qddot;

    // Eigen::VectorXd     xi_d;
    // Eigen::VectorXd     xidot_d;
    // Eigen::VectorXd     xiddot_d;
    Eigen::VectorXd     qpos_d;
    Eigen::VectorXd     qvel_d;
    Eigen::VectorXd     qacc_d;

    static constexpr int dim = 0;
    static constexpr int active_dim = 0;

    // functions
    KinWBC();
    ~KinWBC();

    void setContactJacobianSize(int size);
    void clearTask();
    void assignTask(int task_DoF, Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd x, Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d);
    void computeKinWBC(CARBML robot);
    void checkComputation();

    void BuildProjectionMatrix(const Eigen::MatrixXd & J, Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> & N);
	void svd_pseudoInverse(Eigen::MatrixXd Matrix, Eigen::MatrixXd& invMatrix);
};