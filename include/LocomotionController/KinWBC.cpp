#include "KinWBC.hpp"

/**
 *	@brief compute kinematic level using null space projection technique
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


KinWBC::KinWBC()
{
	n_tasks = 0;

	// xi_d = Eigen::VectorXd(TOTAL_DOF).setZero();
	// xidot_d = Eigen::VectorXd(TOTAL_DOF).setZero();
	// xiddot_d = Eigen::VectorXd(TOTAL_DOF).setZero();
}

KinWBC::~KinWBC()
{

}

void KinWBC::setContactJacobianSize(int size)
{
	J_c = Eigen::MatrixXd(3 * size, TOTAL_DOF).setZero();
	Jdot_c = Eigen::MatrixXd(3 * size, TOTAL_DOF).setZero();
}

void KinWBC::clearTask()
{
    tasks.clear();
    n_tasks = 0;
}

void KinWBC::assignTask(int task_DoF, Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd x, \
								Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d)
{
    Task task;
    task.setTaskSize(task_DoF, TOTAL_DOF);

	task.J = J;
	task.Jdot = Jdot;
    task.x = x;
    task.x_d = x_d;
	task.err = x_d - x;
	task.xdot_d = xdot_d;
	task.xddot_d = xddot_d;
    
	tasks.push_back(task);
    n_tasks++;
}

void KinWBC::computeKinWBC(CARBML robot)
{
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> N_c;
	BuildProjectionMatrix(J_c, N_c);

	Eigen::Matrix<double, TOTAL_DOF, 1> delta_q, qdot, qddot;
	Eigen::MatrixXd JtPre, JtPre_pinv;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> N_nx, N_pre;

	// First Task
    JtPre = tasks[0].J * N_c;
    svd_pseudoInverse(JtPre, JtPre_pinv);
	
	delta_q = JtPre_pinv * (tasks[0].err);
	qdot = JtPre_pinv * tasks[0].xdot_d;
	qddot = JtPre_pinv * tasks[0].xddot_d;

    Eigen::Matrix<double, TOTAL_DOF, 1> prev_delta_q = delta_q;
    Eigen::Matrix<double, TOTAL_DOF, 1> prev_qdot = qdot;
    Eigen::Matrix<double, TOTAL_DOF, 1> prev_qddot = qddot;

	BuildProjectionMatrix(JtPre, N_nx);
	N_pre = N_c * N_nx;

	// std::cout << "Contact Vel: " << (J_c * delta_q).transpose().format(fmt) << std::endl;
	// std::cout << "Jt0: \n" << J_task[0].format(fmt) << std::endl;
	// std::cout << "J_C: \n" << J_c.format(fmt) << std::endl;
	// std::cout << "Nc: \n" << N_c.format(fmt) << std::endl;
	// std::cout << "JtNc: \n" << JtPre.format(fmt) << std::endl;
	// std::cout << "JtNc_pinv: \n" << JtPre_pinv.format(fmt) << std::endl;
	// std::cout << "delta q: \n" << delta_q.transpose().format(fmt) << std::endl;
    
	for(int i = 1; i<tasks.size(); i++) 
	{
		JtPre = tasks[i].J * N_pre;

		svd_pseudoInverse(JtPre, JtPre_pinv);
		// delta_q = prev_delta_q + JtPre_pinv * (task_x_d[i] - task_x[i] - J_task[i] * prev_delta_q);
		delta_q = prev_delta_q + JtPre_pinv * (tasks[i].err - tasks[i].J * prev_delta_q);
		qdot = prev_qdot + JtPre_pinv * (tasks[i].xdot_d - tasks[i].J * prev_qdot);
		qddot = prev_qddot + JtPre_pinv * (tasks[i].xddot_d - tasks[i].Jdot * robot.xidot - tasks[i].J * prev_qddot);

		// std::cout << "Jt" << i << std::endl << J_task[i].format(fmt) << std::endl;s
		// std::cout << "JtN" << i << std::endl << JtPre.format(fmt) << std::endl;
		// std::cout << "J_pre = J" << i << "N" << i-1 << std::endl << J_pre[i].format(fmt) << std::endl;
		// std::cout << "J_pre_pinv" << std::endl << J_pre_pinv.format(fmt) << std::endl;
		// std::cout << "delta_q" << i << std::endl << delta_q[i].transpose().format(fmt) << std::endl;

		BuildProjectionMatrix(JtPre, N_nx);
		N_pre *= N_nx;
		prev_delta_q = delta_q;
        prev_qdot = qdot;
        prev_qddot = qddot;
	}
	qpos_d = robot.q + delta_q.segment(6, ACTIVE_DOF);
	// qpos_d = delta_q.segment(6, ACTIVE_DOF);
	qvel_d = qdot.segment(6, ACTIVE_DOF);
	// qpos_d = qdot.segment(6, ACTIVE_DOF) * 0.001 + qpos_d;
	qacc_d = qddot.segment(6, ACTIVE_DOF);
}

void KinWBC::BuildProjectionMatrix(const Eigen::MatrixXd & J, Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> & N)
{
    Eigen::MatrixXd J_pinv;
    Eigen::MatrixXd I_mtx(TOTAL_DOF, TOTAL_DOF);
    I_mtx.setIdentity();
    svd_pseudoInverse(J, J_pinv);
    N = I_mtx  - J_pinv * J;
}

void KinWBC::svd_pseudoInverse(Eigen::MatrixXd Matrix, Eigen::MatrixXd& invMatrix)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd S = svd.singularValues();

    Eigen::VectorXd Sinv = S;
    for (int i = 0; i < S.size(); i++)
    {
        if (S(i) > 0.0001)
            Sinv(i) = 1.0 / S(i);
        else
            Sinv(i) = 0.0;
    }

    invMatrix = svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose();		
}

void KinWBC::checkComputation()
{
	std::cout << "[ KinWBC ] " << "Contact Jacbi Size: " << J_c.rows() << "x" << J_c.cols() << std::endl;
	std::cout << "[ KinWBC ] " << "Number of Assinged Tasks: " << tasks.size() << std::endl;
	std::cout << "[ KinWBC ] " << "qpos_d: " << qpos_d.transpose().format(fmt) << std::endl;
	std::cout << "[ KinWBC ] " << "qvel_d: " << qvel_d.transpose().format(fmt) << std::endl;
	std::cout << "[ KinWBC ] " << "qacc_d: " << qacc_d.transpose().format(fmt) << std::endl;
}