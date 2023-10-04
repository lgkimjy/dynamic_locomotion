#pragma once

#include <chrono>
#include <fstream>

#include "ARBMLlib/ARBML.h"
#include "Contact/ContactWrenchCone.h"
#include "Trajectory/Trajectory.h"
#include "WalkingPatternGeneration/WalkingPatternGeneration.h"

using namespace std;

//	Time Variables
constexpr sysReal TIME_START = 3.0;				//	Second
constexpr sysReal TIME_TASK_EXECUTION = 2.0;	//	Second


//	Multiple of Simulation Step(Forward dynamics) > 1
constexpr int CONTROL_RATE = 1;


//	Control Related ...
enum control_list
{
	noControl = 0,
	PointJointPD,
	TrackingJointPD
};


//	Task List
enum task_list
{
	noControlSubtask = 0,
	task_standing
};

// Gait State
typedef enum{
  SINGLE_STANCE     = 0X00,
  RIGHT_CONTACT     = 0X01, // right leg contact (double: 1)
  LEFT_CONTACT      = 0X02, // left leg contact (double: 2)
  DOUBLE_STANCE     = 0X03, // both leg contact
}stateMachineTypeDef;


class CRobotControl
{
private:
	CARBML robot;

public:
	CRobotControl();
	~CRobotControl() {}

	unsigned count_sim;
	unsigned count_ctrl;
	unsigned CtrlFlag;
	unsigned TaskFlag;
	unsigned TaskScheduleFlag;

	//////////	Active joint variables	//////////
	Eigen::Matrix<double, ACTIVE_DOF, 1>			joint_torq;					//	Active joint torque
	Eigen::Matrix<double, ACTIVE_DOF, 1>			qpos_d, qvel_d, qacc_d;		//	Desired position & velocity of active joint

	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	K_qp, K_qv;					//	Joint gain matrices for active joint


	///////////////////////////////////////////////////////////////////////////
	/////	Motion parameters for body frame : NOT NECESSARY !!!!
	///////////////////////////////////////////////////////////////////////////
	Eigen::Vector3d								p_lnk[NO_OF_BODY];			//	Position vector of i-th link frame in {I}
	Eigen::Matrix3d								R_lnk[NO_OF_BODY];			//	Rotation matrix of i-th link frame in {I}

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk[NO_OF_BODY];			//	Linear Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk[NO_OF_BODY];			//	Angular Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_lnkCoM[NO_OF_BODY];		//	CoM Jacobian of i-th link in {I}

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk[NO_OF_BODY];		//	Time derivative of Jp_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk[NO_OF_BODY];		//	Time derivative of Jr_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_lnkCoM[NO_OF_BODY];	//	Time derivative of Jp_lnkCoM


	///////////////////////////////////////////////////////////////////////////
	/////	Motion parameters for end-effectors : Expressed in {I}
	///////////////////////////////////////////////////////////////////////////
	int													no_of_EE;			//	Number of end-effectors
	vector<int>											id_body_EE;			//	End-effector ID
	vector<Eigen::Vector3d>								p0_lnk2EE;			//	Local position offset from link frame to end-effector
	vector<Eigen::Matrix3d>								R0_lnk2EE;			//	Local rotation offset from link frame to end-effector

	vector<Eigen::Vector3d>								p_EE;				//	Position vector of i-th end-effector
	vector<Eigen::Vector3d>								pdot_EE;			//	Linear velocity of i-th end-effector
	vector<Eigen::Vector3d>								omega_EE;			//	Angular velocity of i-th end-effector
	vector<Eigen::Matrix3d>								R_EE;				//	End-effector rotation matrix
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jp_EE;				//	i-th End-effector linear Jacobian
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jr_EE;				//	i-th End-effector angular Jacobian
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jdotp_EE;			//	Time derivative of Jp_EE
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jdotr_EE;			//	Time derivative of Jr_EE


	///////////////////////////////////////////////////////////////////////////
	// Command on Robot
	Eigen::Vector3d		des_lin_vel; // desired linear velocity of the robot
	Eigen::Vector3d		des_ang_vel; // desired angular velocity of the robot
	double sim_time;
	double prev_sim_time;

	stateMachineTypeDef stateMachine;
	stateMachineTypeDef prev_state;

	// Walking Pattern Generation
	WalkingPatternGeneration	WPG;
	double step_time;
	Eigen::Vector3d des_com_pos;
	Eigen::Vector3d des_com_vel;
	Eigen::Vector3d des_com_acc;

	// Trajectory Generation
	QuinticTrajecotryProfile	swing_foot_traj_x;
	QuinticTrajecotryProfile	swing_foot_traj_y;
	vector<Eigen::Vector3d>		prev_p_EE_d;
	vector<Eigen::Vector3d>		p_EE_d;
	vector<Eigen::Vector3d>		pdot_EE_d;

	// CoM Dyanmics
	Eigen::Matrix<double, 6, 6> 	A;
	Eigen::Matrix<double, 6, 1> 	b_d;
	Eigen::Matrix<double, 6, 6>		f;
	Eigen::Matrix<double, 6, 1> 	rho;
	Eigen::Matrix<double, 6, 6> 	G;
	Eigen::Matrix<double, 6, 1> 	g0;
	Eigen::Matrix<double, 12, 12>	Ce;
	Eigen::Matrix<double, 12, 1>	ce;
	Eigen::Matrix<double, 12, 12>	Ci;
	Eigen::Matrix<double, 12, 1>	ci;

	// Contact
	CContactWrenchCone	ContactWrench;


	//////////	Functions	//////////
	void InitializeSystem(const mjModel* model_mj);
	void initEEParameters(const mjModel* model);
	void initCtrlParameters(const mjModel* model_mj);
	void initLocomotionVariables();
	void outputEEInformation();

	void UserControl(mjModel* uModel, mjData* uData);
	void getFeedbackInformation(const mjData* data);
	void computeControlInput();

	void computeCoMMotion();
	void computeEEKinematics(Eigen::Matrix<double, TOTAL_DOF, 1>& xidot);
	void computeLinkKinematics();	//	Compute kinematics and Jacobian, Jdot, etc

	//	Check code validaty
	void compareModelComputation(const mjModel* model, mjData* data, const int& count);
};