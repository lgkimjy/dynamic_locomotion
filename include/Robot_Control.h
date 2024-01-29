#pragma once

#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ARBMLlib/ARBML.h"
#include "Contact/ContactWrenchCone.h"
#include "Trajectory/Trajectory.h"
#include "Trajectory/Opt_Trajectory_EE.h"
#include "Trajectory/polynomial_end_effector_trajectory.hpp"
#include "WalkingPatternGeneration/WalkingPatternGeneration.h"
// #include "WalkingPatternGeneration/dcm_vrp_planner.hpp"

#include "QuadProgpp/QuadProg++.h"
#include "WalkingPatternGeneration/reactive_stepper.hpp"
#include "LocomotionController/ReactionForce.hpp"
#include "LocomotionController/KinWBC.hpp"
#include "LocomotionController/DynWBC.hpp"

#include "Logger/log.hpp"

using namespace std;

//	Multiple of Simulation Step(Forward dynamics) > 1
constexpr int CONTROL_RATE = 1;

// Gait State
typedef enum{
  SINGLE_STANCE     = 0X00, // single stance
  RIGHT_CONTACT     = 0X01, // right leg contact (double: 1)
  LEFT_CONTACT      = 0X02, // left leg contact (double: 2)
  DOUBLE_STANCE     = 0X03, // both leg contact
}stateMachineTypeDef;

class CRobotControl
{
private:
public:

	CARBML robot;
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
	Eigen::Matrix<double, TOTAL_DOF, 1>				q_d, qdot_d, qddot_d;		//	Desired position & velocity of total joint

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
	double sim_time;
	double prev_sim_time;

	stateMachineTypeDef stateMachine;
	stateMachineTypeDef prev_state;
	std::string contactState;
	std::string tmp_contactState;

	////////////////////////////////
	// Walking Pattern Generation
	////////////////////////////////
	WalkingPatternGeneration	WPG;
	double step_time;
	double moving_time;

	////////////////////////////////
	// Swing Foot Trajectory Generation
	////////////////////////////////
	ReactiveStepper reactive_stepper;
	Eigen::Vector3d 			p_CoM_d;
	Eigen::Vector3d 			pdot_CoM_d;
	Eigen::Vector3d 			pddot_CoM_d;
	vector<Eigen::Vector3d>		prev_p_EE_d;
	vector<Eigen::Vector3d>		next_p_EE_d;
	vector<Eigen::Vector3d>		p_EE_d;
	vector<Eigen::Vector3d>		pdot_EE_d;
	vector<Eigen::Vector3d>		pddot_EE_d;

	////////////////////////////////
	// Gain
	////////////////////////////////
	Eigen::MatrixXd Kp_qcmd, Kd_qcmd;
	Eigen::MatrixXd Kp_q, Kd_q;

	////////////////////////////////
	// Centroidal Dyanmics
	// Task-Priority-based Kinematics Control
	// Dynamics Level WBC using Quadratic Programming to find delta values
	////////////////////////////////
	CoMDynamics 	CCoMDynamics;
	KinWBC 			kinWBC;
	DynWBC 			dynWBC;

	Eigen::Matrix<double, ACTIVE_DOF, TOTAL_DOF> M_mat_q;
	Eigen::Matrix<double, ACTIVE_DOF, TOTAL_DOF> C_mat_q;
	Eigen::Matrix<double, ACTIVE_DOF, 1>		 g_vec_q;

	Eigen::Matrix<double, TOTAL_DOF, 1> 	qddot_cmd;
	Eigen::Matrix<double, ACTIVE_DOF, 1>	torq_ff;
	Eigen::VectorXd							qddot;
	Eigen::VectorXd							rho;
	Eigen::VectorXd							f;

	////////////////////////////////
	// Logger
	////////////////////////////////
	Logger CLogger;

	//////////	Functions	//////////
	void InitializeSystem(const mjModel* model_mj);
	void initEEParameters(const mjModel* model);
	void initCtrlParameters(const mjModel* model_mj);
	void outputEEInformation();

	void UserControl(mjModel* uModel, mjData* uData);
	void getFeedbackInformation(const mjData* data);
	void computeControlInput();

	void computeCoMMotion();
	void computeEEKinematics(Eigen::Matrix<double, TOTAL_DOF, 1>& xidot);
	void computeLinkKinematics();	//	Compute kinematics and Jacobian, Jdot, etc

	//	Check code validaty
	void compareModelComputation(const mjModel* model, mjData* data, const int& count);

	/////////// Extra Functions ///////////
	void contactEstimator();
	void computeCentroidalDynamics(stateMachineTypeDef stateMachine);
	void assignTaskPriority(int n_task, stateMachineTypeDef stateMachine);
	void computeTaskPriorityKinematics();
	void computeDynamicLevelWBC();

	Eigen::Quaterniond QuatMultiply(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2)
	{
			Eigen::Quaterniond ret_q;
			
			// IJRR
            ret_q.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z(),
            ret_q.x() = q1.w()*q2.x() + q1.x()*q2.w() + q1.y()*q2.z() - q1.z()*q2.y(),
            ret_q.y() = q1.w()*q2.y() - q1.x()*q2.z() + q1.y()*q2.w() + q1.z()*q2.x(),
            ret_q.z() = q1.w()*q2.z() + q1.x()*q2.y() - q1.y()*q2.x() + q1.z()*q2.w();
			
            // ret_q.w() = q1.w()*q2.w() + q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z(),
            // ret_q.x() = -q1.w()*q2.x() + q1.x()*q2.w() - q1.y()*q2.z() + q1.z()*q2.y(),
            // ret_q.y() = -q1.w()*q2.y() + q1.x()*q2.z() + q1.y()*q2.w()  q1.z()*q2.x(),
            // ret_q.z() = -q1.w()*q2.z() - q1.x()*q2.y() + q1.y()*q2.x() + q1.z()*q2.w();

			// ret_q.w() = q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();
			// ret_q.x() = q1.w() * q2.x() - q1.x() * q2.w() - q1.y() * q2.z() + q1.z() * q2.y();
			// ret_q.y() = q1.w() * q2.y() + q1.x() * q2.z() - q1.y() * q2.w() - q1.z() * q2.x();
			// ret_q.z() = q1.w() * q2.z() - q1.x() * q2.y() + q1.y() * q2.x() - q1.z() * q2.w();

			if(ret_q.w() < 0){
				ret_q.w() *= -1.;
				ret_q.x() *= -1.;
				ret_q.y() *= -1.;
				ret_q.z() *= -1.;
			}

			return ret_q;
	}

    void convert(Eigen::Quaterniond const & from, Eigen::Vector3d & to)
	{
        double w = from.w();
        Eigen::Vector3d img_vec;
        img_vec[0] = from.x();
        img_vec[1] = from.y();
        img_vec[2] = from.z();
        // double theta = 2.0*acos(w);
        double theta = 2.0*asin(sqrt(img_vec[0] * img_vec[0] +
                                     img_vec[1] * img_vec[1] +
                                     img_vec[2] * img_vec[2]) );
        
        if(fabs(theta)<0.0000001) {
            to = Eigen::Vector3d::Zero();
            return ;
        }
        to = img_vec/sin(theta/2.0);
        // printf("quaternion (theta, w, x, y, z): %f, %f, %f, %f, %f\n", theta, w, to[0], to[1], to[2]);
        
        // if(theta > M_PI){
        //     theta -= 2*M_PI;
        // }
        // if(theta < -M_PI){
        //     theta += 2*M_PI;
        // }
        to = to * theta;
    }

	std::string vectorToString(const Eigen::VectorXd &vec) 
	{
		std::stringstream ss;
		for (int i = 0; i < vec.size(); ++i) {
			if (i > 0) ss << " ";
			ss << vec[i];
		}
			return ss.str();
	}

	bool yamlReader()
	{
		YAML::Node yaml_node;
		std::string path = CMAKE_SOURCE_DIR "/config/config.yaml";
		try
		{
			yaml_node = YAML::LoadFile(path.c_str());

			Kp_qcmd.resize(yaml_node["qcmd_gain"]["kp"].size(), yaml_node["qcmd_gain"]["kp"].size());
			for(int i=0; i<yaml_node["qcmd_gain"]["kp"].size(); i++)
				Kp_qcmd(i, i) = yaml_node["qcmd_gain"]["kp"][i].as<double>();
			
			Kd_qcmd.resize(yaml_node["qcmd_gain"]["kd"].size(), yaml_node["qcmd_gain"]["kd"].size());
			for(int i=0; i<yaml_node["qcmd_gain"]["kd"].size(); i++)
				Kd_qcmd(i, i) = yaml_node["qcmd_gain"]["kd"][i].as<double>();

			Kp_q.resize(yaml_node["joint_level_gain"]["kp"].size(), yaml_node["joint_level_gain"]["kp"].size());
			for(int i=0; i<yaml_node["joint_level_gain"]["kp"].size(); i++)
				Kp_q(i, i) = yaml_node["joint_level_gain"]["kp"][i].as<double>();

			Kd_q.resize(yaml_node["joint_level_gain"]["kd"].size(), yaml_node["joint_level_gain"]["kd"].size());
			for(int i=0; i<yaml_node["joint_level_gain"]["kd"].size(); i++)
				Kd_q(i, i) = yaml_node["joint_level_gain"]["kd"][i].as<double>();
		}
		catch(const std::exception& e)
		{
			std::cerr << "fail to read particle yaml file" <<std::endl;
			exit(0);
		}

		return true;
	}
};