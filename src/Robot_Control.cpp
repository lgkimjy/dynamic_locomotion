//
//
#include "Robot_Control.h"


CRobotControl::CRobotControl() : count_sim(0), count_ctrl(0), CtrlFlag(0), TaskFlag(0), TaskScheduleFlag(0)
{
	qpos_d.setZero();
	qvel_d.setZero();
	qacc_d.setZero();

	joint_torq.setZero();

	no_of_EE = 0;
	id_body_EE.clear();

	p0_lnk2EE.clear();
	R0_lnk2EE.clear();

	p_EE.clear();
	R_EE.clear();
	pdot_EE.clear();
	omega_EE.clear();

	Jp_EE.clear();
	Jr_EE.clear();
	Jdotp_EE.clear();
	Jdotr_EE.clear();
}



////////////////////////////////////////////////////////////////////////////////
//	Initialize control related parameters !
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::InitializeSystem(const mjModel* model_mj)
{
	robot.initRobot(model_mj);

	/////	Set joint range of motion, end-effector, etc
	initEEParameters(model_mj);

	/////	Set controller parameters
	initCtrlParameters(model_mj);

	/////	Set locomotion parameters
	initLocomotionVariables();

#ifdef PRINT_END_EFFECTOR_INFO
	outputEEInformation();
#endif
}



////////////////////////////////////////////////////////////////////////////////
//	Initialize End-effector Parameters !
//	 * Joint motion range
//	 * Local End-effector pose !
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::initEEParameters(const mjModel* model)
{
	int i;
	Eigen::Vector3d temp_vec;
	Eigen::Vector4d temp_quat;

	//////////	Get body ID for end-effectors (defined in XML file via model->site !)
	no_of_EE = model->nsite;

	for (i = 0; i < no_of_EE; i++) {
		id_body_EE.push_back(model->site_bodyid[i] - 1);

		temp_vec = {(sysReal)model->site_pos[i * 3],
					(sysReal)model->site_pos[i * 3 + 1],
					(sysReal)model->site_pos[i * 3 + 2]};
		p0_lnk2EE.push_back(temp_vec);				//	Set rel. position of end-effector ???

		temp_quat = {(sysReal)model->site_quat[i * 4],
					 (sysReal)model->site_quat[i * 4 + 1],
					 (sysReal)model->site_quat[i * 4 + 2],
					 (sysReal)model->site_quat[i * 4 + 3]};
		R0_lnk2EE.push_back(_Quat2Rot(temp_quat));	//	Set rel. orientation of end-effector
	}

	id_body_EE.shrink_to_fit();
	p0_lnk2EE.shrink_to_fit();
	R0_lnk2EE.shrink_to_fit();


	/////	Initialize transformation matrix about base, end-effector, contact wheel
	p_EE.reserve(no_of_EE);
	R_EE.reserve(no_of_EE);
	pdot_EE.reserve(no_of_EE);
	omega_EE.reserve(no_of_EE);

	Jp_EE.reserve(no_of_EE);		//	Linear Jacobian of end-effectors
	Jr_EE.reserve(no_of_EE);		//	Angular Jacobian of end-effectors
	Jdotp_EE.reserve(no_of_EE);		//	Time derivative of Jp_EE
	Jdotr_EE.reserve(no_of_EE);		//	Time derivative of Jr_EE

	prev_p_EE_d.reserve(no_of_EE);
	next_p_EE_d.reserve(no_of_EE);
	p_EE_d.reserve(no_of_EE);
	pdot_EE_d.reserve(no_of_EE);
	pddot_EE_d.reserve(no_of_EE);

	for (int i = 0; i < no_of_EE; i++) {
		p_EE[i].setZero();
		R_EE[i].setIdentity();
		pdot_EE[i].setZero();
		omega_EE[i].setZero();

		Jp_EE[i].setZero();
		Jr_EE[i].setZero();
		Jdotp_EE[i].setZero();
		Jdotr_EE[i].setZero();

		prev_p_EE_d[i].setZero();
		next_p_EE_d[i].setZero();
		p_EE_d[i].setZero();
		pdot_EE_d[i].setZero();
		pddot_EE_d[i].setZero();
	}
}


void CRobotControl::outputEEInformation()
{
	int i;

	cout.precision(3);
	cout << endl << "No. of End-effector : " << no_of_EE;
	cout << endl << "ID of End-effectors : ";
	for (auto& it : id_body_EE)
		cout << it << " ";
	cout << endl;

	cout << endl << "Local Position of End-effector : p0_lnk2EE()" << endl;
	for (i = 0; i < p0_lnk2EE.size(); i++) {
		cout << "[" << i << "] : ";
		cout << " [ " << p0_lnk2EE[i].transpose() << " ]" << endl;
	}

	cout << endl << "Local Rotation of End-effector : R0_lnk2EE()" << endl;
	for (i = 0; i < R0_lnk2EE.size(); i++) {
		cout << "[" << i << "] : " << endl;
		cout << R0_lnk2EE[i] << endl;
	}

	cout << "================================================================================" << endl;
}



void CRobotControl::initCtrlParameters(const mjModel* model_mj)
{
	K_qp = 1000.0 * K_qp.setIdentity();
	K_qv = 100.0 * K_qv.setIdentity();

	Kp_p = 100.0 * Kp_p.setIdentity();
	Kd_p = 20.0 * Kd_p.setIdentity();

	// Centroidal Dynamics Gain
	double K_f = 1.0;
	double K_mu = 1.0;
	ContactWrench.init(K_f, K_mu);

	Kp_p.setIdentity();
	Kd_p.setIdentity();
	Kp_omega.setIdentity();
	Kd_omega.setIdentity();
}

void CRobotControl::initLocomotionVariables()
{
	des_lin_vel.setZero();
	des_lin_vel = {0.25, 0.0, 0.0};
	// des_lin_vel = {0.0, 0.0, 0.0};
	des_ang_vel.setZero();
	step_time = 0.0;

	stateMachine = RIGHT_CONTACT;
	WPG.initialize_nominal_param();

	p_EE_d[0] << 0.0, WPG.lp/2, 0.0;
	prev_p_EE_d[0] << 0.0, WPG.lp/2, 0.0;
	next_p_EE_d[0] << 0.0, WPG.lp/2, 0.0;
	p_EE_d[1] << 0.0, -WPG.lp/2, 0.0;
	prev_p_EE_d[1] << 0.0, -WPG.lp/2, 0.0;
	next_p_EE_d[1] << 0.0, -WPG.lp/2, 0.0;
	trajectory.set_mid_air_height(0.15);
	trajectory.set_costs(1e1, 1e1, 1e1, 1e-6);
}

////////////////////////////////////////////////////////////////////////////////
//	USER Control Core !!
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::UserControl(mjModel* model, mjData* data)
{
	int i;

	//mj_warning(data, warning_index, warning_info);
	if ((data->warning[mjWARN_INERTIA].lastinfo != 0) || (data->warning[mjWARN_BADQPOS].lastinfo != 0) ||
		(data->warning[mjWARN_BADQVEL].lastinfo != 0) || (data->warning[mjWARN_BADQACC].lastinfo != 0)) {
		_ErrorMsg("Check Inertia, Position, Velocity & Acceleration !!");
	}

	/////	Check the kinematics and dynamics of model
	if (count_sim > 0)	compareModelComputation(model, data, count_ctrl);

	////////////////////	Main Control Routine	////////////////////
	if (count_sim % CONTROL_RATE == 0) {
		//	Time Stamp Routine : Start
		std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();

		/////	01. Input feedback variables(joint pos/vel, base pos/ori/vel)
		getFeedbackInformation(data);

		/////	02. Compute Kinematic Motion Core (w.r.t Base frame) - Called BEFORE All others !!!
		robot.computeMotionCore();

		/////	03. Compute CoM Kinematics : "computeMotionCore()" is followed by "computeCoMKinematics()"
		robot.computeCoMKinematics();

		//computeCoMMotion(robot);

		/////	Compute Dynamics : "computeCoMKinematics()" is followes by "computeDynamics()"
		robot.computeDynamics();

		/////	Compute Link/CoM Kinematics (w.r.t Inertial frame) 
		computeLinkKinematics();

		/////	Compute End-effector Kinematics : "computeMotionCore()" is followed by "computeEEKinematics()"
		computeEEKinematics(robot.xidot);

		/////	Compute Control Input & Output Control Input
		computeControlInput();

		for (i = 0; i < model->nu; i++) {
			data->ctrl[i] = joint_torq(i);
		}
		++count_ctrl;
	}
	++count_sim;
}



///////////////////////////////////////////////////////////////////////////
/////	Read Devices : State Feedback !
///////////////////////////////////////////////////////////////////////////
void CRobotControl::getFeedbackInformation(const mjData* data)
{
#ifdef _FLOATING_BASE
	/////	Position vector of floating-base body w.r.t {I}
	robot.p_B(0) = data->qpos[0];
	robot.p_B(1) = data->qpos[1];
	robot.p_B(2) = data->qpos[2];

	/////	Orientation of floating-base body w.r.t {I}
	robot.quat_B(0) = data->qpos[3];
	robot.quat_B(1) = data->qpos[4];
	robot.quat_B(2) = data->qpos[5];	
	robot.quat_B(3) = data->qpos[6];

	robot.R_B = _Quat2Rot(robot.quat_B);

	/////	Linear velocity of floating-base body w.r.t {I}
	robot.pdot_B(0) = data->qvel[0];
	robot.pdot_B(1) = data->qvel[1];
	robot.pdot_B(2) = data->qvel[2];

	/////	Angular velocity of floating-base body expressed in {B}
	robot.varphi_B(0) = data->qvel[3];
	robot.varphi_B(1) = data->qvel[4];
	robot.varphi_B(2) = data->qvel[5];

	/////	Convert to absolute angular velocity
	robot.omega_B = robot.R_B * robot.varphi_B;

	/////	Set generalized coordinates
	robot.xi_quat.segment(0, DOF3) = robot.p_B;
	robot.xi_quat.segment(DOF3, DOF4) = robot.quat_B;

	robot.xidot.segment(0, DOF3) = robot.pdot_B;
	robot.xidot.segment(DOF3, DOF3) = robot.omega_B;
#endif

	for (int i = 0; i < ACTIVE_DOF; i++) {
		robot.q(i) = data->qpos[DOF_BASEBODY_QUAT + i];
		robot.qdot(i) = data->qvel[DOF_BASEBODY + i];
	}

	/////	Set joint coordinates and joint velocity
	robot.xi_quat.segment(DOF_BASEBODY_QUAT, ACTIVE_DOF) = robot.q;
	robot.xidot.segment(DOF_BASEBODY, ACTIVE_DOF) = robot.qdot;

	/////	Compute joint acceleration by numerical diff.
	robot.xiddot = (robot.xidot - robot.xidot_tmp) / robot.getSamplingTime();
	robot.xidot_tmp = robot.xidot;

	sim_time = data->time;
}

/**
 *	@brief compute centroidal dynamics using qp solver, w/ state of robot stance
 * 
 *	@param[in] stateMachine : state of robot stance
 *	@todo
 *		1. orientation error using exponential map representation must be re-checked.
 *		2. centroidal rotational inertia matrix must be re-checked.
 *		3. R_C, which is roation matrix is defined w/ identity matrix as assuming Contact frame equals to Inertial frame,
 *		but in future, it could need to be re-defined w.r.t Contact frame.
 */
void CRobotControl::computeCentroidalDynamics(stateMachineTypeDef stateMachine)
{
	std::cout << "--------------------------------------------" << FUNC_TAG << "--------------------------------------------" << std::endl;
	S = 10 * S.setIdentity();
	alpha = 0.001;

	A.setZero();
	A.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
	A.block(3, 0, 3, 3) = Skew(p_EE[0] - robot.p_CoM);	// left leg
	A.block(3, 3, 3, 3) = Skew(p_EE[1] - robot.p_CoM);	// right leg

	R_C.setZero();
	R_C_left.setIdentity();
	R_C_right.setIdentity();
	R_C.block(0, 0, 3, 3) = R_C_left;
	R_C.block(3, 3, 3, 3) = R_C_right;

	C_contact_cone.setZero();
	ContactWrench.computeContactWrenchCone();
	C_contact_cone.block(0, 0, 3, 6) = ContactWrench.W;
	C_contact_cone.block(3, 6, 3, 6) = ContactWrench.W;
	
	f_prime = R_C * C_contact_cone;

	// std::cout << FUNC_TAG << "A matrix rank: " << A.fullPivLu().rank() << std::endl;
	// std::cout << FUNC_TAG << "Desired_com_pos: " << desired_com_pos.transpose().format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "robot.p_CoM : " << robot.p_CoM.transpose().format(fmt) << std::endl;

	// Desired Centroidal Dynamics (PD control law)
	Eigen::Matrix3d desiredRotationMatrix;
	Eigen::Vector3d desired_omega_B;
	desiredRotationMatrix.setIdentity();
	desired_omega_B.setZero();
    Eigen::Matrix3d errorMatrix = desiredRotationMatrix * robot.R_B.transpose();
    Eigen::AngleAxisd errorAngleAxis(errorMatrix);
 
	pddot_c_d = Kp_p * (desired_com_pos - robot.p_CoM) + Kd_p * (desired_com_vel - robot.pdot_CoM);		// desired COM acceleration using PD Controller
	omegadot_b_d = Kp_omega * (errorAngleAxis.axis()) + Kd_omega * (desired_omega_B - robot.omega_B);	// desired angular acceleration using PD Controller
	g_vec << 0.0, 0.0, robot.getGravityConst();

	b_d.segment<3>(0) = (robot.getTotalMass() * (g_vec));						// total mass * (CoM acceleration + gravity)
	// b_d.segment<3>(0) = (robot.getTotalMass() * (pddot_c_d + g_vec));		// total mass * (gravity) -> just for checking reaction ground force w/o desired COM acceleration
	b_d.segment<3>(3) = (robot.I_G_BCS[0] * omegadot_b_d);						// centroidal rotational inertia * body angular acceleration

	/*  define objective function  */
	G =	(A * f_prime).transpose() * S * (A * f_prime);
	G += alpha * Eigen::MatrixXd::Identity(12, 12);								// w/o alpha, which is force nomralization factor, cholesky decomposition will be errroed in quadprogpp
	g0 = -1 * (A * f_prime).transpose() * S * b_d;
	
	/*  Inequality constraints.  */
	Ci.block(0, 0, 6, 6) = ContactWrench.E_cone;
	Ci.block(6, 6, 6, 6) = ContactWrench.E_cone;
	ci = Eigen::Matrix<double, 12, 1>::Zero();
	
	/*  Equality constraints.	*/
	Ce = Eigen::Matrix<double, 12, 12>::Zero();
	ce = Eigen::Matrix<double, 12, 1>::Zero();
	
	// std::cout << FUNC_TAG << std::endl;
	// std::cout << "G: " << std::endl << G.format(fmt) << std::endl;
	// std::cout << "g0: " << g0.transpose().format(fmt) << std::endl;
	// std::cout << "CE: : " << std::endl << Ce.format(fmt) << std::endl;
	// std::cout << "ce: " << ce.transpose().format(fmt) << std::endl;
	// std::cout << "CI: " << std::endl << Ci.format(fmt) << std::endl;
	// std::cout << "ci: " << ci.transpose().format(fmt) << std::endl;

	/* solve qp */
	f_qp.solve_QuadProg(G, g0, Ci, ci, opt_rho);			// w/o equality constraints
	// f_qp.solve_QuadProg(G, g0, Ce, ce, Ci, ci, opt_rho);	// w/ equality constraints, error occurs: terminating with uncaught exception of type std::runtime_error: Constraints are linearly dependent, when Ce and ci is set to zero
	f = f_prime * opt_rho;

	// std::cout << FUNC_TAG << "p_EE[0] - robot.p_CoM: " << (p_EE[0] - robot.p_CoM).transpose().format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "p_EE[1] - robot.p_CoM: " << (p_EE[1] - robot.p_CoM).transpose().format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "skew(p_EE[0] - robot.p_CoM): " << std::endl << Skew(p_EE[0] - robot.p_CoM).format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "skew(p_EE[1] - robot.p_CoM): " << std::endl << Skew(p_EE[1] - robot.p_CoM).format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "RC: " << std::endl << R_C.format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "U: " << std::endl << C_contact_cone.format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "opt_rho: " << opt_rho.transpose().format(fmt) << std::endl;
	// std::cout << FUNC_TAG << "l_foot reaction force: " << f.head(3).transpose().format(full_fmt) << std::endl;
	// std::cout << FUNC_TAG << "r_foot reaction force: " << f.tail(3).transpose().format(full_fmt) << std::endl;
}


/**
 * @brief 
 * 
 * @param stateMachine 
 */
void CRobotControl::assignTaskPriority(stateMachineTypeDef stateMachine)
{
	//// 0: supporting leg in fixed position
	//// 1:	maintaining posture5
	//// 2:	maintaining CoM position
	//// 3: swing leg to follow predetermined trajectory
	std::cout << "--------------------------------------------" << FUNC_TAG << "--------------------------------------------" << std::endl;
}


void CRobotControl::computeTaskPriorityKinematics()
{
	std::cout << "--------------------------------------------" << FUNC_TAG << "--------------------------------------------" << std::endl;

	Eigen::Vector3d delta_q;
	std::cout << delta_q.transpose() << std::endl;
}


void CRobotControl::computeControlInput()
{
	///////////////////////////////////////////////////////////////////////////
    /*  step cycle  */
	///////////////////////////////////////////////////////////////////////////
	/////// @todo : 1) gait switcher & scheduler
	/////// @todo : 2) Walking Pattern Generation
	if(sim_time - prev_sim_time > step_time)
	{
        std::cout << BLUE << "--------------------------------------------STEP CYCLE--------------------------------------------" << RESET << std::endl;

		prev_sim_time = sim_time;

        if(prev_state == LEFT_CONTACT)
            stateMachine = RIGHT_CONTACT;
        else
            stateMachine = LEFT_CONTACT;
		
		prev_p_EE_d[0] = next_p_EE_d[0];
		prev_p_EE_d[1] = next_p_EE_d[1];
		WPG.specifying_nominal_values(des_lin_vel, stateMachine);
		step_time = WPG.nominal_T;

		if(stateMachine == LEFT_CONTACT) {
			// LEFT SWING
			next_p_EE_d[1](0) = prev_p_EE_d[0](0) + WPG.nominal_L;
			next_p_EE_d[1](1) = WPG.nominal_W - WPG.lp/2;
			WPG.dcm_offset(0) = prev_p_EE_d[0](0) + WPG.nominal_bx;
			WPG.dcm_offset(1) = prev_p_EE_d[0](1) - WPG.nominal_by;
			WPG.dcm_offset(2) = 0.0;
        	prev_state = LEFT_CONTACT;
		}
        else if(stateMachine == RIGHT_CONTACT) {
			// RIGHT SWING
			next_p_EE_d[0](0) = prev_p_EE_d[1](0) + WPG.nominal_L;
			next_p_EE_d[0](1) = WPG.nominal_W + WPG.lp/2;
			WPG.dcm_offset(0) = prev_p_EE_d[1](0) + WPG.nominal_bx;
			WPG.dcm_offset(1) = prev_p_EE_d[1](1) - WPG.nominal_by;
			WPG.dcm_offset(2) = 0.0;
        	prev_state = RIGHT_CONTACT;
		}
		succeed = true;
	}
    moving_time = sim_time - prev_sim_time;

	if(stateMachine == LEFT_CONTACT) {
		WPG.dcm_trajectory = (WPG.dcm_offset - prev_p_EE_d[0]) * std::exp(WPG.omega * moving_time) + prev_p_EE_d[0];
		if (moving_time <= step_time - WPG.control_period)
		{
			succeed = succeed && trajectory.compute(prev_p_EE_d[1],
							p_EE_d[1], pdot_EE_d[1], pddot_EE_d[1],
							next_p_EE_d[1],
							0.0, moving_time, step_time);
		}
		trajectory.get_next_state(moving_time + WPG.control_period, p_EE_d[1], pdot_EE_d[1], pddot_EE_d[1]);
		// The current support foot does not move
		p_EE_d[0] = prev_p_EE_d[0];
		pdot_EE_d[0].setZero();
		pddot_EE_d[0].setZero();
	}
	else {
		WPG.dcm_trajectory = (WPG.dcm_offset - prev_p_EE_d[1]) * std::exp(WPG.omega * moving_time) + prev_p_EE_d[1];
		if (moving_time <= step_time - WPG.control_period)
		{
			succeed = succeed && trajectory.compute(prev_p_EE_d[0],
							p_EE_d[0], pdot_EE_d[0], pddot_EE_d[0],
							next_p_EE_d[0],
							0.0, moving_time, step_time);
		}
		trajectory.get_next_state(moving_time + WPG.control_period, p_EE_d[0], pdot_EE_d[0], pddot_EE_d[0]);
		// The current support foot does not move
		p_EE_d[1] = prev_p_EE_d[1];
		pdot_EE_d[1].setZero();
		pddot_EE_d[1].setZero();
	}
	WPG.com_trajectory_generation();
	desired_com_pos = WPG.com_trajectory;
	desired_com_vel = WPG.com_dot_trajectory;
	desired_com_acc = WPG.com_ddot_trajectory;

	// LOG stance, swing foot trajectory and CoM trajectory
	SWING.add(sim_time, p_EE_d[0](0), p_EE_d[0](1), p_EE_d[0](2), p_EE_d[1](0), p_EE_d[1](1), p_EE_d[1](2),
						pdot_EE_d[0](0), pdot_EE_d[0](1), pdot_EE_d[0](2), pdot_EE_d[1](0), pdot_EE_d[1](1), pdot_EE_d[1](2),
						pddot_EE_d[0](0), pddot_EE_d[0](1), pddot_EE_d[0](2), pddot_EE_d[1](0), pddot_EE_d[1](1), pddot_EE_d[1](2));


	///////////////////////////////////////////////////////////////////////////
    /*  control cycle   */
    ///////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// 3) Centroidal Dynamics Ground Reaction Force Deployment ( QP solve )
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	computeCentroidalDynamics(LEFT_CONTACT);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// @todo : 4) KinWBC ( Task Priority-based Control )
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	assignTaskPriority(LEFT_CONTACT);
	computeTaskPriorityKinematics();
	n_tasks = 4;

	task_x.reserve(n_tasks);
	task_x.reserve(n_tasks);
	task_x_d.reserve(n_tasks);
	task_xdot_d.reserve(n_tasks);
	task_xddot_d.reserve(n_tasks);
	J_task.reserve(n_tasks);
	Jdot_task.reserve(n_tasks);

	if(stateMachine == LEFT_CONTACT) {
		// TASK 0: Supporting Leg
		J_task[0] = Jp_EE[0];
		Jdot_task[0] = Jdotp_EE[0];
		task_x[0] = p_EE[0];
		task_x_d[0] = p_EE_d[0];
		task_xdot_d[0] = pdot_EE_d[0];
		task_xddot_d[0] = pddot_EE_d[0];
		// TASK 3: Swing Leg
		J_task[1] = Jp_EE[1];
		Jdot_task[1] = Jdotp_EE[1];
		task_x[1] = p_EE[1];
		task_x_d[1] = p_EE_d[1];
		task_xdot_d[1] = pdot_EE_d[1];
		task_xddot_d[1] = pddot_EE_d[1]; 
	}
	else if(stateMachine == RIGHT_CONTACT) {
		// TASK 0: Supporting Leg
		J_task[0] = Jp_EE[1];
		Jdot_task[0] = Jdotp_EE[1];
		task_x[0] = p_EE[1];
		task_x_d[0] = p_EE_d[1];
		task_xdot_d[0] = pdot_EE_d[1];
		task_xddot_d[0] = pddot_EE_d[1]; 
		// TASK 3: Swing Leg
		J_task[1] = Jp_EE[0];
		Jdot_task[1] = Jdotp_EE[0];
		task_x[1] = p_EE[0];
		task_x_d[1] = p_EE_d[0];
		task_xdot_d[1] = pdot_EE_d[0];
		task_xddot_d[1] = pddot_EE_d[0];
	}

	// TASK 1: Maintaining Posture  --> Check needed
	J_task[3] = Jr_lnk[0];
	Jdot_task[3] = Jdotr_lnk[0];
	task_x[3] = robot.R_B.eulerAngles(0, 1, 2);
	task_x_d[3] = {0.0, 0.0, 0.0,};
	task_xdot_d[3] = {0.0, 0.0, 0.0,};
	task_xddot_d[3] = {0.0, 0.0, 0.0,};

	// TASK 2: Maintaining CoM Position
	J_task[2] = robot.J_CoM;
	Jdot_task[2] = robot.Jdot_CoM;
	task_x[2] = robot.p_CoM;
	task_x_d[2] = desired_com_pos;
	task_xdot_d[2] = desired_com_vel;
	task_xddot_d[2] = desired_com_acc;

	// re-initialize variables for recursive task-priority kinematics
	delta_q.reserve(n_tasks);
	qdot.reserve(n_tasks);
	qddot.reserve(n_tasks);
	J_pre.reserve(n_tasks);
	N.reserve(n_tasks);
	N_pre.reserve(n_tasks);

	for(int i=0; i<n_tasks; i++) {
		delta_q[i].setZero();
		qdot[i].setZero();
		qddot[i].setZero();
		J_pre[i].setZero();
		N[i].setZero();
		N_pre[i].setZero();
	}

	// std::cout << "----------------------------------------------------" << std::endl;
	// Eigen::MatrixXd mp_J_psudo_inv;
	// Eigen::MatrixXd svd_J_psudo_inv;
	// mp_J_psudo_inv = J_task[0].completeOrthogonalDecomposition().pseudoInverse();
	// svd_pseudoInverse(J_task[0], svd_J_psudo_inv);
	// std::cout << "Moore Penrose PseudoInverse: \n" <<mp_J_psudo_inv << std::endl;
	// std::cout << "SVD PseudoInverse: \n" << svd_J_psudo_inv << std::endl;
	// std::cout << "Compare: \n" << (mp_J_psudo_inv - svd_J_psudo_inv).format(full_fmt) << std::endl;
	// std::cout << "----------------------------------------------------" << std::endl << std::endl;

	// Task 및 Desired Task를 한번씩 전부 출력해 볼 것.
	// 첫자세에서는 어느정도 비슷한 값을 갖고 있어야 하는데, 그렇지 않다면 문제가 있는 것.
	// 첫 initialize에 문제가 있거나, com 높이에 문제가 있을 수도 있고, 어떤 frame에서 바라본 값들인지에 대한 확인도 필요 할 것
	// 그게 전부 정확하다면 무조건 아래 recursive loop에서 문제가 있는 것, 그러니 한번 값들 task값들 한번 출력해 볼 것.

	delta_q[0].setZero();
	qdot[0].setZero();
	qddot[0] = J_task[0].completeOrthogonalDecomposition().pseudoInverse() * (-J_task[0] * robot.xidot);
	N[0] = Eigen::MatrixXd::Identity(TOTAL_DOF, TOTAL_DOF) - J_task[0].completeOrthogonalDecomposition().pseudoInverse() * J_task[0];

	std::cout << "stateMachine: " << ((stateMachine == LEFT_CONTACT)? "LEFT CONTACT: ":"RIGHT CONTACT: ") << std::endl;
	std::cout << "--------------------------------------TASK 0--------------------------------------" << std::endl;
	std::cout << "Jc" << std::endl << J_task[0].format(fmt) << std::endl;
	std::cout << "Jc_pinv" << std::endl << J_task[0].completeOrthogonalDecomposition().pseudoInverse().format(fmt) << std::endl;
	std::cout << "N0 = I - Jc_pinv * Jc" << std::endl << N[0].format(fmt) << std::endl;

	for(int i = 1; i<n_tasks; i++) 
	{
		////// Priority: TASK 1 ~ N
		std::cout << "--------------------------------------TASK " << i << "--------------------------------------" << std::endl;
		J_pre[i] = J_task[i] * N[i-1];
		Eigen::MatrixXd J_pre_pinv;
		svd_pseudoInverse(J_pre[i], J_pre_pinv);

		delta_q[i] = delta_q[i-1] + J_pre_pinv * (task_x_d[i] - task_x[i] - J_task[i] * delta_q[i-1]);
		qdot[i] = qdot[i-1] + J_pre_pinv * (task_xdot_d[i] - J_task[i] * qdot[i-1]);
		qddot[i] = qddot[i-1] + J_pre_pinv * (task_xddot_d[i] - Jdot_task[i] * robot.xidot - J_task[i] * qddot[i-1]);

		std::cout << "J" << i << std::endl << J_task[i].format(fmt) << std::endl;
		std::cout << "N" << i-1 << std::endl << N[i-1].format(fmt) << std::endl;
		std::cout << "J_pre = J" << i << "N" << i-1 << std::endl << J_pre[i].format(fmt) << std::endl;
		std::cout << "J_pre_pinv" << std::endl << J_pre_pinv.format(fmt) << std::endl;
		std::cout << "delta_q" << i << std::endl << delta_q[i].transpose().format(fmt) << std::endl;

		N_pre[i] = Eigen::MatrixXd::Identity(TOTAL_DOF, TOTAL_DOF) - J_pre_pinv * J_pre[i];
		N[i] = N[i-1] * N_pre[i];
	}

	// qpos_d = robot.q + delta_q.back().segment(6, ACTIVE_DOF);
	// qvel_d = qdot.back().segment(6, ACTIVE_DOF);
	// qacc_d = qddot.back().segment(6, ACTIVE_DOF);
	qpos_d = robot.q + delta_q[3].segment(6, ACTIVE_DOF);
	qvel_d = qdot[3].segment(6, ACTIVE_DOF);
	qacc_d = qddot[3].segment(6, ACTIVE_DOF);

	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "[computeTaskPriorityKinematics] qpos_d: " << qpos_d.transpose().format(fmt) << std::endl;
	std::cout << "[computeTaskPriorityKinematics] qvel_d: " << qvel_d.transpose().format(fmt) << std::endl;
	std::cout << "[computeTaskPriorityKinematics] qacc_d: " << qacc_d.transpose().format(fmt) << std::endl << std::endl;



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// @todo : 5) DynWBC, solve qp to find delta qddot and delta contact force, finding optimal solution
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	torq_ff.setZero();
	// qpos_d.setZero();
	// qvel_d.setZero();
	// qddot_d +=  qddot_delta
	// F_C = f_qp + f_delta
	// torq_ff = M_mat_q * xi_ddot_C + C_mat_q * robot.xidot + g_vec_q;
	// for(int i=0; i < # of contact; i++) {
	//	 joint_torq += J_C * F_C;
	// }
	// std::cout << "---------------------------------------------------------------------" << std::endl;
	// std::cout << "[computeDynamicLevelWBC] torq_ff: " << torq_ff.transpose().format(fmt) << std::endl;
	// std::cout << "[computeDynamicLevelWBC] delta_f: " << delta_f.transpose().format(fmt) << std::endl;
	// std::cout << "[computeDynamicLevelWBC] delta_rho: " << delta_rho.transpose().format(fmt) << std::endl;



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// @todo : 6) Joint Level Controller
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	joint_torq = torq_ff + K_qp * (qpos_d - robot.q) + K_qv * (qvel_d - robot.qdot);
	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "[computeJointLevelController] joint_torq: " << joint_torq.transpose().format(fmt) << std::endl << std::endl;
}



////////////////////////////////////////////////////////////////////////////////
//	Compute pose, Jacobian and its time derivative for ALL link w.r.t {I}
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::computeLinkKinematics()
{
	for (int i = 0; i < NO_OF_BODY; i++) {
		robot.getLinkPose(i, p_lnk[i], R_lnk[i]);

		robot.getBodyJacob(i, p_lnk[i], Jp_lnk[i], Jr_lnk[i]);

		robot.getBodyJacobDeriv(i, Jdotp_lnk[i], Jdotr_lnk[i]);
	}
}



void CRobotControl::computeCoMMotion()
{
	///	Time Stamp Routine : Start
	std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();

	int i;
	int error;
	sysReal m_G = robot.getTotalMass();
	Eigen::Vector3d								p_lnkCoM[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM[NO_OF_BODY];
	Eigen::Vector3d								p_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_G;
	p_G.setZero();
	J_G.setZero();
	Jdot_G.setZero();

	for (int k = 0; k < 20; k++) {
		p_G.setZero();
		J_G.setZero();
		Jdot_G.setZero();

		for (i = 0; i < NO_OF_BODY; i++) {
			error = 0;
			robot.getLinkCoMPose(i, p_lnkCoM[i], R_lnkCoM[i]);
			p_G += robot.body[i].get_mass() * p_lnkCoM[i];

			robot.getBodyJacob(i, p_lnkCoM[i], J_lnkCoM[i]);
			J_G += robot.body[i].get_mass() * J_lnkCoM[i];

			robot.getBodyJacobDeriv(i, Jdot_lnkCoM[i]);
			Jdot_G += robot.body[i].get_mass() * Jdot_lnkCoM[i];
		}

		p_G = p_G / m_G;
		J_G = J_G / m_G;
		Jdot_G = Jdot_G / m_G;

		robot.p_B2CoM = p_G - robot.p_B;
		robot.pdot_CoM = J_G * robot.xidot;
		robot.pdot_B2CoM = robot.pdot_CoM - robot.pdot_B;
	}
}


////////////////////////////////////////////////////////////////////////////////
//	Compute End-effector Kinematics w.r.t {I}
//	 * Pose, CoM Jacobian and its time derivative
//	 * Linear/angular velocity vectors
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::computeEEKinematics(Eigen::Matrix<double, TOTAL_DOF, 1>& xidot)
{
	int i, j, k;

	/////	Position / Rotation matrix of end-effector w.r.t {I}
	for (i = 0; i < id_body_EE.size(); i++) {
		robot.getBodyPose(id_body_EE[i], p0_lnk2EE[i], R0_lnk2EE[i], p_EE[i], R_EE[i]);
	}

	/////	End-effector Jacobian & its time derivative w.r.t {I}  (Geometric Jacobian NOT analytic Jacobian !)
	for (i = 0; i < id_body_EE.size(); i++) {
		robot.getBodyJacob(id_body_EE[i], p_EE[i], Jp_EE[i], Jr_EE[i]);
		robot.getBodyJacobDeriv(id_body_EE[i], Jdotp_EE[i], Jdotr_EE[i]);
	}

	/////	Compute end-effector velocity expressed in {I}
	for (i = 0; i < id_body_EE.size(); i++) {
		pdot_EE[i].setZero();
		omega_EE[i].setZero();

#ifdef _FLOATING_BASE
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < DOF6; k++) {
				pdot_EE[i](j) += Jp_EE[i](j, k) * xidot(k);
				omega_EE[i](j) += Jr_EE[i](j, k) * xidot(k);
			}
		}
#endif
		for (j = 0; j < DOF3; j++) {
			for (unsigned& idx : robot.kinematic_chain[id_body_EE[i]]) {
				k = idx + DOF_BASEBODY - robot.BodyID_ActJntStart();
				pdot_EE[i](j) += Jp_EE[i](j, k) * xidot(k);
				omega_EE[i](j) += Jr_EE[i](j, k) * xidot(k);
			}
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_EE_mj[NO_OF_ENDEFFECTOR], Pre_Jr_EE_mj[NO_OF_ENDEFFECTOR];
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnk_mj[NO_OF_BODY], Pre_Jr_lnk_mj[NO_OF_BODY];
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnkCoM_mj[NO_OF_BODY];

void CRobotControl::compareModelComputation(const mjModel* uModel, mjData* uData, const int& count)
{
	int i, j, k;
	int error = 0;
	int mjBodyID;
	int start_BodyID = robot.BodyID_ActJntStart();

	sysReal dT = robot.getSamplingTime();
	sysReal error_precision = uModel->opt.tolerance;

	mjtNum* jacp = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* jacr = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* dense_M = new mjtNum[TOTAL_DOF * TOTAL_DOF]{ 0 };
	mjtNum* temp_vec3 = new mjtNum[DOF3]{ 0 };

	Eigen::Vector3d								p_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix3d								R_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE[NO_OF_ENDEFFECTOR];

	Eigen::Vector3d								p_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix3d								R_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE_mj[NO_OF_ENDEFFECTOR];

	Eigen::Vector3d								p_lnk[NO_OF_BODY];
	Eigen::Vector3d								p_lnkCoM[NO_OF_BODY];
	Eigen::Matrix3d								R_lnk[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM[NO_OF_BODY];

	Eigen::Vector3d								p_lnk_mj[NO_OF_BODY];
	Eigen::Vector3d								p_lnkCoM_mj[NO_OF_BODY];
	Eigen::Matrix3d								R_lnk_mj[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM_mj[NO_OF_BODY];

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_lnkCoM[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk[NO_OF_BODY];

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk_mj[NO_OF_BODY];		//	Linear Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk_mj[NO_OF_BODY];		//	Angular Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk_mj[NO_OF_BODY];	//	Time derivative of Jp_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk_mj[NO_OF_BODY];	//	Time derivative of Jr_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnkCoM_mj[NO_OF_BODY];

	Eigen::Matrix<double, TOTAL_DOF, 1>			hvec;
	Eigen::Matrix<double, TOTAL_DOF, 1>			hvec_mj;
	Eigen::Matrix<double, TOTAL_DOF, 1>			gvec_mj;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>	Mmat_mj;

	Eigen::Vector3d								p_G;
	Eigen::Vector3d								p_CoM_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_G;

	Eigen::Vector3d								tempVec3_1, tempVec3_2, tempVec3_3, tempVec3_4;
	Eigen::Vector3d								tmpPosVec1, tmpPosVec2;
	Eigen::Matrix3d								TmpRot_Mat1, TmpRot_Mat2;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> X_O;

	for(int i = 0; i<NO_OF_ENDEFFECTOR; i++) {
		p_EE[i].setZero();
		R_EE[i].setZero();
		Jp_EE[i].setZero();
		Jr_EE[i].setZero();
		Jdotp_EE[i].setZero();
		Jdotr_EE[i].setZero();
		p_EE_mj[i].setZero();
		R_EE_mj[i].setZero();
		Jp_EE_mj[i].setZero();
		Jr_EE_mj[i].setZero();
		Jdotp_EE_mj[i].setZero();
		Jdotr_EE_mj[i].setZero();
	}

	for(int i = 0; i<NO_OF_BODY; i++) {
		Jp_lnk[i].setZero();
		Jr_lnk[i].setZero();
		J_lnkCoM[i].setZero();
		Jdotp_lnk[i].setZero();
		Jdotr_lnk[i].setZero();
		Jp_lnk_mj[i].setZero();
		Jr_lnk_mj[i].setZero();
		Jp_lnkCoM_mj[i].setZero();
		Jr_lnkCoM_mj[i].setZero();
		Jdotp_lnk_mj[i].setZero();
		Jdotr_lnk_mj[i].setZero();
		Jdotp_lnkCoM_mj[i].setZero();
	}

	X_O.setIdentity();

#ifdef _FLOATING_BASE
	X_O.block(DOF3, DOF3, DOF3, DOF3) = robot.R_B;
#endif

	//////////////////////////////////////////////////////////////////////
	/////	Get All information from MuJoCo !! --> Convert to {I}
	//////////////////////////////////////////////////////////////////////
	for (i = 0; i < NO_OF_BODY; i++) {
		mjBodyID = i + 1;

		/////	01. Position & rotation of each body/link frame and link CoM w.r.t {I}
		for (j = 0; j < DOF3; j++) {
			p_lnk_mj[i](j) = uData->xpos[mjBodyID * DOF3 + j];						//	Position of Each Link from MuJoCo
			p_lnkCoM_mj[i](j) = uData->xipos[mjBodyID * DOF3 + j];					//	CoM Position of Each Link from MuJoCo

			for (k = 0; k < DOF3; k++) {
				R_lnk_mj[i](j, k) = uData->xmat[mjBodyID * 9 + j * DOF3 + k];		//	Link Rotation from MuJoCo
				R_lnkCoM_mj[i](j, k) = uData->ximat[mjBodyID * 9 + j * DOF3 + k];	//	Link CoM Rotation from MuJoCo
			}
		}

		/////	02. ALL Link Jacobian
		mj_jacBody(uModel, uData, jacp, jacr, mjBodyID);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_lnk_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_lnk_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_lnk_mj[i] = Jp_lnk_mj[i] * Eigen::Transpose(X_O);
		Jr_lnk_mj[i] = Jr_lnk_mj[i] * Eigen::Transpose(X_O);


		/////	03. Time derivative of Link Jacobian via numerical diff.
		Jdotp_lnk_mj[i] = (Jp_lnk_mj[i] - Pre_Jp_lnk_mj[i]) / dT;
		Jdotr_lnk_mj[i] = (Jr_lnk_mj[i] - Pre_Jr_lnk_mj[i]) / dT;
		Pre_Jp_lnk_mj[i] = Jp_lnk_mj[i];
		Pre_Jr_lnk_mj[i] = Jr_lnk_mj[i];


		/////	04. ALL Link CoM Jacobian
		mj_jacBodyCom(uModel, uData, jacp, jacr, mjBodyID);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_lnkCoM_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_lnkCoM_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_lnkCoM_mj[i] = Jp_lnkCoM_mj[i] * Eigen::Transpose(X_O);
		Jr_lnkCoM_mj[i] = Jr_lnkCoM_mj[i] * Eigen::Transpose(X_O);


		/////	05. Time derivative of Link CoM Jacobian via numerical diff.
		Jdotp_lnkCoM_mj[i] = (Jp_lnkCoM_mj[i] - Pre_Jp_lnkCoM_mj[i]) / dT;
		Pre_Jp_lnkCoM_mj[i] = Jp_lnkCoM_mj[i];
	}

	/////	06. CoM Position of the system from MuJoCo !
	p_CoM_mj(0) = uData->subtree_com[start_BodyID * DOF3];
	p_CoM_mj(1) = uData->subtree_com[start_BodyID * DOF3 + 1];
	p_CoM_mj(2) = uData->subtree_com[start_BodyID * DOF3 + 2];


	/////	07. CoM Jacobian of the system from MuJoCo
	mj_jacSubtreeCom(uModel, uData, jacp, start_BodyID);
	for (j = 0; j < DOF3; j++) {
		for (k = 0; k < TOTAL_DOF; k++) {
			J_G_mj(j, k) = jacp[j * TOTAL_DOF + k];
		}
	}
	//	Transform from MuJoCo coordinate to inertial coordinate
	J_G_mj = J_G_mj * Eigen::Transpose(X_O);


	for (i = 0; i < id_body_EE.size(); i++) {
		/////	08. Compute End-effector position & rotation from MuJoCo !!
		for (j = 0; j < DOF3; j++) {
			p_EE_mj[i](j) = uData->site_xpos[i * DOF3 + j];						//	End-effecor Position from MuJoCo
			for (k = 0; k < DOF3; k++) {
				R_EE_mj[i](j, k) = uData->site_xmat[i * 9 + j * DOF3 + k];		//	End-effector Rotation from MuJoCo
			}
		}

		///// 09. End-Effector Jacobian & end-effector velocity from MuJoCo
		temp_vec3[0] = p_EE_mj[i](0);
		temp_vec3[1] = p_EE_mj[i](1);
		temp_vec3[2] = p_EE_mj[i](2);

		//	Linear & angular end-effector Jacobian matrix
		mj_jac(uModel, uData, jacp, jacr, temp_vec3, id_body_EE[i] + 1);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_EE_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_EE_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_EE_mj[i] = Jp_EE_mj[i] * Eigen::Transpose(X_O);
		Jr_EE_mj[i] = Jr_EE_mj[i] * Eigen::Transpose(X_O);

		/////	10. Time derivative of end-effector Jacobian
		Jdotp_EE_mj[i] = (Jp_EE_mj[i] - Pre_Jp_EE_mj[i]) / dT;
		Jdotr_EE_mj[i] = (Jr_EE_mj[i] - Pre_Jr_EE_mj[i]) / dT;

		Pre_Jp_EE_mj[i] = Jp_EE_mj[i];
		Pre_Jr_EE_mj[i] = Jr_EE_mj[i];
	}


	/////	11. Compute Inertia Matrix
	mj_fullM(uModel, dense_M, uData->qM);
	// Mmat_mj = Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>::CstMatrix('a', dense_M);
	int temp_k = 0;
	for (int i=0;i<TOTAL_DOF;i++)
	{
		for(int j=0;j<TOTAL_DOF;j++)
		{
			Mmat_mj(i,j) = dense_M[temp_k++];
		}
	}
	Mmat_mj = X_O * Mmat_mj * Eigen::Transpose(X_O);


	/////	12. Compute Gravity Force
	gvec_mj = mj_getTotalmass(uModel) * robot.getGravityConst() * J_G_mj.row(Z_AXIS);

	/////	13. Compute Coriolis & Centrifugl Forces
	for (i = 0; i < TOTAL_DOF; i++) {
		hvec_mj(i) = uData->qfrc_bias[i];
	}
	hvec_mj = X_O * hvec_mj - gvec_mj;

	////////////////////////////////////////////////////////////
	//	Check Error between MuJoCo & ARBML
	////////////////////////////////////////////////////////////
	if (count_ctrl % int(3.0 / dT) == 0) {
		//////////	01. Check Total Mass
		error = 0;
		sysReal m_G = robot.getTotalMass();
		error = max(error, (abs(m_G - mj_getTotalmass(uModel)) > error_precision ? 1 : 0));
		if (error == 1) {
			cout << "# 01 : System mass error = " << m_G - mj_getTotalmass(uModel) << endl;
		}

		//////////	02 : Kinematics(pose, Jacob and Jdot) of ALL body frames expressed in {I}
		for (i = 0; i < NO_OF_BODY; i++) {
			///	02-1. Pose(position & rotation)
			error = 0;
			robot.getLinkPose(i, p_lnk[i], R_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_lnk_mj[i](j) - p_lnk[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_lnk_mj[i](j, k) - R_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-1. " << i << "-th Link : Position error (p_lnk_mj - p_lnk)" << endl;
				cout << (p_lnk_mj[i] - p_lnk[i])[2] << endl;
				cout << "# 02-1. " << i << "-th Link : Rotation error (R_lnk_mj - R_lnk)" << endl;
				cout << (R_lnk_mj[i] - R_lnk[i]) << endl;
			}


			///	02-2. Link Jacobian
			error = 0;
			robot.getBodyJacob(i, p_lnk[i], Jp_lnk[i], Jr_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_lnk_mj[i](j, k) - Jp_lnk[i](j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jr_lnk_mj[i](j, k) - Jr_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-2. " << i << "-th Link : Linear Jacobian error (Jp_lnk_mj - Jp_lnk)" << endl;
				cout << (Jp_lnk_mj[i] - Jp_lnk[i]) << endl;
				cout << "# 02-2. " << i << "-th Link : Angular Jacobian error (Jr_lnk_mj - Jr_lnk)" << endl;
				cout << (Jr_lnk_mj[i] - Jr_lnk[i]) << endl;
			}

			///	02-3. Time derivative of link Jacobian
			error = 0;
			robot.getBodyJacobDeriv(i, Jdotp_lnk[i], Jdotr_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_lnk_mj[i](j, k) - Jdotp_lnk[i](j, k)) > 0.01 ? 1 : 0));
					error = max(error, (abs(Jdotr_lnk_mj[i](j, k) - Jdotr_lnk[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-3. " << i << "-th Link : Time derivative of linear Jacobian error (Jdotp_lnk_mj - Jdotp_lnk)" << endl;
				cout << (Jdotp_lnk_mj[i] - Jdotp_lnk[i]) << endl;
				cout << "# 02-3. " << i << "-th Link : Time derivative of angular Jacobian error (Jdotr_lnk_mj - Jdotr_lnk)" << endl;
				cout << (Jdotr_lnk_mj[i] - Jdotr_lnk[i]) << endl;
			}
		}

		//////////	03 : Link CoM Kinematics (pose, J_CoM, Jdot_CoM) for ALL bodies expressed in {I}
		p_G.setZero();
		J_G.setZero();
		Jdot_G.setZero();
		for (i = 0; i < NO_OF_BODY; i++) {
			///	03-1. Link CoM pose(position & rotation)
			error = 0;
			robot.getLinkCoMPose(i, p_lnkCoM[i], R_lnkCoM[i]);
			p_G += robot.body[i].get_mass() * p_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_lnkCoM_mj[i](j) - p_lnkCoM[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_lnkCoM_mj[i](j, k) - R_lnkCoM[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-1. " << i << "-th Link : CoM position error (p_lnkCoM_mj - p_lnkCoM)" << endl;
				cout << (p_lnkCoM_mj[i] - p_lnkCoM[i])[2] << endl;
				cout << "# 03-1. " << i << "-th Link : CoM rotation error (R_lnkCoM_mj - R_lnkCoM)" << endl;
				cout << (R_lnkCoM_mj[i] - R_lnkCoM[i]) << endl;
			}

			///	03-2. Link CoM Jacobian
			error = 0;
			robot.getBodyJacob(i, p_lnkCoM[i], J_lnkCoM[i]);
			J_G += robot.body[i].get_mass() * J_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_lnkCoM_mj[i](j, k) - J_lnkCoM[i](j, k)) > error_precision ? 1 : 0));
					// error = max(error, (abs(Jr_lnkCoM_mj[i](j, k) - Jr_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-2. " << i << "-th Link : Linear CoM Jacobian error (Jp_lnkCoM_mj - J_lnkCoM)" << endl;
				cout << (Jp_lnkCoM_mj[i] - J_lnkCoM[i]) << endl;
			}

			///	03-3. Time derivative of link CoM Jacobian
			error = 0;
			robot.getBodyJacobDeriv(i, Jdot_lnkCoM[i]);
			Jdot_G += robot.body[i].get_mass() * Jdot_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_lnkCoM_mj[i](j, k) - Jdot_lnkCoM[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-3. " << i << "-th Link : Time derivative of CoM Jacobian error (Jdot_lnkCoM_mj - Jdot_lnkCoM)" << endl;
				cout << (Jdotp_lnkCoM_mj[i] - Jdot_lnkCoM[i]) << endl;
			}
		}

		//////////	04 : CoM Kinematics of the System expressed in {I}
		{
			///	04-1. CoM pose of the system
			error = 0;
			p_G = p_G / m_G;
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_CoM_mj(j) - robot.p_CoM(j)) > error_precision ? 1 : 0));
				error = max(error, (abs(robot.p_CoM(j) - p_G(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 04-1. CoM position error (p_CoM_mj - p_CoM)" << endl;
				cout << (p_CoM_mj - robot.p_CoM) << endl;
				cout << "# 04-1. CoM position error (p_CoM - p_G)" << endl;
				cout << (robot.p_CoM - p_G) << endl;
			}

			///	04-2. CoM Jacobian of system
			error = 0;
			J_G = J_G / m_G;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(J_G_mj(j, k) - robot.J_CoM(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(J_G_mj(j, k) - J_G(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-2. CoM Jacobian error (J_G_mj - J_CoM)" << endl;
				cout << (J_G_mj - robot.J_CoM) << endl;
				cout << "# 04-2. CoM Jacobian error (J_G_mj - J_G)" << endl;
				cout << (J_G_mj - J_G) << endl;
			}

			///	04-3. Time derivative of CoM Jacobian of system
			error = 0;
			Jdot_G = Jdot_G / m_G;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-3 : Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
				cout << (Jdot_G - robot.Jdot_CoM) << endl;
			}
#ifdef _FLOATING_BASE
			error = 0;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(robot.C_p(j, k) - m_G * robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-3. Time derivative CoM Jacobian error (C_p -  m_G * Jdot_CoM)" << endl;
				cout << (robot.C_p - m_G * robot.Jdot_CoM) << endl;
				cout << "# 04-3. Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
				cout << (Jdot_G - robot.Jdot_CoM) << endl;
			}
#endif
		}

		//////////	05 : End-effector Kinematics expressed in {I}
		for (i = 0; i < id_body_EE.size(); i++) {
			///	05-1. Global end-effector position & rotation
			error = 0;
			robot.getBodyPose(id_body_EE[i], p0_lnk2EE[i], R0_lnk2EE[i], p_EE[i], R_EE[i]);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_EE_mj[i](j) - p_EE[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_EE_mj[i](j, k) - R_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-1. " << i << "-th End-effector : Position error (p_EE_mj - p_EE)" << endl;
				cout << (p_EE_mj[i] - p_EE[i]) << endl;
				cout << "# 05-1. " << i << "-th End-effector : Rotation error (R_EE_mj - R_EE)" << endl;
				cout << (R_EE_mj[i] - R_EE[i]) << endl;
			}

			///	05-2. End-effector Jacobian expressed in {I}
			error = 0;
			robot.getBodyJacob(id_body_EE[i], p_EE[i], Jp_EE[i], Jr_EE[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_EE_mj[i](j, k) - Jp_EE[i](j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jr_EE_mj[i](j, k) - Jr_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-2. " << i << "-th End-effector : Linear Jacobian error (Jp_EE_mj - Jp_EE)" << endl;
				cout << (Jp_EE_mj[i] - Jp_EE[i]) << endl;
				cout << "# 05-2. " << i << "-th End-effector : Angular Jacobian error (Jr_EE_mj - Jr_EE)" << endl;
				cout << (Jr_EE_mj[i] - Jr_EE[i]) << endl;
			}

			///	05-3. Check Time derivative of End-effector Jacobian : ��� �� �ٸ� �������, �� �� ��Ȯ�ϰ�~~~
			error = 0;
			robot.getBodyJacobDeriv(id_body_EE[i], Jdotp_EE[i], Jdotr_EE[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_EE_mj[i](j, k) - Jdotp_EE[i](j, k)) > 0.01 ? 1 : 0));
					error = max(error, (abs(Jdotr_EE_mj[i](j, k) - Jdotr_EE[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-3. " << i << "-th End-effector : Time derivative of linear Jacobian error (Jdotp_EE - Jdotp_EE_mj)" << endl;
				cout << (Jdotp_EE[i] - Jdotp_EE_mj[i]) << endl;
				cout << "# 05-3. " << i << "-th End-effector : Time derivative of angular Jacobian error (Jdotr_EE - Jdotr_EE_mj)" << endl;
				cout << (Jdotr_EE[i] - Jdotr_EE_mj[i]) << endl;
			}
		}

		//////////	06 : Joint Inertia Matrix (M_mat)
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Mmat_mj(j, k) - robot.M_mat(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 06 : Inertia matrix error (Mmat_mj - M_mat)" << endl;
				cout << (Mmat_mj - robot.M_mat) << endl;
			}
		}

		//////////	07. Gravity Forces Vector : g_vec
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				error = max(error, (abs(gvec_mj(j) - robot.g_vec(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 07 : Gravity force error (gvec_mj - gvec)" << endl;
				cout << (gvec_mj - robot.g_vec) << endl;
			}
		}

		//////////	08. Check Coriolis & Centrifugal forces : C_mat
		{
			error = 0;
			hvec = robot.C_mat * robot.xidot;
			for (j = 0; j < TOTAL_DOF; j++) {
				error = max(error, (abs(hvec_mj(j) - hvec(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 08 : Coriolis & Centrifugal force error (hvec_mj - C_mat * xidot)" << endl;
				cout << (hvec_mj - hvec) << endl;
			}
		}

#ifdef INERTIADOT
		//////////	09. Time derivative of Inertia matrix, Mdot = C + C^T
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(robot.Mdot_mat(j, k) - robot.C_mat(j, k) - robot.C_mat(k, j)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 09 : Mdot - C_mat - C_mat^T error" << endl;
				cout << (robot.Mdot_mat - robot.C_mat - Eigen::Transpose(robot.C_mat)) << endl;
			}
		}

#ifdef _FLOATING_BASE
		Eigen::Matrix<double, DOF3, TOTAL_DOF> Mdot_p, Mdot_o;

		Mdot_p = robot.Mdot_mat.block(0, 0, DOF3, TOTAL_DOF);
		Mdot_o = robot.Mdot_mat.block(DOF3, 0, DOF3, TOTAL_DOF);

		//////////	10 : Mdot_p = C_p = m_G * Jdot_CoM
		{
			error = 0;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Mdot_p(j, k) - robot.C_p(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(m_G * Jdot_G(j, k) - robot.C_p(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 10-1. Mdot_p - C_p" << endl;
				cout << (Mdot_p - robot.C_p) << endl;
				cout << "# 10-1. m_G * Jdot_G - C_p" << endl;
				cout << (m_G* Jdot_G - robot.C_p) << endl;
			}
		}

		//////////	11 : Mdot_o * xidot = (pdot_bG x M_p + C_o) * xidot !!!
		{
			error = 0;
			Eigen::Matrix3d Sk_pdot_BG = Skew(robot.pdot_B2CoM);
			tempVec3_1 = Mdot_o * robot.xidot;
			tempVec3_2 = (Sk_pdot_BG * robot.M_p + robot.C_o) * robot.xidot;
			for (j = 0; j < DOF3; j++) {
				error = max(error, (tempVec3_1(j) - tempVec3_2(j)) > error_precision ? 1 : 0);
			}
			if (error == 1) {
				cout << "# 11-1 : Mdot_o * xidot - (pdot_B2CoM x M_p + C_o) * xidot" << endl;
				cout << (tempVec3_1 - tempVec3_2) << endl;
			}

			error = 0;
			tempVec3_2 = Sk_pdot_BG * robot.M_p * robot.xidot;
			tempVec3_2(0) += hvec_mj(3);
			tempVec3_2(1) += hvec_mj(4);
			tempVec3_2(2) += hvec_mj(5);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (tempVec3_1(j) - tempVec3_2(j)) > error_precision ? 1 : 0);
			}
			if (error == 1) {
				cout << "# 11-2 : Mdot_o * xidot - (Sk_pdot_BG * M_p * xidot + hvec_o_mj)" << endl;
				cout << (tempVec3_1 - tempVec3_2) << endl;
			}
		}
#endif
#endif
		cout << endl; cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl << endl;
	}

	/////	memory free !!!
	delete[] jacp;
	delete[] jacr;
	delete[] dense_M;
	delete[] temp_vec3;
}
