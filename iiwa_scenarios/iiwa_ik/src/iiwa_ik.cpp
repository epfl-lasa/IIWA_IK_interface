/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "iiwa_ik.h"

iiwa_ik::iiwa_ik()
	: RobotInterface()
{
}
iiwa_ik::~iiwa_ik()
{
}

void iiwa_ik::chatterCallback_Desired_end(const geometry_msgs::Pose & msg)
{
	Desired_EndPos(0)=msg.position.x;
	Desired_EndPos(1)=msg.position.y;
	Desired_EndPos(2)=msg.position.z;
	rotation_temp.w() = msg.orientation.w;
	rotation_temp.x() = msg.orientation.x;
	rotation_temp.y() = msg.orientation.y;
	rotation_temp.z() = msg.orientation.z;
	rot_mat_temp = rotation_temp.toRotationMatrix();
	Desired_EndDirY(0)=rot_mat_temp(0,1); Desired_EndDirZ(0)=rot_mat_temp(0,2);
	Desired_EndDirY(1)=rot_mat_temp(1,1);	Desired_EndDirZ(1)=rot_mat_temp(1,2);
	Desired_EndDirY(2)=rot_mat_temp(2,1);	Desired_EndDirZ(2)=rot_mat_temp(2,2);
}

void iiwa_ik::chatterCallback_position(const sensor_msgs::JointState &msg)
{
	JointPos_handle(0) = msg.position[0];
	JointPos_handle(1) = msg.position[1];
	JointPos_handle(2) = msg.position[2];
	JointPos_handle(3) = msg.position[3];
	JointPos_handle(4) = msg.position[4];
	JointPos_handle(5) = msg.position[5];
	JointPos_handle(6) = msg.position[6];
	JointPos = JointPos_handle;
	Position_of_the_robot_recieved = true;
}

void iiwa_ik::Send_Postion_To_Robot(VectorXd Position)
{

	kuka_fri_bridge::JointStateImpedance msg;
	msg.position.resize(KUKA_DOF);
	msg.stiffness.resize(KUKA_DOF);
	for (int i = 0; i < KUKA_DOF; i = i + 1)
	{
		msg.position[i] = Position(i);
		msg.stiffness[i] = 2000;
	}
	pub_command_robot_real.publish(msg);
}

void iiwa_ik::reset_the_bool()
{

	Position_of_the_robot_recieved = false;
}
bool iiwa_ik::everythingisreceived()
{

	return Position_of_the_robot_recieved;
}
void iiwa_ik::Parameter_initialization()
{
	Desired_JointVel.resize(KUKA_DOF);
	Desired_JointVel.setZero();

	cJob.resize(KUKA_DOF);

	EndPos.setZero();
	EndDirY.setZero();
	EndDirZ.setZero();
	Desired_EndDirY.setZero();
	Desired_EndDirZ.setZero();
	Desired_EndPos.setZero();

	cJob(0) = -60.0 * PI / 180;
	cJob(1) = -30 * PI / 180;
	cJob(2) = -0 * PI / 180;
	cJob(3) = -60 * PI / 180;
	cJob(4) = 0;
	cJob(5) = 90.0 * PI / 180;
	cJob(6) = 0;

	reset_the_bool();
}
void iiwa_ik::initKinematics()
{

	mSKinematicChain = new sKinematics(KUKA_DOF, dt);

	mSKinematicChain->setDH(0, 0.0, 0.34, M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
	mSKinematicChain->setDH(1, 0.0, 0.00, -M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(98.0));
	mSKinematicChain->setDH(2, 0.0, 0.40, -M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
	mSKinematicChain->setDH(3, 0.0, 0.00, M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(120.0));
	mSKinematicChain->setDH(4, 0.0, 0.39, M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
	mSKinematicChain->setDH(5, 0.0, 0.00, -M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(180.0));
	mSKinematicChain->setDH(6, 0.0, 0.126, 0.0, 0.0, 1, DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0));
	//                                 0.126 is the distance between the fifth joint and the real of the robot
	T0.setZero();

	T0(0, 0) = 1;
	T0(1, 1) = 1;
	T0(2, 2) = 1;
	T0(3, 3) = 1;
	T0(0, 3) = 0;
	T0(1, 3) = 0;
	T0(2, 3) = 0;
	mSKinematicChain->setT0(T0);
	mSKinematicChain->readyForKinematics();

	MatrixXd W(KUKA_DOF, KUKA_DOF);
	W.setZero();

	// variable for ik
	Jacobian3.resize(3, KUKA_DOF);
	Jacobian3_Inve.resize(3, KUKA_DOF);
	JacobianDirY.resize(3, KUKA_DOF);
	JacobianDirZ.resize(3, KUKA_DOF);
	Jacobian9.resize(9, KUKA_DOF);

	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);
	msg_robot_end.position.x = EndPos(0);
	msg_robot_end.position.y = EndPos(1);
	msg_robot_end.position.z = EndPos(2);
	msg_robot_end.orientation.x = 0;
	msg_robot_end.orientation.y = 0;
	msg_robot_end.orientation.z = 0;
	pub_end_of_robot_measured.publish(msg_robot_end);
	pub_end_of_robot_measured.publish(msg_robot_end);
	pub_end_of_robot_measured.publish(msg_robot_end);
	pub_end_of_robot_measured.publish(msg_robot_end);

	MathLib::Vector mJointVelLimitsUp;
	MathLib::Vector mJointVelLimitsDn;
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
}
void iiwa_ik::Topic_initialization()
{
	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();

	sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, &iiwa_ik::chatterCallback_position, this);
	pub_command_robot_real = n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);

	pub_end_of_robot_desired = n->advertise<geometry_msgs::Pose>("/IIWA/Desired_E_Pos", 3);

	pub_command = n->advertise<std_msgs::Int64>("/command", 3);
}

void iiwa_ik::prepare_sovlve_IK()
{

	mSKinematicChain->getJacobianPos(Jacobian3);

	mSKinematicChain->getJacobianPos(mJacobian3);				   // Get jacobian matrix which contributes to the position of the end-effector
	mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ); // Get jacobian matrix which contributes to the orientation of the end-effector around Z axis
	mSKinematicChain->getJacobianDirection(AXIS_Y, lJacobianDirY); // Get jacobian around which contributes to the orientation of the end-effector around Y axis

	for (int i = 0; i < 3; i++)
	{
		mJacobian9.SetRow(mJacobian3.GetRow(i), i);
		mJacobian9.SetRow((lJacobianDirZ.GetRow(i)) * 0.5, i + 3);
		mJacobian9.SetRow((lJacobianDirY.GetRow(i)) * 0.2, i + 6);
	}
}

RobotInterface::Status iiwa_ik::RobotInit()
{
	// This runs only one time, once the robot-toolkid module is started.

	JointPos.resize(KUKA_DOF);
	JointPos.setZero();
	Desired_JointPos.resize(KUKA_DOF);
	Desired_JointPos.setZero();
	JointPos_handle.resize(KUKA_DOF);
	JointPos_handle.setZero();

	// Inverse kinematic solver initialization.
	mEndEffectorId = mRobot->GetLinksCount() - 1;
	mKinematicChain.SetRobot(mRobot);
	mKinematicChain.Create(0, 0, mEndEffectorId);

	mIKSolver.SetSizes(KUKA_DOF);
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);
	mIKSolver.SetVerbose(false);									// No comments
	mIKSolver.SetThresholds(0.0001, 0.00001);						// Singularities thresholds
	mIKSolver.Enable(true, 0);										// Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(), 0); // Joint maps for first solver
	MathLib::Vector lJointWeight;
	lJointWeight.Resize(KUKA_DOF);

	// All the joints are going to contribute equally.
	lJointWeight(0) = 1;
	lJointWeight(1) = 1;
	lJointWeight(2) = 1;
	lJointWeight(3) = 1;
	lJointWeight(4) = 1.0;
	lJointWeight(5) = 1.0;
	lJointWeight(6) = 1.0;

	mIKSolver.SetDofsWeights(lJointWeight);

	mTargetVelocity9.Resize(IK_CONSTRAINTS);
	mJacobian9.Resize(9, KUKA_DOF);
	mJacobian3.Resize(3, KUKA_DOF);
	lJacobianDirZ.Resize(3, KUKA_DOF);
	lJacobianDirY.Resize(3, KUKA_DOF);
	mJointDesVel.Resize(KUKA_DOF);
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);

	Topic_initialization();
	Parameter_initialization();

	initKinematics();

	mPlanner = PLANNER_NONE;
	mCommand = COMMAND_NONE;

	flag_job = true;
	AddConsoleCommand("job");	// Move the robot to its initial position which is defined by cJob
	AddConsoleCommand("init");   // Initialize all the parameters, stops the robot and make it ready for polish!
	AddConsoleCommand("task"); // Excuting the task! reads the desired motion of the robot at the task level and convert them to the joint level motion while preparing the robot for contacting the surface.

	srand(time(NULL));

	Initia_time = ros::Time::now().toSec();
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotFree()
{
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotStart()
{

	while (!Position_of_the_robot_recieved)
	{
		ros::spinOnce();
	}

	Desired_JointPos = JointPos;

	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);
	Desired_EndPos = EndPos;

	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotStop()
{
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotUpdate()
{

	ros::spinOnce();

	switch (mCommand)
	{
	case COMMAND_INITIAL:
		mPlanner = PLANNER_NONE;
		if (!flag_init[0])
		{
			cout << "Initialization" << endl;
			ros::spinOnce();
			msg_command.data = COMMAND_INITIAL;
			Parameter_initialization();
			initKinematics();
			ros::spinOnce();
			Desired_JointPos = JointPos;
			flag_init[0] = true;
			reset_the_bool();
		}
		if (everythingisreceived() && !flag_init[1])
		{
			ros::spinOnce();
			Desired_JointPos = JointPos;
			mSKinematicChain->setJoints(JointPos.data());
			mSKinematicChain->getEndPos(EndPos);
			Desired_EndPos = EndPos;
			flag_init[1] = true;
			cout << "Initialization finished" << endl;
		}
		break;
	case COMMAND_JOB:
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_NONE;
		msg_command.data = COMMAND_JOB;
		ros::spinOnce();
		Desired_JointPos = JointPos;
		break;
	case COMMAND_Polish:
		if (everythingisreceived())
		{
			Desired_JointPos = JointPos;
			mSKinematicChain->setJoints(JointPos.data());
			mSKinematicChain->getEndPos(EndPos);
			Desired_EndPos = EndPos;
			mPlanner = PLANNER_CARTESIAN;
			mCommand = COMMAND_NONE;
			msg_command.data = COMMAND_Polish;
		}
		else
		{
			cout << "Position_of_the_robot_recieved " << Position_of_the_robot_recieved << endl;
		}
		break;
	}

	pub_command.publish(msg_command);

	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotUpdateCore()
{

	ros::spinOnce();


	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);

	mSKinematicChain->getEndDirAxis(AXIS_X, EndDirX);
	mSKinematicChain->getEndDirAxis(AXIS_Y, EndDirY);
	mSKinematicChain->getEndDirAxis(AXIS_Z, EndDirZ);

	MOrientation.block(0, 0, 3, 1) = EndDirX;
	MOrientation.block(0, 1, 3, 1) = EndDirY;
	MOrientation.block(0, 2, 3, 1) = EndDirZ;

	Orientation = MOrientation;

	switch (mPlanner)
	{
	case PLANNER_CARTESIAN:


		prepare_sovlve_IK();

		// Preparing the Inverse-Kinematic solver
		mTargetVelocity9(0) = (Desired_EndPos - EndPos)(0, 0) / dt;
		mTargetVelocity9(1) = (Desired_EndPos - EndPos)(1, 0) / dt;
		mTargetVelocity9(2) = (Desired_EndPos - EndPos)(2, 0) / dt;
		mTargetVelocity9(3) = (Desired_EndDirZ - EndDirZ)(0, 0) / (0.1 * Gain_Orientation);
		mTargetVelocity9(4) = (Desired_EndDirZ - EndDirZ)(1, 0) / (0.1 * Gain_Orientation);
		mTargetVelocity9(5) = (Desired_EndDirZ - EndDirZ)(2, 0) / (0.1 * Gain_Orientation);
		mTargetVelocity9(6) = (Desired_EndDirY - EndDirY)(0, 0) / (0.1 * Gain_Orientation);
		mTargetVelocity9(7) = (Desired_EndDirY - EndDirY)(1, 0) / (0.1 * Gain_Orientation);
		mTargetVelocity9(8) = (Desired_EndDirY - EndDirY)(2, 0) / (0.1 * Gain_Orientation);

		/*		cout<<"mTargetVelocity9 "<<mTargetVelocity9<<endl;*/

		// Preparing the Inverse-Kinematic solver, settign up the joint limits
		for (int i = 0; i < KUKA_DOF; i++)
		{
			double deltaLow = JointPos(i) + 0.9 * mSKinematicChain->getMax(i);
			double deltaHigh = 0.9 * mSKinematicChain->getMax(i) - JointPos(i);
			mJointVelLimitsDn(i) = -Gain_velocity_limit * mSKinematicChain->getMaxVel(i);
			mJointVelLimitsUp(i) = Gain_velocity_limit * mSKinematicChain->getMaxVel(i);
			if (deltaLow < 0.0)
				mJointVelLimitsDn(i) *= 0.0;
			else if (deltaLow < DEG2RAD(5.0))
				mJointVelLimitsDn(i) *= deltaLow / DEG2RAD(5.0);
			if (deltaHigh < 0.0)
				mJointVelLimitsUp(i) *= 0.0;
			else if (deltaHigh < DEG2RAD(5.0))
				mJointVelLimitsUp(i) *= deltaHigh / DEG2RAD(5.0);
		}

		mIKSolver.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);
		// Preparing the Inverse-Kinematic solver
		mIKSolver.SetJacobian(mJacobian9);
		mIKSolver.SetTarget(mTargetVelocity9, 0);
		// Solve the Inverse-Kinematic problem
		mIKSolver.Solve();
		// Get the joint space results, It is at the velocity level!
		mJointDesVel = mIKSolver.GetOutput();

		for (int i = 0; i < KUKA_DOF; i++)
		{
			Desired_JointVel(i) = mJointDesVel(i);
		}

		Desired_JointPos = JointPos + Desired_JointVel * dt;

		break;
	case PLANNER_JOINT:
		
		Desired_JointPos = JointPos + 0.005 * (1 / dt) * (cJob - JointPos) * dt;
		Desired_EndPos = EndPos;

		break;
	}

	msg_robot_end.position.x = EndPos(0);
	msg_robot_end.position.y = EndPos(1);
	msg_robot_end.position.z = EndPos(2);
	msg_robot_end.orientation.x = Orientation.x();
	msg_robot_end.orientation.y = Orientation.y();
	msg_robot_end.orientation.z = Orientation.z();
	msg_robot_end.orientation.w = Orientation.w();	

	pub_end_of_robot_measured.publish(msg_robot_end);

	Send_Postion_To_Robot(Desired_JointPos);

	return STATUS_OK;
}
int iiwa_ik::RespondToConsoleCommand(const string cmd, const vector<string> &args)
{

	if (cmd == "init")
	{
		if (!flag_job)
		{
			mPlanner = PLANNER_NONE;
			mCommand = COMMAND_INITIAL;
			flag_init[2] = true;
			flag_init[0] = false;
			flag_init[1] = false;
		}
	}
	else if (cmd == "job")
	{
		mCommand = COMMAND_JOB;
		mPlanner = PLANNER_NONE;
		flag_job = false;
		flag_init[2] = false;
	}
	else if (cmd == "task")
	{
		if ((!flag_job) && (flag_init[2]))
		{
			cout << "task!" << endl;
			mCommand = COMMAND_Polish;
			mPlanner = PLANNER_NONE;
			flag_job = true;
			flag_init[2] = false;
		}
	}

	return 0;
}

extern "C"
{
	// These two "C" functions manage the creation and destruction of the class
	iiwa_ik *create() { return new iiwa_ik(); }
	void destroy(iiwa_ik *module) { delete module; }
}