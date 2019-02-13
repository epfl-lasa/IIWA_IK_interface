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

#ifndef iiwa_ik_H_
#define iiwa_ik_H_

#include "RobotLib/RobotInterface.h"

#include "eigen3/Eigen/Dense"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"

#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "sKinematics.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"

const int KUKA_DOF = 7;			  // The number of robotic arm's joints!
int IK_CONSTRAINTS = 9;			  // Inverse kinematic constrains, 3 position, 3 orientation around Z axis and 3 for orientation around Y axis.
double Gain_velocity_limit = 1.5; // Velocity constrains of the robot is multiplied to Gain_velocity_limit.
enum ENUM_AXIS
{
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z
};
enum ENUM_PLANNER
{
	PLANNER_CARTESIAN = 0,
	PLANNER_JOINT,
	PLANNER_NONE
};
enum ENUM_COMMAND
{
	COMMAND_INITIAL = 0,
	COMMAND_JOB,
	COMMAND_Polish,
	COMMAND_NONE
};
double dt = 0.002;			 // Time sample
double Gain_Orientation = 1; // Orientation vs position constraint!. It should be more than zero and if it is more than one it works in favour of position and if it is between zero and one it works in favour of orientation.

using namespace std;
using namespace Eigen;
class iiwa_ik : public RobotInterface
{
  public:
	iiwa_ik();
	virtual ~iiwa_ik();

	virtual Status RobotInit();
	virtual Status RobotFree();

	virtual Status RobotStart();
	virtual Status RobotStop();

	virtual Status RobotUpdate();
	virtual Status RobotUpdateCore();

	virtual int RespondToConsoleCommand(const string cmd, const vector<string> &args);

  private:
	KinematicChain mKinematicChain;

	IKGroupSolver mIKSolver;

	int mEndEffectorId;
	MathLib::Matrix mJacobian9;
	MathLib::Vector mTargetVelocity9;
	MathLib::Matrix mJacobian3;
	MathLib::Matrix lJacobianDirZ;
	MathLib::Matrix lJacobianDirY;
	Vector mJointDesVel;
	Vector mJointVelLimitsDn;
	Vector mJointVelLimitsUp;
	void chatterCallback_position(const sensor_msgs::JointState &msg); // Callback for the position of the joints.
	void chatterCallback_Desired_end(const geometry_msgs::Pose &msg);
	void Send_Postion_To_Robot(VectorXd Position);

	void Topic_initialization();
	void Parameter_initialization();
	void initKinematics();
	void prepare_sovlve_IK(); // Prepare the inverse kinematic solver
	void reset_the_bool();
	bool everythingisreceived(); // Safety check

	bool flag_job;						 // Check if job command has been sent or not
	bool flag_init[3];					 // Check if init command has been sent or not
	bool Position_of_the_robot_recieved; // Check if robot's joint state is received or not

	ros::Subscriber sub_orginal_DS;
	ros::Subscriber sub_position_robot;

	ros::Publisher pub_command_robot_real;
	ros::Publisher pub_end_of_robot_desired;
	ros::Publisher pub_end_of_robot_measured;
	ros::Publisher pub_command;

	geometry_msgs::Pose msg_robot_end;
	std_msgs::Int64 msg_command;

	sKinematics *mSKinematicChain;
	Matrix4d T0;

	MatrixXd Jacobian3;
	MatrixXd Jacobian3_Inve;
	MatrixXd Jacobian9;
	MatrixXd JacobianDirY;
	MatrixXd JacobianDirZ;

	VectorXd cJob;

	VectorXd JointPos;
	VectorXd JointPos_handle;
	VectorXd Desired_JointVel;
	VectorXd Desired_JointPos;

	Vector3d EndPos;
	Vector3d EndDirX;
	Vector3d EndDirY;
	Vector3d EndDirZ;

	Vector3d Desired_EndPos;
	Vector3d Desired_EndDirY;
	Vector3d Desired_EndDirZ;

	Eigen::Quaternionf rotation_temp;
	Eigen::Matrix3f rot_mat_temp;

	Quaterniond Orientation;
	Matrix3d MOrientation;


	ENUM_COMMAND mCommand;
	ENUM_PLANNER mPlanner;
	double begin;
	double Initia_time;
};

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &mat)
{
	double eps = 0.0001;
	if (mat.rows() == mat.cols() && mat.determinant() > eps)
		return mat.inverse();

	else
	{
		if (mat.cols() == 1)
		{
			if (mat.isApproxToConstant(0))
			{
				return Eigen::VectorXd::Zero(mat.rows());
			}

			return mat.transpose() / (pow(mat.norm(), 2));
		}

		/// Use SVD decomposition.
		Eigen::JacobiSVD<Eigen::MatrixXd> jacSVD(
			mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd U = jacSVD.matrixU();
		Eigen::MatrixXd V = jacSVD.matrixV();
		Eigen::VectorXd S = jacSVD.singularValues();

		Eigen::MatrixXd S_inv(Eigen::MatrixXd::Zero(mat.cols(), mat.rows()));

		for (int i = 0; i < S.rows(); i++)
		{
			if (S(i) > eps)
			{
				S_inv(i, i) = 1.0 / S(i);
			}
			else
			{
				S_inv(i, i) = 0;
			}
		}

		return V * S_inv * U.transpose();
	}
}
#endif