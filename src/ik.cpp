#include <vector>
#include "CoreRobotics.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

#define convention CR_EULER_MODE_XYZ

using namespace CoreRobotics;

class Solver {
public:
	Solver(ros::NodeHandle node);
	void setPoseElements(const geometry_msgs::Twist::ConstPtr& msg) {
		this->solver->setPoseElements((Eigen::Matrix<bool, 6, 1>() << (bool) msg->linear.x, (bool) msg->linear.y, (bool) msg->linear.z, (bool) msg->angular.x, (bool) msg->angular.y, (bool) msg->angular.z).finished());
	};
	void setConfiguration(const sensor_msgs::JointState::ConstPtr& msg) {
		this->solver->setQ0(Eigen::VectorXd::Map(msg->position.data(), msg->position.size()));
		this->fk();
	};
	void solve(const geometry_msgs::Twist::ConstPtr& msg);
private:
	void fk(void);
	int toolIndex;
	CRHardLimits* solver;
	CRManipulator* MyRobot;
	ros::Publisher joint_pub;
	ros::Publisher pose_pub;
};

Solver::Solver(ros::NodeHandle node) {
	this->joint_pub = node.advertise<sensor_msgs::JointState>("q", 10);
	this->pose_pub = node.advertise<geometry_msgs::Twist>("pose", 10);
	#include "robot.h"
	this->MyRobot = new CRManipulator();
	CRRigidBody* link;
	int linkIndex;
	for (std::vector<CRFrameEuler*>::iterator frame = frames.begin(); frame < frames.end(); frame++) {
		link = new CRRigidBody(*frame);
		linkIndex = this->MyRobot->addLink(link);
	}
	CRFrameEuler* tool = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	this->toolIndex = this->MyRobot->addTool(linkIndex, tool);
	this->solver = new CRHardLimits(*this->MyRobot, toolIndex, convention, true);
	this->solver->setQ0(Eigen::VectorXd::Zero(MyRobot->getDegreesOfFreedom()));
	this->solver->setJointMotion(Eigen::VectorXd::Zero(MyRobot->getDegreesOfFreedom()));
}

void Solver::solve(const geometry_msgs::Twist::ConstPtr& msg) {
	std::cout << "start solve" << std::endl;
	Eigen::VectorXd goal(6);
	goal << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
	this->solver->setToolPose(goal);
	Eigen::VectorXd q;
	CRResult result = this->solver->solve(q);
	if (result == CR_RESULT_SUCCESS) {
		std::vector<double> qv(&q[0], q.data() + q.size());
		sensor_msgs::JointState soln;
		soln.position = qv;
		this->solver->getIKSolver()->getRobot().setConfiguration(q);
		std::cout << goal << std::endl;
		std::cout << this->solver->getIKSolver()->getRobot().getForwardKinematics() << std::endl;
		std::cout << q << std::endl << std::endl;
		this->joint_pub.publish(soln);
		this->fk();
	} else {
		std::cout << "IK failed with error " << result << std::endl;
	}
}

void Solver::fk(void) {
	this->MyRobot->setConfiguration(this->solver->getQ0());
	Eigen::VectorXd pose = this->MyRobot->getToolPose(this->toolIndex, convention);
	geometry_msgs::Twist msg;
	msg.linear.x = pose[0];
	msg.linear.y = pose[1];
	msg.linear.z = pose[2];
	msg.angular.x = pose[3];
	msg.angular.y = pose[4];
	msg.angular.z = pose[5];
	this->pose_pub.publish(msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ik");
	ros::NodeHandle node;
	Solver solver = Solver(node);
	ros::Subscriber goal_sub = node.subscribe("goal", 10, &Solver::solve, &solver);
	ros::Subscriber config_sub = node.subscribe("config", 10, &Solver::setConfiguration, &solver);
	ros::spin();
}
