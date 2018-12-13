#include <vector>
#include "CoreRobotics.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

using namespace CoreRobotics;

class Solver {
public:
	Solver(ros::NodeHandle node);
	void setPoseElements(const geometry_msgs::Twist::ConstPtr& msg) {
		this->solver->setPoseElements((Eigen::Matrix<bool, 6, 1>() << (bool) msg->linear.x, (bool) msg->linear.y, (bool) msg->linear.z, (bool) msg->angular.x, (bool) msg->angular.y, (bool) msg->angular.z).finished());
	};
	//void setConfiguration(const sensor_msgs::JointState::ConstPtr& msg)
	//	this->solver.setConfiguration(Eigen::VectorXd(req.positions));
	//};
	void solve(const geometry_msgs::Twist::ConstPtr& msg);
private:
	int toolIndex;
	CRHardLimits* solver;
	ros::Publisher pub;
};

Solver::Solver(ros::NodeHandle node) {
	this->pub = node.advertise<sensor_msgs::JointState>("q", 10);
	#include "robot.h"
	CRManipulator* MyRobot = new CRManipulator();
	CRRigidBody* link;
	int linkIndex;
	for (std::vector<CRFrameEuler*>::iterator frame = frames.begin(); frame < frames.end(); frame++) {
		link = new CRRigidBody(*frame);
		linkIndex = MyRobot->addLink(link);
	}
	CRFrameEuler* tool = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	this->toolIndex = MyRobot->addTool(linkIndex, tool);
	this->solver = new CRHardLimits(*MyRobot, toolIndex, convention, true);
	this->solver->setQ0(Eigen::VectorXd::Zero(MyRobot->getDegreesOfFreedom()));
	this->solver->setJointMotion(Eigen::VectorXd::Zero(MyRobot->getDegreesOfFreedom()));
}

void Solver::solve(const geometry_msgs::Twist::ConstPtr& msg) {
	Eigen::VectorXd goal(6);
	goal << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
	this->solver->setToolPose(goal);
	Eigen::VectorXd q;
	this->solver->solve(q);
	std::vector<double> qv(&q[0], q.data() + q.size());
	sensor_msgs::JointState soln;
	soln.position = qv;
	this->pub.publish(soln);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ik");
	ros::NodeHandle node;
	Solver solver = Solver(node);
	ros::Subscriber solve_sub = node.subscribe("goal", 10, &Solver::solve, &solver);
	ros::spin();
}
